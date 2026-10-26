"""
cameraSetup.py

Camera access for the Arducam ezBOX: V4L2 control reads/writes (Linux only,
via v4l2-ctl), format listing, and FrameGrabber, which drains the capture
queue in a background thread and hands out the newest grayscale frame.

On macOS the camera is opened through AVFoundation and the V4L2 controls are
unavailable, so the camera runs on its own settings.
"""

import collections
import glob
import re
import subprocess
import sys
import threading
import time

import cv2
import numpy as np


# ============================================================================
# Camera controls
# ============================================================================

IS_MAC = sys.platform == "darwin"


def device_label(index):
    """V4L2 device path on Linux; AVFoundation camera index on macOS."""
    return f"avfoundation:{index}" if IS_MAC else f"/dev/video{index}"


CTRL_RE = re.compile(r"^\s*(\w+)\s+0x[0-9a-fA-F]+\s+\((\w+)\)\s*:(.*)$")


def v4l2(device, *args):
    """Run v4l2-ctl. Returns stdout, or None if the tool is missing or failed."""
    try:
        r = subprocess.run(["v4l2-ctl", "-d", device, *args],
                           capture_output=True, text=True, timeout=5)
        return r.stdout if r.returncode == 0 else None
    except (subprocess.SubprocessError, FileNotFoundError):
        return None


def read_ctrls(device):
    """{name: (value, inactive)} for every control the camera reports."""
    out = v4l2(device, "--list-ctrls")
    if out is None:
        return None
    ctrls = {}
    for line in out.splitlines():
        m = CTRL_RE.match(line)
        if not m:
            continue
        name, _type, rest = m.groups()
        v = re.search(r"\bvalue=(-?\d+)", rest)
        if v:
            ctrls[name] = (int(v.group(1)), "inactive" in rest)
    return ctrls


def set_ctrl(device, names, value):
    """Set one control (first supported name) and return the read-back value."""
    ctrls = read_ctrls(device) or {}
    name = next((n for n in names if n in ctrls), None)
    if name is None:
        return None
    v4l2(device, "--set-ctrl", f"{name}={value}")
    after = read_ctrls(device) or {}
    return after.get(name, (None, False))[0]


def exposure_units(exposure_us):
    """UVC exposure_time_absolute is in 100 us units; 1 == 100 us."""
    return max(1, int(round(exposure_us / 100.0)))


EXPOSURE_NAMES = ("exposure_time_absolute", "exposure_absolute")


def desired_controls(settings):
    """
    Ordered (candidate names, value) pairs. Order matters: auto_exposure must
    be manual before exposure_time_absolute will accept a write. Legacy names
    are listed as fallbacks for older kernels.
    """
    return [
        # 1 == Manual Mode. NOT 0 -- 0 is full auto. 3 (Aperture Priority) is
        # the camera default, which is why exposure shows flags=inactive.
        (("auto_exposure", "exposure_auto"), 1),
        # Never let the camera drop framerate to fit a longer exposure.
        (("exposure_dynamic_framerate", "exposure_auto_priority"), 0),
        (EXPOSURE_NAMES, settings.exp_units),
        (("gain",), settings.gain),
        # Mains-flicker avoidance. Lighting is DC-driven, so this only adds a
        # hidden constraint on exposure.
        (("power_line_frequency",), 0),
        (("backlight_compensation",), 0),
        # Edge enhancement adds ringing around the ball and biases centroids.
        (("sharpness",), 0),
    ]


def apply_controls(device, desired):
    """Write each control, then read everything back. Returns True if all stuck."""
    before = read_ctrls(device)
    if before is None:
        if IS_MAC:
            print("  camera controls need v4l2-ctl (Linux only) -- using camera defaults")
        else:
            print("  v4l2-ctl unavailable -- cannot set controls (sudo apt install v4l-utils)")
        return False

    for names, value in desired:
        name = next((n for n in names if n in before), None)
        if name is not None:
            v4l2(device, "--set-ctrl", f"{name}={value}")

    after = read_ctrls(device) or {}
    all_ok = True
    for names, value in desired:
        name = next((n for n in names if n in after), None)
        if name is None:
            print(f"    {names[0]:<28} not supported, skipped")
            continue
        got, inactive = after[name]
        if got != value:
            status, all_ok = f"MISMATCH (wanted {value})", False
        elif inactive:
            status, all_ok = "INACTIVE -- camera still owns it", False
        else:
            status = "ok"
        print(f"    {name:<28} = {got:<6} {status}")
    return all_ok


def live_exposure(device):
    """(auto_exposure, exposure_units) as the camera reports them right now."""
    c = read_ctrls(device) or {}
    ae = c.get("auto_exposure", c.get("exposure_auto", (None, False)))[0]
    ex = c.get("exposure_time_absolute", c.get("exposure_absolute", (None, False)))[0]
    return ae, ex


def list_formats(device):
    if IS_MAC:
        # AVFoundation indices follow the order macOS enumerates cameras,
        # which is usually (not always) the order listed here.
        out = subprocess.run(["system_profiler", "SPCameraDataType"],
                             capture_output=True, text=True).stdout
        print(out)
        print("pass --device N to pick a camera (N is its 0-based AVFoundation index)")
        return
    out = v4l2(device, "--list-formats-ext")
    print(out if out else "v4l2-ctl not found. sudo apt install v4l-utils")


def check_cpu_governor():
    govs = set()
    for p in glob.glob("/sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_governor"):
        try:
            with open(p) as f:
                govs.add(f.read().strip())
        except OSError:
            pass
    return ", ".join(sorted(govs)) if govs else None


# ============================================================================
# Frame acquisition
# ============================================================================

def to_gray(frame, w, h):
    """
    Turn whatever OpenCV handed back into a single-channel image.

    Handles every shape the V4L2 backend can produce depending on the OpenCV
    version and whether raw passthrough (CONVERT_RGB=0) took effect.
    """
    if frame.ndim == 2 and frame.shape == (h, w):
        return frame
    if frame.ndim == 3 and frame.shape[2] == 3:
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if frame.ndim == 3 and frame.shape[2] == 2:
        # Raw YUYV. On a mono sensor the Y plane IS the image.
        return np.ascontiguousarray(frame[:, :, 0])
    buf = frame.reshape(-1)
    if buf.size == w * h * 2:
        return np.ascontiguousarray(buf.reshape(h, w, 2)[:, :, 0])
    # Raw MJPEG bitstream: decode straight to grayscale.
    return cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)


class FrameGrabber:
    """
    Drains the V4L2 queue continuously in a background thread and decodes
    every frame to grayscale, so the consumer always gets the newest frame.

    Timestamps are taken the moment read() returns -- after USB transfer,
    before decode -- and capture intervals are measured HERE, on every frame,
    so they reflect true camera cadence regardless of consumer speed.
    """

    LIVE = 400  # samples kept for the viewer (~2 s at 200 FPS)

    def __init__(self, index, width, height, fps, fourcc, raw):
        if IS_MAC:
            # No V4L2 on macOS; AVFoundation also ignores CONVERT_RGB, so
            # frames always arrive as BGR and to_gray() takes the cvtColor path.
            self.cap = cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)
            raw = False
        else:
            self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            hint = (" -- try another --device index, and allow camera access for "
                    "your terminal in System Settings > Privacy & Security > Camera"
                    if IS_MAC else "")
            raise RuntimeError(f"could not open {device_label(index)}{hint}")

        # FOURCC MUST precede frame size or the driver may renegotiate back
        # to YUYV and silently cap you at ~60 FPS.
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # often ignored; the thread is the real fix
        if raw:
            # Ask for the undecoded buffer. Not every OpenCV build honors
            # this; to_gray() copes either way.
            self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)

        self.w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.lock = threading.Lock()
        self.gray = None
        self.stamp = None
        self.seq = 0
        self.first_frame = threading.Event()
        self.raw_active = None          # determined from the first frame
        self.running = True
        self.reset_stats()

        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def reset_stats(self):
        with self.lock:
            self.grab_count = 0
            self.read_fails = 0
            self.decode_fails = 0
            self.intervals = collections.deque(maxlen=50000)
            self.decode_ms = collections.deque(maxlen=50000)
            self.live_intervals = collections.deque(maxlen=self.LIVE)
            self.live_decode = collections.deque(maxlen=self.LIVE)
            self.last_stamp = None

    def negotiated(self):
        cc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        return {
            "width": self.w,
            "height": self.h,
            "fps": self.cap.get(cv2.CAP_PROP_FPS),
            "fourcc": "".join(chr((cc >> 8 * i) & 0xFF) for i in range(4)),
        }

    def _loop(self):
        while self.running:
            ok, frame = self.cap.read()
            t = time.monotonic()
            if not ok or frame is None:
                self.read_fails += 1
                continue

            if self.raw_active is None:
                self.raw_active = not (frame.ndim == 3 and frame.shape[2] == 3)

            t0 = time.perf_counter()
            gray = to_gray(frame, self.w, self.h)
            dt_dec = (time.perf_counter() - t0) * 1000.0
            if gray is None:
                self.decode_fails += 1
                continue

            with self.lock:
                if self.last_stamp is not None:
                    iv = (t - self.last_stamp) * 1000.0
                    self.intervals.append(iv)
                    self.live_intervals.append(iv)
                self.last_stamp = t
                self.decode_ms.append(dt_dec)
                self.live_decode.append(dt_dec)
                self.gray = gray
                self.stamp = t
                self.seq += 1
                self.grab_count += 1
            self.first_frame.set()

    def latest(self):
        with self.lock:
            return self.gray, self.stamp, self.seq

    def release(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.cap.release()
