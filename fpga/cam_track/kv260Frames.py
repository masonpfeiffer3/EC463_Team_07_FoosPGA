"""
kv260Frames.py

Frame source for the KV260: a drop-in replacement for cameraFrames.FrameGrabber
that gets frames from the FPGA (frame_to_bram + AXI GPIO + frame BRAM, see
camera_stage3) instead of a USB camera. Same interface as cameraFrames:

    latest()        -> (gray, stamp, seq) of the newest frame
    get_frames()    -> every frame since the last call, [(seq, stamp, gray), ...]
    wait_frames(t)  -> same, but waits up to t seconds for at least one
    first_frame, lock, cond, live_intervals, dropped, w, h, negotiated(), release()

so tracker.py and view_tracker.render() work on it unchanged.

How one frame is fetched (the 4-phase handshake with frame_to_bram):
    1. capture_req = 1                  (GPIO channel 1)
    2. wait for capture_done = 1        (GPIO channel 2): the FPGA has copied
                                        the next WHOLE frame into BRAM and stopped
    3. copy 640 x 400 bytes out of the BRAM (through /dev/mem)
    4. capture_req = 0, wait for capture_done = 0

Timestamps are time.monotonic() taken the moment capture_done is seen, i.e.
right after the frame finished arriving. Because a new request has to wait for
the next frame start, at most every other camera frame is fetched (~130 fps
with the camera at 260 fps). The Kalman filter uses real dt, so that's fine.

Camera settings (exposure, gain, frame rate) are NOT set here: run
ov9281_init.py first.

Run this file directly for a capture benchmark on the Kria:
    sudo python3 kv260Frames.py --seconds 5
    python3 kv260Frames.py --fake            # no hardware
"""

import collections
import mmap
import os
import struct
import threading
import time

import numpy as np

WIDTH, HEIGHT = 640, 400

GPIO_DATA = 0x00         # AXI GPIO channel 1 (output): bit 0 = capture_req
GPIO2_DATA = 0x08        # AXI GPIO channel 2 (input):  bit 0 = capture_done


# ============================================================================
# Frame sources: each has grab() -> (gray, stamp) and close()
# ============================================================================

class _Regs:
    """A block of FPGA addresses mapped into Python through /dev/mem."""

    def __init__(self, base, size):
        fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
        self.mem = mmap.mmap(fd, size, mmap.MAP_SHARED,
                             mmap.PROT_READ | mmap.PROT_WRITE, offset=base)
        os.close(fd)

    def read(self, off):
        return struct.unpack_from("<I", self.mem, off)[0]

    def write(self, off, value):
        struct.pack_into("<I", self.mem, off, value)


class FpgaSource:
    """Frames from frame_to_bram. Addresses = Vivado Address Editor."""

    def __init__(self, gpio_base=0xB0020000, bram_base=0xB0040000, safe_copy=False, crop=None):
        self.gpio = _Regs(gpio_base, 0x1000)
        self.bram = _Regs(bram_base, 0x40000)          # 256 KB window
        # BRAM as 400 rows x 160 words (4 pixels per word). With a crop only
        # columns x0..x1 are read, so less data crosses the bus.
        words = np.frombuffer(self.bram.mem, np.uint32,
                              count=WIDTH * HEIGHT // 4).reshape(HEIGHT, WIDTH // 4)
        self.x0, self.x1 = crop or (0, WIDTH)
        self.words = words[:, self.x0 // 4:self.x1 // 4]
        self.safe_copy = safe_copy
        self.w, self.h = self.x1 - self.x0, HEIGHT
        self.gpio.write(GPIO_DATA, 0)

    def _wait_done(self, want, timeout):
        end = time.monotonic() + timeout
        while (self.gpio.read(GPIO2_DATA) & 1) != want:
            if time.monotonic() > end:
                return False
            time.sleep(0.0001)       # sleep (not spin) so other threads get the CPU
        return True

    def grab(self):
        self.gpio.write(GPIO_DATA, 1)                   # 1. ask
        if not self._wait_done(1, 0.5):                 # 2. wait for a whole frame
            self.gpio.write(GPIO_DATA, 0)
            raise TimeoutError("no frame from the FPGA in 0.5 s: is the camera "
                               "streaming? (ov9281_init.py, csi_status.py)")
        stamp = time.monotonic()
        buf = np.empty(self.words.shape, np.uint32)     # new array: consumers keep frames
        if self.safe_copy:
            np.bitwise_or(self.words, 0, out=buf)       # word-by-word reads
        else:
            np.copyto(buf, self.words)                  # 3. copy (only the cropped columns)
        self.gpio.write(GPIO_DATA, 0)                   # 4. release
        self._wait_done(0, 0.1)
        return buf.view(np.uint8).reshape(self.h, self.w), stamp

    def close(self):
        self.gpio.write(GPIO_DATA, 0)


class FakeSource:
    """No hardware: dark table, rods, a bright ball bouncing. ~130 fps like the real thing."""

    def __init__(self, fps=130.0, ball_radius=9, crop=None):
        self.x0, self.x1 = crop or (0, WIDTH)
        self.w, self.h = self.x1 - self.x0, HEIGHT
        self.period = 1.0 / fps
        self.r = ball_radius
        self.x, self.y, self.vx, self.vy = 120.0, 150.0, 650.0, 380.0
        self.t = time.monotonic()
        yy, xx = np.mgrid[0:HEIGHT, 0:WIDTH]
        self.yy, self.xx = yy, xx
        self.table = np.full((HEIGHT, WIDTH), 45, np.uint8)
        self.table[(yy % 133) < 6] = 120
        self.rng = np.random.default_rng(1)

    def grab(self):
        time.sleep(max(0.0, self.t + self.period - time.monotonic()))
        now = time.monotonic()
        dt, self.t = now - self.t, now
        self.x += self.vx * dt
        self.y += self.vy * dt
        if self.x < 20:           self.vx = abs(self.vx)
        if self.x > WIDTH - 20:   self.vx = -abs(self.vx)
        if self.y < 20:           self.vy = abs(self.vy)
        if self.y > HEIGHT - 20:  self.vy = -abs(self.vy)
        frame = self.table + self.rng.integers(0, 10, (HEIGHT, WIDTH), dtype=np.uint8)
        frame[(self.xx - self.x) ** 2 + (self.yy - self.y) ** 2 < self.r ** 2] = 230
        return np.ascontiguousarray(frame[:, self.x0:self.x1]), now

    def close(self):
        pass


def parse_crop(text):
    """'200:480' -> (200, 480), rounded to whole 4-pixel words. None -> None."""
    if not text:
        return None
    x0, x1 = (int(v) for v in text.split(":"))
    x0 = max(0, min(WIDTH - 4, x0 // 4 * 4))          # round down
    x1 = max(x0 + 4, min(WIDTH, (x1 + 3) // 4 * 4))   # round up
    return x0, x1


# ============================================================================
# FrameGrabber: same interface as cameraFrames.FrameGrabber
# ============================================================================

class FrameGrabber:
    """
    Background thread that pulls frames from `source` as fast as it can and
    exports them exactly like cameraFrames.FrameGrabber does.
    """

    LIVE = 400

    def __init__(self, source, queue_len=64):
        self.src = source
        self.w, self.h = source.w, source.h
        self.lock = threading.Lock()
        self.cond = threading.Condition(self.lock)
        self.gray = None
        self.stamp = None
        self.seq = 0
        self.frames = collections.deque(maxlen=queue_len)
        self._export = False
        self.first_frame = threading.Event()
        self.raw_active = True
        self.error = None               # last grab error (shown by the viewer)
        self.running = True
        self.reset_stats()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def reset_stats(self):
        with self.lock:
            self.grab_count = 0
            self.read_fails = 0
            self.decode_fails = 0
            self.dropped = 0
            self.intervals = collections.deque(maxlen=50000)
            self.decode_ms = collections.deque(maxlen=50000)    # here: BRAM copy time
            self.live_intervals = collections.deque(maxlen=self.LIVE)
            self.live_decode = collections.deque(maxlen=self.LIVE)
            self.last_stamp = None

    def negotiated(self):
        return {"width": self.w, "height": self.h, "fps": None,
                "fourcc": "Y8 (FPGA frame BRAM)"}

    def _loop(self):
        while self.running:
            t0 = time.perf_counter()
            try:
                gray, t = self.src.grab()
            except TimeoutError as e:
                self.read_fails += 1
                self.error = str(e)
                continue
            self.error = None
            dt_copy = (time.perf_counter() - t0) * 1000.0

            with self.lock:
                if self.last_stamp is not None:
                    iv = (t - self.last_stamp) * 1000.0
                    self.intervals.append(iv)
                    self.live_intervals.append(iv)
                self.last_stamp = t
                self.decode_ms.append(dt_copy)
                self.live_decode.append(dt_copy)
                self.seq += 1
                self.gray = gray
                self.stamp = t
                self.grab_count += 1
                if self._export:
                    if len(self.frames) == self.frames.maxlen:
                        self.dropped += 1
                    self.frames.append((self.seq, t, gray))
                    self.cond.notify_all()
            if not self.first_frame.is_set():
                self.first_frame.set()

    def latest(self):
        with self.lock:
            return self.gray, self.stamp, self.seq

    def get_frames(self):
        with self.lock:
            self._export = True
            out = list(self.frames)
            self.frames.clear()
        return out

    def wait_frames(self, timeout=0.1):
        with self.cond:
            self._export = True
            if not self.frames:
                self.cond.wait(timeout)
            out = list(self.frames)
            self.frames.clear()
        return out

    def release(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.src.close()


# ============================================================================
# Benchmark: how fast can we really pull frames?
# ============================================================================

def _benchmark():
    import argparse
    ap = argparse.ArgumentParser(description="KV260 frame grab benchmark")
    ap.add_argument("--fake", action="store_true")
    ap.add_argument("--gpio-base", type=lambda s: int(s, 0), default=0xB0020000)
    ap.add_argument("--bram-base", type=lambda s: int(s, 0), default=0xB0040000)
    ap.add_argument("--safe-copy", action="store_true")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--crop", metavar="X0:X1", help="keep only columns X0..X1, e.g. 200:480")
    args = ap.parse_args()

    crop = parse_crop(args.crop)
    src = FakeSource(crop=crop) if args.fake else \
        FpgaSource(args.gpio_base, args.bram_base, args.safe_copy, crop)
    grabber = FrameGrabber(src)
    if not grabber.first_frame.wait(5.0):
        print("no frames within 5 s:", grabber.error)
        grabber.release()
        return 1
    grabber.reset_stats()
    got = 0
    t_end = time.monotonic() + args.seconds
    while time.monotonic() < t_end:
        got += len(grabber.wait_frames(0.1))
    with grabber.lock:
        iv = np.array(grabber.intervals)
        cp = np.array(grabber.decode_ms)
        fails = grabber.read_fails
    grabber.release()
    if iv.size:
        print(f"grab rate : {1000.0 / iv.mean():.1f} fps "
              f"(interval mean {iv.mean():.2f} ms, p99 {np.percentile(iv, 99):.2f} ms)")
    if cp.size:
        print(f"grab time : mean {cp.mean():.2f} ms (wait for frame + BRAM copy)")
    print(f"exported  : {got} frames, {fails} timeouts")
    return 0


if __name__ == "__main__":
    raise SystemExit(_benchmark())
