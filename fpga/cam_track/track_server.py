#!/usr/bin/env python3
# =============================================================================
# track_server.py  --  live ball tracking on the KV260, shown in a web browser
# =============================================================================
#
# WHAT IT DOES
#   1. Grabs a frame from the FPGA:
#        - sets capture_req = 1 (AXI GPIO channel 1)
#        - waits for capture_done = 1 (AXI GPIO channel 2)
#        - copies the 640x400 picture out of the frame BRAM
#        - sets capture_req = 0 and waits for capture_done = 0
#   2. Finds the ball: the bright spot (or, with a background picture, the
#      spot that changed), and its centre.
#   3. Serves a web page that shows the live picture with the ball marked.
#
#   Both blocks are reached through /dev/mem, exactly like csi_status.py.
#   Needs: Python 3 + numpy. (OpenCV is NOT needed.)
#
# HOW TO RUN (on the Kria, camera already started with ov9281_init.py)
#   sudo python3 track_server.py                      # live view on port 8080
#   sudo python3 track_server.py --snapshot frame     # save ONE frame and quit
#   python3 track_server.py --fake                    # no hardware: fake ball
#
#   Then on the laptop:
#   ssh -L 8080:localhost:8080 <user>@<kria-ip>       # keep this open
#   open http://localhost:8080
#
# ADDRESSES: the defaults match the guide. If Vivado's Address Editor gave
#   other ones, pass --gpio-base 0x... and --bram-base 0x...
# =============================================================================

import argparse
import collections
import json
import mmap
import os
import struct
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np

WIDTH, HEIGHT = 640, 400

# AXI GPIO registers (AMD PG144)
GPIO_DATA  = 0x00      # channel 1: our output -> capture_req (bit 0)
GPIO2_DATA = 0x08      # channel 2: our input  <- capture_done (bit 0)


# =============================================================================
# 1. Talking to the FPGA
# =============================================================================
class Regs:
    """A block of FPGA addresses, mapped into Python through /dev/mem."""

    def __init__(self, base, size):
        fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
        self.mem = mmap.mmap(fd, size, mmap.MAP_SHARED,
                             mmap.PROT_READ | mmap.PROT_WRITE, offset=base)
        os.close(fd)

    def read(self, offset):
        return struct.unpack_from("<I", self.mem, offset)[0]

    def write(self, offset, value):
        struct.pack_into("<I", self.mem, offset, value)


class FpgaCamera:
    """Gets frames from frame_to_bram through the GPIO handshake."""

    def __init__(self, gpio_base, bram_base, safe_copy=False, crop=None):
        self.gpio = Regs(gpio_base, 0x1000)
        self.bram = Regs(bram_base, 0x40000)          # 256 KB window
        # The BRAM seen as 400 rows x 160 words (each word = 4 pixels)
        words = np.frombuffer(self.bram.mem, dtype=np.uint32,
                              count=WIDTH * HEIGHT // 4).reshape(HEIGHT, WIDTH // 4)
        # Crop: only read the columns x0..x1 (multiples of 4, so whole words).
        # Fewer bytes over the bus = faster grab.
        self.x0, self.x1 = crop or (0, WIDTH)
        self.words = words[:, self.x0 // 4:self.x1 // 4]
        self.buffer = np.empty(self.words.shape, dtype=np.uint32)
        self.safe_copy = safe_copy
        self.gpio.write(GPIO_DATA, 0)                 # start with no request

    def _wait_done(self, want, timeout):
        end = time.perf_counter() + timeout
        while (self.gpio.read(GPIO2_DATA) & 1) != want:
            if time.perf_counter() > end:
                return False
        return True

    def grab(self):
        """Return one frame as a 400 x 640 array of 8-bit pixels."""
        self.gpio.write(GPIO_DATA, 1)                 # 1. ask for a frame
        if not self._wait_done(1, 0.5):               # 2. wait for it
            self.gpio.write(GPIO_DATA, 0)
            raise TimeoutError("no frame from the FPGA in 0.5 s: is the camera "
                               "streaming? (run ov9281_init.py, check csi_status.py)")
        # 3. copy it out while the FPGA holds it still
        if self.safe_copy:
            np.bitwise_or(self.words, 0, out=self.buffer)   # word-by-word reads
        else:
            np.copyto(self.buffer, self.words)              # one fast block copy
        self.gpio.write(GPIO_DATA, 0)                 # 4. done reading
        self._wait_done(0, 0.1)                       # 5. FPGA lets go
        return self.buffer.view(np.uint8).reshape(HEIGHT, self.x1 - self.x0).copy()


class FakeCamera:
    """No hardware: a dark table, two rods and a bright ball bouncing around."""

    def __init__(self, crop=None):
        self.x0, self.x1 = crop or (0, WIDTH)
        self.x, self.y = 100.0, 120.0
        self.vx, self.vy = 420.0, 270.0               # pixels per second
        self.t = time.perf_counter()
        yy, xx = np.mgrid[0:HEIGHT, 0:WIDTH]
        self.yy, self.xx = yy, xx
        self.table = np.full((HEIGHT, WIDTH), 45, np.uint8)
        self.table[(yy % 133) < 6] = 120              # rods
        self.rng = np.random.default_rng(0)

    def grab(self):
        time.sleep(1 / 120)                           # pretend 120 fps
        now = time.perf_counter()
        dt, self.t = now - self.t, now
        self.x += self.vx * dt
        self.y += self.vy * dt
        if self.x < 10:           self.vx = abs(self.vx)      # bounce off the walls
        if self.x > WIDTH - 10:   self.vx = -abs(self.vx)
        if self.y < 10:           self.vy = abs(self.vy)
        if self.y > HEIGHT - 10:  self.vy = -abs(self.vy)
        frame = self.table + self.rng.integers(0, 12, (HEIGHT, WIDTH), dtype=np.uint8)
        ball = (self.xx - self.x) ** 2 + (self.yy - self.y) ** 2 < 9 ** 2
        frame[ball] = 230
        return frame[:, self.x0:self.x1].copy()


# =============================================================================
# 2. Finding the ball
# =============================================================================
class Tracker:
    """
    Very simple tracker, good enough to start with:
      - mask = pixels brighter than `thresh`
        (or, if a background picture is set, pixels that differ from it by
         more than `thresh`: works for any ball colour, ignores the table)
      - rough centre = median of the mask pixels (a few stray pixels can't
        pull it far)
      - final centre = average of the mask pixels within `window` of the
        rough centre
    """

    def __init__(self, thresh=200, min_pixels=15, window=30):
        self.thresh = thresh
        self.min_pixels = min_pixels
        self.window = window
        self.background = None

    def find(self, frame):
        if self.background is not None:
            diff = np.abs(frame.astype(np.int16) - self.background)
            mask = diff > self.thresh
        else:
            mask = frame > self.thresh

        ys, xs = np.nonzero(mask)
        if xs.size < self.min_pixels:
            return None, int(xs.size)

        cx, cy = np.median(xs), np.median(ys)
        near = (np.abs(xs - cx) < self.window) & (np.abs(ys - cy) < self.window)
        n = int(near.sum())
        if n < self.min_pixels:
            return None, n
        return (float(xs[near].mean()), float(ys[near].mean())), n


# =============================================================================
# 3. The capture + tracking loop (runs in its own thread)
# =============================================================================
class Shared:
    """Latest results, shared between the loop and the web server."""

    def __init__(self):
        self.lock = threading.Lock()
        self.frame = np.zeros((HEIGHT, WIDTH), np.uint8)
        self.state = {"found": False, "error": "starting..."}
        self.trail = collections.deque(maxlen=40)
        self.want_background = False     # set by the web page's button


def capture_loop(camera, tracker, shared, log):
    frame_id = 0
    last_pos, last_t = None, None
    fps, fps_count, fps_t = 0.0, 0, time.perf_counter()

    while True:
        t0 = time.perf_counter()
        try:
            frame = camera.grab()
        except TimeoutError as e:
            with shared.lock:
                shared.state["error"] = str(e)
            time.sleep(0.2)
            continue
        t1 = time.perf_counter()

        if shared.want_background:                    # button was pressed
            tracker.background = frame.astype(np.int16)
            shared.want_background = False

        pos, n = tracker.find(frame)
        t2 = time.perf_counter()

        # speed in pixels per second, from the previous position
        vx = vy = 0.0
        if pos and last_pos:
            dt = t1 - last_t
            vx, vy = (pos[0] - last_pos[0]) / dt, (pos[1] - last_pos[1]) / dt
        last_pos, last_t = (pos, t1) if pos else (None, None)

        frame_id += 1
        fps_count += 1
        if t2 - fps_t >= 1.0:
            fps, fps_count, fps_t = fps_count / (t2 - fps_t), 0, t2

        with shared.lock:
            shared.frame = frame
            if pos:
                shared.trail.append((round(pos[0], 1), round(pos[1], 1)))
            shared.state = {
                "frame": frame_id,
                "found": pos is not None,
                "x": round(pos[0], 1) if pos else None,
                "y": round(pos[1], 1) if pos else None,
                "vx": round(vx), "vy": round(vy),
                "pixels": n,
                "fps": round(fps, 1),
                "grab_ms": round((t1 - t0) * 1000, 1),
                "track_ms": round((t2 - t1) * 1000, 1),
                "thresh": tracker.thresh,
                "background": tracker.background is not None,
                "trail": list(shared.trail),
                "error": None,
            }

        if log:
            log.write(f"{t1:.6f},{frame_id},{pos[0] if pos else ''},{pos[1] if pos else ''}\n")


# =============================================================================
# 4. The web server
# =============================================================================
PAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "track_view.html")


def make_handler(shared, tracker):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):                 # keep the terminal quiet
            pass

        def send(self, body, ctype, extra=None):
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            url = urlparse(self.path)
            q = parse_qs(url.query)

            if url.path == "/":
                with open(PAGE, "rb") as f:
                    self.send(f.read(), "text/html; charset=utf-8")

            elif url.path == "/frame":
                # Raw 8-bit pixels (scale 2 = every 2nd pixel, 4x less data),
                # with the tracking result in the X-State header.
                scale = 2 if q.get("scale") == ["2"] else 1
                with shared.lock:
                    img = np.ascontiguousarray(shared.frame[::scale, ::scale])
                    full_w = shared.frame.shape[1]
                    state = dict(shared.state)
                state["w"], state["h"] = img.shape[1], img.shape[0]
                state["fw"] = full_w              # frame width before scaling (for the overlay)
                self.send(img.tobytes(), "application/octet-stream",
                          {"X-State": json.dumps(state)})

            elif url.path == "/set":
                if "thresh" in q:
                    tracker.thresh = max(0, min(255, int(q["thresh"][0])))
                if "background" in q:
                    if q["background"][0] == "1":
                        shared.want_background = True
                    else:
                        tracker.background = None
                self.send(b"ok", "text/plain")

            else:
                self.send_error(404)

    return Handler


# =============================================================================
# 5. Saving one frame (--snapshot)
# =============================================================================
def write_png(path, img):
    """8-bit grayscale PNG with only the standard library."""
    h, w = img.shape
    raw = b"".join(b"\x00" + img[y].tobytes() for y in range(h))

    def chunk(kind, data):
        body = kind + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body))

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(raw, 6)))
        f.write(chunk(b"IEND", b""))


def snapshot(camera, tracker, name):
    t0 = time.perf_counter()
    frame = camera.grab()
    t1 = time.perf_counter()
    write_png(name + ".png", frame)
    pos, n = tracker.find(frame)
    print(f"grabbed a frame in {(t1 - t0) * 1000:.1f} ms -> {name}.png")
    print(f"pixel values: min {frame.min()}, mean {frame.mean():.0f}, max {frame.max()}")
    print(f"pixels above {tracker.thresh}: {int((frame > tracker.thresh).sum())}")
    print(f"ball: {pos} ({n} pixels)" if pos else "ball: not found")


# =============================================================================
# main
# =============================================================================
def parse_crop(text):
    """'200:480' -> (200, 480), rounded to whole 4-pixel words. None -> None."""
    if not text:
        return None
    x0, x1 = (int(v) for v in text.split(":"))
    x0 = max(0, min(WIDTH - 4, x0 // 4 * 4))          # round down
    x1 = max(x0 + 4, min(WIDTH, (x1 + 3) // 4 * 4))   # round up
    return x0, x1


def main():
    p = argparse.ArgumentParser(description="Live ball tracking web view")
    p.add_argument("--fake", action="store_true", help="no hardware, fake ball")
    p.add_argument("--snapshot", metavar="NAME", help="save one frame as NAME.png and quit")
    p.add_argument("--gpio-base", type=lambda s: int(s, 0), default=0xB0020000)
    p.add_argument("--bram-base", type=lambda s: int(s, 0), default=0xB0040000)
    p.add_argument("--host", default="127.0.0.1",
                   help="127.0.0.1 = only via SSH tunnel (safe); 0.0.0.0 = anyone on the network")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--thresh", type=int, default=200, help="brightness threshold 0-255")
    p.add_argument("--min-pixels", type=int, default=15)
    p.add_argument("--log", metavar="CSV", help="write time,frame,x,y for every frame")
    p.add_argument("--safe-copy", action="store_true",
                   help="read the BRAM word by word (try this if you get 'Bus error')")
    p.add_argument("--crop", metavar="X0:X1",
                   help="keep only columns X0..X1, e.g. 200:480 (rounded to multiples of 4)")
    args = p.parse_args()

    crop = parse_crop(args.crop)
    if crop:
        print(f"cropping to columns {crop[0]}..{crop[1]} ({crop[1] - crop[0]} x {HEIGHT})")
    camera = FakeCamera(crop) if args.fake else \
        FpgaCamera(args.gpio_base, args.bram_base, args.safe_copy, crop)
    tracker = Tracker(args.thresh, args.min_pixels)

    if args.snapshot:
        snapshot(camera, tracker, args.snapshot)
        return

    shared = Shared()
    log = open(args.log, "w", buffering=1) if args.log else None
    if log:
        log.write("time,frame,x,y\n")
    threading.Thread(target=capture_loop, args=(camera, tracker, shared, log),
                     daemon=True).start()

    server = ThreadingHTTPServer((args.host, args.port), make_handler(shared, tracker))
    server.daemon_threads = True
    print(f"live view on http://{args.host}:{args.port}  (Ctrl+C to stop)")
    if args.host == "127.0.0.1":
        print(f"from the laptop: ssh -L {args.port}:localhost:{args.port} <user>@<kria-ip>, "
              f"then open http://localhost:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped")


if __name__ == "__main__":
    main()
