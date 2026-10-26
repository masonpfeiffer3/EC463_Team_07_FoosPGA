"""
test_tracker.py

Synthetic sanity test for tracker.py: a bright ball bounces around a noisy,
gradient-lit 320x240 table at 400 fps, gets kicked to a new velocity every
0.5 s, disappears briefly twice (rod occlusion), and a tall bar sweeps across
as a distractor. Ground truth is known, so we can measure the error.

    python test_tracker.py
"""

import math
import time

import cv2
import numpy as np

from tracker import (BallDetector, BallTracker, DetectorConfig, TrackerConfig,
                     SEARCH, TENTATIVE, TRACK, COAST)

W, H, FPS = 320, 240, 400
R = 5.0
LO = np.array([R + 2, R + 2])
HI = np.array([W - R - 2, H - R - 2])

YY, XX = np.mgrid[0:H, 0:W].astype(np.float32)
BASE = 90 + 25 * np.sin(XX / 55.0) + 20 * (YY / H)
BASE[100:106, :] -= 40                       # static rod shadow: part of the background


def noisy(img, rng):
    return np.clip(img + rng.normal(0, 2.0, img.shape), 0, 255).astype(np.uint8)


def draw_ball(img, c, level=230.0):
    x0, x1 = max(0, int(c[0]) - 9), min(W, int(c[0]) + 10)
    y0, y1 = max(0, int(c[1]) - 9), min(H, int(c[1]) + 10)
    d = np.hypot(XX[y0:y1, x0:x1] - c[0], YY[y0:y1, x0:x1] - c[1])
    a = np.clip(R + 0.5 - d, 0, 1)
    img[y0:y1, x0:x1] = img[y0:y1, x0:x1] * (1 - a) + level * a


def make_clip(n=1600, seed=1):
    rng = np.random.default_rng(seed)
    background = [noisy(BASE, rng) for _ in range(30)]

    pos = np.array([60.0, 100.0])
    vel = np.array([1400.0, 650.0])
    hidden = set(range(700, 715)) | set(range(1210, 1241))
    frames, truth = [], []
    for i in range(n):
        if i % 200 == 0 and i > 0:               # a "hit": new random velocity
            ang = rng.uniform(0, 2 * math.pi)
            vel = rng.uniform(1000, 1800) * np.array([math.cos(ang), math.sin(ang)])
        pos = pos + vel / FPS
        for k in range(2):                        # wall bounces
            if pos[k] < LO[k]:
                pos[k], vel[k] = 2 * LO[k] - pos[k], abs(vel[k])
            elif pos[k] > HI[k]:
                pos[k], vel[k] = 2 * HI[k] - pos[k], -abs(vel[k])

        img = BASE.copy()
        bx = int(160 + 90 * math.sin(2 * math.pi * i / FPS / 1.7))
        img[140:174, bx:bx + 10] += 60            # distractor bar (tall and thin)
        if i not in hidden:
            draw_ball(img, pos)
        frames.append(noisy(img, rng))
        truth.append(pos.copy())
    return background, frames, truth, hidden


def main():
    background, frames, truth, hidden = make_clip()

    det = BallDetector(DetectorConfig(mode="diff", thresh=30, min_area=12, max_area=400))
    det.learn_background(background)
    trk = BallTracker(det, TrackerConfig(ball_area=math.pi * R * R),
                      bounds=(LO[0], LO[1], HI[0], HI[1]))

    counts = {SEARCH: 0, TENTATIVE: 0, TRACK: 0, COAST: 0}
    err_meas, err_coast, times = [], [], []
    lost_events, first_track, prev = 0, None, SEARCH
    for i, (f, gt) in enumerate(zip(frames, truth)):
        t0 = time.perf_counter()
        s = trk.update(f, i / FPS)
        times.append((time.perf_counter() - t0) * 1000.0)
        counts[s.state] += 1
        if s.state == TRACK and first_track is None:
            first_track = i
        if prev in (TRACK, COAST) and s.state == SEARCH:
            lost_events += 1
        prev = s.state
        if s.x is not None and s.state in (TRACK, COAST):
            e = math.hypot(s.x - gt[0], s.y - gt[1])
            (err_meas if s.measured else err_coast).append(e)

    n = len(frames)
    print(f"frames: {n}   first TRACK at frame {first_track}")
    print("state share: " + "  ".join(f"{k}={100.0 * v / n:.1f}%" for k, v in counts.items()))
    print(f"lost-track events: {lost_events}   (expected 0-1: the 77 ms occlusion follows a visible kick)")
    em = np.array(err_meas)
    print(f"measured frames: n={em.size}  rms={np.sqrt((em ** 2).mean()):.2f}px  "
          f"median={np.median(em):.2f}px  p95={np.percentile(em, 95):.2f}px  max={em.max():.2f}px")
    if err_coast:
        ec = np.array(err_coast)
        print(f"coasting frames: n={ec.size}  mean={ec.mean():.2f}px  max={ec.max():.2f}px")
    t = np.array(times)
    print(f"update() time on this machine: mean={t.mean():.3f} ms  p99={np.percentile(t, 99):.3f} ms")

    assert counts[TRACK] / n > 0.85, "tracked less than 85% of frames"
    assert np.sqrt((em ** 2).mean()) < 1.5, "measured-frame RMS error above 1.5 px"
    print("PASS")


if __name__ == "__main__":
    main()