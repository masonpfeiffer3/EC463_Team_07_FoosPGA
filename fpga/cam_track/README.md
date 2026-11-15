# cam_track (USB Arducam test branch)

Arducam ezBOX VGA GS Mono (USB) → Python on the KV260's ARM → ball tracker → live view in a browser.
No FPGA bitstream on this branch; the MIPI/OV9281 version lives on `ethan-manfredi`.

## Getting started with this branch

1. **Get the code on your laptop.**
   ```
   git clone https://github.com/masonpfeiffer3/EC463_Team_07_FoosPGA.git   # first time only
   cd EC463_Team_07_FoosPGA
   git fetch origin
   git checkout phillip-test-cam
   git pull
   ```
2. **Plug in the hardware.** Arducam USB cable into the KV260, Ethernet into the network, power on.
   Find the Kria's IP from its serial console with `ip addr show eth0` (last used: `10.210.31.187`).
3. **Copy the folder to the Kria** (from the repo root on the laptop):
   ```
   scp -r fpga/cam_track ubuntu@<kria-ip>:~/
   ```
   Re-run this after every code change. `table_setup.json` (masks and corners) lives only on the Kria,
   so copying the folder again does not erase it.
4. **First time on a fresh Kria only**, see [One-time setup](#one-time-setup-on-the-kria).
5. **Run it** as in [Every session](#every-session), open **http://localhost:8080**, then do the
   [Setup](#the-page) steps (background, rod masks, corners).

## Files

| File | What it is |
|---|---|
| `Arducam_frames.py` | **Run this.** Entry point: camera, tracker, web page, command-line options |
| `cameraFrames.py` | Reads frames from the camera (V4L2, MJPEG decode) and sets exposure/gain. Run it directly for an fps/decode benchmark |
| `tracker.py` | Ball detector + Kalman tracker (SEARCH / TENTATIVE / TRACK / COAST / HIDDEN, rod dwell) |
| `web_ui.py` | The web page, its endpoints and the render loop |
| `metrics_display.py` | Frame counts, 1-second windows and the numbers on the page |
| `table_setup.py` | Rod masks, table corners, table coordinates, overlay drawing; saved to `table_setup.json` |
| `view_tracker.py` | Draws the tracker graphics and learns the background (also a laptop viewer with an OpenCV window) |
| `kv260_view_tracker.py` | Tracking thread and `Live` shared state, used by `Arducam_frames.py` |
| `kv260Frames.py` | MIPI frame source; unused here but imported by `kv260_view_tracker.py` |

## One-time setup on the Kria

```
sudo apt install python3-numpy python3-opencv v4l-utils
sudo usermod -aG video $USER          # then log out and back in
```

## Every session

On the Kria (SSH in or use the serial console):
```
cd ~/cam_track
v4l2-ctl --list-devices               # the Arducam is /dev/video0 (video1 is metadata)
python3 cameraFrames.py --device 0 --width 640 --height 480 --fps 200 --seconds 5   # optional speed check
python3 Arducam_frames.py --device 0
```

On the laptop, in a second terminal (keep it open):
```
ssh -L 8080:localhost:8080 ubuntu@<kria-ip>
```
Open **http://localhost:8080**. Clear the table and press **b** (learn background) before tracking.
Stop the tracker with **Ctrl+C** on the Kria.

## The page

Video on top (tracker graphics only), metrics below, **Setup** collapsed at the bottom.

| Card | Shows |
|---|---|
| Ball | Position in table units (X 0–47 along the length, Y 0–27 across, inches), speed in in/s, and the same in image pixels |
| Camera | Frames passed (total), then per closed 1-second window: frames, mean and max interval, fps |
| Tracker | State (TRACK / COAST / HIDDEN / SEARCH), tracking time per frame, live camera fps, queue drops |

**Setup** (open it to edit; close it for the demo):
1. **b** to learn the background (clear the table first).
2. Click mode **rod mask**: click a rod to drop a full-height strip (`--mask-width`, 20 px); click a strip to remove it.
3. Click mode **set (0,0)**, then click the bottom-left corner of the playing surface. Then **set (47,27)** and the top-right corner.
4. **o** (or the overlay button) hides the masks and corners from the picture. They stay active; clicks on the picture only
   edit while the overlay is shown.

Masks and corners are saved to `table_setup.json` (full-camera pixels, so `--crop` changes don't move them) and loaded on the
next start. An old `rod_masks.json` is imported automatically the first time. If the camera is moved, clear the masks and
set the corners again.

A ball passing under a rod stays **COAST** (its prediction keeps moving). It only parks as **HIDDEN** when it
should have come out and didn't. A ball kicked out faster than the exit box allows is caught by the escape
catcher: two frames of a blob moving away from that rod, lined up with where it went in.

## Handy options (`Arducam_frames.py`)

| Option | Default | Meaning |
|---|---|---|
| `--width --height --fps` | 640 480 200 | Camera mode (the Kria tops out around 145–150 fps at 640x480) |
| `--exposure-us`, `--gain` | 500, 0 | Camera exposure and gain |
| `--crop X0:X1` | none | Keep only the table's columns; `--bounds` is then in cropped pixels |
| `--crop-y Y0:Y1` | none | Keep only the table's rows; same rule for `--bounds` |
| `--ball-radius` | 10 | Ball radius in pixels |
| `--min-area --max-area` | 48 1600 | Blob size limits in pixels |
| `--thresh` | 30 | Detector threshold (also a live slider on the page) |
| `--px-scale` | 2 | Scales the tracker's pixel distances (2 for 640x480) |
| `--mask-width` | 20 | Rod mask strip width in pixels |
| `--log file.csv` | off | Save every tracked position |
| `--table-size WxH` | 47x27 | Playing surface size; W along the image's horizontal axis (use `27x47` if the camera is rotated) |
| `--units` | in | Label for the table units |
| `--overlay-off` | off | Start with the setup overlay hidden |
| `--metrics-log file.csv` | off | One row per 1-second frame window |
| `--hidden-margin` | 20 | Ball under a rod mask: exit box = strip ± (this + kick speed × frame gap), around the y where it went in |
| `--kick-speed` | 4000 | Fastest the ball leaves from under a rod, px/s. Raise it if hard shots off a rod are missed |
| `--drift-speed` | 300 | How fast a pinned ball can be dragged along the rod, px/s (y window grows by up to 100 px) |
| `--hidden-max-s` | 3 | Parked under a rod (state **HIDDEN**) this long with no sighting → search the whole table |
| `--jpeg-quality` | 80 | Quality of the streamed picture (raise it if the page looks blocky) |
| `--port` | 8080 | Web page port (change it in the `ssh -L` command too) |

If the page's `dropped` count keeps climbing, the Kria can't decode 640x480 MJPEG fast enough: set the
CPU governor to `performance` or lower `--fps`.
