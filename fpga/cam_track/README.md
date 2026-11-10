# cam_track (USB Arducam test branch)

Arducam ezBOX VGA GS Mono (USB) → Python on the KV260's ARM → ball tracker → live view in a browser.
No FPGA bitstream on this branch; the MIPI/OV9281 version lives on `ethan-manfredi`.

## Files

| File | What it is |
|---|---|
| `Arducam_frames.py` | **Run this.** Live tracker viewer for the USB Arducam, served as a web page |
| `cameraFrames.py` | Reads frames from the camera (V4L2, MJPEG decode) and sets exposure/gain. Run it directly for an fps/decode benchmark |
| `tracker.py` | Ball detector + Kalman tracker |
| `view_tracker.py` | Draws the overlay and learns the background (also a laptop viewer with an OpenCV window) |
| `kv260_view_tracker.py` | Web page, tracking and render threads, used by `Arducam_frames.py` |
| `kv260Frames.py` | MIPI frame source; unused here but imported by `kv260_view_tracker.py` |

## One-time setup on the Kria

```
sudo apt install python3-numpy python3-opencv v4l-utils
sudo usermod -aG video $USER          # then log out and back in
```

Copy the folder from the laptop: `scp -r fpga/cam_track ubuntu@<kria-ip>:~/`

## Every session

On the Kria:
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

## Handy options (`Arducam_frames.py`)

| Option | Default | Meaning |
|---|---|---|
| `--width --height --fps` | 640 480 200 | Camera mode |
| `--exposure-us`, `--gain` | 500, 0 | Camera exposure and gain |
| `--crop X0:X1` | none | Keep only the table's columns; `--bounds` is then in cropped pixels |
| `--ball-radius` | 10 | Ball radius in pixels |
| `--min-area --max-area` | 48 1600 | Blob size limits in pixels |
| `--thresh` | 30 | Detector threshold (also a live slider on the page) |
| `--px-scale` | 2 | Scales the tracker's pixel distances (2 for 640x480) |
| `--log file.csv` | off | Save every tracked position |
| `--port` | 8080 | Web page port (change it in the `ssh -L` command too) |

If the page's `dropped` count keeps climbing, the Kria can't decode 640x480 MJPEG fast enough: set the
CPU governor to `performance` or lower `--fps`.
