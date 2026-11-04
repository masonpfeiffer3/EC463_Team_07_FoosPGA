# cam_track: live ball tracking on the KV260

OV9281 camera → MIPI → FPGA (`frame_to_bram` copies one frame into block RAM on request) → Python on the ARM reads the frame through `/dev/mem`, tracks the ball, and serves a live view at `http://localhost:8080`.

Full build steps: `claude/camera_stage3.md` (the Stage 3 guide) in the project.

## Files

| File | What it is |
|---|---|
| `cam_track.bit.bin` | FPGA bitstream (camera pipeline + frame_to_bram + AXI GPIO + frame BRAM) |
| `cam_track.dtsi` | Device-tree overlay source: loads the bitstream, turns on pl_clk0 (100 MHz) and pl_clk1 (200 MHz) |
| `shell.json` | Tells `xmutil` this is a flat (single-slot) design |
| `ov9281_init.py` | Camera setup over I2C: 640 × 400 RAW10, frame length, exposure, gain, test pattern, start streaming |
| `csi_status.py` | Reads the MIPI CSI-2 receiver's status and error registers (is the link healthy?) |
| `track_server.py` | Quick tracker: grabs frames, finds the brightest/changed spot, serves the live page. Also `--snapshot` (save one frame) and `--fake` (no hardware) |
| `track_view.html` | The web page `track_server.py` serves (keep it next to it) |
| `kv260Frames.py` | `FrameGrabber` with the same interface as `cameraFrames.FrameGrabber`, but fed from the FPGA. Run it directly for a grab-rate benchmark |
| `kv260_view_tracker.py` | Our real tracker (`tracker.py`, Kalman) with `view_tracker.render()`, shown as a live web page |
| `tracker.py` | Ball detector + Kalman tracker (unchanged) |
| `view_tracker.py` | Drawing (`render()`) and background learning, used by kv260_view_tracker (unchanged) |
| `cameraFrames.py` | USB-camera version of the frame grabber; needed only because view_tracker imports it (unchanged) |

## One-time install on the Kria

```
cd ~/cam_track
dtc -@ -O dtb -o cam_track.dtbo cam_track.dtsi
sudo mkdir -p /lib/firmware/xilinx/cam_track
sudo cp cam_track.bit.bin cam_track.dtbo shell.json /lib/firmware/xilinx/cam_track/
sudo apt install python3-numpy python3-opencv     # if missing
```

After a new bitstream, only copy `cam_track.bit.bin` to `/lib/firmware/xilinx/cam_track/` again.

## Every session

```
sudo xmutil unloadapp && sudo xmutil loadapp cam_track
sudo python3 ov9281_init.py --vts 422 --gain 64      # camera on (after every loadapp)
sudo python3 csi_status.py                           # optional: link check
```

Then pick one:

```
# quick check: save one frame as frame.png
sudo python3 track_server.py --snapshot frame --crop 208:472

# simple tracker + live view
sudo python3 track_server.py --crop 208:472

# our Kalman tracker + live view
sudo python3 kv260_view_tracker.py --crop 208:472 --ball-radius 10 --min-area 40 --max-area 1600 --px-scale 2
```

Run only one tracker at a time. On the laptop, keep this open:
```
ssh -L 8080:localhost:8080 ubuntu@<kria-ip>
```
Then open **http://localhost:8080**. Clear the table and press **b** (or **Set background**) before tracking.

## Handy options

| Option | Script | Meaning |
|---|---|---|
| `--vts N` | ov9281_init | Frame length in lines: 422 = fastest (~260 fps), 1422 = 77 fps |
| `--exposure N` | ov9281_init | Exposure in lines (9.12 µs each), at most VTS − 25 |
| `--gain N` | ov9281_init | 16 = 1×, 32 = 2×, 64 = 4×, 128 = 8× |
| `--pattern` | ov9281_init | Test pattern instead of the image |
| `--crop X0:X1` | trackers | Keep only columns X0..X1 (the table); rounded to multiples of 4 |
| `--log file.csv` | trackers | Save every tracked position |
| `--fake` | trackers | Fake bouncing ball, no hardware needed |
| `--safe-copy` | trackers | Use if Python dies with "Bus error" |

Frame rate: at most ~130 tracked fps (every other camera frame), less if Python is slower.
