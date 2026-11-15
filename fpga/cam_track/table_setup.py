"""
table_setup.py

Everything you set up by clicking on the picture, with no web code in it:

  * rod masks   full-height strips over the rods. Blobs centered in one are
                ignored by the detector, and the strips are the tracker's rod
                zones (rod-dwell release).
  * table axes  two clicked corners of the playing surface: bottom-left is
                (0, 0), top-right is (W, H) = the table's real size (default
                47 x 27 inches), so table units are the same size on both axes.

Points are stored in FULL-CAMERA pixels so changing --crop does not move them,
and saved to one JSON file that is loaded again on the next start.

Mapping (camera roughly square to the table, so axis-aligned is enough):
    X = W * (x - ox) / (cx - ox)        Y = H * (y - oy) / (cy - oy)
Image y points down and the top-right corner is above the origin (cy < oy),
so Y grows upward on its own.
"""

import json
import threading

import cv2

from view_tracker import put

OVERLAY_COLOR = (255, 200, 0)     # BGR: cyan-ish, stands out from the red masks


class TableSetup:
    def __init__(self, path, mask_width, table_size, grabber, det, trk, legacy_masks=None):
        """
        grabber: the cropping grabber (needs x0, y0, w, h). table_size: (W, H)
        in table units along image x and image y. legacy_masks: an old
        rod_masks.json to import from if `path` does not exist yet.
        """
        self.path, self.mask_width = path, mask_width
        self.size = table_size
        self.g, self.det, self.trk = grabber, det, trk
        self.lock = threading.Lock()
        self.spans = []          # [(x0, x1)] rod masks, full-camera px
        self.origin = None       # (x, y) full-camera px of table (0, 0)
        self.corner = None       # (x, y) full-camera px of table (W, H)
        self._load(legacy_masks)
        self._apply()

    # -- persistence -----------------------------------------------------------

    def _load(self, legacy_masks):
        try:
            with open(self.path) as f:
                d = json.load(f)
            self.spans = [tuple(s) for s in d.get("masks", [])]
            self.origin = tuple(d["origin"]) if d.get("origin") else None
            self.corner = tuple(d["corner"]) if d.get("corner") else None
            print(f"loaded {len(self.spans)} rod masks, origin {self.origin}, "
                  f"corner {self.corner} from {self.path}")
            return
        except FileNotFoundError:
            pass
        except (OSError, ValueError, KeyError, TypeError) as e:
            print(f"ignoring unreadable {self.path}: {e}")
            return
        if legacy_masks:
            try:
                with open(legacy_masks) as f:
                    self.spans = [tuple(s) for s in json.load(f)["masks"]]
                print(f"imported {len(self.spans)} rod masks from {legacy_masks}")
                self._save()
            except FileNotFoundError:
                pass
            except (OSError, ValueError, KeyError, TypeError) as e:
                print(f"ignoring unreadable {legacy_masks}: {e}")

    def _save(self):
        d = {"note": "full-camera pixels; masks are [x0, x1] strips",
             "masks": self.spans, "origin": self.origin, "corner": self.corner}
        try:
            with open(self.path, "w") as f:
                json.dump(d, f)
        except OSError as e:
            print(f"could not save table setup to {self.path}: {e}")

    # -- rod masks -------------------------------------------------------------

    def click_mask(self, x):
        """x in cropped image px: remove the strip under it, or add one there."""
        fx = x + self.g.x0
        with self.lock:
            hit = next((s for s in self.spans if s[0] <= fx < s[1]), None)
            if hit:
                self.spans.remove(hit)
            else:
                half = self.mask_width / 2.0
                self.spans.append((int(round(fx - half)), int(round(fx + half))))
                self.spans.sort()
            self._apply()
            self._save()

    def clear_masks(self):
        with self.lock:
            self.spans = []
            self._apply()
            self._save()

    def _apply(self):
        """Push the masks to the detector and tracker as cropped-pixel rectangles."""
        ox, w, h = self.g.x0, self.g.w, self.g.h
        rects = tuple((max(0, x0 - ox), 0, min(w, x1 - ox), h)
                      for x0, x1 in self.spans if x1 - ox > 0 and x0 - ox < w)
        self.det.cfg.masks = rects        # whole-tuple swaps: safe across threads
        self.trk.cfg.rod_zones = rects

    # -- table coordinates -----------------------------------------------------

    def set_point(self, which, x, y):
        """which = 'origin' or 'corner'; x, y in cropped image px."""
        p = (round(x + self.g.x0, 1), round(y + self.g.y0, 1))
        with self.lock:
            if which == "origin":
                self.origin = p
            elif which == "corner":
                self.corner = p
            else:
                raise ValueError(which)
            self._save()

    def _scale(self):
        """(ox, oy, kx, ky): table = k * (full px - o), or None if not calibrated."""
        o, c = self.origin, self.corner
        if o is None or c is None or c[0] == o[0] or c[1] == o[1]:
            return None
        return o[0], o[1], self.size[0] / (c[0] - o[0]), self.size[1] / (c[1] - o[1])

    def to_table(self, x, y):
        """Cropped image px -> table units, or None if not calibrated."""
        s = self._scale()
        if s is None:
            return None
        ox, oy, kx, ky = s
        return kx * (x + self.g.x0 - ox), ky * (y + self.g.y0 - oy)

    def vel_to_table(self, vx, vy):
        """px/s -> table units/s, or None if not calibrated."""
        s = self._scale()
        if s is None:
            return None
        return s[2] * vx, s[3] * vy

    def status(self):
        return {"masks": len(self.spans), "origin": self.origin is not None,
                "corner": self.corner is not None, "size": list(self.size)}

    # -- overlay ---------------------------------------------------------------

    def draw(self, img, scale):
        """Corner markers + table outline on a rendered frame (masks are tinted by render())."""
        def P(p):
            return (int(round((p[0] - self.g.x0 + 0.5) * scale)),
                    int(round((p[1] - self.g.y0 + 0.5) * scale)))

        o, c = self.origin, self.corner
        if o and c:
            cv2.rectangle(img, P(o), P(c), OVERLAY_COLOR, 1, cv2.LINE_AA)
        W, H = self.size
        for p, label, dy in ((o, "(0,0)", 18), (c, f"({W:g},{H:g})", -8)):
            if p is None:
                continue
            q = P(p)
            cv2.drawMarker(img, q, OVERLAY_COLOR, cv2.MARKER_CROSS, 16, 2, cv2.LINE_AA)
            tx = min(max(q[0] + 6, 2), img.shape[1] - 70)
            ty = min(max(q[1] + dy, 14), img.shape[0] - 4)
            put(img, label, (tx, ty), OVERLAY_COLOR)


def parse_size(text):
    """'47x27' -> (47.0, 27.0)."""
    w, h = (float(v) for v in text.lower().split("x"))
    if w <= 0 or h <= 0:
        raise ValueError("table size must be positive")
    return w, h
