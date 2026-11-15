"""
web_ui.py

The browser page for the USB Arducam viewer: video on top, metrics panel
below, and a collapsible Setup section with the tracker keys, the overlay
toggle and the click modes (rod mask / set (0,0) / set far corner).

Endpoints added on top of kv260_view_tracker.make_handler (which still serves
/stream, /snapshot.jpg, /cmd and /set):
    /             this page
    /metrics      JSON from metrics_display.snapshot()
    /overlay      ?on=0|1 (no argument = toggle)
    /mask         ?x=     add / remove a rod mask (only while the overlay is shown)
    /calib        ?pt=origin|corner&x=&y=   set a table corner (same rule)
    /masks/clear
"""

import json
import time
from urllib.parse import parse_qs, urlparse

import cv2

from kv260_view_tracker import make_handler as base_handler
from metrics_display import snapshot
from view_tracker import render


# ============================================================================
# Render thread: tracker graphics only; all text lives in the page
# ============================================================================

def render_loop(det, grabber, live, args, setup, ui):
    period = 1.0 / args.display_fps
    while live.running:
        t0 = time.monotonic()
        with live.lock:
            snap = live.snap
        if snap is not None:
            gray, s, view, trail, info, show_mask, ready = snap
            img = render(gray, s, view, det, trail, info, args.scale, show_mask, ready,
                         text=False, show_masks=ui.show_overlay)
            if ui.show_overlay:
                setup.draw(img, args.scale)
            ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality])
            if ok:
                with live.cond:
                    live.jpeg = jpg.tobytes()
                    live.jpeg_id += 1
                    live.cond.notify_all()
        time.sleep(max(0.0, period - (time.monotonic() - t0)))


# ============================================================================
# Page
# ============================================================================

PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>FoosPGA Tracker</title>
<style>
  :root { --bg:#101214; --panel:#1a1d21; --line:#2b3036; --text:#e7eaee; --muted:#8c95a0;
          --ok:#5ad16a; --warn:#ffb020; --bad:#ff5a5a; --accent:#4cc3ff; }
  * { box-sizing:border-box; }
  body { margin:0; background:var(--bg); color:var(--text);
         font:14px/1.45 system-ui,-apple-system,"Segoe UI",sans-serif; }
  main { max-width:1300px; margin:0 auto; padding:16px; }
  h1 { font-size:18px; font-weight:600; margin:0 0 12px; }
  #video { width:100%; height:auto; max-height:70vh; object-fit:contain; display:block; background:#000;
           border:1px solid var(--line); border-radius:6px; }
  #video.edit { cursor:crosshair; }
  .grid { display:grid; grid-template-columns:repeat(auto-fit, minmax(260px, 1fr)); gap:12px; margin-top:12px; }
  .card { background:var(--panel); border:1px solid var(--line); border-radius:6px; padding:12px 14px; }
  .card h2 { font-size:12px; font-weight:600; letter-spacing:.06em; text-transform:uppercase;
             color:var(--muted); margin:0 0 8px; }
  .big { font:600 34px/1.1 ui-monospace,SFMono-Regular,Menlo,monospace; font-variant-numeric:tabular-nums; }
  .big small { font-size:15px; color:var(--muted); font-weight:500; }
  .row { display:flex; justify-content:space-between; gap:12px; padding:2px 0; }
  .row span:first-child { color:var(--muted); }
  .num { font-family:ui-monospace,SFMono-Regular,Menlo,monospace; font-variant-numeric:tabular-nums; }
  .TRACK { color:var(--ok); } .COAST { color:var(--warn); } .TENTATIVE { color:#ffe14d; }
  .SEARCH, .WAITING { color:var(--bad); }  .HIDDEN { color:#d77cf0; }
  #err { color:var(--bad); margin-top:8px; min-height:1em; }
  details { margin-top:12px; background:var(--panel); border:1px solid var(--line); border-radius:6px; }
  summary { cursor:pointer; padding:10px 14px; font-weight:600; }
  .setup { padding:0 14px 12px; display:flex; flex-direction:column; gap:10px; }
  .bar { display:flex; flex-wrap:wrap; gap:8px 16px; align-items:center; }
  label { display:inline-flex; align-items:center; gap:6px; color:var(--muted); }
  input[type=range] { width:180px; }
  button { background:#2a3038; color:var(--text); border:1px solid #3a414b; border-radius:5px;
           padding:6px 10px; font:inherit; cursor:pointer; }
  button:hover { background:#343b45; }
  button.on { border-color:var(--accent); color:var(--accent); }
  kbd { font:12px ui-monospace,monospace; border:1px solid #3a414b; border-radius:3px; padding:0 4px; }
  .hint { color:var(--muted); }
</style></head>
<body><main>
  <h1>FoosPGA tracker &middot; KV260 &middot; USB Arducam</h1>
  <img id="video" src="/stream" alt="live tracker view">
  <div id="err"></div>

  <div class="grid">
    <div class="card">
      <h2>Ball &middot; table position (__UNITS__)</h2>
      <div class="big"><small>X</small> <span id="tx">&mdash;</span> &nbsp;<small>Y</small> <span id="ty">&mdash;</span></div>
      <div class="row"><span>speed</span><span class="num" id="tspd">&mdash;</span></div>
      <div class="row"><span>image px</span><span class="num" id="px">&mdash;</span></div>
      <div class="row"><span>image speed</span><span class="num" id="pspd">&mdash;</span></div>
    </div>
    <div class="card">
      <h2>Camera &middot; frames</h2>
      <div class="row"><span>frames passed</span><span class="num" id="total">&mdash;</span></div>
      <div class="row"><span>last second</span><span class="num" id="wf">&mdash;</span></div>
      <div class="row"><span>mean interval</span><span class="num" id="wm">&mdash;</span></div>
      <div class="row"><span>max interval</span><span class="num" id="wx">&mdash;</span></div>
      <div class="row"><span>fps (from mean)</span><span class="num" id="wfps">&mdash;</span></div>
    </div>
    <div class="card">
      <h2>Tracker</h2>
      <div class="row"><span>state</span><span class="num" id="state">&mdash;</span></div>
      <div class="row"><span>track time / frame</span><span class="num" id="proc">&mdash;</span></div>
      <div class="row"><span>camera fps (live)</span><span class="num" id="cfps">&mdash;</span></div>
      <div class="row"><span>queue drops</span><span class="num" id="drops">&mdash;</span></div>
    </div>
  </div>

  <details id="setupbox">
    <summary>Setup</summary>
    <div class="setup">
      <div class="bar">
        <button data-k="b"><kbd>b</kbd> learn background</button>
        <button data-k="r"><kbd>r</kbd> reset tracker</button>
        <button data-k="m"><kbd>m</kbd> diff view</button>
        <label>thresh <input id="t" type="range" min="0" max="255" value="__THRESH__">
          <output id="tv">__THRESH__</output></label>
        <span class="hint">mode: __MODE__</span>
      </div>
      <div class="bar">
        <button id="ov"><kbd>o</kbd> overlay</button>
        <span class="hint">click mode:</span>
        <label><input type="radio" name="cm" value="mask" checked> rod mask</label>
        <label><input type="radio" name="cm" value="origin"> set (0,0) &middot; bottom-left</label>
        <label><input type="radio" name="cm" value="corner"> set __FAR__ &middot; top-right</label>
        <button id="clr">clear masks</button>
      </div>
      <div class="bar hint" id="calib">&mdash;</div>
      <div class="hint">Clicks on the picture edit only while the overlay is shown. Hidden masks still apply.</div>
    </div>
  </details>
</main>
<script>
  const W = __W__, H = __H__, S = __SCALE__;
  const $ = (id) => document.getElementById(id);
  const img = $("video");
  let overlay = true;

  const send = (k) => fetch("/cmd?k=" + k);
  document.querySelectorAll("button[data-k]").forEach(b => b.onclick = () => send(b.dataset.k));
  const toggleOverlay = () => fetch("/overlay").then(poll);
  $("ov").onclick = toggleOverlay;
  document.addEventListener("keydown", e => {
    if (e.target.tagName === "INPUT" && e.target.type !== "radio") return;
    if ("brm".includes(e.key)) send(e.key);
    if (e.key === "o") toggleOverlay();
  });
  const t = $("t"), tv = $("tv");
  t.oninput = () => { tv.textContent = t.value; fetch("/set?thresh=" + t.value); };
  $("clr").onclick = () => { if (confirm("Remove all rod masks?")) fetch("/masks/clear").then(poll); };

  const mode = () => document.querySelector("input[name=cm]:checked").value;
  img.addEventListener("click", e => {
    if (!overlay) return;
    // the picture is letterboxed (object-fit: contain): undo that first
    const r = img.getBoundingClientRect();
    const k = Math.min(r.width / W, r.height / H);
    const x = (e.clientX - r.left - (r.width - W * k) / 2) / k;
    const y = (e.clientY - r.top - (r.height - H * k) / 2) / k;
    if (x < 0 || x >= W || y < 0 || y >= H) return;
    const ix = (x / S).toFixed(1), iy = (y / S).toFixed(1), m = mode();
    if (m === "mask") fetch("/mask?x=" + ix).then(poll);
    else {
      fetch(`/calib?pt=${m}&x=${ix}&y=${iy}`).then(poll);
      document.querySelector("input[name=cm][value=mask]").checked = true;   // one click per corner
    }
  });

  const f = (v, nd = 1, unit = "") => v === null || v === undefined ? "\\u2014" : v.toFixed(nd) + unit;
  function poll() {
    return fetch("/metrics").then(r => r.json()).then(d => {
      $("tx").textContent = d.table ? d.table[0].toFixed(1) : "\\u2014";
      $("ty").textContent = d.table ? d.table[1].toFixed(1) : "\\u2014";
      $("tspd").textContent = f(d.speed_table, 1, " __UNITS__/s");
      $("px").textContent = d.px ? `${d.px[0].toFixed(1)}, ${d.px[1].toFixed(1)}` : "\\u2014";
      $("pspd").textContent = f(d.speed_px, 0, " px/s");
      $("total").textContent = d.total.toLocaleString();
      const w = d.window || {};
      $("wf").textContent = w.frames === undefined ? "\\u2014" : w.frames;
      $("wm").textContent = f(w.mean_ms, 2, " ms");
      $("wx").textContent = f(w.max_ms, 2, " ms");
      $("wfps").textContent = f(w.fps, 1);
      const st = d.ready ? d.state : "WAITING";
      $("state").textContent = d.ready ? st : "WAITING (Setup \\u2192 learn background)";
      $("state").className = "num " + st;
      $("proc").textContent = f(d.proc_ms, 2, " ms");
      $("cfps").textContent = f(d.cam_fps, 0);
      $("drops").textContent = d.dropped;
      $("err").textContent = d.error || "";
      overlay = d.overlay;
      $("ov").classList.toggle("on", overlay);
      $("ov").innerHTML = "<kbd>o</kbd> overlay: " + (overlay ? "shown" : "hidden");
      img.classList.toggle("edit", overlay);
      const s = d.setup;
      $("calib").textContent = `rod masks: ${s.masks}   \\u00b7   (0,0): ${s.origin ? "set" : "not set"}` +
        `   \\u00b7   __FAR__: ${s.corner ? "set" : "not set"}` +
        (s.origin && s.corner ? "" : "   \\u2192 pick a click mode above, then click the corner");
    }).catch(() => {});
  }
  setInterval(poll, 250);
  poll();
</script></body></html>"""


def make_handler(live, det, args, setup, metrics, ui, w, h):
    W, H = setup.size
    far = f"({W:g},{H:g})"
    page = (PAGE.replace("__W__", str(w * args.scale))
                .replace("__H__", str(h * args.scale))
                .replace("__SCALE__", str(args.scale))
                .replace("__UNITS__", args.units)
                .replace("__FAR__", far)
                .replace("__MODE__", det.cfg.mode))

    class Handler(base_handler(live, det, args)):
        def do_GET(self):
            url = urlparse(self.path)
            q = parse_qs(url.query)
            p = url.path
            if p == "/":
                body = page.replace("__THRESH__", str(det.cfg.thresh))
                self.small(body.encode(), "text/html; charset=utf-8")
            elif p == "/metrics":
                self.small(json.dumps(snapshot(live, metrics, setup, ui)).encode(), "application/json")
            elif p == "/overlay":
                on = q.get("on", [None])[0]
                ui.show_overlay = (not ui.show_overlay) if on is None else on == "1"
                self.small(b"ok", "text/plain")
            elif p in ("/mask", "/calib", "/masks/clear"):
                if not ui.show_overlay:
                    self.send_error(409, "show the overlay to edit")
                    return
                try:
                    if p == "/mask":
                        setup.click_mask(float(q["x"][0]))
                    elif p == "/calib":
                        setup.set_point(q["pt"][0], float(q["x"][0]), float(q["y"][0]))
                    else:
                        setup.clear_masks()
                except (KeyError, ValueError):
                    self.send_error(400)
                    return
                self.small(b"ok", "text/plain")
            else:
                super().do_GET()

    return Handler
