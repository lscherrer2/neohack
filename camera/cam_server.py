"""Local camera server for ONE camera, opened by device NAME (never by index), so no other camera
(FaceTime, iPhone/Continuity, ...) is ever opened, listed, or probed.

Uses ffmpeg's AVFoundation input to open the camera by name and pipes MJPEG frames to us.

Run in a terminal that HAS macOS camera permission:
    conda activate lerobot
    python ~/lerobot-tools/cam_server.py                     # "icspring camera", 640x480 @ 30 fps
    python ~/lerobot-tools/cam_server.py --size 1280x720     # other mode (must be one the camera supports)

Endpoints (127.0.0.1 only):
    /             live view + focus/sharpness meter
    /snap.jpg     latest frame (JPEG)
    /stream       MJPEG live stream
    /sharpness    JSON: focus score (Laplacian variance, center box) + brightness
    /info         JSON: camera name, size, fps, frames received
"""

import argparse
import json
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import cv2
import numpy as np

DEFAULT_NAME = "icspring camera"


class Camera:
    """Reads JPEG frames from an ffmpeg subprocess that opens exactly one AVFoundation device by name."""

    def __init__(self, name: str, size: str, fps: int, test_pattern: bool = False):
        self.name, self.size, self.fps = name, size, fps
        if test_pattern:  # for testing without camera permission
            src = ["-re", "-f", "lavfi", "-i", f"testsrc=size={size}:rate={fps}"]
        else:
            # "<video name>:none" = this video device only, no audio device.
            src = ["-f", "avfoundation", "-framerate", str(fps), "-video_size", size, "-i", f"{name}:none"]
        self.proc = subprocess.Popen(
            ["ffmpeg", "-hide_banner", "-loglevel", "error", *src,
             "-f", "image2pipe", "-vcodec", "mjpeg", "-q:v", "4", "-"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, stdin=subprocess.DEVNULL,
        )
        self.jpeg: bytes | None = None
        self.ts = 0.0
        self.count = 0
        self.cond = threading.Condition()
        self.stderr = b""
        threading.Thread(target=self._read_frames, daemon=True).start()
        threading.Thread(target=self._read_stderr, daemon=True).start()

    def _read_stderr(self):
        for line in self.proc.stderr:
            self.stderr += line
            print("[ffmpeg]", line.decode(errors="replace").rstrip())

    def _read_frames(self):
        buf = b""
        while True:
            chunk = self.proc.stdout.read1(65536)
            if not chunk:
                break
            buf += chunk
            while True:
                start = buf.find(b"\xff\xd8")
                end = buf.find(b"\xff\xd9", start + 2) if start >= 0 else -1
                if end < 0:
                    if start > 0:
                        buf = buf[start:]
                    break
                frame, buf = buf[start:end + 2], buf[end + 2:]
                with self.cond:
                    self.jpeg, self.ts = frame, time.time()
                    self.count += 1
                    self.cond.notify_all()

    def alive(self) -> bool:
        return self.proc.poll() is None

    def latest(self, newer_than: float | None = None, timeout: float = 2.0) -> bytes:
        with self.cond:
            ok = self.cond.wait_for(
                lambda: self.jpeg is not None and (newer_than is None or self.ts > newer_than) or not self.alive(),
                timeout=timeout)
            if self.jpeg is None or not ok:
                raise RuntimeError(self.error() or "no frames from camera yet")
            return self.jpeg

    def error(self) -> str:
        if self.alive():
            return ""
        return f"ffmpeg exited ({self.proc.returncode}): " + self.stderr.decode(errors="replace").strip()[-600:]

    def close(self):
        if self.alive():
            self.proc.terminate()
            try:
                self.proc.wait(3)
            except subprocess.TimeoutExpired:
                self.proc.kill()


def sharpness(jpeg: bytes) -> tuple[float, float]:
    """Variance of the Laplacian over the center 50% of the frame (standard focus metric) + brightness."""
    f = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
    h, w = f.shape
    roi = f[h // 4: 3 * h // 4, w // 4: 3 * w // 4]
    return float(cv2.Laplacian(roi, cv2.CV_64F).var()), float(roi.mean())


PAGE = """<html><head><title>%(name)s</title></head>
<body style="background:#111;color:#eee;font-family:sans-serif;margin:16px">
<h3>%(name)s — %(size)s @ %(fps)d fps</h3>
<p>To focus: twist the lens slowly, watch Sharpness, stop where it peaks (go slightly past and come back).
Point it at something detailed (text, ruler, box edge) at your working distance.</p>
<div style="display:flex;gap:24px;align-items:flex-start">
 <div style="position:relative">
  <img src="/stream" style="max-width:960px;width:70vw;border:1px solid #444">
  <div style="position:absolute;left:25%%;top:25%%;width:50%%;height:50%%;border:2px dashed #0f0;pointer-events:none"></div>
 </div>
 <div style="min-width:260px">
  <div>Sharpness (center box)</div>
  <div id=score style="font-size:48px;font-weight:bold">–</div>
  <div style="background:#333;width:260px;height:24px"><div id=bar style="background:#0c0;height:24px;width:0"></div></div>
  <div style="margin-top:8px">Best so far: <b id=peak>–</b> <button onclick="peak=0">reset</button></div>
  <div id=hint style="margin-top:8px;font-size:20px"></div>
  <div style="margin-top:16px;color:#aaa">Brightness: <span id=bright>–</span></div>
  <div id=err style="margin-top:16px;color:#f66"></div>
 </div>
</div>
<script>
let peak=0;
async function tick(){
 try{const r=await fetch('/sharpness'); const d=await r.json();
  if(d.error){document.getElementById('err').textContent=d.error}else{
  document.getElementById('err').textContent='';
  peak=Math.max(peak,d.score);
  document.getElementById('score').textContent=d.score.toFixed(0);
  document.getElementById('peak').textContent=peak.toFixed(0);
  const q=peak?d.score/peak:0;
  document.getElementById('bar').style.width=Math.min(100,100*q)+'%%';
  document.getElementById('hint').textContent=q>0.95?'✅ at/near best focus':q>0.8?'close…':'keep turning';
  document.getElementById('bright').textContent=d.brightness.toFixed(0)+' / 255'+(d.brightness<40?' (too dark)':d.brightness>220?' (too bright)':'');}
 }catch(e){}
 setTimeout(tick,200)}
tick();
</script></body></html>"""


def make_handler(cam: Camera):
    class Handler(BaseHTTPRequestHandler):
        def _send(self, code, body: bytes, ctype: str):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(code, json.dumps(obj, indent=1).encode(), "application/json")

        def do_GET(self):
            path = urlparse(self.path).path.rstrip("/") or "/"
            try:
                if path == "/":
                    page = PAGE % {"name": cam.name, "size": cam.size, "fps": cam.fps}
                    return self._send(200, page.encode(), "text/html; charset=utf-8")
                if path == "/info":
                    return self._json({"name": cam.name, "size": cam.size, "fps": cam.fps,
                                       "frames_received": cam.count, "alive": cam.alive(), "error": cam.error()})
                if path in ("/snap", "/snap.jpg"):
                    return self._send(200, cam.latest(newer_than=time.time()), "image/jpeg")
                if path == "/sharpness":
                    s, b = sharpness(cam.latest())
                    return self._json({"score": s, "brightness": b})
                if path == "/stream":
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.end_headers()
                    last = 0.0
                    while True:
                        jpg = cam.latest(newer_than=last)
                        last = cam.ts
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpg))
                        self.wfile.write(jpg + b"\r\n")
                return self._json({"error": "not found"}, 404)
            except (BrokenPipeError, ConnectionResetError):
                pass
            except RuntimeError as e:
                self._json({"error": str(e)}, 503)

        def log_message(self, fmt, *args):
            if args and isinstance(args[0], str) and args[0].split(" ")[1:2] in (["/stream"], ["/sharpness"]):
                return
            super().log_message(fmt, *args)

    return Handler


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default=DEFAULT_NAME, help="exact macOS camera name")
    ap.add_argument("--size", default="640x480")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--test-pattern", action="store_true", help="use a synthetic test image (no camera)")
    args = ap.parse_args()

    cam = Camera(args.name, args.size, args.fps, args.test_pattern)
    srv = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(cam))  # localhost only
    print(f"Serving '{args.name}' {args.size}@{args.fps} on http://127.0.0.1:{args.port}  (Ctrl+C to stop)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        cam.close()


if __name__ == "__main__":
    main()
