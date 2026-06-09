import time
import numpy as np
import cv2
from flask import Flask, Response
from j2cdynamics_camera.annotator import draw_detections, draw_fps, encode_jpeg

app = Flask(__name__)

_cam_output = None
_get_detections = None
_ros_node = None


def init_streamer(cam_output, get_detections, ros_node):
    global _cam_output, _get_detections, _ros_node
    _cam_output = cam_output
    _get_detections = get_detections
    _ros_node = ros_node


def generate_frames():
    prev_time = time.time()

    while True:
        with _cam_output.condition:
            _cam_output.condition.wait()
            jpeg_bytes = _cam_output.frame

        frame = cv2.imdecode(np.frombuffer(jpeg_bytes, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            continue

        dets = _get_detections()

        now = time.time()
        fps = 1.0 / (now - prev_time + 1e-9)
        prev_time = now

        frame = draw_detections(frame, dets)
        frame = draw_fps(frame, fps)
        out = encode_jpeg(frame)

        yield (b"--frame\r\nContent-Type: image/jpeg\r\n"
               b"Content-Length: " + str(len(out)).encode() + b"\r\n\r\n"
               + out + b"\r\n")


@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/snapshot")
def snapshot():
    # Non-blocking: serve the LATEST frame with detections drawn on it. Zero CPU
    # when nobody's polling, unlike /video_feed which runs an inference-driven loop.
    if _cam_output is None or _cam_output.frame is None:
        return ("not ready", 503)
    with _cam_output.condition:
        jpeg_bytes = _cam_output.frame
    frame = cv2.imdecode(np.frombuffer(jpeg_bytes, np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        return ("decode failed", 503)
    dets = _get_detections() if _get_detections else []
    frame = draw_detections(frame, dets)
    out = encode_jpeg(frame)
    headers = {"Cache-Control": "no-cache, no-store, must-revalidate"}
    return Response(out, mimetype='image/jpeg', headers=headers)


_INDEX_HTML = """<!doctype html><html><head><meta charset=utf-8><title>camera</title>
<style>
  body{margin:0;background:#000;color:#ddd;font-family:ui-monospace,Menlo,monospace;
       display:flex;flex-direction:column;align-items:center}
  #img{max-width:100%;max-height:88vh;object-fit:contain;background:#111;display:block}
  .bar{padding:8px 12px;display:flex;gap:8px;align-items:center;flex-wrap:wrap}
  button{background:#1a1a1a;color:#ddd;border:1px solid #333;padding:6px 12px;
         border-radius:4px;cursor:pointer;font:inherit}
  button.on{border-color:#3fb950;color:#3fb950}
  button:hover{border-color:#666}
  select{background:#1a1a1a;color:#ddd;border:1px solid #333;padding:5px;
         border-radius:4px;font:inherit}
  .mut{color:#666;font-size:11px}
</style></head><body>
<img id=img src="/snapshot" alt="snapshot">
<div class=bar>
  <button onclick="refresh()">Refresh</button>
  <button id=auto onclick="toggleAuto()">Auto: OFF</button>
  <select id=rate onchange="restartAuto()">
    <option value=2000>0.5 Hz</option>
    <option value=1000 selected>1 Hz</option>
    <option value=500>2 Hz</option>
    <option value=250>4 Hz</option>
  </select>
  <span class=mut>snapshot mode — use /video_feed for live MJPEG</span>
</div>
<script>
let timer=null;
function refresh(){ document.getElementById('img').src='/snapshot?t='+Date.now(); }
function startAuto(){
  const ms=parseInt(document.getElementById('rate').value);
  timer=setInterval(refresh, ms);
}
function toggleAuto(){
  const b=document.getElementById('auto');
  if(timer){clearInterval(timer); timer=null; b.textContent='Auto: OFF'; b.classList.remove('on');}
  else{startAuto(); b.textContent='Auto: ON'; b.classList.add('on');}
}
function restartAuto(){ if(timer){clearInterval(timer); startAuto();} }
</script></body></html>"""


@app.route("/")
def index():
    return _INDEX_HTML