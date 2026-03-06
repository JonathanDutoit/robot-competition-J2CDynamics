import io
import time
import threading
from threading import Condition

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from flask import Flask, Response
from inference import get_model

from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput

app = Flask(__name__)

# ── Config ────────────────────────────────────────────────────────────────────
MAIN_SIZE   = (640, 480)   # streaming resolution
LORES_SIZE  = (320, 320)   # inference resolution (must be divisible by 32)
CONF_THRESH = 0.4
MODEL_NAME = "duplo-merged-v3-tkqpb/1"
# ─────────────────────────────────────────────────────────────────────────────

model = get_model(MODEL_NAME)

# Shared detection state
latest_detections = []
det_lock = threading.Lock()


# ── Streaming output ──────────────────────────────────────────────────────────
class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


output = StreamingOutput()


# ── Camera setup ──────────────────────────────────────────────────────────────
picam2 = Picamera2()

config = picam2.create_video_configuration(
    main={"size": MAIN_SIZE, "format": "RGB888"},
    lores={"size": LORES_SIZE, "format": "RGB888"}, 
    buffer_count=4,
    queue=False,
    encode="main",
)
picam2.configure(config)
picam2.start_recording(JpegEncoder(), FileOutput(output))


# ── Inference thread (lores → YOLO) ──────────────────────────────────────────
def inference_thread():
    """Grab lores frames and run YOLO."""
    global latest_detections
    while True:
        frame = picam2.capture_array("lores") 
        pil_img = Image.fromarray(frame) 

        results = model.infer(
            pil_img,
            confidence=CONF_THRESH,
            img_size=LORES_SIZE[0]
        )

        dets = []
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls   = int(box.cls[0])
                conf  = float(box.conf[0])
                label = model.names[cls]
                dets.append((x1, y1, x2, y2, label, conf))

        with det_lock:
            latest_detections = dets


# ── Overlay drawing (Pillow only) ─────────────────────────────────────────────
def draw_detections_on_jpeg(jpeg_bytes, dets, main_size, lores_size):
    """
    Decode JPEG → draw boxes with Pillow → re-encode to JPEG bytes.
    OpenCV is NOT used here.
    """
    sx = main_size[0] / lores_size[0]
    sy = main_size[1] / lores_size[1]

    img = Image.open(io.BytesIO(jpeg_bytes)).convert("RGB")
    draw = ImageDraw.Draw(img)

    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 14)
    except OSError:
        font = ImageFont.load_default()

    for (x1, y1, x2, y2, label, conf) in dets:
        rx1, ry1 = int(x1 * sx), int(y1 * sy)
        rx2, ry2 = int(x2 * sx), int(y2 * sy)
        draw.rectangle([rx1, ry1, rx2, ry2], outline=(0, 255, 0), width=2)
        text = f"{label} {conf:.2f}"
        draw.text((rx1 + 2, max(ry1 - 16, 0)), text, fill=(0, 255, 0), font=font)

    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=80)
    return buf.getvalue()


# ── MJPEG generator ───────────────────────────────────────────────────────────
def generate_frames():
    prev_time = time.time()

    while True:
        with output.condition:
            output.condition.wait()
            jpeg_bytes = output.frame  # already a JPEG buffer from JpegEncoder

        with det_lock:
            dets = list(latest_detections)

        # Only decode/redraw if there's something to annotate
        if dets:
            jpeg_bytes = draw_detections_on_jpeg(jpeg_bytes, dets, MAIN_SIZE, LORES_SIZE)

        # FPS — drawn with Pillow as well
        now = time.time()
        fps = 1.0 / (now - prev_time + 1e-9)
        prev_time = now

        img = Image.open(io.BytesIO(jpeg_bytes))
        draw = ImageDraw.Draw(img)
        try:
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 16)
        except OSError:
            font = ImageFont.load_default()
        draw.text((10, 8), f"FPS: {fps:.1f}", fill=(0, 200, 255), font=font)
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=80)
        jpeg_bytes = buf.getvalue()

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n"
            b"Content-Length: " + str(len(jpeg_bytes)).encode() + b"\r\n\r\n" +
            jpeg_bytes +
            b"\r\n"
        )


# ── Routes ────────────────────────────────────────────────────────────────────
@app.route("/video_feed")
def video_feed():
    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )

@app.route("/")
def index():
    return """
    <html><body style="margin:0;background:#000;">
      <img src="/video_feed" style="width:100%;height:100vh;object-fit:contain;" />
    </body></html>
    """


# ── Entry point ───────────────────────────────────────────────────────────────
threading.Thread(target=inference_thread, daemon=True).start()

if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=7123, threaded=True)
    finally:
        picam2.stop_recording()
