import io
import time
import threading
from threading import Condition
import numpy as np
import cv2
from flask import Flask, Response
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput
import onnxruntime as ort

app = Flask(__name__)

# ── Config ────────────────────────────────────────────────────────────────────
MAIN_SIZE    = (640, 480)
LORES_SIZE   = (320, 320)
CONF_THRESH  = 0.4
MODEL_PATH   = "model/best_int8.onnx"
CLASS_NAMES  = ["duplo"]
INFER_FPS    = 5   # ✅ cap inference rate (lower = less heat)
JPEG_QUALITY = 60  # ✅ lower quality = less CPU on encode
# ─────────────────────────────────────────────────────────────────────────────

# ── Load Model ─────────────────────────────────
sess_options = ort.SessionOptions()
sess_options.intra_op_num_threads = 2   # limit ONNX CPU threads
sess_options.inter_op_num_threads = 1
session    = ort.InferenceSession(MODEL_PATH, sess_options,
                                  providers=["CPUExecutionProvider"])
input_name = session.get_inputs()[0].name
print("[startup] ONNX model loaded OK")

SCALE_X = MAIN_SIZE[0] / LORES_SIZE[0]
SCALE_Y = MAIN_SIZE[1] / LORES_SIZE[1]

latest_detections: list = []
det_lock = threading.Lock()

# ── Load Camera ────────────────────────────────────────────────────────────────────
class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()

cam_output = StreamingOutput()
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": MAIN_SIZE, "format": "RGB888"},
    lores={"size": LORES_SIZE, "format": "RGB888"},
    buffer_count=2,          # ✅ reduced from 4 — less memory pressure
    queue=False,
    encode="main",
)
picam2.configure(config)
picam2.start_recording(JpegEncoder(), FileOutput(cam_output))
print("[startup] Camera started")

# ── Shared preprocessed input buffer (avoid alloc every frame) ───────────────
_inp_buffer = np.empty((1, 3, LORES_SIZE[1], LORES_SIZE[0]), dtype=np.float32)  # ✅ reuse

# ── Inference thread ──────────────────────────────────────────────────────────
def inference_thread():
    global latest_detections
    target_dt = 1.0 / INFER_FPS
    print("[inference] Thread started")

    while True:
        t0 = time.time()
        try:
            frame = picam2.capture_array("lores")

            np.divide(frame, 255.0, out=_inp_buffer[0].transpose(1, 2, 0)
                      .reshape(LORES_SIZE[1], LORES_SIZE[0], 3)
                      .view(np.float32))

            _inp_buffer[0] = frame.transpose(2, 0, 1).astype(np.float32) / 255.0

            output_np = session.run(None, {input_name: _inp_buffer})[0][0]  # (300, 6)

            dets = []
            for row in output_np:
                x1, y1, x2, y2, conf, cls = row
                if conf < CONF_THRESH:
                    continue

                x1 = int(np.clip(x1 * SCALE_X, 0, MAIN_SIZE[0]))
                y1 = int(np.clip(y1 * SCALE_Y, 0, MAIN_SIZE[1]))
                x2 = int(np.clip(x2 * SCALE_X, 0, MAIN_SIZE[0]))
                y2 = int(np.clip(y2 * SCALE_Y, 0, MAIN_SIZE[1]))

                if x2 > x1 and y2 > y1:
                    dets.append((x1, y1, x2, y2, CLASS_NAMES[int(cls)], float(conf)))

            with det_lock:
                latest_detections = dets

        except Exception as e:
            print(f"[inference] Error: {e}")

        # Sleep only the remaining time to hit target FPS
        elapsed = time.time() - t0
        time.sleep(max(0.0, target_dt - elapsed))


# ── MJPEG generator ───────────────────────────────────────────────────────────
def generate_frames():
    prev_time = time.time()

    while True:
        with cam_output.condition:
            cam_output.condition.wait()
            jpeg_bytes = cam_output.frame

        with det_lock:
            dets = list(latest_detections)

        frame_np = cv2.imdecode(np.frombuffer(jpeg_bytes, np.uint8), cv2.IMREAD_COLOR)
        if frame_np is None:
            continue

        for (x1, y1, x2, y2, label, conf) in dets:
            cv2.rectangle(frame_np, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame_np, f"{label} {conf:.2f}",
                        (x1 + 2, max(y1 - 6, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        now = time.time()
        fps = 1.0 / (now - prev_time + 1e-9)
        prev_time = now
        cv2.putText(frame_np, f"FPS: {fps:.1f}", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2, cv2.LINE_AA)

        _, jpeg_buf = cv2.imencode(".jpg", frame_np, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        out = jpeg_buf.tobytes()

        yield (b"--frame\r\nContent-Type: image/jpeg\r\n"
               b"Content-Length: " + str(len(out)).encode() + b"\r\n\r\n"
               + out + b"\r\n")


# ── Routes ────────────────────────────────────────────────────────────────────
@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/")
def index():
    return '<html><body style="margin:0;background:#000;"><img src="/video_feed" style="width:100%;height:100vh;object-fit:contain;"/></body></html>'


# ── Entry point ───────────────────────────────────────────────────────────────
threading.Thread(target=inference_thread, daemon=True).start()

if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=7123, threaded=True)
    finally:
        picam2.stop_recording()