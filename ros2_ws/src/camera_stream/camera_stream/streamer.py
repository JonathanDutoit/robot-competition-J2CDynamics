import time
import numpy as np
import cv2
from flask import Flask, Response
from camera_stream.annotator import draw_detections, draw_fps, encode_jpeg

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


@app.route("/")
def index():
    return '<html><body style="margin:0;background:#000;"><img src="/video_feed" style="width:100%;height:100vh;object-fit:contain;"/></body></html>'