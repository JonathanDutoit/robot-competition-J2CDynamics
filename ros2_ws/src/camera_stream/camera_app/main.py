# main.py
import time
import rclpy
import threading
from config import INFER_FPS
from camera import Camera
from detector import Detector
from ros_node import init_ros
from streamer import app, init_streamer

# ── Shared state ──────────────────────────────────────────────────────────────
import threading
latest_detections = []
det_lock = threading.Lock()

def get_detections():
    with det_lock:
        return list(latest_detections)

# ── Inference loop ────────────────────────────────────────────────────────────
def inference_loop(camera: Camera, detector: Detector, ros_node):
    global latest_detections
    target_dt = 1.0 / INFER_FPS
    print("[inference] Thread started")

    while True:
        t0 = time.time()
        try:
            frame = camera.capture_lores()
            dets = detector.detect(frame)

            with det_lock:
                latest_detections = dets

            ros_node.publish_detections(dets)

        except Exception as e:
            print(f"[inference] Error: {e}")

        elapsed = time.time() - t0
        time.sleep(max(0.0, target_dt - elapsed))


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    ros_node = init_ros()
    camera = Camera()
    detector = Detector()

    camera.start()

    init_streamer(camera.output, get_detections, ros_node)

    threading.Thread(
        target=inference_loop,
        args=(camera, detector, ros_node),
        daemon=True
    ).start()

    try:
        app.run(host="0.0.0.0", port=7123, threaded=True, use_reloader=False)
    finally:
        camera.stop()
        rclpy.shutdown()


if __name__ == "__main__":
    main()