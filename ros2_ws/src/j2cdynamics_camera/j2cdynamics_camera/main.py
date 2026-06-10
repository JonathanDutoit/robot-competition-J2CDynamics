import time
import threading
import rclpy

from j2cdynamics_camera.config import INFER_FPS
from j2cdynamics_camera.camera import Camera
from j2cdynamics_camera.detector import Detector
from j2cdynamics_camera.ros_node import init_ros


# ── Inference loop ────────────────────────────────────────────────────────────
def inference_loop(camera: Camera, detector: Detector, ros_node, stop_event: threading.Event):
    target_dt = 1.0 / INFER_FPS
    print("[inference] Thread started")

    while not stop_event.is_set():
        t0 = time.time()
        try:
            frame = camera.capture_lores()
            dets = detector.detect(frame)

            ros_node.publish_frame(frame)
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

    stop_event = threading.Event()
    inference_thread = threading.Thread(
        target=inference_loop,
        args=(camera, detector, ros_node, stop_event),
        daemon=True,
    )
    inference_thread.start()

    try:
        # Block forever; ROS executor runs in its own thread (see init_ros).
        stop_event.wait()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        camera.stop()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
