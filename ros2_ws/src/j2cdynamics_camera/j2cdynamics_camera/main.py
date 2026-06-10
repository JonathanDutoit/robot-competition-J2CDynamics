import time
import threading
import rclpy

from j2cdynamics_camera.config import INFER_FPS, IMAGE_PUB_EVERY_N
from j2cdynamics_camera.camera import Camera
from j2cdynamics_camera.detector import Detector
from j2cdynamics_camera.ros_node import init_ros


# ── Inference loop ────────────────────────────────────────────────────────────
def inference_loop(camera: Camera, detector: Detector, ros_node, stop_event: threading.Event):
    target_dt = 1.0 / INFER_FPS
    print(f"[inference] Thread started ({INFER_FPS} FPS, image every {IMAGE_PUB_EVERY_N})")
    frame_idx = 0

    while not stop_event.is_set():
        t0 = time.time()
        try:
            frame = camera.capture_lores()
            dets = detector.detect(frame)

            # Detections always go out at full rate (they're cheap and small).
            ros_node.publish_detections(dets)
            # Image publishing is decimated — the dashboard MJPEG doesn't need
            # 10 FPS, and serializing a 640x480 BGR8 message each tick is real
            # CPU on the Pi. publish_frame() also skips when no subscribers.
            if IMAGE_PUB_EVERY_N <= 1 or (frame_idx % IMAGE_PUB_EVERY_N) == 0:
                ros_node.publish_frame(frame)
            frame_idx += 1

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
