from picamera2 import Picamera2
from j2cdynamics_camera.config import MAIN_SIZE, LORES_SIZE

class Camera:
    """
    Headless capture: no JPEG encoder, no MJPEG sink. Only the lores stream is
    read (for the detector). The main stream is configured at a small size
    because picamera2 requires one, but it is never encoded or transmitted.
    """

    def __init__(self):
        self._cam = Picamera2()

        config = self._cam.create_video_configuration(
            main={"size": MAIN_SIZE, "format": "RGB888"},
            lores={"size": LORES_SIZE, "format": "RGB888"},
            buffer_count=2,
            queue=False,
        )
        self._cam.configure(config)

    def start(self):
        self._cam.start()
        print("[camera] Started")

    def stop(self):
        self._cam.stop()
        self._cam.stop()
        print("[camera] Stopped")

    def capture_lores(self):
        return self._cam.capture_array("lores")
