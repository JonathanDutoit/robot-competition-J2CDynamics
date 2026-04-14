import io
from threading import Condition
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput
from config import MAIN_SIZE, LORES_SIZE

class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()

class Camera:
    def __init__(self):
        self.output = StreamingOutput()
        self._cam = Picamera2()

        config = self._cam.create_video_configuration(
            main={"size": MAIN_SIZE, "format": "RGB888"},
            lores={"size": LORES_SIZE, "format": "RGB888"},
            buffer_count=2,
            queue=False,
            encode="main",
        )
        self._cam.configure(config)

    def start(self):
        self._cam.start_recording(JpegEncoder(), FileOutput(self.output))
        print("[camera] Started")

    def stop(self):
        self._cam.stop_recording()
        print("[camera] Stopped")

    def capture_lores(self):
        return self._cam.capture_array("lores")