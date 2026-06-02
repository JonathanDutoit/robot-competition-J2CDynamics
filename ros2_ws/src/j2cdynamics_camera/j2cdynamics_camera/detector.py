import numpy as np
import onnxruntime as ort
from j2cdynamics_camera.config import MODEL_PATH, LORES_SIZE, MAIN_SIZE, CONF_THRESH, CLASS_NAMES

SCALE_X = MAIN_SIZE[0] / LORES_SIZE[0]
SCALE_Y = MAIN_SIZE[1] / LORES_SIZE[1]

class Detector:
    def __init__(self):
        sess_options = ort.SessionOptions()
        sess_options.intra_op_num_threads = 2
        sess_options.inter_op_num_threads = 1

        self.session = ort.InferenceSession(
            MODEL_PATH,
            sess_options,
            providers=["CPUExecutionProvider"]
        )
        self.input_name = self.session.get_inputs()[0].name
        self._buffer = np.empty((1, 3, LORES_SIZE[1], LORES_SIZE[0]), dtype=np.float32)
        print("[detector] ONNX model loaded OK")

    def detect(self, frame: np.ndarray) -> list:
        """Takes a raw HxWx3 RGB frame, returns list of (x1, y1, x2, y2, label, conf)."""
        rgb = frame[:, :, ::-1]
        self._buffer[0] = rgb.transpose(2, 0, 1).astype(np.float32) / 255.0
        output_np = self.session.run(None, {self.input_name: self._buffer})[0][0]

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

        return dets