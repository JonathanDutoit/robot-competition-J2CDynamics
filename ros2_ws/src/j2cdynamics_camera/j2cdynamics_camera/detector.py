import numpy as np
import cv2
import onnxruntime as ort
from j2cdynamics_camera.config import (
    MODEL_PATH, LORES_SIZE, MAIN_SIZE, YOLO_INPUT, YOLO_OUTPUT_FORMAT,
    CONF_THRESH, CLASS_NAMES,
)

# Letterbox: LORES_SIZE (4:3) → YOLO_INPUT square, aspect-preserving with gray pad
_SRC_W, _SRC_H = LORES_SIZE
_SCALE = min(YOLO_INPUT / _SRC_W, YOLO_INPUT / _SRC_H)
_NEW_W = int(round(_SRC_W * _SCALE))
_NEW_H = int(round(_SRC_H * _SCALE))
_PAD_X = (YOLO_INPUT - _NEW_W) // 2
_PAD_Y = (YOLO_INPUT - _NEW_H) // 2

# After unletterboxing the YOLO output back to LORES coords, scale to MAIN coords
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
        # Model now expects 640x640 (export the .onnx with imgsz=640)
        self._buffer = np.empty((1, 3, YOLO_INPUT, YOLO_INPUT), dtype=np.float32)
        # 114 = standard YOLO letterbox pad colour
        self._canvas = np.full((YOLO_INPUT, YOLO_INPUT, 3), 114, dtype=np.uint8)
        out_shape = self.session.get_outputs()[0].shape
        print(f"[detector] ONNX loaded; YOLO {YOLO_INPUT}×{YOLO_INPUT}, "
              f"letterbox {_NEW_W}×{_NEW_H} pad ({_PAD_X},{_PAD_Y}), "
              f"LORES {LORES_SIZE} → MAIN {MAIN_SIZE}, "
              f"out_shape={out_shape}, output_format={YOLO_OUTPUT_FORMAT}")
        self._debug_first = True

    def detect(self, frame: np.ndarray) -> list:
        """Takes a raw HxWx3 RGB frame from lores; returns
        [(x1, y1, x2, y2, label, conf), ...] in MAIN_SIZE coords."""
        # picamera2 RGB888 capture_array already returns RGB — no flip needed
        # Aspect-preserving resize + paste into the gray canvas (letterbox)
        resized = cv2.resize(frame, (_NEW_W, _NEW_H), interpolation=cv2.INTER_LINEAR)
        self._canvas[:] = 114
        self._canvas[_PAD_Y:_PAD_Y + _NEW_H, _PAD_X:_PAD_X + _NEW_W] = resized

        # NCHW float32 in [0, 1]
        self._buffer[0] = self._canvas.transpose(2, 0, 1).astype(np.float32) / 255.0
        output_np = self.session.run(None, {self.input_name: self._buffer})[0][0]

        # One-shot diagnostic: print the top-3 raw rows so you can verify the
        # output format (xyxy vs xywh, pixel vs normalized).
        if self._debug_first:
            top = sorted(output_np, key=lambda r: -float(r[4]))[:3]
            print(f"[detector] sample output rows (top-3 by conf): {top}")
            self._debug_first = False

        dets = []
        for row in output_np:
            # YOLO output row format: configurable in config.py.
            # Whatever the format, downstream wants (x1, y1, x2, y2) in YOLO_INPUT coords.
            if YOLO_OUTPUT_FORMAT == "xywh":
                cx, cy, w_box, h_box, conf, cls = row
                x1 = cx - w_box / 2.0
                y1 = cy - h_box / 2.0
                x2 = cx + w_box / 2.0
                y2 = cy + h_box / 2.0
            else:  # "xyxy"
                x1, y1, x2, y2, conf, cls = row
            if conf < CONF_THRESH:
                continue

            # YOLO (640×640) → LORES coords (unletterbox)
            x1 = (x1 - _PAD_X) / _SCALE
            y1 = (y1 - _PAD_Y) / _SCALE
            x2 = (x2 - _PAD_X) / _SCALE
            y2 = (y2 - _PAD_Y) / _SCALE

            # LORES → MAIN coords (this is what duplo_approach + ros_node expect)
            x1 = int(np.clip(x1 * SCALE_X, 0, MAIN_SIZE[0]))
            y1 = int(np.clip(y1 * SCALE_Y, 0, MAIN_SIZE[1]))
            x2 = int(np.clip(x2 * SCALE_X, 0, MAIN_SIZE[0]))
            y2 = int(np.clip(y2 * SCALE_Y, 0, MAIN_SIZE[1]))

            h = y2 - y1
            w = x2 - x1

            area = w * h
            img_area = MAIN_SIZE[0] * MAIN_SIZE[1]

            # reject huge boxes (bug)
            if area > 0.9 * img_area:
                continue

            # reject tiny noise
            if area < 0.001 * img_area:
                continue

            # reject extreme aspect ratios
            if w / h > 5 or h / w > 5:
                continue

            if x2 > x1 and y2 > y1:
                dets.append((x1, y1, x2, y2, CLASS_NAMES[int(cls)], float(conf)))

        return dets