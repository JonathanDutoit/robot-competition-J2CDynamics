# annotator.py
import cv2
import numpy as np
from camera_stream.config import JPEG_QUALITY

def draw_detections(frame: np.ndarray, dets: list) -> np.ndarray:
    for (x1, y1, x2, y2, label, conf) in dets:
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"{label} {conf:.2f}",
                    (x1 + 2, max(y1 - 6, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0), 1, cv2.LINE_AA)
    return frame

def draw_fps(frame: np.ndarray, fps: float) -> np.ndarray:
    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 200, 255), 2, cv2.LINE_AA)
    return frame

def encode_jpeg(frame: np.ndarray) -> bytes:
    _, jpeg_buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    return jpeg_buf.tobytes()