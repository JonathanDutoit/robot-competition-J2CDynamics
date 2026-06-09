MAIN_SIZE  = (1640, 1232)  # full IMX219 FOV (2x2 binned, 4:3); used for streaming + final bbox coord space
LORES_SIZE = (640, 480)    # 4:3 lores fed to the detector (cheap to access; same FOV as MAIN)
YOLO_INPUT = 640           # square model input (must match the imgsz= used at export)
# YOLO output row layout. Ultralytics exports with `nms=True` give "xyxy"; the
# default raw export (no NMS) gives "xywh" (center + size). Wrong choice =
# every bbox glued to the top-left of the image. Watch the detector log line at
# startup — it prints the model's output shape + a sample row so you can verify.
YOLO_OUTPUT_FORMAT = "xyxy"   # "xyxy"  OR  "xywh"
CONF_THRESH  = 0.3
MODEL_PATH   = "model/best_int8.onnx"
CLASS_NAMES  = ["duplo"]
INFER_FPS    = 10
JPEG_QUALITY = 60