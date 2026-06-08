MAIN_SIZE  = (1640, 1232)  # full IMX219 FOV (2x2 binned, 4:3); used for streaming + final bbox coord space
LORES_SIZE = (640, 480)    # 4:3 lores fed to the detector (cheap to access; same FOV as MAIN)
YOLO_INPUT = 640           # square model input (must match the imgsz= used at export)
CONF_THRESH  = 0.3
MODEL_PATH   = "model/best_int8.onnx"
CLASS_NAMES  = ["duplo"]
INFER_FPS    = 10
JPEG_QUALITY = 60