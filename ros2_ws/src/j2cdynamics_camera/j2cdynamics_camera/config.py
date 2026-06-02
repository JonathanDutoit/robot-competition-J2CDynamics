MAIN_SIZE  = (640, 480)   # streaming only
LORES_SIZE = (640, 640)   # inference space and detection output space
CONF_THRESH  = 0.3
MODEL_PATH   = "model/best_int8.onnx"
CLASS_NAMES  = ["duplo"]
INFER_FPS    = 10
JPEG_QUALITY = 60