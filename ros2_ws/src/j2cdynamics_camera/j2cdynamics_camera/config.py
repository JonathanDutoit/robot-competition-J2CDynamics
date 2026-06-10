MAIN_SIZE  = (3200, 2464)  # bbox coordinate space published on /detections — consumers
                           # (duplo_approach, ground_projection) interpret bbox centers in
                           # these pixel coords, so don't change without updating them.
                           # The main stream is no longer encoded/streamed (see camera.py).
LORES_SIZE = (640, 480)    # 4:3 lores fed to the detector (cheap to access; same FOV as MAIN)
YOLO_INPUT = 640           # square model input (must match the imgsz= used at export)
# YOLO output row layout. Ultralytics exports with `nms=True` give "xyxy"; the
# default raw export (no NMS) gives "xywh" (center + size). Wrong choice =
# every bbox glued to the top-left of the image. Watch the detector log line at
# startup — it prints the model's output shape + a sample row so you can verify.
YOLO_OUTPUT_FORMAT = "xyxy"   # "xyxy"  OR  "xywh"
CONF_THRESH  = 0.8
MODEL_PATH   = "model/best_int8.onnx"
CLASS_NAMES  = ["duplo"]
# Inference rate. Each detection cycle ≈ 80-150 ms on Pi 4 CPU; at 10 FPS that's
# ~80-100% of one core. 6 FPS is enough for the visual-servo loop (running at
# CONTROL_HZ=10 in duplo_approach.py) since the controller uses last_seen_time +
# REACQUIRE_TIME for persistence between detections. Drop further if needed.
INFER_FPS    =  2
# Image publishing decimation: skip /camera/image_raw on most detection cycles.
# Dashboard MJPEG only needs a few FPS. 3 = publish every 3rd frame (2 FPS at
# INFER_FPS=6). 1 = publish every frame. Detections always go out at full rate.
IMAGE_PUB_EVERY_N = 3
JPEG_QUALITY = 60