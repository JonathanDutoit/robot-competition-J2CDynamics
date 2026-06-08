from ultralytics import YOLO

model = YOLO("best.pt")

model.info(detailed=True)
print(model.model.args)
