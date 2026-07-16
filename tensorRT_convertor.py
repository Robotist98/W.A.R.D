from ultralytics import YOLO

model = YOLO("yolov8n.pt")

model.export(
    format="engine",
    imgsz=640,
    quantize=16,
    device=0,
    workspace=4,
    simplify=False,
)