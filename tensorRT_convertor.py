from ultralytics import YOLO

model = YOLO("yolov8n.pt")

model.export(
    format="engine",
    imgsz=640,
    half=True,
    device=0,
    workspace=4,
    simplify=False,
)