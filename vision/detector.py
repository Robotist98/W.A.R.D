import time
from pathlib import Path

from .bootstrap import ensure_runtime_environment


ensure_runtime_environment()

from ultralytics import YOLO


class YoloDetector:
    def __init__(
        self,
        model_path,
        inference_size: int,
        confidence_threshold: float,
        iou_threshold: float,
        classes=None,
        device=0,
        half=True,
    ):
        self.model_path = Path(model_path)
        self.inference_size = inference_size
        self.confidence_threshold = confidence_threshold
        self.iou_threshold = iou_threshold
        self.classes = classes
        self.device = device
        self.half = half

        self.model = YOLO(str(self.model_path))

    @property
    def class_names(self):
        return self.model.names

    def predict(self, frame):
        inference_start = time.perf_counter()

        results = self.model.predict(
            source=frame,
            imgsz=self.inference_size,
            conf=self.confidence_threshold,
            iou=self.iou_threshold,
            classes=self.classes,
            device=self.device,
            half=self.half,
            verbose=False,
        )

        inference_ms = (time.perf_counter() - inference_start) * 1000.0

        return results[0], inference_ms
