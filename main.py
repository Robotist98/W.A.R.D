#!/usr/bin/env python3

import sys
import time
from pathlib import Path

from vision.bootstrap import ensure_runtime_environment


ensure_runtime_environment()

import cv2

from vision.bounding_boxes import draw_detections, draw_status
from vision.camera_input import LatestFrameCamera, make_gstreamer_pipeline
from vision.detector import YoloDetector


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

CAMERA_URL = "rtsp://169.254.3.154/stream2"

# TensorRT model generated on this Jetson.
MODEL_PATH = Path(__file__).parent / "models" / "yolov8n.engine"

# Must normally match the size used when exporting the TensorRT engine.
INFERENCE_SIZE = 640

CONFIDENCE_THRESHOLD = 0.30
IOU_THRESHOLD = 0.45

# Set to a class ID list to detect only selected classes.
# Example: [0] for person on a standard COCO model.
CLASSES = None

DEVICE = 0
USE_HALF_PRECISION = True

WINDOW_NAME = "Low-latency YOLO"


def wait_for_first_frame(camera: LatestFrameCamera) -> None:
    print("Waiting for the first frame...")

    while camera.get_latest_frame() is None:
        time.sleep(0.01)


def count_detections(result) -> int:
    if result.boxes is None:
        return 0

    return len(result.boxes)


def main() -> int:
    if not MODEL_PATH.exists():
        print(f"Error: model does not exist: {MODEL_PATH}")
        print("Export your trained .pt model to TensorRT first.")
        return 1

    print(f"Loading TensorRT model: {MODEL_PATH}")
    detector = YoloDetector(
        model_path=MODEL_PATH,
        inference_size=INFERENCE_SIZE,
        confidence_threshold=CONFIDENCE_THRESHOLD,
        iou_threshold=IOU_THRESHOLD,
        classes=CLASSES,
        device=DEVICE,
        half=USE_HALF_PRECISION,
    )

    pipeline = make_gstreamer_pipeline(CAMERA_URL)

    print(f"Opening camera: {CAMERA_URL}")
    camera = LatestFrameCamera(pipeline)

    try:
        camera.start()
    except RuntimeError as error:
        print(error)
        return 1

    wait_for_first_frame(camera)

    print("Detection running. Press Q or Escape to exit.")

    displayed_frames = 0
    fps_start_time = time.perf_counter()
    display_fps = 0.0

    try:
        while True:
            frame = camera.get_latest_frame()

            if frame is None:
                continue

            result, inference_ms = detector.predict(frame)

            draw_detections(
                frame,
                result,
                detector.class_names,
            )

            displayed_frames += 1
            elapsed = time.perf_counter() - fps_start_time

            if elapsed >= 1.0:
                display_fps = displayed_frames / elapsed
                displayed_frames = 0
                fps_start_time = time.perf_counter()

            draw_status(
                frame,
                inference_ms,
                display_fps,
                count_detections(result),
            )

            cv2.imshow(WINDOW_NAME, frame)

            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        camera.stop()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    sys.exit(main())
