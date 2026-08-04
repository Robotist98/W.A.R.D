#!/usr/bin/env python3

import sys
import time
from pathlib import Path

from vision.bootstrap import ensure_runtime_environment


ensure_runtime_environment()

import cv2

from vision.bounding_boxes import draw_status, draw_tracks, extract_detections, get_frame_centre_point
from vision.camera_input import LatestFrameCamera, make_gstreamer_pipeline
from vision.detector import YoloDetector
from vision.tracker import CentroidTracker
from algorithm import pid


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

TRACK_MAX_DISTANCE = 120.0
TRACK_MAX_MISSED_FRAMES = 10

KP = 0.1
KI = 0.01
KD = 0.05

x_axis_pid = pid.PID(KP, KI, KD)
y_axis_pid = pid.PID(KP, KI, KD)


def wait_for_first_frame(camera: LatestFrameCamera) -> None:
    print("Waiting for the first frame...")

    while camera.get_latest_frame() is None:
        time.sleep(0.01)


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

    tracker = CentroidTracker(max_distance=TRACK_MAX_DISTANCE, max_missed_frames=TRACK_MAX_MISSED_FRAMES)

    displayed_frames = 0
    fps_start_time = time.perf_counter()
    display_fps = 0.0

    try:
        while True:
            # Gets frame from the camera thread.
            frame = camera.get_latest_frame()
            if frame is None:
                continue

            # Run inference on the frame and update the tracker with the detections.
            result, inference_ms = detector.predict(frame)
            detections = extract_detections(result,detector.class_names,)
            tracks = tracker.update(detections)

            # FPS calculation and display.
            displayed_frames += 1
            elapsed = time.perf_counter() - fps_start_time

            if elapsed >= 1.0:
                display_fps = displayed_frames / elapsed
                displayed_frames = 0
                fps_start_time = time.perf_counter()

            draw_tracks(frame, tracks)
            draw_status(frame, inference_ms, display_fps, len(tracks),)

            #PID control for the first track (if available)
            if tracks:
                first_track = tracks[0]
                frame_centre = get_frame_centre_point(frame)
                x_target = frame_centre[0] - first_track.center[0]
                y_target = frame_centre[1] - first_track.center[1]

                dt = 1.0 / display_fps if display_fps > 0 else 0.01
                x_output = x_axis_pid.update(x_target, x_error, dt)
                y_output = y_axis_pid.update(y_target, y_error, dt)

                print(f"PID Output - X: {x_output:.2f}, Y: {y_output:.2f}")

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

    print("Starting W.A.R.D. vision system...")
    sys.exit(main())
