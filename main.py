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
from communications.canbus import WardCanBus, CanBusConfig, clamp


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

TARGETTING = True  # Set to True to enable PID control for the priority target.

CANBUS_ENABLED = False  # Set to True to enable CAN bus communication for PID output.

X_STEPPER_MAX_SPEED = 1000
Y_STEPPER_MAX_SPEED = 800
X_STEPPER_SPEED_SCALE = 1.0
Y_STEPPER_SPEED_SCALE = 1.0
X_STEPPER_DIRECTION = 1
Y_STEPPER_DIRECTION = 1
STEPPER_SPEED_DEADBAND = 2

KP = 0.1
KI = 0.01
KD = 0.05

canbus_config = CanBusConfig(
    interface="socketcan",
    channel="can0",
    bitrate=500000,
    timeout=1.0,
)

canbus = None


def get_canbus() -> WardCanBus:
    global canbus

    if canbus is None:
        canbus = WardCanBus(config=canbus_config)

    return canbus


def pid_output_to_stepper_speed(output: float,scale: float,max_speed: int,direction: int) -> int:
    speed = int(round(output * scale * direction))
    if abs(speed) <= STEPPER_SPEED_DEADBAND:
        return 0
    return clamp(speed, -max_speed, max_speed)


def canbus_send(x_output: float, y_output: float) -> None:
    x_speed = pid_output_to_stepper_speed(
        x_output,
        X_STEPPER_SPEED_SCALE,
        X_STEPPER_MAX_SPEED,
        X_STEPPER_DIRECTION,
    )
    y_speed = pid_output_to_stepper_speed(
        y_output,
        Y_STEPPER_SPEED_SCALE,
        Y_STEPPER_MAX_SPEED,
        Y_STEPPER_DIRECTION,
    )

    get_canbus().set_speed(x_speed, y_speed)
    print(f"CAN speed sent - X: {x_speed} steps/s, Y: {y_speed} steps/s")


def canbus_stop() -> None:
    if canbus is not None:
        canbus.stop()


def wait_for_first_frame(camera: LatestFrameCamera) -> None:
    print("Waiting for the first frame...")

    while camera.get_latest_frame() is None:
        time.sleep(0.01)


def main() -> int:
    x_axis_pid = pid.PID(KP, KI, KD)
    y_axis_pid = pid.PID(KP, KI, KD)

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
            detections = extract_detections(result, detector.class_names)
            tracks = tracker.update(detections)

            # FPS calculation and display.
            displayed_frames += 1
            elapsed = time.perf_counter() - fps_start_time

            if elapsed >= 1.0:
                display_fps = displayed_frames / elapsed
                displayed_frames = 0
                fps_start_time = time.perf_counter()

            draw_tracks(frame, tracks)
            draw_status(frame, inference_ms, display_fps, len(tracks))

            # PID control for the priority target, if one is visible.
            if TARGETTING:
                target_id = tracker.priority_target(tracks, class_name="person")
                priority_track = next(
                    (track for track in tracks if track.track_id == target_id),
                    None,
                )

                if priority_track is not None:
                    frame_centre = get_frame_centre_point(frame)
                    dt = 1.0 / display_fps if display_fps > 0 else 0.01
                    x_output = x_axis_pid.update(
                        frame_centre[0],
                        priority_track.center[0],
                        dt,
                    )
                    y_output = y_axis_pid.update(
                        frame_centre[1],
                        priority_track.center[1],
                        dt,
                    )

                    print(
                        f"Priority Target ID: {priority_track.track_id}, "
                        f"Center: {priority_track.center}, "
                        f"BBox: {priority_track.bbox}, "
                        f"PID Output - X: {x_output:.2f}, Y: {y_output:.2f}"
                    )

                    if CANBUS_ENABLED:
                        canbus_send(x_output, y_output)
                elif CANBUS_ENABLED:
                    canbus_stop()

            cv2.imshow(WINDOW_NAME, frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        if CANBUS_ENABLED:
            canbus_stop()

        if canbus is not None:
            canbus.close()

        camera.stop()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":

    print("Starting W.A.R.D. vision system...")
    sys.exit(main())
