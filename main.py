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

CANBUS_ENABLED = True  # Set to True to enable CAN bus communication for PID output.

X_STEPPER_MAX_SPEED = 5000
Y_STEPPER_MAX_SPEED = 800
X_STEPPER_SPEED_SCALE = 25.0
Y_STEPPER_SPEED_SCALE = 1.0
# Set to -1 if positive visual error needs negative X motor speed on this turret.
X_STEPPER_DIRECTION = 1
Y_STEPPER_DIRECTION = 1
# Hold X still when the target is within this many image pixels of centre.
# This is applied before PID/scaling, so the visual target—not motor speed—sets
# the dead zone. Increase it if the target still chatters around centre.
X_VISION_DEADBAND_PIXELS = 5
Y_VISION_DEADBAND_PIXELS = 5

X_TARGET_OFFSET_PIXELS = 80
Y_TARGET_OFFSET_PIXELS = 0

# Exponential moving average for the detected target centre before PID.  A
# lower value smooths detector noise more, but also adds tracking delay.
TARGET_CENTER_EMA_ALPHA = 0.5

KP = 0.3
KI = 0.00
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


def pid_output_to_stepper_speed(
    output: float,
    scale: float,
    max_speed: int,
    direction: int,
) -> int:
    speed = int(round(output * scale * direction))
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
    last_pid_update_time = time.perf_counter()
    smoothed_target_id = None
    smoothed_target_centre = None

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

            # Use the real interval between control-loop updates.  The display
            # FPS is intentionally averaged over a second and is unsuitable for
            # the PID derivative/integral calculations.
            pid_update_time = time.perf_counter()
            dt = max(pid_update_time - last_pid_update_time, 1e-3)
            last_pid_update_time = pid_update_time

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
                    detected_centre = priority_track.center
                    if smoothed_target_id != priority_track.track_id:
                        # Never blend a newly selected target with the previous
                        # target's position.
                        smoothed_target_id = priority_track.track_id
                        smoothed_target_centre = (
                            float(detected_centre[0]),
                            float(detected_centre[1]),
                        )
                    else:
                        # Smooth small frame-to-frame detector-centre jumps
                        # before they reach the PID (and especially its D term).
                        previous_x, previous_y = smoothed_target_centre
                        smoothed_target_centre = (
                            (1.0 - TARGET_CENTER_EMA_ALPHA) * previous_x
                            + TARGET_CENTER_EMA_ALPHA * detected_centre[0],
                            (1.0 - TARGET_CENTER_EMA_ALPHA) * previous_y
                            + TARGET_CENTER_EMA_ALPHA * detected_centre[1],
                        )

                    frame_centre = get_frame_centre_point(frame)
                    x_error_pixels = frame_centre[0] - smoothed_target_centre[0] + X_TARGET_OFFSET_PIXELS
                    y_error_pixels = frame_centre[1] - smoothed_target_centre[1] + Y_TARGET_OFFSET_PIXELS
                    if abs(x_error_pixels) <= X_VISION_DEADBAND_PIXELS:
                        # Clear PID state so a prior correction cannot cause a
                        # kick when the target later leaves the visual dead zone.
                        x_axis_pid.reset()
                        x_output = 0.0
                    else:
                        x_output = x_axis_pid.update(
                            0,
                            -x_error_pixels,
                            dt,
                        )
                    if abs(y_error_pixels) <= Y_VISION_DEADBAND_PIXELS:
                        y_axis_pid.reset()
                        y_output = 0.0
                    else:
                        y_output = y_axis_pid.update(
                            0,
                            -y_error_pixels,
                            dt,
                        )

                    print(
                        f"Priority Target ID: {priority_track.track_id}, "
                        f"Center: {detected_centre}, "
                        f"Smoothed center: {smoothed_target_centre}, "
                        f"BBox: {priority_track.bbox}, "
                        f"PID Output - X: {x_output:.2f}, Y: {y_output:.2f}"
                    )

                    if CANBUS_ENABLED:
                        canbus_send(x_output, y_output)
                else:
                    # Reset the filter so reacquiring a target cannot pull its
                    # centre toward the last target's position.
                    smoothed_target_id = None
                    smoothed_target_centre = None
                    if CANBUS_ENABLED:
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
