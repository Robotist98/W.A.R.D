#!/usr/bin/env python3
#!/usr/bin/env python3

import os
import sys

LIB_GL_DISPATCH = "/lib/aarch64-linux-gnu/libGLdispatch.so.0"

if os.environ.get("LD_PRELOAD") != LIB_GL_DISPATCH:
    environment = os.environ.copy()
    environment["LD_PRELOAD"] = LIB_GL_DISPATCH

    os.execve(
        sys.executable,
        [sys.executable] + sys.argv,
        environment,
    )


import numpy as np

if "bool" not in np.__dict__:
    np.bool = bool

import threading
import time
from pathlib import Path

import cv2
from ultralytics import YOLO


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

CAMERA_URL = "rtsp://192.168.144.26:8554/main.264"
# TensorRT model generated on this Jetson.
MODEL_PATH = Path(__file__).parent / "yolov8n.engine"

# Must normally match the size used when exporting the TensorRT engine.
INFERENCE_SIZE = 640

CONFIDENCE_THRESHOLD = 0.30
IOU_THRESHOLD = 0.45

# Set to a class ID list to detect only selected classes.
# Example: [0] for person on a standard COCO model.
CLASSES = None

WINDOW_NAME = "Low-latency YOLO"


# ---------------------------------------------------------------------------
# Low-latency GStreamer pipeline
# ---------------------------------------------------------------------------

def make_gstreamer_pipeline() -> str:
    """
    Receive H.265 RTSP over TCP, decode using the Jetson hardware decoder,
    and expose BGR frames to OpenCV.

    appsink drop=true and max-buffers=1 prevent old frames accumulating.
    """

    return (
        f'rtspsrc location="{CAMERA_URL}" '
        "latency=100 "
        "protocols=tcp "
        "drop-on-latency=true "
        "buffer-mode=none ! "
        "rtph265depay ! "
        "h265parse ! "
        "nvv4l2decoder "
        "disable-dpb=true "
        "enable-max-performance=true ! "
        "nvvidconv ! "
        "video/x-raw,format=BGRx ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink "
        "drop=true "
        "max-buffers=1 "
        "sync=false "
        "enable-last-sample=false"
    )


# ---------------------------------------------------------------------------
# Latest-frame camera reader
# ---------------------------------------------------------------------------

class LatestFrameCamera:
    """
    Continuously reads the camera in a background thread.

    Only the newest frame is retained. This prevents inference from processing
    an increasingly old queue of frames when the camera FPS is higher than the
    model inference FPS.
    """

    def __init__(self, pipeline: str):
        self.pipeline = pipeline
        self.capture = None

        self.frame = None
        self.frame_lock = threading.Lock()

        self.running = False
        self.thread = None

        self.frames_received = 0

    def start(self) -> None:
        self.capture = cv2.VideoCapture(
            self.pipeline,
            cv2.CAP_GSTREAMER,
        )

        if not self.capture.isOpened():
            raise RuntimeError(
                "Could not open the RTSP stream through GStreamer.\n"
                "Check that OpenCV has GStreamer support and that the camera "
                "is reachable."
            )

        self.running = True
        self.thread = threading.Thread(
            target=self._reader,
            name="camera-reader",
            daemon=True,
        )
        self.thread.start()

    def _reader(self) -> None:
        while self.running:
            success, frame = self.capture.read()

            if not success:
                print("Warning: failed to receive camera frame.")
                time.sleep(0.01)
                continue

            with self.frame_lock:
                self.frame = frame
                self.frames_received += 1

    def get_latest_frame(self):
        with self.frame_lock:
            if self.frame is None:
                return None

            # Copy so the camera thread can safely replace its frame.
            return self.frame.copy()

    def stop(self) -> None:
        self.running = False

        if self.thread is not None:
            self.thread.join(timeout=1.0)

        if self.capture is not None:
            self.capture.release()


# ---------------------------------------------------------------------------
# Bounding-box drawing
# ---------------------------------------------------------------------------

def draw_detections(frame, result, class_names) -> None:
    """
    Draw boxes manually instead of calling result.plot().

    Manual drawing gives more control and avoids some additional plotting
    overhead.
    """

    if result.boxes is None or len(result.boxes) == 0:
        return

    boxes = result.boxes.xyxy.cpu().numpy()
    confidences = result.boxes.conf.cpu().numpy()
    class_ids = result.boxes.cls.cpu().numpy().astype(int)

    for box, confidence, class_id in zip(
        boxes,
        confidences,
        class_ids,
    ):
        x1, y1, x2, y2 = box.astype(int)

        class_name = class_names.get(class_id, str(class_id))
        if class_name == "person":
            label = f"{class_name} {confidence:.2f}"

            cv2.rectangle(
                frame,
                (x1, y1),
                (x2, y2),
                (0, 255, 0),
                2,
            )

            text_size, baseline = cv2.getTextSize(
                label,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                1,
            )

            text_width, text_height = text_size
            text_y = max(y1, text_height + baseline + 2)

            cv2.rectangle(
                frame,
                (x1, text_y - text_height - baseline - 4),
                (x1 + text_width + 4, text_y),
                (0, 255, 0),
                -1,
            )

            cv2.putText(
                frame,
                label,
                (x1 + 2, text_y - baseline - 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                1,
                cv2.LINE_AA,
            )


# ---------------------------------------------------------------------------
# Main application
# ---------------------------------------------------------------------------

def main() -> int:
    if not MODEL_PATH.exists():
        print(f"Error: model does not exist: {MODEL_PATH}")
        print("Export your trained .pt model to TensorRT first.")
        return 1

    print(f"Loading TensorRT model: {MODEL_PATH}")
    model = YOLO(str(MODEL_PATH), task="detect")

    pipeline = make_gstreamer_pipeline()

    print(f"Opening camera: {CAMERA_URL}")
    camera = LatestFrameCamera(pipeline)

    try:
        camera.start()
    except RuntimeError as error:
        print(error)
        return 1

    print("Waiting for the first frame...")

    while camera.get_latest_frame() is None:
        time.sleep(0.01)

    print("Detection running. Press Q or Escape to exit.")

    displayed_frames = 0
    fps_start_time = time.perf_counter()
    display_fps = 0.0

    try:
        while True:
            frame = camera.get_latest_frame()

            if frame is None:
                continue

            inference_start = time.perf_counter()

            results = model.predict(
                source=frame,
                imgsz=INFERENCE_SIZE,
                conf=CONFIDENCE_THRESHOLD,
                iou=IOU_THRESHOLD,
                classes=CLASSES,
                device=0,
                half=True,
                verbose=False,
            )

            inference_ms = (
                time.perf_counter() - inference_start
            ) * 1000.0

            result = results[0]

            draw_detections(
                frame,
                result,
                model.names,
            )

            displayed_frames += 1
            elapsed = time.perf_counter() - fps_start_time

            if elapsed >= 1.0:
                display_fps = displayed_frames / elapsed
                displayed_frames = 0
                fps_start_time = time.perf_counter()

            detection_count = (
                len(result.boxes)
                if result.boxes is not None
                else 0
            )

            status = (
                f"Inference: {inference_ms:.1f} ms  "
                f"Display: {display_fps:.1f} FPS  "
                f"Objects: {detection_count}"
            )

            cv2.putText(
                frame,
                status,
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
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