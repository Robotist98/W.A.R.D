#!/usr/bin/env python3

"""Run YOLO on an incoming RTSP camera and serve the annotated video as RTSP."""

import argparse
import os
import socket
import sys
import threading
import time
from pathlib import Path


LIB_GL_DISPATCH = "/lib/aarch64-linux-gnu/libGLdispatch.so.0"

# Keep the same Jetson OpenGL workaround used by ip_vision.py.
if os.environ.get("LD_PRELOAD") != LIB_GL_DISPATCH:
    environment = os.environ.copy()
    environment["LD_PRELOAD"] = LIB_GL_DISPATCH
    os.execve(sys.executable, [sys.executable] + sys.argv, environment)


import cv2
import numpy as np
from ultralytics import YOLO

if "bool" not in np.__dict__:
    np.bool = bool

try:
    import gi

    gi.require_version("Gst", "1.0")
    gi.require_version("GstRtspServer", "1.0")
    from gi.repository import GLib, Gst, GstRtspServer

    RTSP_IMPORT_ERROR = None
except (ImportError, ValueError) as error:
    GLib = Gst = GstRtspServer = None
    RTSP_IMPORT_ERROR = error


DEFAULT_CAMERA_URL = "rtsp://192.168.144.26:8554/main.264"
DEFAULT_MODEL_PATH = Path(__file__).parent / "yolov8n.engine"


def make_camera_pipeline(camera_url: str, latency_ms: int) -> str:
    """Return a low-latency H.265/Jetson pipeline that exposes BGR frames."""

    return (
        f'rtspsrc location="{camera_url}" '
        f"latency={latency_ms} "
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


class LatestFrameCamera:
    """Read continuously while retaining only the newest camera frame."""

    def __init__(self, pipeline: str):
        self.pipeline = pipeline
        self.capture = None
        self.frame = None
        self.frame_lock = threading.Lock()
        self.running = False
        self.thread = None

    def start(self) -> None:
        self.capture = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if not self.capture.isOpened():
            raise RuntimeError(
                "Could not open the input RTSP stream through GStreamer. "
                "Check the camera URL, connectivity, and OpenCV GStreamer support."
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
                print("Warning: failed to receive an input camera frame.")
                time.sleep(0.01)
                continue

            with self.frame_lock:
                self.frame = frame

    def get_latest_frame(self):
        with self.frame_lock:
            return None if self.frame is None else self.frame.copy()

    def stop(self) -> None:
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        if self.capture is not None:
            self.capture.release()


def draw_detections(frame, result, class_names) -> None:
    """Draw all detections onto a BGR frame."""

    if result.boxes is None or len(result.boxes) == 0:
        return

    boxes = result.boxes.xyxy.cpu().numpy()
    confidences = result.boxes.conf.cpu().numpy()
    class_ids = result.boxes.cls.cpu().numpy().astype(int)

    for box, confidence, class_id in zip(boxes, confidences, class_ids):
        x1, y1, x2, y2 = box.astype(int)
        class_name = class_names.get(class_id, str(class_id))
        label = f"{class_name} {confidence:.2f}"

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
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


class AnnotatedRtspServer:
    """Serve BGR NumPy frames as a shared, low-latency H.264 RTSP stream."""

    def __init__(
        self,
        host: str,
        port: int,
        path: str,
        width: int,
        height: int,
        fps: int,
        bitrate_kbps: int,
    ):
        Gst.init(None)

        if Gst.ElementFactory.find("x264enc") is None:
            raise RuntimeError(
                "The GStreamer x264enc element is unavailable. Install the "
                "gstreamer1.0-plugins-ugly package."
            )

        self.width = width
        self.height = height
        self.frame_duration = Gst.util_uint64_scale_int(1, Gst.SECOND, fps)
        self.frame_number = 0
        self.source = None
        self.source_lock = threading.Lock()
        self.loop = GLib.MainLoop()

        mount_path = "/" + path.strip("/")
        launch = (
            "( appsrc name=source is-live=true block=false format=time "
            "do-timestamp=true "
            f"caps=video/x-raw,format=BGR,width={width},height={height},framerate={fps}/1 ! "
            "queue leaky=downstream max-size-buffers=1 ! "
            "videoconvert ! video/x-raw,format=I420 ! "
            f"x264enc tune=zerolatency speed-preset=ultrafast bitrate={bitrate_kbps} "
            f"key-int-max={fps} bframes=0 byte-stream=true ! "
            "h264parse config-interval=1 ! "
            "rtph264pay name=pay0 pt=96 config-interval=1 )"
        )

        factory = GstRtspServer.RTSPMediaFactory.new()
        factory.set_launch(launch)
        factory.set_shared(True)
        factory.connect("media-configure", self._on_media_configure)

        self.server = GstRtspServer.RTSPServer.new()
        self.server.set_address(host)
        self.server.set_service(str(port))
        self.server.get_mount_points().add_factory(mount_path, factory)

        if self.server.attach(None) == 0:
            raise RuntimeError(f"Could not bind the RTSP server to {host}:{port}.")

        self.thread = threading.Thread(
            target=self.loop.run,
            name="rtsp-server",
            daemon=True,
        )

    def _on_media_configure(self, _factory, media) -> None:
        source = media.get_element().get_child_by_name("source")
        source.set_property("format", Gst.Format.TIME)
        with self.source_lock:
            self.source = source
            self.frame_number = 0
        media.connect("unprepared", self._on_media_unprepared, source)

    def _on_media_unprepared(self, _media, source) -> None:
        with self.source_lock:
            if self.source == source:
                self.source = None

    def start(self) -> None:
        self.thread.start()

    def push(self, frame) -> None:
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))
        frame = np.ascontiguousarray(frame)

        with self.source_lock:
            source = self.source
            if source is None:
                return

            buffer = Gst.Buffer.new_allocate(None, frame.nbytes, None)
            buffer.fill(0, frame.tobytes())
            buffer.duration = self.frame_duration
            self.frame_number += 1

        result = source.emit("push-buffer", buffer)
        if result != Gst.FlowReturn.OK:
            print(f"Warning: RTSP pipeline rejected a frame: {result.value_nick}")

    def stop(self) -> None:
        self.loop.quit()
        self.thread.join(timeout=1.0)


def parse_classes(value: str | None):
    if value is None or value.strip().lower() in {"", "all", "none"}:
        return None
    return [int(item.strip()) for item in value.split(",")]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Detect objects in an RTSP camera and host the annotated stream over RTSP."
    )
    parser.add_argument("--camera-url", default=DEFAULT_CAMERA_URL)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL_PATH)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.30)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument(
        "--classes",
        help="Comma-separated class IDs, for example 0 for person; default: all",
    )
    parser.add_argument("--device", default="0")
    parser.add_argument("--input-latency", type=int, default=100)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8555)
    parser.add_argument("--path", default="detected")
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--bitrate", type=int, default=4000, help="H.264 bitrate in kbit/s")
    parser.add_argument("--display", action="store_true", help="Also show a local preview")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if RTSP_IMPORT_ERROR is not None:
        print(f"Error: GStreamer RTSP server bindings are unavailable: {RTSP_IMPORT_ERROR}")
        print("Install them with:")
        print("  sudo apt install gir1.2-gst-rtsp-server-1.0")
        return 1
    if not args.model.exists():
        print(f"Error: model does not exist: {args.model}")
        return 1
    if not 1 <= args.port <= 65535 or args.fps < 1 or args.bitrate < 1:
        print("Error: port, fps, and bitrate must be positive and valid.")
        return 1

    print(f"Loading model: {args.model}")
    model = YOLO(str(args.model), task="detect")
    camera = LatestFrameCamera(
        make_camera_pipeline(args.camera_url, max(0, args.input_latency))
    )

    try:
        print(f"Opening input camera: {args.camera_url}")
        camera.start()
        print("Waiting for the first input frame...")
        first_frame = None
        while first_frame is None:
            first_frame = camera.get_latest_frame()
            time.sleep(0.01)

        height, width = first_frame.shape[:2]
        server = AnnotatedRtspServer(
            args.host,
            args.port,
            args.path,
            width,
            height,
            args.fps,
            args.bitrate,
        )
        server.start()
    except (KeyboardInterrupt, RuntimeError) as error:
        camera.stop()
        if str(error):
            print(f"Error: {error}")
        return 1

    stream_host = socket.gethostname() if args.host == "0.0.0.0" else args.host
    stream_url = f"rtsp://{stream_host}:{args.port}/{args.path.strip('/')}"
    print(f"Annotated RTSP stream ready at: {stream_url}")
    print("Press Ctrl+C to stop.")

    classes = parse_classes(args.classes)
    frame_count = 0
    fps_started = time.perf_counter()
    measured_fps = 0.0

    try:
        while True:
            frame = camera.get_latest_frame()
            if frame is None:
                time.sleep(0.001)
                continue

            started = time.perf_counter()
            result = model.predict(
                source=frame,
                imgsz=args.imgsz,
                conf=args.conf,
                iou=args.iou,
                classes=classes,
                device=args.device,
                quantize=16,
                verbose=False,
            )[0]
            inference_ms = (time.perf_counter() - started) * 1000.0
            draw_detections(frame, result, model.names)

            frame_count += 1
            elapsed = time.perf_counter() - fps_started
            if elapsed >= 1.0:
                measured_fps = frame_count / elapsed
                frame_count = 0
                fps_started = time.perf_counter()

            object_count = len(result.boxes) if result.boxes is not None else 0
            status = (
                f"Inference: {inference_ms:.1f} ms  "
                f"Stream: {measured_fps:.1f} FPS  Objects: {object_count}"
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

            server.push(frame)
            if args.display:
                cv2.imshow("YOLO RTSP output", frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        server.stop()
        camera.stop()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    sys.exit(main())
