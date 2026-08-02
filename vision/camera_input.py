import threading
import time

from .bootstrap import ensure_runtime_environment


ensure_runtime_environment()

import cv2


def make_gstreamer_pipeline(camera_url: str) -> str:
    """
    Receive H.264 RTSP over UDP, decode using the Jetson hardware decoder,
    and expose BGR frames to OpenCV.

    appsink drop=true and max-buffers=1 prevent old frames accumulating.
    """

    return (
        f'rtspsrc location="{camera_url}" '
        "latency=0 "
        "protocols=udp "
        "drop-on-latency=true "
        "buffer-mode=none ! "
        "rtph264depay ! "
        "h264parse ! "
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
