import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO


def build_pipeline(width: int, height: int, fps: int) -> rs.pipeline:
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    pipeline.start(config)
    return pipeline


def start_mjpeg_server(
    host: str,
    port: int,
    get_rgb_frame,
    get_depth_frame,
    stream_fps: int,
    jpeg_quality: int,
) -> HTTPServer:
    class MJPEGHandler(BaseHTTPRequestHandler):
        def log_message(self, format: str, *args) -> None:
            return

        def do_GET(self) -> None:
            if self.path in ("/", "/index.html"):
                page = (
                    "<html><body>"
                    "<h2>RealSense + YOLO</h2>"
                    '<div><img src="/rgb.mjpg" /></div>'
                    "<h2>RealSense Depth</h2>"
                    '<div><img src="/depth.mjpg" /></div>'
                    "</body></html>"
                )
                body = page.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if self.path not in ("/rgb.mjpg", "/depth.mjpg"):
                self.send_response(404)
                self.end_headers()
                return

            self.send_response(200)
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=frame"
            )
            self.end_headers()
            is_depth = self.path == "/depth.mjpg"

            try:
                while True:
                    frame = get_depth_frame() if is_depth else get_rgb_frame()
                    if frame is None:
                        time.sleep(0.01)
                        continue
                    ok, jpg = cv2.imencode(
                        ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]
                    )
                    if not ok:
                        continue
                    data = jpg.tobytes()
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(data)}\r\n\r\n".encode())
                    self.wfile.write(data)
                    self.wfile.write(b"\r\n")
                    if stream_fps > 0:
                        time.sleep(1.0 / stream_fps)
            except (BrokenPipeError, ConnectionResetError):
                return

    server = HTTPServer((host, port), MJPEGHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def run(
    model_path: str,
    width: int,
    height: int,
    fps: int,
    conf: float,
    imgsz: int,
    infer_every: int,
    device: str,
    half: bool,
    display: bool,
    stream: bool,
    host: str,
    port: int,
    stream_fps: int,
    jpeg_quality: int,
) -> None:
    model = YOLO(model_path)
    if half:
        model.to(device)
    pipeline = build_pipeline(width, height, fps)
    align = rs.align(rs.stream.color)
    latest = {"rgb": None, "depth": None}
    lock = threading.Lock()
    server = None

    def get_rgb():
        with lock:
            return latest["rgb"]

    def get_depth():
        with lock:
            return latest["depth"]

    if stream:
        server = start_mjpeg_server(
            host, port, get_rgb, get_depth, stream_fps, jpeg_quality
        )

    try:
        frame_idx = 0
        last_annotated = None
        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03),
                cv2.COLORMAP_JET,
            )

            frame_idx += 1
            if infer_every <= 1 or frame_idx % infer_every == 0:
                results = model.predict(
                    image,
                    conf=conf,
                    imgsz=imgsz,
                    device=device,
                    quantize=16 if half else None,
                    verbose=False,
                )
                annotated = results[0].plot()
                last_annotated = annotated
            else:
                annotated = last_annotated if last_annotated is not None else image

            with lock:
                latest["rgb"] = annotated
                latest["depth"] = depth_colormap

            if display:
                cv2.imshow("RealSense + YOLO (RGB)", annotated)
                cv2.imshow("RealSense Depth (Aligned)", depth_colormap)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q") or key == 27:
                    break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        if server is not None:
            server.shutdown()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RealSense RGB + YOLO inference")
    parser.add_argument("--model", default="yolov8n.pt", help="Model path or name")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--imgsz", type=int, default=640, help="YOLO inference size")
    parser.add_argument(
        "--infer-every",
        type=int,
        default=1,
        help="Run inference every N frames (higher = lower load)",
    )
    parser.add_argument("--device", default="cuda", help="Inference device")
    parser.add_argument("--half", action="store_true", help="Use FP16 if supported")
    parser.add_argument("--headless", action="store_true", help="Disable local display")
    parser.add_argument("--stream", action="store_true", help="Enable MJPEG streaming")
    parser.add_argument("--host", default="0.0.0.0", help="Stream bind host")
    parser.add_argument("--port", type=int, default=8080, help="Stream port")
    parser.add_argument("--stream-fps", type=int, default=15, help="MJPEG FPS cap")
    parser.add_argument(
        "--jpeg-quality", type=int, default=80, help="MJPEG JPEG quality (1-100)"
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    run(
        args.model,
        args.width,
        args.height,
        args.fps,
        args.conf,
        args.imgsz,
        max(1, args.infer_every),
        args.device,
        args.half,
        display=not args.headless,
        stream=args.stream,
        host=args.host,
        port=args.port,
        stream_fps=max(0, args.stream_fps),
        jpeg_quality=max(1, min(100, args.jpeg_quality)),
    )


if __name__ == "__main__":
    main()
