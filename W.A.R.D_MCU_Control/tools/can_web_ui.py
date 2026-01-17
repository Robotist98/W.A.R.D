import argparse
import struct
import threading
import time

import cv2

import can
from flask import Flask, jsonify, render_template_string, request

APP_HTML = """<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <title>W.A.R.D CAN Control</title>
    <style>
      :root {
        color-scheme: light;
        --bg: #0f1a24;
        --panel: #132838;
        --accent: #f4b23f;
        --text: #f6f2ea;
        --muted: #a6b6c6;
      }
      body {
        margin: 0;
        font-family: "Fira Sans", "Trebuchet MS", sans-serif;
        background: radial-gradient(circle at top, #1c3b52, var(--bg));
        color: var(--text);
        min-height: 100vh;
        display: flex;
        align-items: center;
        justify-content: center;
        padding: 32px;
      }
      .panel {
        width: min(720px, 100%);
        background: var(--panel);
        border-radius: 20px;
        padding: 28px;
        box-shadow: 0 20px 60px rgba(0, 0, 0, 0.45);
      }
      h1 {
        font-weight: 700;
        letter-spacing: 0.04em;
        text-transform: uppercase;
        margin: 0 0 8px 0;
      }
      p {
        margin: 0 0 20px 0;
        color: var(--muted);
      }
      .grid {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
        gap: 16px;
      }
      .card {
        background: rgba(255, 255, 255, 0.04);
        border-radius: 16px;
        padding: 16px;
      }
      label {
        display: block;
        margin-bottom: 6px;
        font-size: 0.9rem;
        color: var(--muted);
      }
      input, button {
        width: 100%;
        box-sizing: border-box;
        border-radius: 10px;
        border: 1px solid transparent;
        padding: 10px 12px;
        font-size: 1rem;
        margin-bottom: 10px;
      }
      input {
        background: #0d1c28;
        color: var(--text);
        border-color: #203445;
      }
      button {
        background: var(--accent);
        color: #1b1b1b;
        font-weight: 700;
        cursor: pointer;
        transition: transform 0.1s ease;
      }
      button:active {
        transform: scale(0.98);
      }
      .row {
        display: flex;
        gap: 8px;
      }
      .row button {
        flex: 1;
      }
      .status {
        font-size: 0.85rem;
        color: var(--muted);
      }
      input[type="range"] {
        accent-color: var(--accent);
      }
    </style>
  </head>
  <body>
    <div class="panel">
      <h1>W.A.R.D Control</h1>
      <p>Send CAN commands to the Feather board.</p>
      <div class="grid">
        <div class="card">
          <label for="power">Main Power</label>
          <div class="row">
            <button onclick="setPower(true)">On</button>
            <button onclick="setPower(false)">Off</button>
          </div>
          <div class="status" id="powerStatus">Idle</div>
        </div>
        <div class="card">
          <label for="xSteps">Move X (steps)</label>
          <input id="xSteps" type="number" value="100" />
          <div class="row">
            <button onclick="moveAxis('x', 1)">Forward</button>
            <button onclick="moveAxis('x', -1)">Reverse</button>
          </div>
          <div class="status" id="xStatus">Idle</div>
        </div>
        <div class="card">
          <label for="ySteps">Move Y (steps)</label>
          <input id="ySteps" type="number" value="100" />
          <div class="row">
            <button onclick="moveAxis('y', 1)">Forward</button>
            <button onclick="moveAxis('y', -1)">Reverse</button>
          </div>
          <div class="status" id="yStatus">Idle</div>
        </div>
        <div class="card">
          <label for="servoAngle">Servo Angle (90-145)</label>
          <input id="servoAngle" type="range" min="90" max="145" value="100" oninput="servoValue.textContent = this.value" />
          <div class="status">Angle: <span id="servoValue">100</span></div>
          <button onclick="setServo()">Set Servo</button>
          <button onclick="fireServo()">Fire</button>
          <div class="status" id="servoStatus">Idle</div>
        </div>
        <div class="card">
          <label>Fire Settings</label>
          <label for="fireForward">Forward Angle</label>
          <input id="fireForward" type="number" min="90" max="145" value="145" />
          <label for="fireBack">Return Angle</label>
          <input id="fireBack" type="number" min="90" max="145" value="90" />
          <label for="fireDelay">Delay (sec)</label>
          <input id="fireDelay" type="number" min="0" max="5" step="0.05" value="0.30" />
          <button onclick="setFireConfig()">Apply Fire Settings</button>
          <div class="status" id="fireStatus">Idle</div>
        </div>
        <div class="card">
          <label>Camera Stream</label>
          <img src="/stream.mjpg" alt="Camera stream" style="width: 100%; border-radius: 12px; border: 1px solid #203445;" />
          <div class="status">MJPEG stream</div>
        </div>
        <div class="card">
          <label>Camera Settings</label>
          <label for="camIndex">Index</label>
          <input id="camIndex" type="number" min="0" max="10" value="0" />
          <label for="camWidth">Width</label>
          <input id="camWidth" type="number" min="160" max="1920" value="640" />
          <label for="camHeight">Height</label>
          <input id="camHeight" type="number" min="120" max="1080" value="480" />
          <label for="camFps">FPS</label>
          <input id="camFps" type="number" min="1" max="60" value="15" />
          <button onclick="setCamera()">Apply Camera Settings</button>
          <div class="status" id="cameraStatus">Idle</div>
        </div>
      </div>
    </div>
    <script>
      async function postJson(url, payload) {
        const res = await fetch(url, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(payload),
        });
        return res.json();
      }

      async function setPower(state) {
        const data = await postJson("/api/power", { state });
        document.getElementById("powerStatus").textContent = data.status;
      }

      async function moveAxis(axis, direction) {
        const inputId = axis === "x" ? "xSteps" : "ySteps";
        const statusId = axis === "x" ? "xStatus" : "yStatus";
        const steps = Number(document.getElementById(inputId).value) * direction;
        const data = await postJson("/api/move", { axis, steps });
        document.getElementById(statusId).textContent = data.status;
      }

      async function setServo() {
        const angle = Number(document.getElementById("servoAngle").value);
        const data = await postJson("/api/servo", { angle });
        document.getElementById("servoStatus").textContent = data.status;
      }

      async function fireServo() {
        const data = await postJson("/api/fire", {});
        document.getElementById("servoStatus").textContent = data.status;
      }

      async function setFireConfig() {
        const forward = Number(document.getElementById("fireForward").value);
        const back = Number(document.getElementById("fireBack").value);
        const delay = Number(document.getElementById("fireDelay").value);
        const data = await postJson("/api/fire-config", { forward, back, delay });
        document.getElementById("fireStatus").textContent = data.status;
      }

      async function setCamera() {
        const index = Number(document.getElementById("camIndex").value);
        const width = Number(document.getElementById("camWidth").value);
        const height = Number(document.getElementById("camHeight").value);
        const fps = Number(document.getElementById("camFps").value);
        const data = await postJson("/api/camera", { index, width, height, fps });
        document.getElementById("cameraStatus").textContent = data.status;
      }
    </script>
  </body>
</html>
"""

CAN_CMD_PTM_STOP = 0x00
CAN_CMD_PTM_START_FW = 0x01
CAN_CMD_PTM_START_REV = 0x02

CAN_CMD_SET_POWER = 0x10
CAN_CMD_MOVE_X = 0x11
CAN_CMD_MOVE_Y = 0x12
CAN_CMD_SET_SERVO = 0x13

SERVO_MIN = 90
SERVO_MAX = 145
SERVO_FIRE_FORWARD = SERVO_MAX
SERVO_FIRE_RETURN = SERVO_MIN
SERVO_FIRE_DELAY_SEC = 0.3

app = Flask(__name__)
bus = None
bus_lock = threading.Lock()
camera_lock = threading.Lock()
camera = None
camera_index = 0
camera_width = 640
camera_height = 480
camera_fps = 15


def clamp(value, min_value, max_value):
  return max(min_value, min(value, max_value))


def send_can(cmd_id, payload):
  msg = can.Message(
    arbitration_id=cmd_id,
    is_extended_id=False,
    data=payload,
  )
  with bus_lock:
    bus.send(msg)


@app.route("/")
def index():
  return render_template_string(APP_HTML)


@app.route("/api/power", methods=["POST"])
def api_power():
  data = request.get_json(silent=True) or {}
  state = bool(data.get("state", False))
  send_can(CAN_CMD_SET_POWER, bytes([1 if state else 0]))
  return jsonify(status=f"Power {'ON' if state else 'OFF'} sent")


@app.route("/api/move", methods=["POST"])
def api_move():
  data = request.get_json(silent=True) or {}
  axis = str(data.get("axis", "x")).lower()
  steps = int(data.get("steps", 0))
  steps = clamp(steps, -32768, 32767)
  payload = struct.pack(">h", steps)
  if axis == "x":
    send_can(CAN_CMD_MOVE_X, payload)
  else:
    send_can(CAN_CMD_MOVE_Y, payload)
  return jsonify(status=f"Move {axis.upper()} {steps} steps sent")


@app.route("/api/servo", methods=["POST"])
def api_servo():
  data = request.get_json(silent=True) or {}
  angle = int(data.get("angle", SERVO_MIN))
  angle = clamp(angle, SERVO_MIN, SERVO_MAX)
  send_can(CAN_CMD_SET_SERVO, bytes([angle]))
  return jsonify(status=f"Servo angle {angle} sent")


def fire_servo_sequence():
  send_can(CAN_CMD_SET_SERVO, bytes([SERVO_FIRE_FORWARD]))
  time.sleep(SERVO_FIRE_DELAY_SEC)
  send_can(CAN_CMD_SET_SERVO, bytes([SERVO_FIRE_RETURN]))


@app.route("/api/fire", methods=["POST"])
def api_fire():
  threading.Thread(target=fire_servo_sequence, daemon=True).start()
  return jsonify(status="Fire sequence sent")


@app.route("/api/fire-config", methods=["POST"])
def api_fire_config():
  global SERVO_FIRE_FORWARD, SERVO_FIRE_RETURN, SERVO_FIRE_DELAY_SEC
  data = request.get_json(silent=True) or {}
  forward = int(data.get("forward", SERVO_FIRE_FORWARD))
  back = int(data.get("back", SERVO_FIRE_RETURN))
  delay = float(data.get("delay", SERVO_FIRE_DELAY_SEC))

  forward = clamp(forward, SERVO_MIN, SERVO_MAX)
  back = clamp(back, SERVO_MIN, SERVO_MAX)
  delay = max(0.0, min(delay, 5.0))
  SERVO_FIRE_FORWARD = forward
  SERVO_FIRE_RETURN = back
  SERVO_FIRE_DELAY_SEC = delay

  return jsonify(status=f"Fire config set (fwd {forward}, back {back}, delay {delay:.2f}s)")


def get_camera():
  global camera
  with camera_lock:
    if camera is None or not camera.isOpened():
      cam = cv2.VideoCapture(camera_index)
      cam.set(cv2.CAP_PROP_FRAME_WIDTH, camera_width)
      cam.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_height)
      cam.set(cv2.CAP_PROP_FPS, camera_fps)
      camera = cam
  return camera


@app.route("/api/camera", methods=["POST"])
def api_camera():
  data = request.get_json(silent=True) or {}
  index = int(data.get("index", camera_index))
  width = int(data.get("width", camera_width))
  height = int(data.get("height", camera_height))
  fps = int(data.get("fps", camera_fps))

  width = max(160, min(width, 1920))
  height = max(120, min(height, 1080))
  fps = max(1, min(fps, 60))

  global camera_index, camera_width, camera_height, camera_fps, camera
  camera_index = index
  camera_width = width
  camera_height = height
  camera_fps = fps

  with camera_lock:
    if camera is not None:
      camera.release()
      camera = None

  return jsonify(status=f"Camera set (index {index}, {width}x{height}@{fps})")


def mjpeg_stream():
  while True:
    cam = get_camera()
    ok, frame = cam.read()
    if not ok:
      time.sleep(0.05)
      continue
    ok, buf = cv2.imencode(".jpg", frame)
    if not ok:
      continue
    jpg = buf.tobytes()
    yield (
      b"--frame\r\n"
      b"Content-Type: image/jpeg\r\n"
      b"Content-Length: " + str(len(jpg)).encode("ascii") + b"\r\n\r\n" +
      jpg + b"\r\n"
    )


@app.route("/stream.mjpg")
def stream_mjpg():
  return app.response_class(mjpeg_stream(), mimetype="multipart/x-mixed-replace; boundary=frame")


def main():
  parser = argparse.ArgumentParser(description="CAN web UI controller")
  parser.add_argument("--interface", default="socketcan", help="python-can interface (e.g. socketcan, pcan)")
  parser.add_argument("--channel", default="can0", help="CAN channel (e.g. can0, PCAN_USBBUS1)")
  parser.add_argument("--bitrate", type=int, default=500000, help="CAN bitrate")
  parser.add_argument("--camera-index", type=int, default=0, help="OpenCV camera index")
  parser.add_argument("--camera-width", type=int, default=640, help="Camera width")
  parser.add_argument("--camera-height", type=int, default=480, help="Camera height")
  parser.add_argument("--camera-fps", type=int, default=15, help="Camera FPS")
  parser.add_argument("--host", default="0.0.0.0", help="Host to bind")
  parser.add_argument("--port", type=int, default=8000, help="Port to bind")
  args = parser.parse_args()

  global bus
  global camera_index, camera_width, camera_height, camera_fps
  bus = can.Bus(interface=args.interface, channel=args.channel, bitrate=args.bitrate)
  camera_index = args.camera_index
  camera_width = args.camera_width
  camera_height = args.camera_height
  camera_fps = args.camera_fps
  app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
  main()
