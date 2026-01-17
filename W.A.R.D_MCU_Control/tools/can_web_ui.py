import argparse
import struct
import threading

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
          <div class="status" id="servoStatus">Idle</div>
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

app = Flask(__name__)
bus = None
bus_lock = threading.Lock()


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


def main():
  parser = argparse.ArgumentParser(description="CAN web UI controller")
  parser.add_argument("--interface", default="socketcan", help="python-can interface (e.g. socketcan, pcan)")
  parser.add_argument("--channel", default="can0", help="CAN channel (e.g. can0, PCAN_USBBUS1)")
  parser.add_argument("--bitrate", type=int, default=500000, help="CAN bitrate")
  parser.add_argument("--host", default="0.0.0.0", help="Host to bind")
  parser.add_argument("--port", type=int, default=8000, help="Port to bind")
  args = parser.parse_args()

  global bus
  bus = can.Bus(interface=args.interface, channel=args.channel, bitrate=args.bitrate)
  app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
  main()
