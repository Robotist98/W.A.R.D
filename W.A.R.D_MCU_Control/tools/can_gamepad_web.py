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
    <title>W.A.R.D Gamepad Control</title>
    <style>
      :root {
        color-scheme: light;
        --bg-top: #f5efe6;
        --bg-bottom: #d2dde5;
        --panel: #0c1a1f;
        --panel-alt: #162a31;
        --accent: #f05d23;
        --accent-2: #2a9d8f;
        --text: #f4f1ea;
        --muted: #9db1b6;
      }
      * {
        box-sizing: border-box;
      }
      body {
        margin: 0;
        font-family: "Space Grotesk", "Segoe UI", sans-serif;
        background: radial-gradient(circle at top, #ffffff, var(--bg-top), var(--bg-bottom));
        min-height: 100vh;
        display: flex;
        justify-content: center;
        align-items: center;
        padding: 24px;
        color: var(--text);
      }
      .panel {
        width: min(960px, 100%);
        background: linear-gradient(160deg, var(--panel), #0b1418);
        border-radius: 24px;
        padding: 28px;
        box-shadow: 0 30px 80px rgba(0, 0, 0, 0.35);
        border: 1px solid rgba(255, 255, 255, 0.05);
      }
      header {
        display: flex;
        justify-content: space-between;
        align-items: flex-start;
        gap: 16px;
        margin-bottom: 20px;
      }
      h1 {
        margin: 0;
        font-size: 1.8rem;
        letter-spacing: 0.05em;
        text-transform: uppercase;
      }
      .subtitle {
        margin: 6px 0 0 0;
        color: var(--muted);
      }
      .grid {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(240px, 1fr));
        gap: 16px;
      }
      .card {
        background: var(--panel-alt);
        padding: 16px;
        border-radius: 16px;
        border: 1px solid rgba(255, 255, 255, 0.06);
      }
      .card h2 {
        margin: 0 0 10px 0;
        font-size: 1rem;
        letter-spacing: 0.04em;
        text-transform: uppercase;
      }
      label {
        display: block;
        margin-bottom: 6px;
        color: var(--muted);
        font-size: 0.85rem;
      }
      input, button {
        width: 100%;
        border-radius: 10px;
        border: 1px solid transparent;
        padding: 10px 12px;
        font-size: 0.95rem;
      }
      input {
        background: #0a1114;
        color: var(--text);
        border-color: rgba(255, 255, 255, 0.08);
      }
      button {
        background: var(--accent);
        color: #160b08;
        font-weight: 700;
        cursor: pointer;
        transition: transform 0.1s ease;
      }
      button.secondary {
        background: var(--accent-2);
        color: #081413;
      }
      button:active {
        transform: scale(0.98);
      }
      .status {
        font-size: 0.85rem;
        color: var(--muted);
      }
      .row {
        display: flex;
        gap: 8px;
      }
      .row button {
        flex: 1;
      }
      .pill {
        display: inline-flex;
        align-items: center;
        gap: 8px;
        padding: 6px 12px;
        border-radius: 999px;
        background: rgba(255, 255, 255, 0.08);
        font-size: 0.8rem;
      }
      .meter {
        height: 6px;
        width: 100%;
        background: rgba(255, 255, 255, 0.08);
        border-radius: 999px;
        overflow: hidden;
        margin-top: 6px;
      }
      .meter span {
        display: block;
        height: 100%;
        background: var(--accent-2);
        width: 0%;
        transition: width 0.1s ease;
      }
      .toggle {
        display: flex;
        justify-content: space-between;
        align-items: center;
        gap: 8px;
        margin-bottom: 8px;
      }
      .toggle input {
        width: auto;
      }
      .footer {
        margin-top: 16px;
        display: flex;
        justify-content: space-between;
        flex-wrap: wrap;
        gap: 12px;
        color: var(--muted);
        font-size: 0.85rem;
      }
    </style>
  </head>
  <body>
    <div class="panel">
      <header>
        <div>
          <h1>W.A.R.D Gamepad Control</h1>
          <p class="subtitle">Left stick drives turret motion. Data streams as CAN speed commands.</p>
        </div>
        <div class="pill" id="padStatus">Gamepad: Not connected</div>
      </header>
      <div class="grid">
        <div class="card">
          <h2>Speed Mapping</h2>
          <label for="maxSpeed">Max speed (steps/sec)</label>
          <input id="maxSpeed" type="number" value="600" min="0" max="2000" />
          <label for="deadzone">Deadzone (0-0.4)</label>
          <input id="deadzone" type="number" value="0.08" min="0" max="0.4" step="0.01" />
          <div class="toggle">
            <label for="invertX">Invert X</label>
            <input id="invertX" type="checkbox" />
          </div>
          <div class="toggle">
            <label for="invertY">Invert Y</label>
            <input id="invertY" type="checkbox" checked />
          </div>
          <div class="toggle">
            <label for="swapAxes">Swap axes</label>
            <input id="swapAxes" type="checkbox" />
          </div>
          <div class="toggle">
            <label for="enableSend">Send CAN updates</label>
            <input id="enableSend" type="checkbox" checked />
          </div>
          <button class="secondary" onclick="sendStop()">Send Stop</button>
          <div class="status" id="sendStatus">Idle</div>
        </div>
        <div class="card">
          <h2>Live Input</h2>
          <div class="status">Axis X: <span id="axisX">0.00</span></div>
          <div class="meter"><span id="axisXMeter"></span></div>
          <div class="status">Axis Y: <span id="axisY">0.00</span></div>
          <div class="meter"><span id="axisYMeter"></span></div>
          <div class="status">Speed X: <span id="speedX">0</span></div>
          <div class="status">Speed Y: <span id="speedY">0</span></div>
        </div>
        <div class="card">
          <h2>Power</h2>
          <div class="row">
            <button onclick="setPower(true)">Power On</button>
            <button onclick="setPower(false)">Power Off</button>
          </div>
          <div class="status" id="powerStatus">Idle</div>
        </div>
        <div class="card">
          <h2>Notes</h2>
          <p class="status">Connect a gamepad and move the left stick to drive motion. Speeds are clamped to 16-bit signed values.</p>
          <p class="status">If you lose connection, press Send Stop or disable updates.</p>
        </div>
      </div>
      <div class="footer">
        <span>CAN Command: 0x14 (Speed X/Y)</span>
        <span>Polling: 20 Hz with change detection</span>
      </div>
    </div>
    <script>
      const state = {
        gamepadIndex: null,
        lastSend: 0,
        lastPayload: null,
      };

      function deadzone(value, dz) {
        const abs = Math.abs(value);
        if (abs < dz) return 0;
        const scaled = (abs - dz) / (1 - dz);
        return Math.sign(value) * scaled;
      }

      function updateMeter(el, value) {
        const pct = Math.min(100, Math.max(0, (Math.abs(value) * 100)));
        el.style.width = pct.toFixed(0) + "%";
      }

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

      async function sendSpeed(xSpeed, ySpeed) {
        const data = await postJson("/api/speed", { x_speed: xSpeed, y_speed: ySpeed });
        document.getElementById("sendStatus").textContent = data.status;
      }

      async function sendStop() {
        await sendSpeed(0, 0);
      }

      function getConfig() {
        return {
          maxSpeed: Number(document.getElementById("maxSpeed").value) || 0,
          deadzone: Number(document.getElementById("deadzone").value) || 0,
          invertX: document.getElementById("invertX").checked,
          invertY: document.getElementById("invertY").checked,
          swapAxes: document.getElementById("swapAxes").checked,
          enableSend: document.getElementById("enableSend").checked,
        };
      }

      function update() {
        const pads = navigator.getGamepads ? navigator.getGamepads() : [];
        const pad = state.gamepadIndex !== null ? pads[state.gamepadIndex] : null;
        const status = document.getElementById("padStatus");
        if (!pad) {
          status.textContent = "Gamepad: Not connected";
          requestAnimationFrame(update);
          return;
        }

        status.textContent = `Gamepad: ${pad.id}`;
        const config = getConfig();
        let axisX = pad.axes[0] || 0;
        let axisY = pad.axes[1] || 0;
        if (config.swapAxes) {
          const temp = axisX;
          axisX = axisY;
          axisY = temp;
        }
        axisX = deadzone(axisX, config.deadzone);
        axisY = deadzone(axisY, config.deadzone);
        if (config.invertX) axisX *= -1;
        if (config.invertY) axisY *= -1;

        const xSpeed = Math.round(axisX * config.maxSpeed);
        const ySpeed = Math.round(axisY * config.maxSpeed);

        document.getElementById("axisX").textContent = axisX.toFixed(2);
        document.getElementById("axisY").textContent = axisY.toFixed(2);
        updateMeter(document.getElementById("axisXMeter"), axisX);
        updateMeter(document.getElementById("axisYMeter"), axisY);
        document.getElementById("speedX").textContent = xSpeed;
        document.getElementById("speedY").textContent = ySpeed;

        const now = performance.now();
        const payload = `${xSpeed},${ySpeed}`;
        if (config.enableSend && (payload !== state.lastPayload || now - state.lastSend > 1000)) {
          if (now - state.lastSend > 50) {
            state.lastSend = now;
            state.lastPayload = payload;
            sendSpeed(xSpeed, ySpeed).catch(() => {});
          }
        }

        requestAnimationFrame(update);
      }

      window.addEventListener("gamepadconnected", (event) => {
        state.gamepadIndex = event.gamepad.index;
        document.getElementById("padStatus").textContent = `Gamepad: ${event.gamepad.id}`;
      });

      window.addEventListener("gamepaddisconnected", () => {
        state.gamepadIndex = null;
        sendStop().catch(() => {});
      });

      update();
    </script>
  </body>
</html>
"""

CAN_CMD_SET_POWER = 0x10
CAN_CMD_SET_SPEED = 0x14

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


@app.route("/api/speed", methods=["POST"])
def api_speed():
  data = request.get_json(silent=True) or {}
  x_speed = int(data.get("x_speed", 0))
  y_speed = int(data.get("y_speed", 0))
  x_speed = clamp(x_speed, -32768, 32767)
  y_speed = clamp(y_speed, -32768, 32767)
  payload = struct.pack(">hh", x_speed, y_speed)
  send_can(CAN_CMD_SET_SPEED, payload)
  return jsonify(status=f"Speed X {x_speed}, Y {y_speed} sent")


def main():
  parser = argparse.ArgumentParser(description="CAN gamepad web controller")
  parser.add_argument("--interface", default="socketcan", help="python-can interface (e.g. socketcan, pcan)")
  parser.add_argument("--channel", default="can0", help="CAN channel (e.g. can0, PCAN_USBBUS1)")
  parser.add_argument("--bitrate", type=int, default=500000, help="CAN bitrate")
  parser.add_argument("--host", default="0.0.0.0", help="Host to bind")
  parser.add_argument("--port", type=int, default=8001, help="Port to bind")
  args = parser.parse_args()

  global bus
  bus = can.Bus(interface=args.interface, channel=args.channel, bitrate=args.bitrate)
  app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
  main()
