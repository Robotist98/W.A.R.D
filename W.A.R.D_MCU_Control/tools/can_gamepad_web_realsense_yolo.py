import argparse
import json
import os
import struct
import threading
import time

import can
import cv2
import numpy as np
import pyrealsense2 as rs
from flask import Flask, jsonify, redirect, render_template_string, request
from ultralytics import YOLO

DEV_HTML = """<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <title>W.A.R.D Developer Control</title>
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
      a {
        color: var(--text);
        text-decoration: none;
        font-weight: 600;
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
          <h1>W.A.R.D Developer Control</h1>
          <p class="subtitle">Left stick drives turret motion. Data streams as CAN speed commands.</p>
        </div>
        <div>
          <div class="pill" id="padStatus">Gamepad: Not connected</div>
          <div class="status"><a href="/control">Back to Control</a></div>
        </div>
      </header>
      <div class="grid">
        <div class="card">
          <h2>Speed Mapping</h2>
          <label for="maxSpeedX">Max speed X (steps/sec)</label>
          <input id="maxSpeedX" type="number" value="{{ settings.maxSpeedX }}" min="0" max="2000" />
          <label for="maxSpeedY">Max speed Y (steps/sec)</label>
          <input id="maxSpeedY" type="number" value="{{ settings.maxSpeedY }}" min="0" max="2000" />
          <label for="accelX">Acceleration X (steps/sec^2)</label>
          <input id="accelX" type="number" value="{{ settings.accelX }}" min="0" max="5000" />
          <label for="accelY">Acceleration Y (steps/sec^2)</label>
          <input id="accelY" type="number" value="{{ settings.accelY }}" min="0" max="5000" />
          <label for="deadzone">Deadzone (0-0.4)</label>
          <input id="deadzone" type="number" value="{{ settings.deadzone }}" min="0" max="0.4" step="0.01" />
          <div class="toggle">
            <label for="invertX">Invert X</label>
            <input id="invertX" type="checkbox" {% if settings.invertX %}checked{% endif %} />
          </div>
          <div class="toggle">
            <label for="invertY">Invert Y</label>
            <input id="invertY" type="checkbox" {% if settings.invertY %}checked{% endif %} />
          </div>
          <div class="toggle">
            <label for="swapAxes">Swap sticks</label>
            <input id="swapAxes" type="checkbox" {% if settings.swapAxes %}checked{% endif %} />
          </div>
          <div class="toggle">
            <label for="enableSend">Send CAN updates</label>
            <input id="enableSend" type="checkbox" {% if settings.enableSend %}checked{% endif %} />
          </div>
          <button class="secondary" onclick="sendStop()">Send Stop</button>
          <button onclick="sendAccel()">Send Accel</button>
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
          <h2>Fire Control</h2>
          <label for="fireMode">Input mode</label>
          <select id="fireMode">
            <option value="button" {% if settings.fireMode == 'button' %}selected{% endif %}>Button</option>
            <option value="axis" {% if settings.fireMode == 'axis' %}selected{% endif %}>Axis</option>
          </select>
          <label for="fireButton">Button index</label>
          <input id="fireButton" type="number" value="{{ settings.fireButton }}" min="0" max="16" />
          <label for="fireAxis">Axis index</label>
          <input id="fireAxis" type="number" value="{{ settings.fireAxis }}" min="0" max="8" />
          <label for="fireThreshold">Axis threshold (0-1)</label>
          <input id="fireThreshold" type="number" value="{{ settings.fireThreshold }}" min="0" max="1" step="0.05" />
          <button onclick="sendFire()">Fire Now</button>
          <div class="status" id="fireStatus">Idle</div>
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
          <h2>RealSense Settings</h2>
          <label for="camWidth">Width</label>
          <input id="camWidth" type="number" min="160" max="1920" value="{{ settings.cameraWidth }}" />
          <label for="camHeight">Height</label>
          <input id="camHeight" type="number" min="120" max="1080" value="{{ settings.cameraHeight }}" />
          <label for="camFps">FPS</label>
          <input id="camFps" type="number" min="1" max="60" value="{{ settings.cameraFps }}" />
          <button onclick="setCamera()">Apply RealSense Settings</button>
          <div class="status" id="cameraStatus">Idle</div>
        </div>
        <div class="card">
          <h2>YOLO (Person Only)</h2>
          <div class="toggle">
            <label for="yoloEnabled">Enable YOLO</label>
            <input id="yoloEnabled" type="checkbox" {% if settings.yoloEnabled %}checked{% endif %} />
          </div>
          <label for="yoloModel">Model path</label>
          <input id="yoloModel" type="text" value="{{ settings.yoloModel }}" />
          <label for="yoloConf">Confidence</label>
          <input id="yoloConf" type="number" min="0" max="1" step="0.01" value="{{ settings.yoloConf }}" />
          <label for="yoloImgsz">Image size</label>
          <input id="yoloImgsz" type="number" min="320" max="1280" step="32" value="{{ settings.yoloImgSz }}" />
          <label for="yoloInferEvery">Infer every N frames</label>
          <input id="yoloInferEvery" type="number" min="1" max="30" value="{{ settings.yoloInferEvery }}" />
          <button onclick="applyYolo()">Apply YOLO Settings</button>
          <div class="status" id="yoloStatus">Idle</div>
        </div>
        <div class="card">
          <h2>Notes</h2>
          <p class="status">Left stick X controls X. Right stick Y controls Y. Speeds are clamped to 16-bit signed values.</p>
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
        lastFirePressed: false,
        lastFireTime: 0,
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

      async function sendAccel() {
        const accelX = Number(document.getElementById("accelX").value) || 0;
        const accelY = Number(document.getElementById("accelY").value) || 0;
        const data = await postJson("/api/accel", { x_accel: accelX, y_accel: accelY });
        document.getElementById("sendStatus").textContent = data.status;
      }

      async function sendFire() {
        const data = await postJson("/api/fire", {});
        document.getElementById("fireStatus").textContent = data.status;
      }

      async function sendStop() {
        await sendSpeed(0, 0);
      }

      async function saveSettings(payload) {
        await postJson("/api/settings", payload);
      }

      async function setCamera() {
        const width = Number(document.getElementById("camWidth").value);
        const height = Number(document.getElementById("camHeight").value);
        const fps = Number(document.getElementById("camFps").value);
        const data = await postJson("/api/camera", { width, height, fps });
        document.getElementById("cameraStatus").textContent = data.status;
      }

      function collectYoloSettings() {
        return {
          yoloEnabled: document.getElementById("yoloEnabled").checked,
          yoloModel: document.getElementById("yoloModel").value,
          yoloConf: Number(document.getElementById("yoloConf").value) || 0,
          yoloImgSz: Number(document.getElementById("yoloImgsz").value) || 0,
          yoloInferEvery: Number(document.getElementById("yoloInferEvery").value) || 1,
        };
      }

      async function applyYolo() {
        const payload = collectYoloSettings();
        const data = await postJson("/api/yolo", payload);
        document.getElementById("yoloStatus").textContent = data.status || data.error || "Updated";
        await saveSettings(payload);
      }

      function collectDevSettings() {
        return {
          maxSpeedX: Number(document.getElementById("maxSpeedX").value) || 0,
          maxSpeedY: Number(document.getElementById("maxSpeedY").value) || 0,
          accelX: Number(document.getElementById("accelX").value) || 0,
          accelY: Number(document.getElementById("accelY").value) || 0,
          deadzone: Number(document.getElementById("deadzone").value) || 0,
          invertX: document.getElementById("invertX").checked,
          invertY: document.getElementById("invertY").checked,
          swapAxes: document.getElementById("swapAxes").checked,
          enableSend: document.getElementById("enableSend").checked,
          fireMode: document.getElementById("fireMode").value,
          fireButton: Number(document.getElementById("fireButton").value) || 0,
          fireAxis: Number(document.getElementById("fireAxis").value) || 0,
          fireThreshold: Number(document.getElementById("fireThreshold").value) || 0,
          cameraWidth: Number(document.getElementById("camWidth").value) || 0,
          cameraHeight: Number(document.getElementById("camHeight").value) || 0,
          cameraFps: Number(document.getElementById("camFps").value) || 0,
          yoloEnabled: document.getElementById("yoloEnabled").checked,
          yoloModel: document.getElementById("yoloModel").value,
          yoloConf: Number(document.getElementById("yoloConf").value) || 0,
          yoloImgSz: Number(document.getElementById("yoloImgsz").value) || 0,
          yoloInferEvery: Number(document.getElementById("yoloInferEvery").value) || 1,
        };
      }

      function getConfig() {
        return {
          maxSpeedX: Number(document.getElementById("maxSpeedX").value) || 0,
          maxSpeedY: Number(document.getElementById("maxSpeedY").value) || 0,
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
        let axisY = pad.axes[3] || 0;
        if (config.swapAxes) {
          const temp = axisX;
          axisX = axisY;
          axisY = temp;
        }
        axisX = deadzone(axisX, config.deadzone);
        axisY = deadzone(axisY, config.deadzone);
        if (config.invertX) axisX *= -1;
        if (config.invertY) axisY *= -1;

        const xSpeed = Math.round(axisX * config.maxSpeedX);
        const ySpeed = Math.round(axisY * config.maxSpeedY);

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

        const fireMode = document.getElementById("fireMode").value;
        const fireButton = Number(document.getElementById("fireButton").value) || 0;
        const fireAxis = Number(document.getElementById("fireAxis").value) || 0;
        const fireThreshold = Number(document.getElementById("fireThreshold").value) || 0;
        let fired = false;
        if (fireMode === "axis") {
          const axisValue = pad.axes[fireAxis] || 0;
          const pressed = axisValue >= fireThreshold;
          if (pressed && !state.lastFirePressed) {
            fired = true;
          }
          state.lastFirePressed = pressed;
        } else {
          const pressed = Boolean(pad.buttons[fireButton] && pad.buttons[fireButton].pressed);
          if (pressed && !state.lastFirePressed) {
            fired = true;
          }
          state.lastFirePressed = pressed;
        }
        if (fired && now - state.lastFireTime > 300) {
          state.lastFireTime = now;
          sendFire().catch(() => {});
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

      document.querySelectorAll(
        "#maxSpeedX, #maxSpeedY, #accelX, #accelY, #deadzone, #invertX, #invertY, #swapAxes, #enableSend," +
        "#fireMode, #fireButton, #fireAxis, #fireThreshold, #camWidth, #camHeight, #camFps"
      ).forEach((el) => {
        el.addEventListener("change", () => saveSettings(collectDevSettings()).catch(() => {}));
      });

      document.querySelectorAll(
        "#yoloEnabled, #yoloModel, #yoloConf, #yoloImgsz, #yoloInferEvery"
      ).forEach((el) => {
        el.addEventListener("change", () => applyYolo().catch(() => {}));
      });

      update();
    </script>
  </body>
</html>
"""

CONTROL_HTML = """<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <title>W.A.R.D Control</title>
    <style>
      :root {
        color-scheme: light;
        --bg-top: #f5efe6;
        --bg-bottom: #d2dde5;
        --panel: #0c1a1f;
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
        width: min(1100px, 100%);
        background: linear-gradient(160deg, var(--panel), #0b1418);
        border-radius: 24px;
        padding: 24px;
        box-shadow: 0 30px 80px rgba(0, 0, 0, 0.35);
        border: 1px solid rgba(255, 255, 255, 0.05);
      }
      header {
        display: flex;
        justify-content: space-between;
        align-items: center;
        gap: 16px;
        margin-bottom: 16px;
      }
      h1 {
        margin: 0;
        font-size: 1.6rem;
        letter-spacing: 0.05em;
        text-transform: uppercase;
      }
      .stream {
        width: 100%;
        border-radius: 16px;
        border: 1px solid rgba(255, 255, 255, 0.08);
        background: #0a1114;
      }
      .toolbar {
        margin-top: 16px;
        display: flex;
        flex-wrap: wrap;
        gap: 10px;
        align-items: center;
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
      button {
        border-radius: 10px;
        border: 1px solid transparent;
        padding: 10px 14px;
        font-size: 0.95rem;
        background: var(--accent);
        color: #160b08;
        font-weight: 700;
        cursor: pointer;
      }
      button.secondary {
        background: var(--accent-2);
        color: #081413;
      }
      label {
        display: inline-flex;
        align-items: center;
        gap: 8px;
        color: var(--muted);
        font-size: 0.85rem;
      }
      .status {
        margin-top: 10px;
        color: var(--muted);
        font-size: 0.85rem;
      }
      a {
        color: var(--text);
        text-decoration: none;
        font-weight: 600;
      }
    </style>
  </head>
  <body>
    <div class="panel">
      <header>
        <h1>W.A.R.D Control</h1>
        <div class="pill" id="padStatus">Gamepad: Not connected</div>
      </header>
      <img class="stream" src="/stream.mjpg" alt="Camera stream" />
      <div class="toolbar">
        <label>
          <span>Enable gamepad</span>
          <input id="enableGamepad" type="checkbox" {% if settings.enableSend %}checked{% endif %} />
        </label>
        <label>
          <span>Enable YOLO</span>
          <input id="enableYolo" type="checkbox" {% if settings.yoloEnabled %}checked{% endif %} />
        </label>
        <button onclick="setPower(true)">Power On</button>
        <button onclick="setPower(false)">Power Off</button>
        <button class="secondary" onclick="sendFire()">Fire</button>
        <button class="secondary" onclick="sendStop()">Stop</button>
        <a href="/dev">Developer Controls</a>
      </div>
      <div class="status" id="controlStatus">Idle</div>
    </div>
    <script>
      const config = {
        maxSpeedX: {{ settings.maxSpeedX }},
        maxSpeedY: {{ settings.maxSpeedY }},
        deadzone: {{ settings.deadzone }},
        invertX: {{ "true" if settings.invertX else "false" }},
        invertY: {{ "true" if settings.invertY else "false" }},
        swapAxes: {{ "true" if settings.swapAxes else "false" }},
        enableSend: {{ "true" if settings.enableSend else "false" }},
        yoloEnabled: {{ "true" if settings.yoloEnabled else "false" }},
        fireMode: "{{ settings.fireMode }}",
        fireButton: {{ settings.fireButton }},
        fireAxis: {{ settings.fireAxis }},
        fireThreshold: {{ settings.fireThreshold }},
      };

      const state = {
        gamepadIndex: null,
        lastSend: 0,
        lastPayload: null,
        lastFirePressed: false,
        lastFireTime: 0,
      };

      async function postJson(url, payload) {
        const res = await fetch(url, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(payload),
        });
        return res.json();
      }

      async function setPower(stateValue) {
        const data = await postJson("/api/power", { state: stateValue });
        document.getElementById("controlStatus").textContent = data.status;
      }

      async function sendSpeed(xSpeed, ySpeed) {
        const data = await postJson("/api/speed", { x_speed: xSpeed, y_speed: ySpeed });
        document.getElementById("controlStatus").textContent = data.status;
      }

      async function sendStop() {
        await sendSpeed(0, 0);
      }

      async function sendFire() {
        const data = await postJson("/api/fire", {});
        document.getElementById("controlStatus").textContent = data.status;
      }

      async function saveSettings(payload) {
        await postJson("/api/settings", payload);
      }

      async function setYoloEnabled(stateValue) {
        const data = await postJson("/api/yolo", { yoloEnabled: stateValue });
        const status = data.status || data.error || "YOLO updated";
        document.getElementById("controlStatus").textContent = status;
      }

      function deadzone(value, dz) {
        const abs = Math.abs(value);
        if (abs < dz) return 0;
        const scaled = (abs - dz) / (1 - dz);
        return Math.sign(value) * scaled;
      }

      function applySettings(data) {
        if (!data) return;
        if (typeof data.maxSpeedX === "number") config.maxSpeedX = data.maxSpeedX;
        if (typeof data.maxSpeedY === "number") config.maxSpeedY = data.maxSpeedY;
        if (typeof data.deadzone === "number") config.deadzone = data.deadzone;
        if (typeof data.invertX === "boolean") config.invertX = data.invertX;
        if (typeof data.invertY === "boolean") config.invertY = data.invertY;
        if (typeof data.swapAxes === "boolean") config.swapAxes = data.swapAxes;
        if (typeof data.enableSend === "boolean") config.enableSend = data.enableSend;
        if (typeof data.yoloEnabled === "boolean") config.yoloEnabled = data.yoloEnabled;
        if (typeof data.fireMode === "string") config.fireMode = data.fireMode;
        if (typeof data.fireButton === "number") config.fireButton = data.fireButton;
        if (typeof data.fireAxis === "number") config.fireAxis = data.fireAxis;
        if (typeof data.fireThreshold === "number") config.fireThreshold = data.fireThreshold;
        const enable = document.getElementById("enableGamepad");
        enable.checked = Boolean(config.enableSend);
        const enableYolo = document.getElementById("enableYolo");
        enableYolo.checked = Boolean(config.yoloEnabled);
      }

      async function refreshSettings() {
        try {
          const res = await fetch("/api/settings");
          const data = await res.json();
          applySettings(data);
        } catch (err) {}
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
        let axisX = pad.axes[0] || 0;
        let axisY = pad.axes[3] || 0;
        if (config.swapAxes) {
          const temp = axisX;
          axisX = axisY;
          axisY = temp;
        }
        axisX = deadzone(axisX, config.deadzone);
        axisY = deadzone(axisY, config.deadzone);
        if (config.invertX) axisX *= -1;
        if (config.invertY) axisY *= -1;

        const xSpeed = Math.round(axisX * config.maxSpeedX);
        const ySpeed = Math.round(axisY * config.maxSpeedY);

        const now = performance.now();
        const payload = `${xSpeed},${ySpeed}`;
        if (config.enableSend && (payload !== state.lastPayload || now - state.lastSend > 1000)) {
          if (now - state.lastSend > 50) {
            state.lastSend = now;
            state.lastPayload = payload;
            sendSpeed(xSpeed, ySpeed).catch(() => {});
          }
        }

        let fired = false;
        if (config.fireMode === "axis") {
          const axisValue = pad.axes[config.fireAxis] || 0;
          const pressed = axisValue >= config.fireThreshold;
          if (pressed && !state.lastFirePressed) {
            fired = true;
          }
          state.lastFirePressed = pressed;
        } else {
          const pressed = Boolean(pad.buttons[config.fireButton] && pad.buttons[config.fireButton].pressed);
          if (pressed && !state.lastFirePressed) {
            fired = true;
          }
          state.lastFirePressed = pressed;
        }
        if (fired && now - state.lastFireTime > 300) {
          state.lastFireTime = now;
          sendFire().catch(() => {});
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

      document.getElementById("enableGamepad").addEventListener("change", (event) => {
        config.enableSend = event.target.checked;
        saveSettings({ enableSend: config.enableSend }).catch(() => {});
      });

      document.getElementById("enableYolo").addEventListener("change", (event) => {
        config.yoloEnabled = event.target.checked;
        setYoloEnabled(config.yoloEnabled).catch(() => {});
        saveSettings({ yoloEnabled: config.yoloEnabled }).catch(() => {});
      });

      window.addEventListener("pageshow", () => {
        refreshSettings();
      });

      refreshSettings();
      update();
    </script>
  </body>
</html>
"""

CAN_CMD_SET_POWER = 0x10
CAN_CMD_SET_SPEED = 0x14
CAN_CMD_SET_ACCEL = 0x15
CAN_CMD_SET_SERVO = 0x13

SERVO_MIN = 90
SERVO_MAX = 145
SERVO_FIRE_FORWARD = SERVO_MAX
SERVO_FIRE_RETURN = SERVO_MIN
SERVO_FIRE_DELAY_SEC = 0.3

app = Flask(__name__)
bus = None
bus_lock = threading.Lock()
rs_lock = threading.Lock()
rs_pipeline = None
rs_align = None
rs_width = 640
rs_height = 480
rs_fps = 15
latest_lock = threading.Lock()
latest_frame = None
capture_stop = threading.Event()
capture_thread = None
yolo_model = None
yolo_model_path = "yolov8n.pt"
yolo_lock = threading.Lock()
yolo_enabled = True
yolo_conf = 0.25
yolo_imgsz = 640
yolo_device = "cuda"
yolo_half = False
yolo_infer_every = 1
settings_lock = threading.Lock()
SETTINGS_PATH = os.path.join(os.path.dirname(__file__), "gamepad_settings_rs_yolo.json")

DEFAULT_SETTINGS = {
  "maxSpeedX": 600,
  "maxSpeedY": 500,
  "accelX": 300,
  "accelY": 250,
  "deadzone": 0.08,
  "invertX": False,
  "invertY": True,
  "swapAxes": False,
  "enableSend": True,
  "fireMode": "button",
  "fireButton": 0,
  "fireAxis": 5,
  "fireThreshold": 0.6,
  "cameraWidth": 640,
  "cameraHeight": 480,
  "cameraFps": 15,
  "yoloEnabled": True,
  "yoloModel": "yolov8n.pt",
  "yoloConf": 0.25,
  "yoloImgSz": 640,
  "yoloInferEvery": 1,
}

def normalize_settings(values):
  mapped = dict(values)
  if "maxSpeedX" not in values and "basicSpeed" in values:
    mapped["maxSpeedX"] = values.get("basicSpeed", mapped.get("maxSpeedX", 0))
  if "deadzone" not in values and "basicDeadzone" in values:
    mapped["deadzone"] = values.get("basicDeadzone", mapped.get("deadzone", 0))
  if "invertY" not in values and "basicInvertY" in values:
    mapped["invertY"] = values.get("basicInvertY", mapped.get("invertY", False))
  if "enableSend" not in values and "basicEnableGamepad" in values:
    mapped["enableSend"] = values.get("basicEnableGamepad", mapped.get("enableSend", True))
  return mapped


def load_settings():
  settings = dict(DEFAULT_SETTINGS)
  if os.path.exists(SETTINGS_PATH):
    try:
      with open(SETTINGS_PATH, "r", encoding="utf-8") as handle:
        stored = json.load(handle)
      if isinstance(stored, dict):
        settings.update(normalize_settings(stored))
    except (OSError, json.JSONDecodeError):
      pass
  return normalize_settings(settings)


def save_settings(settings):
  try:
    cleaned = {key: value for key, value in settings.items() if not key.startswith("basic")}
    with open(SETTINGS_PATH, "w", encoding="utf-8") as handle:
      json.dump(cleaned, handle, indent=2, sort_keys=True)
  except OSError:
    pass


settings = load_settings()


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
  return redirect("/control")

@app.route("/control")
def control():
  with settings_lock:
    current = dict(settings)
  return render_template_string(CONTROL_HTML, settings=current)


@app.route("/dev")
def dev():
  with settings_lock:
    current = dict(settings)
  return render_template_string(DEV_HTML, settings=current)


@app.route("/api/settings", methods=["GET", "POST"])
def api_settings():
  global settings
  if request.method == "GET":
    with settings_lock:
      settings = load_settings()
      current = dict(settings)
    return jsonify(current)

  data = request.get_json(silent=True) or {}
  if not isinstance(data, dict):
    return jsonify(status="Invalid settings payload"), 400
  with settings_lock:
    settings.update(normalize_settings(data))
    settings.update(normalize_settings(settings))
    save_settings(settings)
    current = dict(settings)
  return jsonify(current)


@app.route("/api/yolo", methods=["POST"])
def api_yolo():
  global yolo_model, yolo_model_path, yolo_enabled, yolo_conf, yolo_imgsz, yolo_infer_every
  data = request.get_json(silent=True) or {}
  if not isinstance(data, dict):
    return jsonify(error="Invalid payload"), 400

  enabled = bool(data.get("yoloEnabled", yolo_enabled))
  model_path = str(data.get("yoloModel", yolo_model_path))
  conf = float(data.get("yoloConf", yolo_conf))
  imgsz = int(data.get("yoloImgSz", yolo_imgsz))
  infer_every = int(data.get("yoloInferEvery", yolo_infer_every))

  conf = max(0.0, min(conf, 1.0))
  imgsz = max(320, min(imgsz, 1280))
  infer_every = max(1, min(infer_every, 30))

  try:
    with yolo_lock:
      if model_path != yolo_model_path:
        new_model = YOLO(model_path)
        if yolo_half:
          new_model.to(yolo_device)
        yolo_model = new_model
        yolo_model_path = model_path

      yolo_enabled = enabled
      yolo_conf = conf
      yolo_imgsz = imgsz
      yolo_infer_every = infer_every
  except Exception as exc:
    return jsonify(error=f"Failed to load YOLO model: {exc}"), 400

  with settings_lock:
    settings["yoloEnabled"] = enabled
    settings["yoloModel"] = model_path
    settings["yoloConf"] = conf
    settings["yoloImgSz"] = imgsz
    settings["yoloInferEvery"] = infer_every
    save_settings(settings)

  status = "YOLO enabled" if enabled else "YOLO disabled"
  return jsonify(status=f"{status} (person only)")


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


@app.route("/api/accel", methods=["POST"])
def api_accel():
  data = request.get_json(silent=True) or {}
  x_accel = int(data.get("x_accel", 0))
  y_accel = int(data.get("y_accel", 0))
  x_accel = clamp(x_accel, 0, 5000)
  y_accel = clamp(y_accel, 0, 5000)
  payload = struct.pack(">hh", x_accel, y_accel)
  send_can(CAN_CMD_SET_ACCEL, payload)
  with settings_lock:
    settings["accelX"] = x_accel
    settings["accelY"] = y_accel
    save_settings(settings)
  return jsonify(status=f"Accel X {x_accel}, Y {y_accel} sent")


def fire_sequence():
  send_can(CAN_CMD_SET_SERVO, bytes([SERVO_FIRE_FORWARD]))
  time.sleep(SERVO_FIRE_DELAY_SEC)
  send_can(CAN_CMD_SET_SERVO, bytes([SERVO_FIRE_RETURN]))


@app.route("/api/fire", methods=["POST"])
def api_fire():
  threading.Thread(target=fire_sequence, daemon=True).start()
  return jsonify(status="Fire sequence sent")


def build_realsense_pipeline(width, height, fps):
  pipeline = rs.pipeline()
  config = rs.config()
  config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
  config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
  pipeline.start(config)
  return pipeline, rs.align(rs.stream.color)


def start_realsense():
  global rs_pipeline, rs_align
  with rs_lock:
    if rs_pipeline is None:
      rs_pipeline, rs_align = build_realsense_pipeline(rs_width, rs_height, rs_fps)
  return rs_pipeline, rs_align


def restart_realsense():
  global rs_pipeline, rs_align
  with rs_lock:
    if rs_pipeline is not None:
      try:
        rs_pipeline.stop()
      except Exception:
        pass
    rs_pipeline = None
    rs_align = None
  start_realsense()


def capture_loop():
  global latest_frame
  frame_idx = 0
  last_annotated = None
  while not capture_stop.is_set():
    try:
      pipeline, align = start_realsense()
      frames = pipeline.wait_for_frames(1000)
      aligned_frames = align.process(frames)
      color_frame = aligned_frames.get_color_frame()
      if not color_frame:
        continue

      image = np.asanyarray(color_frame.get_data())
      frame_idx += 1
      with yolo_lock:
        enabled = yolo_enabled
        model = yolo_model
        conf = yolo_conf
        imgsz = yolo_imgsz
        device = yolo_device
        half = yolo_half
        infer_every = yolo_infer_every

      if enabled and model is not None:
        if infer_every <= 1 or frame_idx % infer_every == 0:
          results = model.predict(
            image,
            conf=conf,
            imgsz=imgsz,
            device=device,
            half=half,
            classes=[0],
            verbose=False,
          )
          last_annotated = results[0].plot()
        annotated = last_annotated if last_annotated is not None else image
      else:
        annotated = image

      with latest_lock:
        latest_frame = annotated
    except Exception:
      time.sleep(0.05)


@app.route("/api/camera", methods=["POST"])
def api_camera():
  global rs_width, rs_height, rs_fps
  data = request.get_json(silent=True) or {}
  width = int(data.get("width", rs_width))
  height = int(data.get("height", rs_height))
  fps = int(data.get("fps", rs_fps))

  width = max(160, min(width, 1920))
  height = max(120, min(height, 1080))
  fps = max(1, min(fps, 60))
  rs_width = width
  rs_height = height
  rs_fps = fps
  with settings_lock:
    settings["cameraWidth"] = width
    settings["cameraHeight"] = height
    settings["cameraFps"] = fps
    save_settings(settings)

  restart_realsense()

  return jsonify(status=f"RealSense set ({width}x{height}@{fps})")


def mjpeg_stream():
  while True:
    with latest_lock:
      frame = None if latest_frame is None else latest_frame.copy()
    if frame is None:
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
  parser = argparse.ArgumentParser(description="CAN gamepad web controller (RealSense + YOLO)")
  parser.add_argument("--interface", default="socketcan", help="python-can interface (e.g. socketcan, pcan)")
  parser.add_argument("--channel", default="can0", help="CAN channel (e.g. can0, PCAN_USBBUS1)")
  parser.add_argument("--bitrate", type=int, default=500000, help="CAN bitrate")
  parser.add_argument("--rs-width", type=int, default=640, help="RealSense color width")
  parser.add_argument("--rs-height", type=int, default=480, help="RealSense color height")
  parser.add_argument("--rs-fps", type=int, default=15, help="RealSense FPS")
  parser.add_argument("--model", default="yolov8n.pt", help="YOLO model path or name")
  parser.add_argument("--conf", type=float, default=0.5, help="YOLO confidence")
  parser.add_argument("--imgsz", type=int, default=640, help="YOLO inference size")
  parser.add_argument("--infer-every", type=int, default=1, help="Run YOLO every N frames")
  parser.add_argument("--device", default="cuda", help="YOLO device")
  parser.add_argument("--half", action="store_true", help="Use FP16 if supported")
  parser.add_argument("--host", default="0.0.0.0", help="Host to bind")
  parser.add_argument("--port", type=int, default=8001, help="Port to bind")
  args = parser.parse_args()

  global bus
  bus = can.Bus(interface=args.interface, channel=args.channel, bitrate=args.bitrate)
  global rs_width, rs_height, rs_fps
  with settings_lock:
    rs_width = settings.get("cameraWidth", args.rs_width)
    rs_height = settings.get("cameraHeight", args.rs_height)
    rs_fps = settings.get("cameraFps", args.rs_fps)

  global yolo_model, yolo_model_path, yolo_enabled, yolo_conf, yolo_imgsz, yolo_device, yolo_half, yolo_infer_every
  yolo_device = args.device
  yolo_half = args.half
  with settings_lock:
    yolo_enabled = bool(settings.get("yoloEnabled", True))
    yolo_model_path = settings.get("yoloModel", args.model)
    yolo_conf = float(settings.get("yoloConf", args.conf))
    yolo_imgsz = int(settings.get("yoloImgSz", args.imgsz))
    yolo_infer_every = int(settings.get("yoloInferEvery", args.infer_every))
  yolo_conf = max(0.0, min(yolo_conf, 1.0))
  yolo_imgsz = max(320, min(yolo_imgsz, 1280))
  yolo_infer_every = max(1, yolo_infer_every)
  yolo_model = YOLO(yolo_model_path)
  if yolo_half:
    yolo_model.to(yolo_device)

  global capture_thread
  capture_stop.clear()
  capture_thread = threading.Thread(target=capture_loop, daemon=True)
  capture_thread.start()
  app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
  main()
