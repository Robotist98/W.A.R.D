#!/usr/bin/env python3
"""Interactive terminal for sending W.A.R.D. MCU CAN commands.

Examples:
    python3 can_terminal.py
    python3 can_terminal.py --channel can0 --command "move x 250"
    python3 can_terminal.py --interface pcan --channel PCAN_USBBUS1
"""

import argparse
import shlex
import struct
import sys
import threading
from dataclasses import dataclass
from typing import Callable, Optional

try:
    import can
except ImportError:  # Keep --help useful on machines without python-can.
    can = None

try:
    from textual.app import App, ComposeResult
    from textual.containers import Container, Grid
    from textual.widgets import Button, Footer, Header, Input, RichLog, Static
except ImportError:
    App = None


CAN_ID_MAINPOWER = 0x10
CAN_ID_RUNMOVE_X = 0x11
CAN_ID_RUNMOVE_Y = 0x12
CAN_ID_SERVO_TRIGGER = 0x13
CAN_ID_RUNSPEED_XY = 0x14
CAN_ID_SET_CONFIG_X = 0x15
CAN_ID_SET_CONFIG_Y = 0x16
CAN_ID_MOVE_Y_TO_ZERO = 0x17
CAN_COM_READ_SPEED = 1
CAN_COM_READ_ACCEL = 2
ADDRESS_FIRMWARE_VERSION = 0x600
CAN_ID_STATUS_TELEMETRY = 0x602

SERVO_MIN_ANGLE = 90
SERVO_MAX_ANGLE = 145
INT16_MIN = -32768
INT16_MAX = 32767


@dataclass(frozen=True)
class Frame:
    """A standard 11-bit CAN frame to transmit."""

    arbitration_id: int
    data: bytes


def int_in_range(text: str, minimum: int, maximum: int, name: str) -> int:
    try:
        value = int(text, 0)
    except ValueError as exc:
        raise ValueError(f"{name} must be an integer") from exc
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} must be between {minimum} and {maximum}")
    return value


def parse_command(line: str) -> Optional[Frame]:
    """Convert one terminal command to a CAN frame; None means no transmission."""
    parts = shlex.split(line)
    if not parts:
        return None

    command = parts[0].lower()
    if command in {"help", "?", "quit", "exit", "commands"}:
        return None

    if command == "power" and len(parts) == 2:
        state = parts[1].lower()
        if state not in {"on", "off"}:
            raise ValueError("usage: power on|off")
        return Frame(CAN_ID_MAINPOWER, bytes([state == "on"]))

    if command == "move" and len(parts) == 3:
        axis = parts[1].lower()
        if axis not in {"x", "y"}:
            raise ValueError("axis must be x or y")
        steps = int_in_range(parts[2], INT16_MIN, INT16_MAX, "steps")
        can_id = CAN_ID_RUNMOVE_X if axis == "x" else CAN_ID_RUNMOVE_Y
        return Frame(can_id, struct.pack(">h", steps))

    if command == "speed" and len(parts) == 3:
        x_speed = int_in_range(parts[1], INT16_MIN, INT16_MAX, "X speed")
        y_speed = int_in_range(parts[2], INT16_MIN, INT16_MAX, "Y speed")
        return Frame(CAN_ID_RUNSPEED_XY, struct.pack(">hh", x_speed, y_speed))

    if command == "accel" and len(parts) == 3:
        axis = parts[1].lower()
        if axis not in {"x", "y"}:
            raise ValueError("axis must be x or y")
        acceleration = int_in_range(parts[2], 0, INT16_MAX, "acceleration")
        can_id = CAN_ID_SET_CONFIG_X if axis == "x" else CAN_ID_SET_CONFIG_Y
        return Frame(can_id, bytes([CAN_COM_READ_ACCEL]) + struct.pack(">h", acceleration))

    if command == "config-speed" and len(parts) == 3:
        axis = parts[1].lower()
        if axis not in {"x", "y"}:
            raise ValueError("axis must be x or y")
        speed = int_in_range(parts[2], INT16_MIN, INT16_MAX, "speed")
        can_id = CAN_ID_SET_CONFIG_X if axis == "x" else CAN_ID_SET_CONFIG_Y
        return Frame(can_id, bytes([CAN_COM_READ_SPEED]) + struct.pack(">h", speed))

    if command == "home" and len(parts) == 2 and parts[1].lower() == "y":
        return Frame(CAN_ID_MOVE_Y_TO_ZERO, b"")

    if command == "servo" and len(parts) == 2:
        angle = int_in_range(parts[1], SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, "servo angle")
        return Frame(CAN_ID_SERVO_TRIGGER, bytes([angle]))

    if command == "version" and len(parts) == 1:
        return Frame(ADDRESS_FIRMWARE_VERSION, b"")

    if command == "raw" and len(parts) >= 2:
        can_id = int_in_range(parts[1], 0, 0x7FF, "CAN ID")
        try:
            payload = bytes.fromhex("".join(parts[2:]))
        except ValueError as exc:
            raise ValueError("raw bytes must be hexadecimal, e.g. 01 FF 2A") from exc
        if len(payload) > 8:
            raise ValueError("CAN payload can contain at most 8 bytes")
        return Frame(can_id, payload)

    raise ValueError("unknown command or wrong arguments; enter 'help' for commands")


HELP_TEXT = """Commands:
  power on|off                 Set the main power relay
  move x|y <steps>             Relative move (-32768..32767)
  speed <x> <y>                Set X/Y speed-mode speeds
  accel x|y <steps_per_s2>     Set acceleration for one axis
  config-speed x|y <speed>     Set the saved speed-mode value for one axis
  home y                       Home the Y axis to zero
  servo <angle>                Set servo angle (90..145)
  version                      Request MCU firmware version (ID 0x600)
  raw <can_id> [hex bytes]     Send any standard CAN frame, e.g. raw 0x10 01
  help                         Show this text
  quit                         Exit
"""

APP_CSS = """
Screen { background: #101820; }
#connection { color: #f4b23f; margin: 1 2 0 2; }
#telemetry { color: #6ed8b5; margin: 0 2; height: 1; }
#command { margin: 1 2; }
#controls { grid-size: 4; grid-gutter: 1; margin: 0 2 1 2; }
#controls Button { width: 1fr; }
#controls .danger { background: #9c2d2d; }
#status { color: #a6b6c6; margin: 0 2 1 2; height: 1; }
#log { background: #071018; border: round #26445a; height: 1fr; margin: 0 2 1 2; padding: 1; }
"""


def format_frame(frame: Frame) -> str:
    payload = frame.data.hex(" ").upper() or "(empty)"
    return f"sent 0x{frame.arbitration_id:03X}  [{payload}]"


def decode_status_telemetry(payload: bytes) -> str:
    """Decode the eight-byte status frame produced by the MCU firmware."""
    if len(payload) != 8:
        raise ValueError(f"status telemetry must be 8 bytes, received {len(payload)}")
    x_speed, y_speed = struct.unpack(">hh", payload[:4])
    power = "ON" if payload[4] else "OFF"
    mode_names = {0: "position", 1: "speed", 2: "homing", 3: "idle"}
    x_mode = mode_names.get(payload[5], f"unknown({payload[5]})")
    y_mode = mode_names.get(payload[6], f"unknown({payload[6]})")
    homing = " · Y homing" if payload[7] & 0x01 else ""
    return f"Live MCU: X {x_speed} steps/s ({x_mode}) · Y {y_speed} steps/s ({y_mode}) · Power {power}{homing}"


def send_frame(bus, frame: Frame) -> None:
    bus.send(can.Message(arbitration_id=frame.arbitration_id, data=frame.data,
                         is_extended_id=False))


def run_command(bus, line: str, output: Callable[[str], None]) -> bool:
    """Handle one line. Returns False only when the caller should quit."""
    command = line.strip().lower()
    if command in {"quit", "exit"}:
        return False
    if command in {"help", "?", "commands"}:
        output(HELP_TEXT.rstrip())
        return True
    try:
        frame = parse_command(line)
        if frame is not None:
            send_frame(bus, frame)
            output(format_frame(frame))
    except (ValueError, can.CanError) as exc:
        output(f"error: {exc}")
    return True


class CanTerminalApp(App if App is not None else object):
    """Textual UI around the MCU command parser and CAN transport."""

    CSS = APP_CSS
    TITLE = "W.A.R.D. CAN Terminal"
    SUB_TITLE = "MCU command console"
    BINDINGS = [("ctrl+c", "quit", "Quit"), ("ctrl+l", "clear_log", "Clear log")]

    def __init__(self, bus, interface: str, channel: str, bitrate: int):
        super().__init__()
        self.bus = bus
        self.connection_text = f"Connected: {channel} · {interface} · {bitrate} bit/s"
        self.receiver_stop = threading.Event()

    def compose(self) -> ComposeResult:
        yield Header()
        with Container():
            yield Static(self.connection_text, id="connection")
            yield Static("Live MCU: waiting for status telemetry…", id="telemetry")
            yield Input(placeholder="Enter a CAN command, e.g. move x 250", id="command")
            with Grid(id="controls"):
                yield Button("Power on", id="power-on", variant="success")
                yield Button("Power off", id="power-off", variant="error", classes="danger")
                yield Button("Home Y", id="home-y")
                yield Button("Stop motion", id="stop-motion", variant="warning")
                yield Button("X −100", id="move-x-minus")
                yield Button("X +100", id="move-x-plus")
                yield Button("Y −100", id="move-y-minus")
                yield Button("Y +100", id="move-y-plus")
                yield Button("Servo 90°", id="servo-90")
                yield Button("Servo 100°", id="servo-100")
                yield Button("Servo 145°", id="servo-145")
            yield Static("Type help to list commands.", id="status")
            yield RichLog(highlight=True, markup=True, id="log")
        yield Footer()

    def on_mount(self) -> None:
        self.query_one("#command", Input).focus()
        self.query_one("#log", RichLog).write("[bold #f4b23f]W.A.R.D. MCU CAN terminal ready.[/]")
        self.query_one("#log", RichLog).write("Type [bold]help[/] to view supported commands.")
        threading.Thread(target=self.receive_frames, name="can-receiver", daemon=True).start()

    def receive_frames(self) -> None:
        """Receive CAN frames outside the UI thread and forward them to Textual."""
        while not self.receiver_stop.is_set():
            try:
                message = self.bus.recv(timeout=0.2)
            except can.CanError as exc:
                if not self.receiver_stop.is_set():
                    self.call_from_thread(self.show_receive_error, str(exc))
                return
            if message is not None:
                self.call_from_thread(self.display_received_frame, message)

    def show_receive_error(self, error: str) -> None:
        self.query_one("#status", Static).update(f"CAN receive error: {error}")

    def display_received_frame(self, message) -> None:
        if not message.is_extended_id and message.arbitration_id == CAN_ID_STATUS_TELEMETRY:
            try:
                self.query_one("#telemetry", Static).update(decode_status_telemetry(bytes(message.data)))
            except ValueError as exc:
                self.query_one("#status", Static).update(f"Telemetry error: {exc}")
            return

        payload = bytes(message.data).hex(" ").upper() or "(empty)"
        can_id = f"0x{message.arbitration_id:X}"
        self.query_one("#log", RichLog).write(f"[cyan]received {can_id}  [{payload}][/]")

    def on_input_submitted(self, event: Input.Submitted) -> None:
        line = event.value.strip()
        event.input.value = ""
        if not line:
            return
        self.process_command(line)

    def on_button_pressed(self, event: Button.Pressed) -> None:
        commands = {
            "power-on": "power on",
            "power-off": "power off",
            "home-y": "home y",
            "stop-motion": "speed 0 0",
            "move-x-minus": "move x -100",
            "move-x-plus": "move x 100",
            "move-y-minus": "move y -100",
            "move-y-plus": "move y 100",
            "servo-90": "servo 90",
            "servo-100": "servo 100",
            "servo-145": "servo 145",
        }
        self.process_command(commands[event.button.id])

    def process_command(self, line: str) -> None:
        log = self.query_one("#log", RichLog)
        status = self.query_one("#status", Static)
        log.write(f"[dim]>[/] {line}")
        if line.lower() in {"quit", "exit"}:
            self.exit()
            return
        if line.lower() in {"help", "?", "commands"}:
            log.write(HELP_TEXT.rstrip())
            status.update("Command reference displayed.")
            return
        try:
            frame = parse_command(line)
            if frame is not None:
                send_frame(self.bus, frame)
                message = format_frame(frame)
                log.write(f"[green]{message}[/]")
                status.update(message)
        except (ValueError, can.CanError) as exc:
            message = f"error: {exc}"
            log.write(f"[red]{message}[/]")
            status.update(message)

    def action_clear_log(self) -> None:
        self.query_one("#log", RichLog).clear()

    def on_unmount(self) -> None:
        self.receiver_stop.set()
        self.bus.shutdown()


def parse_args():
    parser = argparse.ArgumentParser(description="Interactive W.A.R.D. MCU CAN terminal")
    parser.add_argument("--interface", default="socketcan",
                        help="python-can interface, e.g. socketcan or pcan")
    parser.add_argument("--channel", default="can0",
                        help="CAN channel, e.g. can0 or PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=500000, help="CAN bitrate (default: 500000)")
    parser.add_argument("-c", "--command", help="Send one command and exit")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if can is None:
        print("python-can is required. Install with: pip3 install python-can", file=sys.stderr)
        return 1

    try:
        bus = can.Bus(interface=args.interface, channel=args.channel, bitrate=args.bitrate)
    except can.CanError as exc:
        print(f"Unable to open CAN bus {args.channel}: {exc}", file=sys.stderr)
        return 1

    if args.command:
        try:
            run_command(bus, args.command, print)
        finally:
            bus.shutdown()
        return 0

    if App is None:
        bus.shutdown()
        print("Textual is required for the interactive UI. Install with: pip3 install textual", file=sys.stderr)
        return 1

    CanTerminalApp(bus, args.interface, args.channel, args.bitrate).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
