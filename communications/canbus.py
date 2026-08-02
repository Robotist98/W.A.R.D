import struct
import time
from dataclasses import dataclass
from typing import Optional, Tuple


ADDRESS_FIRMWARE_VERSION = 0x600
REPLY_FIRMWARE_VERSION = 0x601

CAN_ID_MAINPOWER = 0x10
CAN_CMD_MAINPOWER_OFF = 0x00
CAN_CMD_MAINPOWER_ON = 0x01

CAN_ID_RUNMOVE_X = 0x11
CAN_ID_RUNMOVE_Y = 0x12
CAN_ID_SERVO_TRIGGER = 0x13
CAN_ID_RUNSPEED_XY = 0x14
CAN_ID_SET_CONFIG_X = 0x15
CAN_ID_SET_CONFIG_Y = 0x16
CAN_ID_MOVE_Y_TO_ZERO = 0x17

CAN_COM_SET_SPEED = 1
CAN_COM_SET_ACCEL = 2

CAN_BAUDRATE = 500000

MIN_SERVO_ANGLE = 90
MAX_SERVO_ANGLE = 145
SERVO_FIRE_FORWARD = MAX_SERVO_ANGLE
SERVO_FIRE_RETURN = MIN_SERVO_ANGLE

INT16_MIN = -32768
INT16_MAX = 32767


@dataclass(frozen=True)
class CanBusConfig:
    interface: str = "socketcan"
    channel: str = "can0"
    bitrate: int = CAN_BAUDRATE
    timeout: float = 1.0


def clamp(value: int, minimum: int, maximum: int) -> int:
    return max(minimum, min(int(value), maximum))


def pack_int16(value: int) -> bytes:
    return struct.pack(">h", clamp(value, INT16_MIN, INT16_MAX))


def pack_int16_pair(first: int, second: int) -> bytes:
    return struct.pack(
        ">hh",
        clamp(first, INT16_MIN, INT16_MAX),
        clamp(second, INT16_MIN, INT16_MAX),
    )


def pack_axis_config(command: int, value: int) -> bytes:
    return bytes([command & 0xFF]) + pack_int16(value)


class WardCanBus:
    """
    Python CAN client for the W.A.R.D MCU controller.

    The frame IDs and payload format mirror W.A.R.D_MCU_Control/include/config.h
    and the parser in W.A.R.D_MCU_Control/src/main.cpp.
    """

    def __init__(
        self,
        config: Optional[CanBusConfig] = None,
        bus=None,
    ):
        self.config = config or CanBusConfig()
        self.bus = bus
        self._owns_bus = bus is None

        if self.bus is None:
            import can

            self.bus = can.Bus(
                interface=self.config.interface,
                channel=self.config.channel,
                bitrate=self.config.bitrate,
            )

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def close(self) -> None:
        if self._owns_bus and self.bus is not None:
            shutdown = getattr(self.bus, "shutdown", None)
            if shutdown is not None:
                shutdown()

    def send(self, arbitration_id: int, payload: bytes = b"") -> None:
        if len(payload) > 8:
            raise ValueError("CAN payloads cannot be longer than 8 bytes")

        import can

        message = can.Message(
            arbitration_id=arbitration_id,
            is_extended_id=False,
            data=payload,
        )
        self.bus.send(message)

    def set_power(self, enabled: bool) -> None:
        command = CAN_CMD_MAINPOWER_ON if enabled else CAN_CMD_MAINPOWER_OFF
        self.send(CAN_ID_MAINPOWER, bytes([command]))

    def power_on(self) -> None:
        self.set_power(True)

    def power_off(self) -> None:
        self.set_power(False)

    def move_x(self, delta: int) -> None:
        self.send(CAN_ID_RUNMOVE_X, pack_int16(delta))

    def move_y(self, delta: int) -> None:
        self.send(CAN_ID_RUNMOVE_Y, pack_int16(delta))

    def move_y_to_zero(self) -> None:
        self.send(CAN_ID_MOVE_Y_TO_ZERO)

    def set_speed(self, x_speed: int, y_speed: int) -> None:
        self.send(CAN_ID_RUNSPEED_XY, pack_int16_pair(x_speed, y_speed))

    def stop(self) -> None:
        self.set_speed(0, 0)

    def set_x_speed_mode_speed(self, speed: int) -> None:
        self.send(
            CAN_ID_SET_CONFIG_X,
            pack_axis_config(CAN_COM_SET_SPEED, speed),
        )

    def set_y_speed_mode_speed(self, speed: int) -> None:
        self.send(
            CAN_ID_SET_CONFIG_Y,
            pack_axis_config(CAN_COM_SET_SPEED, speed),
        )

    def set_speed_mode_speeds(self, x_speed: int, y_speed: int) -> None:
        self.set_x_speed_mode_speed(x_speed)
        self.set_y_speed_mode_speed(y_speed)

    def set_x_acceleration(self, acceleration: int) -> None:
        self.send(
            CAN_ID_SET_CONFIG_X,
            pack_axis_config(CAN_COM_SET_ACCEL, acceleration),
        )

    def set_y_acceleration(self, acceleration: int) -> None:
        self.send(
            CAN_ID_SET_CONFIG_Y,
            pack_axis_config(CAN_COM_SET_ACCEL, acceleration),
        )

    def set_accelerations(self, x_acceleration: int, y_acceleration: int) -> None:
        self.set_x_acceleration(x_acceleration)
        self.set_y_acceleration(y_acceleration)

    def set_servo_angle(self, angle: int) -> None:
        clamped_angle = clamp(angle, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE)
        self.send(CAN_ID_SERVO_TRIGGER, bytes([clamped_angle]))

    def fire(self, delay_seconds: float = 0.3) -> None:
        self.set_servo_angle(SERVO_FIRE_FORWARD)
        time.sleep(delay_seconds)
        self.set_servo_angle(SERVO_FIRE_RETURN)

    def request_firmware_version(self) -> Optional[Tuple[int, int, int]]:
        self.send(ADDRESS_FIRMWARE_VERSION)

        deadline = time.monotonic() + self.config.timeout

        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            message = self.bus.recv(remaining)
            if message is None:
                return None

            if (
                not message.is_extended_id
                and message.arbitration_id == REPLY_FIRMWARE_VERSION
                and len(message.data) >= 3
            ):
                return (
                    int(message.data[0]),
                    int(message.data[1]),
                    int(message.data[2]),
                )

        return None
