#include <Arduino.h>
#include "adafruit_telemetry.h"
#include "config.h"
#include "axis_control.h"
#include <Servo.h>
#include <Wire.h>

Telemetry telemetry(SPI_CAN_CS_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_SCK_PIN);
UniversalPacker packer;

AxisControl axisX(X_AXIS_STEP_PIN, X_AXIS_DIR_PIN);
AxisControl axisY(Y_AXIS_STEP_PIN, Y_AXIS_DIR_PIN, true);
Servo triggerServo;

constexpr int16_t kXAccel = 300;
constexpr int16_t kYAccel = 250;
constexpr float kAs5600Offset = 29.0f;
constexpr float kXMaxSpeed = 1000.0f;
constexpr float kYMaxSpeed = 800.0f;
constexpr uint32_t kCanResetCooldownMs = 500;

static void resetCanBus(uint32_t nowMs) {
  static uint32_t lastResetMs = 0;
  if (nowMs - lastResetMs < kCanResetCooldownMs) {
    return;
  }
  lastResetMs = nowMs;

  Serial.println("Resetting CAN bus after RX overflow");
  telemetry.end();
  delay(5);
  telemetry.begin(CAN_BAUDRATE);
}

bool readInt16(size_t offset, int16_t &value) {
  uint16_t raw = 0;
  if (!telemetry.getUint16(offset, raw)) {
    return false;
  }
  value = static_cast<int16_t>(raw);
  return true;
}

bool readConfigPayload(uint8_t &cmd, int16_t &value) {
  if (!telemetry.getUint8(0, cmd)) {
    return false;
  }
  return readInt16(1, value) || readInt16(2, value);
}

void applyAxisConfig(AxisControl &axis, uint8_t cmd, int16_t value) {
  if (cmd == CAN_COM_READ_SPEED) {
    axis.setSpeedModeSpeed(value);
  } else if (cmd == CAN_COM_READ_ACCEL) {
    axis.setAcceleration(value);
  }
}
void setup() {
  Serial.begin(115200);
  //while(!Serial) delay(10);

  pinMode(13, OUTPUT);
  pinMode(MAIN_POWER_PIN, OUTPUT);

  Wire.begin();
  axisX.begin();
  axisY.begin(kAs5600Offset);

  if (!telemetry.begin(CAN_BAUDRATE)) 
  {
    Serial.println("CAN BUS Shield init fail");
    while (1);
  }
  packer.clear();

  digitalWrite(MAIN_POWER_PIN, LOW);
  delay(100);

  axisX.configure(kXMaxSpeed, kXAccel, 0);
  axisY.configure(kYMaxSpeed, kYAccel, 0);

  triggerServo.attach(SERVO_TRIGGER_PIN);
  triggerServo.write(100);
  // delay(500);
  // triggerServo.write(180);

}

void loop() 
{
  const uint32_t nowMs = millis();
  bool rx0Overflow = false;
  bool rx1Overflow = false;
  if (telemetry.readRxOverflowFlags(rx0Overflow, rx1Overflow)) {
    Serial.print("CAN RX overflow: ");
    Serial.print(rx0Overflow ? "RX0 " : "");
    Serial.println(rx1Overflow ? "RX1" : "");
    resetCanBus(nowMs);
  }

  const bool homingYToZero = axisY.isHoming();
  if (!homingYToZero) {
    axisX.update(nowMs);
  }
  axisY.update(nowMs);

  if (telemetry.receive()) {
    const uint32_t cmd = telemetry.getLastReceivedId();
    Serial.print("Received CAN ID: 0x");
    Serial.println(cmd, HEX);
    switch (cmd) {
      case CAN_ID_SET_CONFIG_X: {
        uint8_t configCmd = 0;
        int16_t value = 0;
        if (readConfigPayload(configCmd, value)) {
          applyAxisConfig(axisX, configCmd, value);
        }
        break;
      }
      case CAN_ID_SET_CONFIG_Y: {
        uint8_t configCmd = 0;
        int16_t value = 0;
        if (readConfigPayload(configCmd, value)) {
          applyAxisConfig(axisY, configCmd, value);
        }
        break;
      }
      case CAN_ID_RUNMOVE_X: {
        int16_t delta = 0;
        if (readInt16(0, delta)) {
          axisX.move(delta);
          axisX.setMode(AxisControl::Mode::Position);
          axisY.setMode(AxisControl::Mode::Position);
        }
        break;
      }
      case CAN_ID_MOVE_Y_TO_ZERO: {
        if (axisY.startHomingToZero()) {
          axisX.setMode(AxisControl::Mode::Speed);
          axisY.setMode(AxisControl::Mode::Homing);
        }
        break;
      }
      case CAN_ID_RUNMOVE_Y: {
        int16_t delta = 0;
        if (readInt16(0, delta)) {
          axisY.move(delta);
          axisX.setMode(AxisControl::Mode::Position);
          axisY.setMode(AxisControl::Mode::Position);
        }
        break;
      }
      case CAN_ID_RUNSPEED_XY: {
        int16_t newX = 0;
        int16_t newY = 0;
        if (readInt16(0, newX) && readInt16(2, newY)) {
          axisX.setSpeedModeSpeed(newX);
          axisY.setSpeedModeSpeed(newY);
          axisX.setMode(AxisControl::Mode::Speed);
          axisY.setMode(AxisControl::Mode::Speed);
        }
        break;
      }
      case CAN_ID_SERVO_TRIGGER: {
        uint8_t angle = 0;
        if (telemetry.getUint8(0, angle)) {
          if (angle < MIN_SERVO_ANGLE) {
            angle = MIN_SERVO_ANGLE;
          } else if (angle > MAX_SERVO_ANGLE) {
            angle = MAX_SERVO_ANGLE;
          }
          triggerServo.write(angle);
        }
        break;
      }
      case ADDRESS_FIRMWARE_VERSION: {
        packer.clear();
        packer.addUint8(static_cast<uint8_t>(FIRMWARE_VERSION_MAJOR));
        packer.addUint8(static_cast<uint8_t>(FIRMWARE_VERSION_MINOR));
        packer.addUint8(static_cast<uint8_t>(FIRMWARE_VERSION_PATCH));
        telemetry.send(REPLY_FIRMWARE_VERSION, packer);
        break;
      }
      default:
        break;
    }
  }
}
