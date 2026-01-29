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

constexpr long kStepDelta = 100;
constexpr int16_t kXAccel = 300;
constexpr int16_t kYAccel = 250;
constexpr float kAs5600Offset = 29.0f;
constexpr float kXMaxSpeed = 1000.0f;
constexpr float kYMaxSpeed = 800.0f;

bool readInt16(size_t offset, int16_t &value) {
  uint16_t raw = 0;
  if (!telemetry.getUint16(offset, raw)) {
    return false;
  }
  value = static_cast<int16_t>(raw);
  return true;
}

void setup() {
  Serial.begin(115200);
  //while(!Serial) delay(10);

  Serial.println("MCP2515 Sender test!");
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

  Serial.println("Setup complete.");

}

void loop() 
{
  const uint32_t nowMs = millis();

  const bool homingYToZero = axisY.isHoming();
  if (!homingYToZero) {
    axisX.update(nowMs);
  }
  axisY.update(nowMs);

  if (telemetry.receive()) {
    const uint32_t cmd = telemetry.getLastReceivedId();
    switch (cmd) {
      case CAN_CMD_STOP: {
        axisX.stopAndHold();
        axisY.stopAndHold();
        axisX.setMode(AxisControl::Mode::Speed);
        axisY.setMode(AxisControl::Mode::Speed);
        digitalWrite(MAIN_POWER_PIN, LOW);
        break;
      }
      case CAN_CMD_PTM_START_FW:
      case CAN_CMD_PTM_START_REV: {
        digitalWrite(MAIN_POWER_PIN, HIGH);
        int16_t delta = static_cast<int16_t>(kStepDelta);
        if (!readInt16(0, delta)) {
          delta = static_cast<int16_t>(kStepDelta);
        }
        if (cmd == CAN_CMD_PTM_START_REV) {
          delta = static_cast<int16_t>(-delta);
        }
        axisX.move(delta);
        axisY.move(delta);
        axisX.setMode(AxisControl::Mode::Position);
        axisY.setMode(AxisControl::Mode::Position);
        break;
      }
      case CAN_ID_SET_ACCEL: {
        int16_t newX = 0;
        int16_t newY = 0;
        if (readInt16(0, newX) && readInt16(2, newY)) {
          axisX.setAcceleration(newX);
          axisY.setAcceleration(newY);
        }
        break;
      }
      case CAN_ID_MAINPOWER: {
        uint8_t state = 0;
        if (telemetry.getUint8(0, state)) {
          digitalWrite(MAIN_POWER_PIN, state ? HIGH : LOW);
        }
        break;
      }
      case CAN_ID_MOVE_X: {
        int16_t delta = 0;
        if (readInt16(0, delta)) {
          axisX.move(delta);
          axisX.setMode(AxisControl::Mode::Position);
          axisY.setMode(AxisControl::Mode::Position);
        }
        break;
      }
      case CAN_ID_MOVE_Y_TO_ZERO: {
        axisY.startHomingToZero();
        axisX.setMode(AxisControl::Mode::Speed);
        axisY.setMode(AxisControl::Mode::Homing);
        break;
      }
      case CAN_ID_MOVE_Y: {
        int16_t delta = 0;
        if (readInt16(0, delta)) {
          axisY.move(delta);
          axisX.setMode(AxisControl::Mode::Position);
          axisY.setMode(AxisControl::Mode::Position);
        }
        break;
      }
      case CAN_ID_SET_SPEED: {
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
      default:
        break;
    }
  }
}
