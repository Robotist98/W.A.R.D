#include <Arduino.h>
#include "adafruit_telemetry.h"
#include "config.h"
#include "AccelStepper.h"
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_AS5600.h>

Telemetry telemetry(SPI_CAN_CS_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_SCK_PIN);
UniversalPacker packer;

AccelStepper stepperX(AccelStepper::DRIVER, X_AXIS_STEP_PIN, X_AXIS_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_AXIS_STEP_PIN, Y_AXIS_DIR_PIN);
Servo triggerServo;
Adafruit_AS5600 as5600;
bool as5600Available = false;
float as5600Offset = 29.0f;

constexpr long kStepDelta = 100;
constexpr uint32_t kAs5600ReadIntervalMs = 100;

constexpr uint8_t kCmdSetPower = 0x10;
constexpr uint8_t kCmdMoveX = 0x11;
constexpr uint8_t kCmdMoveY = 0x12;
constexpr uint8_t kCmdSetServo = 0x13;
constexpr uint8_t kCmdSetSpeed = 0x14;
constexpr uint8_t kCmdSetAccel = 0x15;
constexpr uint8_t kCmdMoveYToZero = 0x16;
constexpr uint8_t kServoMinAngle = 0x5A;  // 90 degrees
constexpr uint8_t kServoMaxAngle = 0x91;  // 145 degrees
constexpr float kXHomeSpeed = 50.0f;
constexpr float kXHomeToleranceDeg = 1.0f;

int16_t xSpeed = 0;
int16_t ySpeed = 0;
bool speedMode = false;
int16_t xAccel = 300;
int16_t yAccel = 250;
uint32_t lastAs5600ReadMs = 0;
float as5600Angle = 0.0f;
bool as5600AngleValid = false;
bool homingYToZero = false;

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
  if (!as5600.begin()) {
    Serial.println("AS5600 not found.");
  } else {
    as5600Available = true;
    Serial.println("AS5600 ready.");
  }

  if (!telemetry.begin(CAN_BAUDRATE)) 
  {
    Serial.println("CAN BUS Shield init fail");
    while (1);
  }
  packer.clear();

  digitalWrite(MAIN_POWER_PIN, LOW);
  delay(100);

  stepperX.setMaxSpeed(1000);     // Max steps/sec
  stepperX.setAcceleration(xAccel);  // Smoother movement

  // Configure Y-axis motor
  stepperY.setMaxSpeed(800);      // Less speed if lower microstepping
  stepperY.setAcceleration(yAccel);

  triggerServo.attach(SERVO_TRIGGER_PIN);
  triggerServo.write(100);
  // delay(500);
  // triggerServo.write(180);

  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
  Serial.println("Setup complete.");

}

void loop() 
{
  const uint32_t nowMs = millis();

  if (speedMode) {
    if (!homingYToZero) {
      stepperX.setSpeed(static_cast<float>(xSpeed));
      stepperX.runSpeed();
    }
    stepperY.setSpeed(static_cast<float>(ySpeed));
    stepperY.runSpeed();
  } else {
    if (!homingYToZero) {
      stepperX.run();
    }
    stepperY.run();
  }

  if (as5600Available && (nowMs - lastAs5600ReadMs >= kAs5600ReadIntervalMs)) {
    lastAs5600ReadMs = nowMs;
    if (!as5600.isMagnetDetected()) {
      as5600AngleValid = false;
      Serial.println("AS5600 magnet not detected");
    } else {
      const float rawAngle = (as5600.getRawAngle() * 360.0f) / 4096.0f;
      float angle = -(rawAngle - as5600Offset);
      if (angle > 180.0f) {
        angle -= 360.0f;
      } else if (angle <= -180.0f) {
        angle += 360.0f;
      }
      as5600Angle = angle;
      as5600AngleValid = true;
      Serial.print("AS5600 angle: ");
      Serial.println(angle, 2);
    }
  }
  if (homingYToZero && as5600AngleValid) {
    Serial.println("Homing X to zero...");
    const float absAngle = (as5600Angle < 0.0f) ? -as5600Angle : as5600Angle;
    if (absAngle <= kXHomeToleranceDeg) {
      homingYToZero = false;
      stepperY.setSpeed(0.0f);
    } else {
      const float direction = (as5600Angle > 0.0f) ? -1.0f : 1.0f;
      stepperY.setSpeed(direction * kXHomeSpeed);
    }
    stepperY.runSpeed();
  }

  if (telemetry.receive()) {
    const uint32_t cmd = telemetry.getLastReceivedId();
    switch (cmd) {
      case CAN_CMD_PTM_STOP: {
        stepperX.stop();
        stepperY.stop();
        stepperX.moveTo(stepperX.currentPosition());
        stepperY.moveTo(stepperY.currentPosition());
        xSpeed = 0;
        ySpeed = 0;
        speedMode = true;
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
        stepperX.move(delta);
        stepperY.move(delta);
        speedMode = false;
        break;
      }
      case kCmdSetAccel: {
        int16_t newX = 0;
        int16_t newY = 0;
        if (readInt16(0, newX) && readInt16(2, newY)) {
          if (newX < 0) {
            newX = static_cast<int16_t>(-newX);
          }
          if (newY < 0) {
            newY = static_cast<int16_t>(-newY);
          }
          xAccel = newX;
          yAccel = newY;
          stepperX.setAcceleration(static_cast<float>(xAccel));
          stepperY.setAcceleration(static_cast<float>(yAccel));
        }
        break;
      }
      case kCmdSetPower: {
        uint8_t state = 0;
        if (telemetry.getUint8(0, state)) {
          digitalWrite(MAIN_POWER_PIN, state ? HIGH : LOW);
        }
        break;
      }
      case kCmdMoveX: {
        int16_t delta = 0;
        if (readInt16(0, delta)) {
          stepperX.move(delta);
          speedMode = false;
        }
        break;
      }
      case kCmdMoveYToZero: {
        homingYToZero = true;
        speedMode = true;
        break;
      }
      case kCmdMoveY: {
        int16_t delta = 0;
        if (readInt16(0, delta)) {
          stepperY.move(delta);
          speedMode = false;
        }
        break;
      }
      case kCmdSetSpeed: {
        int16_t newX = 0;
        int16_t newY = 0;
        if (readInt16(0, newX) && readInt16(2, newY)) {
          xSpeed = newX;
          ySpeed = newY;
          speedMode = true;
        }
        break;
      }
      case kCmdSetServo: {
        uint8_t angle = 0;
        if (telemetry.getUint8(0, angle)) {
          if (angle < kServoMinAngle) {
            angle = kServoMinAngle;
          } else if (angle > kServoMaxAngle) {
            angle = kServoMaxAngle;
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
