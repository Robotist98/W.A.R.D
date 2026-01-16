#include <Arduino.h>
#include "adafruit_telemetry.h"
#include "config.h"
#include "AccelStepper.h"
#include <Servo.h>

Telemetry telemetry(SPI_CAN_CS_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_SCK_PIN);
UniversalPacker packer;

AccelStepper stepperX(AccelStepper::DRIVER, X_AXIS_STEP_PIN, X_AXIS_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_AXIS_STEP_PIN, Y_AXIS_DIR_PIN);
Servo triggerServo;

constexpr long kStepDelta = 100;

constexpr uint8_t kCmdSetPower = 0x10;
constexpr uint8_t kCmdMoveX = 0x11;
constexpr uint8_t kCmdMoveY = 0x12;
constexpr uint8_t kCmdSetServo = 0x13;

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
  while(!Serial) delay(10);

  Serial.println("MCP2515 Sender test!");
  pinMode(13, OUTPUT);
  pinMode(MAIN_POWER_PIN, OUTPUT);

  if (!telemetry.begin(CAN_BAUDRATE)) 
  {
    Serial.println("CAN BUS Shield init fail");
    while (1);
  }
  packer.clear();

  digitalWrite(MAIN_POWER_PIN, LOW);
  delay(100);

  stepperX.setMaxSpeed(1000);     // Max steps/sec
  stepperX.setAcceleration(300);  // Smoother movement

  // Configure Y-axis motor
  stepperY.setMaxSpeed(800);      // Less speed if lower microstepping
  stepperY.setAcceleration(250);

  triggerServo.attach(SERVO_TRIGGER_PIN);
  triggerServo.write(0);
  delay(500);
  triggerServo.write(180);

  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
  Serial.println("Setup complete.");

}

void loop() 
{
  stepperX.run();
  stepperY.run();

  if (telemetry.receive()) {
    const uint32_t cmd = telemetry.getLastReceivedId();
    switch (cmd) {
      case CAN_CMD_PTM_STOP: {
        stepperX.stop();
        stepperY.stop();
        stepperX.moveTo(stepperX.currentPosition());
        stepperY.moveTo(stepperY.currentPosition());
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
        }
        break;
      }
      case kCmdMoveY: {
        int16_t delta = 0;
        if (readInt16(0, delta)) {
          stepperY.move(delta);
        }
        break;
      }
      case kCmdSetServo: {
        uint8_t angle = 0;
        if (telemetry.getUint8(0, angle)) {
          if (angle > 180) {
            angle = 180;
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
