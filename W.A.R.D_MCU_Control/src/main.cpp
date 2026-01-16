#include <Arduino.h>
#include "adafruit_telemetry.h"
#include "config.h"
#include "AccelStepper.h"
#include "Adafruit_PWMServoDriver.h"

Telemetry telemetry(SPI_CAN_CS_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_SCK_PIN);
UniversalPacker packer;

AccelStepper stepperX(AccelStepper::DRIVER, X_AXIS_STEP_PIN, X_AXIS_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_AXIS_STEP_PIN, Y_AXIS_DIR_PIN);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

constexpr long kStepDelta = 100;
bool xPositive = true;
bool yPositive = true;

static uint16_t servoPulseForAngle(uint8_t angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
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

  pwm.begin();
  pwm.setPWMFreq(50);
  pwm.setPWM(SERVO_TRIGGER_CHANNEL, 0, servoPulseForAngle(0));
  delay(500);
  pwm.setPWM(SERVO_TRIGGER_CHANNEL, 0, servoPulseForAngle(180));

  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
  stepperX.move(kStepDelta);
  stepperY.move(kStepDelta);
  Serial.println("Setup complete.");

}

void loop() 
{
  static unsigned long previousMillis = 0;
  const long interval = 1000; // 1 second
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    static bool ledState = LOW;
    ledState = !ledState;
    digitalWrite(13, ledState);
  }

  stepperX.run();
  stepperY.run();
  if (stepperX.distanceToGo() == 0) {
    xPositive = !xPositive;
    stepperX.move(xPositive ? kStepDelta : -kStepDelta);
  }
  if (stepperY.distanceToGo() == 0) {
    yPositive = !yPositive;
    stepperY.move(yPositive ? kStepDelta : -kStepDelta);
  }

   if(telemetry.receive()) 
   {
       Serial.print("Received packet with ID 0x");
       Serial.print(telemetry.getLastReceivedId(), HEX);
       Serial.print(" and data: ");
       size_t len = telemetry.getReceivedSize();
       for (size_t i = 0; i < len; i++) {
           uint8_t byte;
           if (telemetry.getUint8(i, byte)) {
               Serial.print("0x");
               Serial.print(byte, HEX);
               Serial.print(" ");
           }
       }
       Serial.println();
   }
}
