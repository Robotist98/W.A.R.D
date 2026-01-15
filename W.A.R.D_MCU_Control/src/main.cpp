#include <Arduino.h>
#include "adafruit_telemetry.h"
#include "config.h"

Telemetry telemetry(SPI_CAN_CS_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_SCK_PIN);
UniversalPacker packer;
void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10);

  Serial.println("MCP2515 Sender test!");
  pinMode(13, OUTPUT);

  if (!telemetry.begin(CAN_BAUDRATE)) 
  {
    Serial.println("CAN BUS Shield init fail");
    while (1);
  }
  packer.clear();
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