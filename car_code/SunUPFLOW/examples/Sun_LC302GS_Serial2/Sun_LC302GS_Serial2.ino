#include "SunUPFLOW.h"

// ESP32
#define TXD2 17
#define RXD2 16
// #define TXD1 32
// #define RXD1 33//23
// ESP32  S3
// #define TXD2 20
// #define RXD2 19
// #define TXD1 1
// #define RXD1 45
unsigned long printTime, lastPrint;
uint16_t last_x = 0;
UPFLOW LC302GS_uart2(&Serial2, RXD2, TXD2, 115200);

void setup()
{
  Serial.begin(115200);

  Serial.println("LC302GS_uart2");
  lastPrint = millis();
}

void loop()
{
  while (Serial2.available())
  {
    LC302GS_uart2.readData(Serial2.read()); // Call
  }
  if ((millis() - lastPrint > 200))
  // if ((last_x != LC302GS_uart2.packData.flow_x_integral))
  {
    lastPrint = millis();
    last_x = LC302GS_uart2.packData.flow_x_integral;
    Serial.println(millis());
    Serial.print("X:");
    Serial.print(LC302GS_uart2.packData.flow_x_integral);
    Serial.print("X_OFFSET:");
    Serial.println(LC302GS_uart2.x_Offset);
    Serial.print("Y:");
    Serial.print(LC302GS_uart2.packData.flow_y_integral);
    Serial.print("Y_OFFSET:");
    Serial.println(LC302GS_uart2.y_Offset);
    //Serial.print("T:");
    //Serial.println(LC302GS_uart2.packData.integration_timespan);
  }
}
