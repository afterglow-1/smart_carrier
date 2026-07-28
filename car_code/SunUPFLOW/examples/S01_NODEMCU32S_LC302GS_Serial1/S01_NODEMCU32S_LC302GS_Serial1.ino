#include "SunUPFLOW.h"

// ESP32
//#define TXD1 32
//#define RXD1 23
#define TXD1 27
#define RXD1 14
// #define TXD1 32
// #define RXD1 33//23
// ESP32  S3
// #define TXD2 20
// #define RXD2 19
//#define TXD1 1
//#define RXD1 45
unsigned long printTime, lastPrint;
uint16_t last_x = 0;
UPFLOW LC302GS_uart1(&Serial1, RXD1, TXD1, 460800);

void setup()
{
  Serial.begin(115200);

  Serial.println("LC302GS_uart1");
  lastPrint = millis();
}

void loop()
{
  while (Serial1.available())
  {
    LC302GS_uart1.readData(Serial1.read()); // Call
  }
  if ((millis() - lastPrint > 200))
  // if ((last_x != LC302GS_uart1.packData.flow_x_integral))
  {
    lastPrint = millis();
    last_x = LC302GS_uart1.packData.flow_x_integral;
    Serial.println(millis());
    Serial.print("X:");
    Serial.print(LC302GS_uart1.packData.flow_x_integral);//采样间隔时间移动量
    Serial.print("X_OFFSET:");
    Serial.println(LC302GS_uart1.x_Offset);
    Serial.print("Y:");
    Serial.print(LC302GS_uart1.packData.flow_y_integral);
    Serial.print("Y_OFFSET:");
    Serial.println(LC302GS_uart1.y_Offset);
    Serial.print("T:");
    Serial.println(LC302GS_uart1.packData.integration_timespan);
  }
}
