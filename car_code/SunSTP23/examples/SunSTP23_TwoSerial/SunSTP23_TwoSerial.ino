#include "SunSTP23.h"

// ESP32
#define TXD2 17
#define RXD2 16
#define TXD1 27
#define RXD1 14
//#define TXD1 32
//#define RXD1 23
//ESP32  S3
//#define TXD2 20
//#define RXD2 19
//#define TXD1 1
//#define RXD1 45
unsigned long printTime, lastPrint;
uint16_t last_distance1 = 0;
uint16_t last_distance2 = 0;
STP23 STP23_uart1(&Serial1, RXD1, TXD1, 921600);
STP23 STP23_uart2(&Serial2, RXD2, TXD2, 921600);

void setup()
{
  Serial.begin(115200);

  Serial.println("STP23 two uart average_distance");
  lastPrint = millis();
}

void loop()
{
  while (Serial1.available())
  {
    STP23_uart1.readData(Serial1.read()); // Call
  }
  while (Serial2.available())
  {
    STP23_uart2.readData(Serial2.read()); // Call
  }

  // if ((millis()-lastPrint>200)&&(last_distance!=STP23_uart2.average_distance))
  if ((millis()-lastPrint>=100)&&((last_distance1 != STP23_uart1.average_distance) || (last_distance2 != STP23_uart2.average_distance)))
  {
    lastPrint = millis();
    last_distance1 = STP23_uart1.average_distance;
    last_distance2 = STP23_uart2.average_distance;
   
    if (STP23_uart1.average_distance != 0)
    {
      Serial.print("x:");
      Serial.print(STP23_uart1.average_distance);
      Serial.print("mm,");
    }
    if (STP23_uart2.average_distance != 0)
    {
      Serial.print("y:");
      Serial.print(STP23_uart2.average_distance);
      Serial.print("mm;");
    }
     Serial.println(lastPrint);
  }
}
