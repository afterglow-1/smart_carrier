//
//    FILE: PCF8575_test.ino
//  AUTHOR: Rob Tillaart
//    DATE: 2020-07-20
// PUPROSE: test PCF8575 library
//     URL: https://github.com/RobTillaart/PCF8575


#include "PCF8575.h"

PCF8575 PCF(0x20);


void setup()
{
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("PCF8575_test version: ");
  Serial.println(PCF8575_LIB_VERSION);

  PCF.begin();

  uint16_t x = PCF.read16();
  Serial.print("Read ");
  Serial.println(x,HEX);
  delay(1000);
}


void loop()
{
    
    PCF.write16(0x5555);
    delay(10);
    uint16_t x = PCF.read16();
    Serial.print("Read ");
    Serial.println(x,HEX);
    delay(1000);
    PCF.write16(0xAAAA);
    delay(10);
    uint16_t y = PCF.read16();
    Serial.print("Read ");
    Serial.println(y,HEX);
    delay(1000);
 
}

// -- END OF FILE --
