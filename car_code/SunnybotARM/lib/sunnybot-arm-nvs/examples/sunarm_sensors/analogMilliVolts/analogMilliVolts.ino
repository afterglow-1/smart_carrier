// https://docs.espressif.com/projects/arduino-esp32/en/latest/api/adc.html?highlight=adc
#include <Arduino.h>
#include <U8g2lib.h>
// oled
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/22, /* data=*/21); // ESP32 Thing, HW I2C with pin remapping
float voltage_k = 0.99;//电压修正系数
void setup()
{
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  // set the resolution to 12 bits (0-4095)
  analogReadResolution(12);
  analogSetPinAttenuation(A0, ADC_2_5db); //可测量的输入电压范围100 mV ~ 1250 mV
  analogSetPinAttenuation(A3, ADC_2_5db);  //可测量的输入电压范围100 mV ~ 1250 mV
  //屏幕初始化
  u8g2.begin();
  u8g2.enableUTF8Print(); // enable UTF8 support for the Arduino print() function
}

void loop()
{
  // read the analog / millivolts毫伏 value for pin A0 A3:
  float analogA0Value = float(analogRead(A0)) * 3.3 * 11.0 / 4096;
  float analogA0Volts = float(analogReadMilliVolts(A0)) * 11.0 * voltage_k / 1000.0;
  float analogA3Volts = float(analogReadMilliVolts(A3)) * 11.0 * voltage_k/ 1000.0;
  Serial.println(analogA0Value);
  Serial.println(analogA0Volts);
  // print out the values you read:
  Serial.printf("ADC analog value = %d\n", analogRead(A0));
  Serial.printf("ADC millivolts value = %d\n", analogReadMilliVolts(A0));

  u8g2.setFont(u8g2_font_unifont_t_chinese2); // use chinese2
  u8g2.firstPage();
  do
  {
    u8g2.setCursor(0, 20);
    u8g2.print("电池V:");
    u8g2.print(analogA0Volts);
    u8g2.setCursor(0, 40);
    u8g2.print("SERVO V:");
    u8g2.print(analogA3Volts);

  } while (u8g2.nextPage());

  delay(1000); // delay in between reads for clear read from serial
}