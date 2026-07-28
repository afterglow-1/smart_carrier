/*
 *
 程序名称：analogVolts
 电池电压检测和舵机电压检测
 ESP32 内置了 2 个 12 位的逐次逼近数字模拟转换器，由 5 个专用转换器控制器管理，可测量来自 18 个管脚的 模拟信号。
 Wi-Fi 驱动程序使用了 ADC2。因此，应用程序只能在未启动 Wi-Fi 驱动程序时使用 ADC2。
 用ESP32读取模拟值意味着你可以测量0 V到3.3 V之间的变化电压等级。然后将测量的电压赋给0到4095之间的一个值，其中0 V对应0,3.3 V对应4095。0 V到3.3 V之间的任何电压都将给出两者之间的相应值。
 ADC是非线性的值
 */
#include <Arduino.h>
#include <U8g2lib.h>
void setup()
{
  Serial.begin(115200); ///初始化串口
  delay(100);           ///<延时等待初始化完成
}

/**
 * @brief 主函数
 *
 */
//float cali_k_a0=1.118;
//float cali_k_a3=1.2;
float cali_k_a0=1.08;
float cali_k_a3=1.133;
void loop()
{
  Serial.println(millis());
  Serial.print("Battery Voltage:");
  Serial.print(analogRead(A0));
  Serial.print(",");
  Serial.println(analogRead(A0) * 3.3 * 11*cali_k_a0 / 4095);
  delay(1000);
  Serial.print("SERVO Voltage:");
  Serial.print(analogRead(A3));
  Serial.print(",");
  Serial.println(analogRead(A3) * 3.3 * 11 *cali_k_a3/ 4095);

  delay(1000);
}