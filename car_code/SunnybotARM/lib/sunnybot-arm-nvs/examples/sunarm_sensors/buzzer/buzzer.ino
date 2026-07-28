/*
 *
 程序名称：buzzer.ino
蜂鸣器测试
 */
#include <Arduino.h>
#include <U8g2lib.h>
// 蜂鸣器
int Buzzer_PIN = 19; // 管脚2连接到蜂鸣器元件的基极
void setup()
{
  Serial.begin(115200); ///初始化串口
    pinMode(Buzzer_PIN, OUTPUT);   // 设置pinBuzzer脚为输出状态
  digitalWrite(Buzzer_PIN, LOW); // 测试蜂鸣器
    digitalWrite(Buzzer_PIN, HIGH); // 测试蜂鸣器
  delay(100);
  digitalWrite(Buzzer_PIN, LOW);         // 测试蜂鸣器
  delay(100);           ///<延时等待初始化完成
}

/**
 * @brief 主函数
 *
 */

void loop()
{

}