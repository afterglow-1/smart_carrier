/*
 * @Author: igcxl acer5502@gmail.com
 * @Date: 2022-07-07 22:07:34
 * @LastEditors: igcxl acer5502@gmail.com
 * @LastEditTime: 2024-07-15 15:28:29
 * @FilePath: \RPGripperf:\PIO\SunnybotARM\src\main.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

//程序名称：ESP32_OpenMV_Serial1
//程序功能：arduino 串口1 和openmv串口3通讯
//使用方法：
// ESP32 硬串口1接收 串口1和串口0同时输出
//将openmv.py 烧入OPENMV

/*openmv.py
import time
from pyb import UART

# UART 3, and baudrate.
uart = UART(3, 115200)

while(True):
    uart.write("Hello ESP32 SunnybotArm!\n")
    if (uart.any()):
        print(uart.read())
    time.sleep_ms(1000)

*/

//程序名称：ESP32_OpenMV_Serial1
//程序功能：arduino 串口1 和openmv串口3通讯
//使用方法：
// ESP32 硬串口1接收 串口1和串口0同时输出
//将openmv.py 烧入OPENMV

/*openmv.py
import time
from pyb import UART

# UART 3, and baudrate.
uart = UART(3, 115200)

while(True):
    uart.write("Hello ESP32 SunnybotArm!\n")
    if (uart.any()):
        print(uart.read())
    time.sleep_ms(1000)

*/

#include <Arduino.h>
#include <U8g2lib.h>
//#include <ESPAsyncWebServer.h>


#define USE_POWER_ARM_BORAD//电源和机械臂主控二合一板


//M2DOCK 扩展板V2,需要交叉官方引脚标识错了
#ifdef USE_POWER_ARM_BORAD
#define TXD1 14//27
#define RXD1 27//14
#endif
void setup()
{
    //Serial1.begin(115200, SERIAL_8N1, 32, 33);//独立版
    //            baud    config      rx  tx
    Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);//二合一版
    Serial.begin(115200);
    Serial.print("uart1 test: ");
}

void loop()
{
    if (Serial1.available())
    {
        // Read the most recent byte
        byte byteRead = Serial1.read();
        // char charRead =Serial1.read();
        // ECHO the value that was read

        Serial1.write(byteRead);
        Serial.write(byteRead);
        // Serial.println(byteRead);
    }
}