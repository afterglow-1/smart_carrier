

//程序名称：ESP32_OpenMV_Serial1_JSON
//程序功能：arduino 串口1 和openmv串口3通讯
//使用方法：
// ESP32 硬串口1接收 串口1和串口0同时输出

/*openmv.py

import json
import time
from pyb import UART
#python字典类型
obj = {
    "qrCode":"123-321",
    "color" :[1,2,3]
}

# UART 3, and baudrate.
uart = UART(3, 115200)

while(True):
    output_str =json.dumps(obj)
    uart.write(output_str+'\n')
    print('I send:',output_str)
    if (uart.any()):
        print('you send:',uart.read())
    time.sleep_ms(1000)

# todo:返回值正确后可以不再发送

*/

#include <Arduino.h>
#include <ArduinoJson.h>//点击这里会自动打开管理库页面: http://librarymanager/All#ArduinoJson
#include <U8g2lib.h>
#include <ESPAsyncWebServer.h>
//选择何种机械臂主控板
//#define USE_ARM_BORAD //独立机械臂主控板
#define USE_POWER_ARM_BORAD//电源和机械臂主控二合一板

#ifdef USE_ARM_BORAD
#define TXD1 33
#define RXD1 32
#endif

#ifdef USE_POWER_ARM_BORAD
#define TXD1 27
#define RXD1 14
#endif

String jsonData = "";

void setup()
{
    Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
    Serial.begin(115200);
}

void loop()
{

    if (Serial1.available() > 0)
    {
        char charTemp = char(Serial1.read());
        jsonData = String(jsonData) + String(charTemp);

        if (charTemp == '\n')
        {
            if (jsonData.length() > 0)
            {
                // String input;
                Serial.print(jsonData);
                Serial1.print(jsonData);
                // DynamicJsonDocument doc(1024); //声明一个JsonDocument对象
                StaticJsonDocument<128> doc;
                DeserializationError error = deserializeJson(doc, jsonData);

                if (error)
                {
                    Serial.print("deserializeJson() failed: ");
                    Serial.println(error.c_str());
                    return;
                }

                const char *qrCode = doc["qrCode"]; // "123-321"

                JsonArray color = doc["color"];
                int color_0 = color[0]; // 1
                int color_1 = color[1]; // 2
                int color_2 = color[2]; // 3
                Serial.println(qrCode);
                Serial1.println(qrCode);
                Serial.println(color_0);
                Serial.println(color_1);
                Serial.println(color_2);
                jsonData = "";
            }
        }
    }
}