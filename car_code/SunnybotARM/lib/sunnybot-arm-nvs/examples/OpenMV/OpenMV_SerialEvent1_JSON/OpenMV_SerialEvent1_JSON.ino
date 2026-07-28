//程序名称：ESP32_OpenMV_SerialEvent1_JSON
//程序功能：ESP32串口1 和openmv串口3通讯 使用伪串口1中断
//数据格式JSON
//使用方法：
// ESP32 硬串口1接收 串口1和串口0同时输出
//使用软串口1中断方式接受数据
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
outQRFlag=True
while(True):
    if(outQRFlag):
        output_str =json.dumps(obj)
        uart.write(output_str+'\n')
        print('I send:',output_str)
        if (uart.any()):
            data = uart.read()  #将串口3读取的数据存入data
            print('you send:',data)
            try:
                #数据能否解析为json
                dictData=json.loads(data)
            except  ValueError as e:
                print(e)
            else:
                print("没有出现异常")
                print(dictData)
                if(dictData["qrCode"]==obj["qrCode"]):
                    outQRFlag=False
    time.sleep_ms(1000)




*/

#include <Arduino.h>
#include <ArduinoJson.h>//点击这里会自动打开管理库页面: http://librarymanager/All#ArduinoJson
//#include <U8g2lib.h>
//#include <ESPAsyncWebServer.h>
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
bool stringComplete = false; // whether the string is complete

void setup()
{
    Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
    Serial.begin(115200);
}

void loop()
{

    if (stringComplete)
    {
        Serial.print(jsonData);
        Serial1.print(jsonData);
        StaticJsonDocument<128> doc;
        DeserializationError error = deserializeJson(doc, jsonData);//反序列化(Deserialization,解析数据)

        if (error)
        {
            Serial.print("deserializeJson() failed: ");
            Serial.println(error.c_str());
            jsonData = ""; //清除数据
            stringComplete = false;
            return;
        }
        JsonObject obj = doc.as<JsonObject>();
        if (obj.containsKey("qrCode"))
        {
            Serial.println("containsKey: qrCode");
            const char *qrCode = doc["qrCode"]; // "123-321"
            JsonArray color = doc["color"];
            int color_0 = color[0]; // 1
            int color_1 = color[1]; // 2
            int color_2 = color[2]; // 3
            Serial.println(qrCode);
            Serial1.println(qrCode);//注释本行可关闭回传数据，openmv会周期性执行
            Serial.println(color_0);
            Serial.println(color_1);
            Serial.println(color_2);
        }

        jsonData = ""; //清除数据
        stringComplete = false;
    }
}

void serialEvent1()
{
    while (Serial1.available())
    {
        // get the new byte:
        char charTemp = (char)Serial1.read();
        // add it to the inputString:
        jsonData += charTemp;
        // if the incoming character is a newline, set a flag so the main loop can
        // do something about it:
        if (charTemp == '\n')
        {
            stringComplete = true;
        }
    }
}