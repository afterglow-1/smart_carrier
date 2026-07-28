/*
*
程序名称：sunarm-gripper-test
舵机开关
初始时为打开状态
需要断电重启
带电压检测
*/
#include <Arduino.h>
#include "FashionStar_SmartGripper.h"
#include <U8g2lib.h>

// 调试串口的配置
#define DEBUG_SERIAL Serial
#define DEBUG_SERIAL_BAUDRATE 115200

// 串口总线舵机配置
#define SERVO_ID 4      //爪子对应的舵机ID号
#define BAUDRATE 115200 // 波特率

// 爪子的配置


float SERVO_ANGLE_GRIPPER_OPEN=0.0 ;   // 爪子张开时的角度
float SERVO_ANGLE_GRIPPER_CLOSE=SERVO_ANGLE_GRIPPER_OPEN-70.0; // 爪子闭合时的角度
float curAngle;
#define GRIPPER_INTERVAL_MS 2000        // 爪子开启闭合的周期, ms
#define GRIPPER_MAX_POWER 400           // 爪子的最大功率
unsigned long previousMillis = 0;       // will store last time run
// 创建舵机的通信协议对象
FSUS_Protocol protocol(BAUDRATE);
// 创建舵机的实例
FSUS_Servo gripper_servo(SERVO_ID, &protocol);
// 创建智能机械爪实例
FSGP_Gripper gripper(&gripper_servo, SERVO_ANGLE_GRIPPER_OPEN, SERVO_ANGLE_GRIPPER_CLOSE);

// oled
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/22, /* data=*/21); // ESP32 Thing, HW I2C with pin remapping

float voltage_k = 0.99;
String statusString = ""; // string to hold input
void setup()
{
    DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUDRATE);       // 软串口
    DEBUG_SERIAL.println("Start To Test Gripper\n"); // 打印日志
                                                     // set the resolution to 12 bits (0-4095)
    analogReadResolution(12);
    analogSetPinAttenuation(A0, ADC_2_5db); //可测量的输入电压范围100 mV ~ 1250 mV
    analogSetPinAttenuation(A3, ADC_2_5db); //可测量的输入电压范围100 mV ~ 1250 mV

    
    
    protocol.init();                        // 舵机通信协议初始化
    gripper_servo.init();                         // 爪子舵机初始化
    curAngle = gripper_servo.queryAngle();
    //取大值重新初始化手爪
    SERVO_ANGLE_GRIPPER_OPEN=(curAngle>SERVO_ANGLE_GRIPPER_OPEN)?curAngle:SERVO_ANGLE_GRIPPER_OPEN;
    SERVO_ANGLE_GRIPPER_CLOSE=SERVO_ANGLE_GRIPPER_OPEN-50.0; 
    gripper.init(&gripper_servo,SERVO_ANGLE_GRIPPER_OPEN,SERVO_ANGLE_GRIPPER_CLOSE);   
    gripper.setMaxPower(GRIPPER_MAX_POWER); // 设置爪子的最大功率

    //屏幕初始化
    u8g2.begin();
    u8g2.enableUTF8Print(); // enable UTF8 support for the Arduino print() function
    previousMillis = millis();
}

void loop()
{

    bool isOnline = gripper_servo.ping(); // 舵机通讯检测
    float analogA0Volts = float(analogReadMilliVolts(A0)) * 11.0 * voltage_k / 1000.0;
    float analogA3Volts = float(analogReadMilliVolts(A3)) * 11.0 * voltage_k / 1000.0;
    unsigned long currentMillis = millis(); // store the current time
    //电压大于阈值才启动
    if (analogA0Volts <= 10.9)
    {
        DEBUG_SERIAL.print("电池电压低，请马上充电！");
    }
    else
    {
        if (isOnline)
        {

            // DEBUG_SERIAL.println("电池电压正常。");
            if (currentMillis - previousMillis >= 4000)
            {
                if (gripper.getStatus() == FSGP_STATUS_OPEN)
                {
                    gripper.close();
                    //delay(1500);
                    DEBUG_SERIAL.println("Gripper Close\n"); // 打印日志
                    statusString = "close";
                    previousMillis = currentMillis;
                }
                else
                {
                    gripper.open();
                    //delay(1500);
                    DEBUG_SERIAL.println("Gripper Open\n"); // 打印日志
                    statusString = "Open";
                    previousMillis = currentMillis;
                }
            }
        }
    }

    //屏幕输出刷新
    u8g2.setFont(u8g2_font_unifont_t_chinese2); // use chinese2
    u8g2.firstPage();
    do
    {
        u8g2.setCursor(0, 20);

        u8g2.print("isOnline:");
        u8g2.print(isOnline);
        u8g2.setCursor(0, 40);
        u8g2.print("手爪状态:");
        u8g2.print(statusString);

        u8g2.setCursor(0, 60);
        u8g2.print("BV:"); //电池电压
        u8g2.print(analogA0Volts);
        u8g2.print("SV:"); //舵机电压
        u8g2.print(analogA3Volts);

    } while (u8g2.nextPage());
}
