#ifndef MaixCam_h
#define MaixCam_h

#include <Arduino.h>

// 定义接收参数
#define PACKET_SIZE 5
#define TIMEOUT_MS 10

// MaixCam串口通信
#define Maix_RX PE7
#define Maix_TX PE8
#define Maix_BAUDRATE 115200

// 视觉识别中颜色的定义
#define Red    1
#define Green  2
#define Blue   3

class Maix
{
public:
    // 传递的x方向与y方向偏差
    int Delta_X = 0;
    int Delta_Y = 0;
    int Head = 0;
    int End = 0;
    int Color = 0;
    // 接收数据变量
    uint8_t packetBuffer[PACKET_SIZE]; // 数据包
    uint8_t byteCount = 0;             // 数据长度
    unsigned long lastByteTime = 0;    // 接收时间

    // 接收数据函数
    void Maix_ReadData(uint8_t data);

    // 初始化以及复位函数
    void Maix_Init();

    // 指定跟随颜色函数
    void Maix_Follow(int RGB);

    //检测圆环
    void Maix_Detect(int RGB);
};

// ==== 添加全局变量声明 ==== //
extern HardwareSerial Serial_Maix;
extern Maix MaixCam;
extern float Move_X_Grab;
extern float Move_Y_Grab;
extern int Current_Color;

#endif