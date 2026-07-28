#include "MaixCam.h"
float Move_X_Grab = 0.0; // 抓取时的移动量
float Move_Y_Grab = 0.0;
int Current_Color = 0; // 当前识别的颜色
//              MaixCam       RX          TX
HardwareSerial Serial_Maix(Maix_RX, Maix_TX);
// 构造对象
Maix MaixCam;
void Maix::Maix_ReadData(uint8_t data)
{
    uint8_t receivedByte = data;
    // 重置超时计时器
    lastByteTime = millis();
    // 将字节存入缓冲区
    if (byteCount < PACKET_SIZE)
    {
        if (receivedByte == 0xAA)
        {
            byteCount = 0;
        }
        packetBuffer[byteCount] = receivedByte;
        byteCount++;
    }
    // 检查是否收到完整数据包
    if (byteCount == PACKET_SIZE)
    {
        // 验证帧头和帧尾
        if (packetBuffer[0] == 0xAA && packetBuffer[6] == 0xBB)
        {
            Head = 0xAA;
            // 可能是数据包读取问题，即坐标理应2位+2位，但是只用了1+1
            // 原来的
            /*Delta_X = (int)packetBuffer[2]; // 第一个有效数据
            Delta_Y = (int)packetBuffer[4]; // 第二个有效数据*/
            Delta_X = packetBuffer[2] + (packetBuffer[3] << 8);
            Delta_Y = packetBuffer[4] + (packetBuffer[5] << 8);
            Color = (int8_t)packetBuffer[1]; // 第三个有效数据
            End = 0xBB;
        }
        else
        {
            Delta_X = 0;
            Delta_Y = 0;
            Color = 0;
            Head = 0;
            End = 0;
        }

        // 重置接收状态
        byteCount = 0;
    }
    // 检查接收超时
    if (byteCount > 0 && (millis() - lastByteTime) > TIMEOUT_MS)
    {
        byteCount = 0; // 超时重置接收状态
    }
}

void Maix::Maix_Init()
{
    Serial_Maix.begin(Maix_BAUDRATE);

    Delta_X = 0;
    Delta_Y = 0;

    byteCount = 0;    // 数据长度
    lastByteTime = 0; // 接收时间
    delay(100);
}

void Maix::Maix_Follow(int RGB)
{
    unsigned char SendBuffer[4] = {0xAA, 0xCC, 0x00, 0xBB};
    switch (RGB)
    {
    case 1:
        SendBuffer[2] = 0x01;
        break;
    case 2:
        SendBuffer[2] = 0x02;
        break;
    case 3:
        SendBuffer[2] = 0x03;
        break;
    }
    Serial_Maix.write(SendBuffer, 4);
    Serial_Maix.flush(); // 等待发送完成
}

void Maix::Maix_Detect(int RGB)
{
    unsigned char SendBuffer[4] = {0xAA, 0xEE, 0x00, 0xBB};
    switch (RGB)
    {
    case 1:
        SendBuffer[2] = 0x01;
        break;
    case 2:
        SendBuffer[2] = 0x02;
        break;
    case 3:
        SendBuffer[2] = 0x03;
        break;
    }
    Serial_Maix.write(SendBuffer, 4);
    Serial_Maix.flush(); // 等待发送完成
}