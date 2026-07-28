#include "MaixCam.h"

// ==== 全局变量定义 ==== //
float Move_X_Grab = 0.0; //抓取时的移动量
float Move_Y_Grab = 0.0;
int Current_Color = 0; //当前识别的颜色
HardwareSerial Serial_Maix(Maix_RX, Maix_TX);
Maix MaixCam;

// ==== 类方法实现 ==== //
void Maix::Maix_ReadData(uint8_t data)
{
    uint8_t receivedByte = data;
    // 重置超时计时器
    lastByteTime = millis();
    
    // 将字节存入缓冲区
    if (byteCount < PACKET_SIZE) 
    {
        if(receivedByte == 0xAA)
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
        if (packetBuffer[0] == 0xAA && packetBuffer[4] == 0xBB) 
        {
            Head = 0xAA;
            Delta_X = (int8_t)packetBuffer[1]; // 第一个有效数据
            Delta_Y = (int8_t)packetBuffer[2]; // 第二个有效数据
            Color = (int8_t)packetBuffer[3];   // 第三个有效数据
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
    unsigned char resetBuffer[4] = {0xAA, 0xFF, 0x00, 0xBB};
    Serial_Maix.begin(Maix_BAUDRATE);
    Serial_Maix.write(resetBuffer, sizeof(resetBuffer));
    Serial_Maix.flush();                // 等待发送完成

    // 重置变量
    Delta_X = 0;
    Delta_Y = 0;
    Head = 0;
    End = 0;
    Color = 0;
    byteCount = 0;
    lastByteTime = millis();

}

void Maix::Maix_Follow(int RGB)
{
    unsigned char SendBuffer[4] = {0xAA, 0xCC, 0x00, 0xBB};
    switch (RGB)
    {
    case Red:
        SendBuffer[2] = 0x01;
        break;
    case Green:
        SendBuffer[2] = 0x02;
        break;
    case Blue:
        SendBuffer[2] = 0x03;
        break;
    }
    Serial_Maix.write(SendBuffer, sizeof(SendBuffer));
    Serial_Maix.flush();
    
}

void Maix::Maix_Detect(int RGB)
{
    unsigned char SendBuffer[4] = {0xAA, 0xEE, 0x00, 0xBB};
    switch (RGB)
    {
    case Red:
        SendBuffer[2] = 0x01;
        break;
    case Green:
        SendBuffer[2] = 0x02;
        break;
    case Blue:
        SendBuffer[2] = 0x03;
        break;
    }
    Serial_Maix.write(SendBuffer, sizeof(SendBuffer));
    Serial_Maix.flush();
    
    }