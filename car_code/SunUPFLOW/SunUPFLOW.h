/**
 * @file SunGrayscale.h
 * @author igcxl (igcxl@qq.com)
 * @brief 光流库 补光灯至少距离5cm
 * @version 0.7
 * @date 2023-6-23
 * @copyright Copyright (c) 2023
 * */
#ifndef SUN_UPFLOW_H_
#define SUN_UPFLOW_H_

#include <Arduino.h>
#include "lowpass_filter.h"
#define USE_LC302GS
//TIME 
// #define  USE_LC302
//TIME 20800
#define LC302GS_BAUD 460800 //345600 // LC302GS波特率
#define LC302_BAUD 19200 // LC302GS波特率
#ifdef USE_LC302GS 
#define FLOW_BAUD 460800//115200//345600
#endif
#ifdef USE_LC302
#define FLOW_BAUD 19200
#endif
#define FLOW_HEIGHT 50           // 镜头高度 ,单位mm

#define FLOW_TIMEOUT_MS 100
#define FLOW_HEADER 0xFE       // 数据结构体帧头
#define FLOWLEN 0x0A      // 光流数据数据结构体字节数（固定值）
#define FLOW_FRAME_END 0x55    // 数据结构体帧尾
#define FLOW_RX_BUFFER_SIZE 14 // 接受缓存区大小
#define FLOW_DATA_SIZE 14      // 封包长度
#define TIMESPAN  8319//两次测量间隔，单位us
#define DEF_X_FILTER_Tf  0.003//低通滤波系数，单位s  Tf = 3ms
#define DEF_Y_FILTER_Tf  0.003//低通滤波系数，单位s  Tf = 3ms


#define UPFLOW_STATUS_SUCCESS 0 // 设置/读取成功
#define UPFLOW_STATUS_FAIL 1 // 设置/读取失败
#define UPFLOW_STATUS_WRONG_HEADER 3 // 响应头不对
#define UPFLOW_STATUS_WRONG_LEN 4 // 第二字节不对
#define UPFLOW_STATUS_SIZE_SHORT 5 // size未到达
#define UPFLOW_STATUS_CHECKSUM_ERROR 6 // 校验错误
#define UPFLOW_STATUS_WRONG_END 7 // 帧尾不对
#define UPFLOW_STATUS_TIMEOUT 8 // 等待超时

class UPFLOW
{
public:
  // 封包结构体
  typedef struct optical_flow_data
  {
    int16_t flow_x_integral;//累计时间内的x位移
    int16_t flow_y_integral;//累计时间内的y位移
    uint16_t integration_timespan;//间隔累计时间
    uint16_t ground_distance;
    uint8_t quality;
    uint8_t version;
  } UpixelsOpticalFlow;

  UpixelsOpticalFlow packData; // 传感器数据结构体
  int32_t x_Offset;            // x方向累计偏移,单位um
  int32_t y_Offset; // y方向累计偏移,单位um
 float flow_x_integral_filtered,flow_y_integral_filtered;
  LowPassFilter flow_x_LPF{DEF_X_FILTER_Tf};//!<  parameter determining the velocity Low pass filter configuration 
  LowPassFilter flow_y_LPF{DEF_Y_FILTER_Tf};//!<  parameter determining the velocity Low pass filter configuration 
  UPFLOW(HardwareSerial *uartPort, int8_t rxPin, int8_t txPin, unsigned long baud = FLOW_BAUD);

 uint8_t readData(unsigned char ucData); // 读取数据
  void emptyCache();                   // 清空缓存
  void initUART();                     // 串口初始化
private:
  HardwareSerial *_uartPort; // 使用的串口号
  int8_t _rxPin, _txPin;
  unsigned long _baud; // 串口通信的波特率
};

#endif