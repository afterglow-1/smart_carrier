/******************************************************************************
                             IM948模块使用示例
适用于 arduino
    1、默认调试口为串口Serial，115200波特率，8数据位、1停止位、无校验 用于实现与用户的操作交互，并实现Dbp日志打印函数 用户可根据需要是否屏蔽
       模块通信口为串口Serial1，115200波特率，8数据位、1停止位、无校验 用于实现和IM948模块串口通信
    2、a.用户把 im948_CMD.cpp 和 im948_CMD.h 添加到自己的项目里
       b.实现 Cmd_Write(U8 *pBuf, int Len) 函数给串口发送数据
       c.并把串口接收到的每字节数据填入 Cmd_GetPkt(U8 byte)
       d.收到模块发来的数据包会回调 Cmd_RxUnpack(U8 *buf, U8 DLen)，用户可在该函数里处理需要的数据功能
       e.参考手册通信协议和本示例的 im948_test()，进行调试每个功能即可
       f.若用户需要扩展485总线方式并联多个模块使用时，可通过设置模块地址 targetDeviceAddress 来指定操作对象
*******************************************************************************/
#include "OOPConfig.h"
float lastyaw = 0;
void setup() {
  Serial.begin(115200);                           // 设置串口0波特率为115200  用做调试控制口
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);  // 设置串口1波特率为115200  用做IM948通信口
  Dbp("--- IM948 arduino test start V1.03 ---\r\n");
  delay(3000);  // 上电后先延迟一会，确保传感器已上电准备完毕

  // 唤醒传感器，并配置好传感器工作参数，然后开启主动上报---------------
  Cmd_03();  // 2 唤醒传感器
  /**
       * 设置设备参数
     * @param accStill    惯导-静止状态加速度阀值 单位dm/s?
     * @param stillToZero 惯导-静止归零速度(单位cm/s) 0:不归零 255:立即归零
     * @param moveToZero  惯导-动态归零速度(单位cm/s) 0:不归零
     * @param isCompassOn 1=需开启磁场 0=需关闭磁场
     * @param barometerFilter 气压计的滤波等级[取值0-3],数值越大越平稳但实时性越差
     * @param reportHz 数据主动上报的传输帧率[取值0-250HZ], 0表示0.5HZ
     * @param gyroFilter    陀螺仪滤波系数[取值0-2],数值越大越平稳但实时性越差
     * @param accFilter     加速计滤波系数[取值0-4],数值越大越平稳但实时性越差
     * @param compassFilter 磁力计滤波系数[取值0-9],数值越大越平稳但实时性越差
     * @param Cmd_ReportTag 功能订阅标识
     */
  Cmd_12(5, 255, 0, 1, 3, 2, 2, 4, 9, 0x00E0);  // 7 设置设备参数(内容1)
  Cmd_19();                                     // 4 开启数据主动上报
}

void loop() {
 
//数据变化时打印
  if (lastyaw != rpy[2]) {
    Serial.print("yaw\t");
    Serial.println(rpy[2]);
    lastyaw = rpy[2];
  }


}

//伪串口1中断 处理传感器发过来的数据
void serialEvent1()
{
  while (Serial1.available())
  {
    U8 rxByte = Serial1.read();      // 读取串口的数据
    Cmd_GetPkt(rxByte);           // 移植 每收到1字节数据都填入该函数，当抓取到有效的数据包就会回调进入 Cmd_RxUnpack(U8 *buf, U8 DLen) 函数处理
  }
}
