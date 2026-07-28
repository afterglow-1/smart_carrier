#include <JY901.h>
/*
Test on ESP32.
JY901   ESP32
TX <---> 0(Rx2)
*/

#define IMU_Serial Serial2
//#define TXD2 17
//#define RXD2 16
#define TXD2 20
#define RXD2 19


unsigned char unlock[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};   //  解锁寄存器 https://wit-motion.yuque.com/wumwnr/ltst03/vl3tpy?#SzruE
unsigned char setBaud[5] = {0xFF, 0xAA, 0x04, 0x06, 0x00};  //  设置波特率为115200 FF AA 04 06 00
unsigned char saveData[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00}; //  保存数据
unsigned char new_Horizontal[5] = {0xFF, 0xAA, 0x23, 0x01, 0x00}; //水平安装
  unsigned char ACCCALSW[5] = {0xFF,0xAA,0x01,0x01,0x00};//进入加速度校准模式
  unsigned char HeadAngle_to_zero[5] = {0xFF,0xAA,0x01,0x04,0x00};// 航向角置零
  unsigned char RSW[5] = {0xFF,0xAA,0x01,0x04,0x00};//输出内容
unsigned char RRATE[5] = {0xFF,0xAA,0x03,0x09,0x00};//输出速率 FF AA 03 09 00（设置100Hz输出）

float initialYaw, initialYawRad, newYaw, newYawRad, realYaw, realYawRad;
bool isFirst = 1;
unsigned long lastTime=0;
float angle2rad(float angle)
{
  return angle * M_PI / 180;
}
float rad2angle(float rad)
{
  return rad * 180 / M_PI;
}
void setup() 
{
  //WT61陀螺仪采集串口2,默认9600
   //IMU_Serial.begin(9600,SERIAL_8N1,RXD2,TXD2);
 IMU_Serial.begin(9600);
  //尝试修改为115200，失败？
  IMU_Serial.write(unlock, 5);                    //解锁
  delay(10);//等待发送完成
  IMU_Serial.write(setBaud, 5);                    //设置波特率为115200
  delay(10);//等待发送完成
  IMU_Serial.write(saveData, 5);                    //保存
  Idelay(10);//等待发送完成
  //delay(10);
  //按115200设置
 IMU_Serial.begin(115200, SERIAL_8N1, RXD1, TXD1);  //二合一版
 //IMU_Serial.updateBaudRate(115200);//重新设置波特率


 
  IMU_Serial.write(unlock, 5);                    //解锁
  delay(10);//等待发送完成
  IMU_Serial.write(setBaud, 5);                    //设置波特率为115200
  delay(10);//等待发送完成
  IMU_Serial.write(new_Horizontal, 5);                    //水平安装
  delay(10);//等待发送完成
  IMU_Serial.write(ACCCALSW, 5);                    //进入加速度校准模式
  delay(10);//等待发送完成
  IMU_Serial.write(HeadAngle_to_zero, 5);                    //航向角置零
  delay(10);//等待发送完成
  
  IMU_Serial.write(saveData, 5);                    //保存
  delay(10);//等待发送完成             

  IMU_Serial.begin(115200, SERIAL_8N1, RXD2, TXD2);  //二合一版


 //锁定后发送会有问题
    //IMU_Serial.write(Horizontal, 3);  // Z轴角度复位指令
  //IMU_Serial.flush();               // 等待发送完成
  //IMU_Serial.write(resetZAngle, 3); // Z轴角度复位指令
  //IMU_Serial.flush();               // 等待发送完成

  delay(50);
    Serial.begin(115200);
      Serial.println("SunnyBot陀螺仪测试");
        delay(50);
      lastTime=millis();
}

void loop() 
{
    while (IMU_Serial.available()) 
  {
    JY901.CopeSerialData(IMU_Serial.read()); //Call JY901 data cope function
  }

    newYawRad = (float)JY901.stcAngle.Angle[2] / 32768 * M_PI;
  updateSensors();
  if (millis()-lastTime>=1000)
  {
    lastTime=millis();
    Serial.print(" yaw: ");
    Serial.println(newYaw);
    Serial.print("Acc:");Serial.print((float)JY901.stcAcc.a[0]/32768*16);Serial.print(" ");Serial.print((float)JY901.stcAcc.a[1]/32768*16);Serial.print(" ");Serial.println((float)JY901.stcAcc.a[2]/32768*16);

  }
  


}




// 更新传感数据
void updateSensors()
{

  // 第一次运行时把当前角度作为初始值
  if (isFirst)
  {
    initialYawRad = newYawRad;
    isFirst = 0;
  }
  realYawRad = newYawRad - initialYawRad; // 偏航角偏差量（弧度）
  realYaw = rad2angle(realYawRad);        // 偏航角偏差量（角度）
  initialYaw = rad2angle(initialYawRad);
  newYaw = rad2angle(newYawRad);

}