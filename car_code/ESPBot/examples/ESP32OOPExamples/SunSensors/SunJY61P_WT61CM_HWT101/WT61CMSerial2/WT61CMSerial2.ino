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
unsigned char Horizontal[3] = {0xFF, 0xAA, 0x65};           // 模块水平放置
unsigned char resetZAngle[3] = {0xFF, 0xAA, 0x52};          // Z轴角度复位指令
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