//程序名称：lib/sunnybot-arm-nvs/examples/sunarm_calibration_u8g2/sunarm_calibration_u8g2.ino
//程序功能：一键标定
//使用方法：1.上传data下网页文件 2.上传程序 3.请手动调整机械臂到竖直态，然后按下标定按钮IO0,IO0状态:0;累计次数:11
// 4.标定后手动扳动机械臂各关节，查看各关节角度变化方向是否和零构型一致，不一致则修改方向0 1。
//测试网页连接
//设计一个竖直标定设备
//经常有个别通讯不上的问题
//检查舵机电压 电压不足提示
// to do 拖动示教保存成文件
// to do 添加手爪校准？

#include <Arduino.h>
#include <Arduino_JSON.h> //Arduino IDE下自动打开管理库页面: http://librarymanager/All#Arduino_JSON
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// https://microcontrollerslab.com/save-data-esp32-flash-permanently-preferences-library/
// https://randomnerdtutorials.com/esp32-save-data-permanently-preferences/
// https://docs.espressif.com/projects/arduino-esp32/en/latest/tutorials/preferences.html?highlight=Preferences#multiple-namespaces
#include <WiFi.h>
#include "SPIFFS.h"
#include "sunnybot-arm-nvs.h"
//数据存储 https://blog.csdn.net/weixin_42880082/article/details/121675013
// https://blog.csdn.net/Naisu_kun/article/details/115514715
// https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/api/preferences.html#:~:text=The%20Preferences%20library%20is%20unique%20to%20arduino-esp32.%20It,and%20loss%20of%20power%20events%20to%20the%20system.
//或者使用littleFS
#include <U8g2lib.h>
#define ButtonPin 0
// 调试串口的配置
#if defined(ARDUINO_AVR_UNO)
#include <SoftwareSerial.h>
#define SOFT_SERIAL_RX 6
#define SOFT_SERIAL_TX 7
SoftwareSerial softSerial(SOFT_SERIAL_RX, SOFT_SERIAL_TX); // 创建软串口
#define DEBUG_SERIAL softSerial
#define DEBUG_SERIAL_BAUDRATE 4800
#elif defined(ARDUINO_AVR_MEGA2560)
#define DEBUG_SERIAL Serial
#define DEBUG_SERIAL_BAUDRATE 115200
#elif defined(ARDUINO_ARCH_ESP32)
#define DEBUG_SERIAL Serial
#define DEBUG_SERIAL_BAUDRATE 115200
#endif
// Replace with your network credentials
const char *ssid = "ESP32654115";
const char *password = "12345678";
FSARM_ARM5DoF arm; //机械臂对象
// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create an Event Source on /events
// EventSource对象，url为/events； 注意EventSource和Http共用url
AsyncEventSource events("/events");

AsyncWebSocket ws("/ws");
// NVS
ARMNVS armnvs;

// Json Variable to Hold Sensor Readings
JSONVar rawReadings, jointReadings, poseReadings;

// Timer variables
unsigned long lastTime = 0;
unsigned long readDelay = 1000;

// //挂载SPIFFS 用工具上传文件时会自动生成SPIFFS
void initSPIFFS()
{
  if (!SPIFFS.begin())
  {
    DEBUG_SERIAL.println("An error has occurred while mounting SPIFFS");
  }
  DEBUG_SERIAL.println("SPIFFS mounted successfully");
}

// Initialize WiFi
void initWiFi()
{
  WiFi.softAP(ssid, password);
  //打印热点IP
  IPAddress IP = WiFi.softAPIP();
  DEBUG_SERIAL.print("AP IP address: ");
  DEBUG_SERIAL.println(IP);
}
FSARM_JOINTS_STATE_T rawThetas;   // 舵机的原始角度
FSARM_JOINTS_STATE_T jointThetas; // 关节的角度
float gripper_raw_angle;          // 爪子的原始角度
float posX, posY, posZ;

//获取舵机原始角度
String getServoRawAngles()
{
  arm.queryRawAngle(&rawThetas);                         // 查询并更新关节舵机的原始角度
  gripper_raw_angle = arm.gripper_servo.queryRawAngle(); // 查询爪子的原始角度

  rawReadings["theta1"] = String(rawThetas.theta1, 1);
  rawReadings["theta2"] = String(rawThetas.theta2, 1);
  rawReadings["theta3"] = String(rawThetas.theta3, 1);
  rawReadings["theta4"] = String(rawThetas.theta4, 1);
  rawReadings["theta5"] = String(gripper_raw_angle, 1);

  String servoString = JSON.stringify(rawReadings);
  return servoString;
}

//获取关节角度
String getJointAngles()
{
  arm.queryAngle(&jointThetas);                          // 查询并更新关节的角度
  gripper_raw_angle = arm.gripper_servo.queryRawAngle(); // 查询爪子的原始角度

  jointReadings["joint1"] = String(jointThetas.theta1, 1);
  jointReadings["joint2"] = String(jointThetas.theta2, 1);
  jointReadings["joint3"] = String(jointThetas.theta3, 1);
  jointReadings["joint4"] = String(jointThetas.theta4, 1);
  jointReadings["gripper1"] = String(gripper_raw_angle, 1);

  String jointString = JSON.stringify(jointReadings);
  return jointString;
}
//获取末端姿态
String getToolPose()
{
  FSARM_POINT3D_T toolPosi; // 末端的位置
  float pitch;
  // gripper_raw_angle = arm.gripper_servo.queryRawAngle();  //
  // 查询爪子的原始角度
  arm.forwardKinematics(jointThetas, &toolPosi, &pitch); // 正向运动学
  poseReadings["posX"] = String(toolPosi.x, 2);
  poseReadings["posY"] = String(toolPosi.y, 2);
  poseReadings["posZ"] = String(toolPosi.z, 2);
  poseReadings["pitch"] = String(pitch, 2);
  // poseReadings["gripper1"] = String(gripper_raw_angle, 1);
  String poseString = JSON.stringify(poseReadings);
  return poseString;
}
void setup()
{
  DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUDRATE); // 初始化串口的波特率
  pinMode(ButtonPin, INPUT_PULLUP);
  initWiFi();
  initSPIFFS();
  arm.init();           //机械臂初始化
  arm.gripperOpen(50);  //测试手爪开启
  arm.setDamping(1000); //设置0-3号舵机为阻尼模式

  //打印原有标定数据
  if (armnvs.readARMNVS())
  {
    armnvs.printARMNVS();
  }
  else
  {
    DEBUG_SERIAL.println("未找到标定数据或未进行过标定");
  }
  uint32_t countC = 0;
  while (digitalRead(ButtonPin) == HIGH)
  {
    if ((millis() - lastTime) > 2000)
    {
      countC++;
      DEBUG_SERIAL.print(
          "请手动调整机械臂到竖直态，然后按下标定按钮IO0,IO0状态:");
      DEBUG_SERIAL.print(!digitalRead(ButtonPin));
      DEBUG_SERIAL.print(";累计次数:");
      DEBUG_SERIAL.println(countC);
      lastTime = millis();
    }
  }
  DEBUG_SERIAL.println("已按下标定按钮IO0。");

  //打印机械臂当前的舵机角度(原始)
  String message = "Raw Angles: " + getServoRawAngles();
  DEBUG_SERIAL.println(message);
  //标定数据    joint 1  2  3  4
  //标定数据    servo 0  1  2  3
  bool flipTurn[] = {0, 0, 1, 1}; //根据手掰角度与建模角度是否一致，调整舵机转向极性。
  arm.calibration(rawThetas.theta1, flipTurn[0], rawThetas.theta2,
                  flipTurn[1], rawThetas.theta3, flipTurn[2],
                  rawThetas.theta4, flipTurn[3]);

  //保存标定数据并打印
  armnvs.writeARMNVS(rawThetas, flipTurn,
                     arm.gripper_servo.queryRawAngle());
  if (armnvs.readARMNVS())
  {
    armnvs.printARMNVS();
  }
  // 打印正向运动学末端位姿的结果
  DEBUG_SERIAL.println("Tool Pose: " + getToolPose());

  // Handle Web Server 注册链接与回调函数；[]C++ lambda函数
  //注册链接"/lambda"与对应回调函数（匿名函数形式声明）
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/index.html", "text/html"); });
  //设置静态文件服务 用户访问/时，返回SPIFFS中/文件
  server.serveStatic("/", SPIFFS, "/");

  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    posX = 0;
    posY = 0;
    posZ = 0;
    request->send(200, "text/plain", "OK"); });

  server.on("/resetX", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    posX = 0;
    request->send(200, "text/plain", "OK"); });

  server.on("/resetY", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    posY = 0;
    request->send(200, "text/plain", "OK"); });

  server.on("/resetZ", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    posZ = 0;
    request->send(200, "text/plain", "OK"); });
  // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient *client)
                   {
    if (client->lastId()) {
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n",
                    client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 1000); });
  // 将EventSource添加到服务器中
  server.addHandler(&events);

  server.begin(); //启动服务器进行请求监听
                  //记录重启次数

  delay(4000);
  lastTime = millis();
}

void loop()
{
  //打印机械臂当前的舵机角度(原始)
  String message1 = "Raw   Angles:  " + getServoRawAngles();
  DEBUG_SERIAL.println(message1);

  delay(10);
  String message2 = "Joint Angles:  " + getJointAngles();
  DEBUG_SERIAL.println(message2);
  delay(10);
  DEBUG_SERIAL.println("Tool    Pose: " + getToolPose());
  DEBUG_SERIAL.println(millis());
  delay(1000);

  if ((millis() - lastTime) > readDelay)
  {
    // Send Events to the Web Server with the Sensor Readings
    // c_str()函数返回一个指向正规C字符串的指针, 内容与本string串相同
    events.send(getServoRawAngles().c_str(), "servo_readings", millis());
    events.send(getJointAngles().c_str(), "joint_readings", millis());
    events.send(getToolPose().c_str(), "pose_readings", millis());
    lastTime = millis();
  }
}