//头文件引入区
#include <Arduino.h>
#include "AccelStepper.h"
#include <JY901.h>
#include "OneButton.h" 
#include "PID.h"
#include <HardwareTimer.h>
#include "FashionStar_UartServo.h"    // Fashion Star串口总线舵机
#include "FashionStar_SmartGripper.h" // Fashion Star智能夹具
#include "TTL_STEPPER.h"              //串口步进电机
#include "MaixCam.h"
#include "MultiStepper.h"
#include <stdio.h>
#include <string.h>

//define区
#define PI 3.14159265358979323846
#define M1_4_EN_PIN  PE13
#define M1_DIR_PIN  PD6
#define M1_STP_PIN  PD4
#define M2_DIR_PIN  PE9
#define M2_STP_PIN  PE11
#define M3_DIR_PIN  PD14
#define M3_STP_PIN  PD15
#define M4_DIR_PIN  PC3_C
#define M4_STP_PIN  PA1
#define M5_EN_PIN  PE10
#define M5_DIR_PIN  PE15
#define M5_STP_PIN  PB11
#define motorInterfaceType 1

//屏幕
#define TJCHMI_RX PB15
#define TJCHMI_TX PB14
#define DATA_NUM 19

//扫码模块
#define QR_RX PE0
#define QR_TX PE1

//IMU
#define WTIMU_RX  PD9
#define WTIMU_TX PD8
#define START_BTN PB9

//波特率
#define TJCHMI_BAUDRATE 115200
#define QR_BAUDRATE 9600
#define WTIMU_BAUDRATE 115200
#define STEPPER_BAUDRATE 115200

//视觉模块参数
#define InitScale 262.0
#define ZeroScale 20.0
#define InitHeight 282.0
#define ItemHeight 147
#define ImageScale 240
#define Arm_Zero_Length 1234

// 串口总线舵机配置
#define SERVO_BAUDRATE 115200
#define SERVO_RX PC7
#define SERVO_TX PC6
#define GRIPPER_SERVO_ID 4
#define STORAGE_SERVO_ID 5

// 串口步进电机
#define Stepper_TX PA2
#define Stepper_RX PA3
#define ARM_Stepper_ID 7
#define Gripper_Stepper_ID 6
#define LUOGAN (12 * 10)
#define CHILUN (36 * 1 * PI * 10)

//变量定义区

// 任务码结构体（12位：四组三位数）
struct TaskCode {
    uint8_t batch1_colors[3];      // 第一批三个物料的颜色（顺序）
    uint8_t batch1_positions[3];   // 第一批物料放置位置（1-3）
    uint8_t batch2_colors[3];      // 第二批三个物料的颜色（顺序）
    uint8_t batch2_positions[3];   // 第二批物料放置位置（1-3）
};

// 执行计划结构体
struct ExecutionStep {
    uint8_t color;          // 要抓取的颜色
    uint8_t targetPos;      // 放置位置（1-3）
    bool isBatch1;          // true=第一批，false=第二批
    bool isGrab;            // true=抓取，false=放置
};

//任务枚举
enum State {
    TASK_IDLE,              // 空闲等待启动
    TASK_WAIT_QR,           // 等待二维码
    TASK_DEPARTURE,         // 出库
    TASK_MOVE_FORWARD,      // X方向前进
    TASK_ROTATE_TO_TARGET,  // 旋转
    TASK_MOVE_LATERAL,      // Y方向前进
    TASK_SECONDARY_ROTATE,  // 二次旋转
    TASK_VISION_ALIGN_X,    // 视觉对齐X
    TASK_VISION_ALIGN_Y,    // 视觉对齐Y
    TASK_VISION_ALIGN,      // 视觉综合对齐
    TASK_GRAB_FROM_TABLE,   // 从转盘抓取
    TASK_PLACE_TO_AREA,     // 放置到区域
    TASK_GRAB_FROM_STORAGE, // 从载物台抓取
    TASK_PLACE_TO_GROUND,   // 放置到地面
    TASK_RETURN_HOME,       // 返回原点
    TASK_COMPLETE           // 任务完成
};

// 全局变量
State currentState = TASK_IDLE;
TaskCode currentTask;
ExecutionStep executionPlan[6];  // 共6个物料（3+3）
int planIndex = 0;
bool taskCodeValid = false;
bool startFlag = false;
bool runFlag = true;
unsigned long stateStartTime = 0;
const unsigned long STATE_TIMEOUT = 30000; // 30秒超时

// 小车运动参数
int motorPolarity[4] = {-1, 1, -1, 1};
const long turn_Pulses = 3140;         // 90°旋转对应的脉冲数
const long PULSES_PER_METER = 10500;
const float PULSES_PER_REV = 200;
const float MICROSTEPS = 16;
const float ROTATION_GEAR_RATIO = 4;
const float ROTATION_PULSES_PER_DEG = (PULSES_PER_REV * MICROSTEPS * ROTATION_GEAR_RATIO) / 360.0;

// 手爪运动参数
int STEPPER_ZHUANPAN = 620;
int STEPPER_GROUND = 1390;
int STEPPER_STORAGE = 450;
int STEPPER_ZERO = 50;
int MATERIAL_HEIGHT = 680;
int STEPPER_GRIPPER[4] = {20, 24, 60, 15};
int STEPPER_GRIPPER_ZHUANPAN_CENTER = 60;
int GRIPPER_OPEN_ANGLE = 0;
int GRIPPER_CLOSE_ANGLE = -85;
int GRIPPER_OPEN_MAX_ANGLE = 0;
float ARM_BASE_STEPPER_ANGLE[4] = {15, -132, -86, -42};
float ARM_BASE_STEPPER_HOME_ANGLE = 0;
int storage[5] = {135, -90, 0, 95, -35};

// IMU相关
float yaw = 0;
float cur_rot = 0;
float last_yaw = 0;
float accumulated_yaw = 0;
int yaw100 = 0;
char str[20];

// 扫码相关
const int QR_BUFFER_SIZE = 16;
char receivedData[QR_BUFFER_SIZE];
int dataIndex = 0;
bool scanFlag = false;
bool scanflag1 = true;

// 视觉相关
int8_t dx = 0;
int8_t dy = 0;
bool vision_updated = false;
float Scale = 0.01;
int cishu = 0;
int time1 = 0;

// 其他参数
int taskIndex = 0;          // 当前任务索引
int colorLoopIndex = 0;     // 颜色循环索引
int rounds = 1;
float MaxError_Move = 2;
float MaxError_Rot = 0.1;
unsigned long nowtime = 0;

// 实例创建

AccelStepper M1_stepper = AccelStepper(motorInterfaceType, M1_STP_PIN, M1_DIR_PIN);
AccelStepper M2_stepper = AccelStepper(motorInterfaceType, M2_STP_PIN, M2_DIR_PIN);
AccelStepper M3_stepper = AccelStepper(motorInterfaceType, M3_STP_PIN, M3_DIR_PIN);
AccelStepper M4_stepper = AccelStepper(motorInterfaceType, M4_STP_PIN, M4_DIR_PIN);
AccelStepper rotationStepper(motorInterfaceType, M5_STP_PIN, M5_DIR_PIN);

HardwareSerial Serial_TJCHMI(TJCHMI_RX, TJCHMI_TX);
HardwareSerial Serial_QR(QR_RX, QR_TX);
HardwareSerial Serial_SERVO(SERVO_RX, SERVO_TX);
HardwareSerial Serial_Stepper(Stepper_RX, Stepper_TX);
HardwareSerial MaixSerial(PE7, PE8);

FSUS_Protocol protocol(&Serial_SERVO, SERVO_BAUDRATE);
FSUS_Servo storageServo(STORAGE_SERVO_ID, &protocol);
FSUS_Servo gripperServo(GRIPPER_SERVO_ID, &protocol);
FSGP_Gripper gripper(&gripperServo, GRIPPER_OPEN_ANGLE, GRIPPER_CLOSE_ANGLE);

TTL_Protocol Stepper_protocol(&Serial_Stepper, STEPPER_BAUDRATE);
TTL_Stepper armStepper(ARM_Stepper_ID, &Stepper_protocol);
TTL_Stepper gripperStepper(Gripper_Stepper_ID, &Stepper_protocol);

OneButton start_btn(START_BTN, true, true);

// PID对象（需要外部定义）
// PID Rot_PID;

// 函数声明
void update_imu_angle();
void parse_task_code();
void generate_execution_plan();
void display_task_info();
void process_vision_data();
bool allMotorsStopped();
void Motor_Setup(float Vm, float Accel_Time);
void RunMotors();
void Run_forward(float distance);
void RotateCar_toTarget(float targetRot, float Rpm, float Accel_Time);
void Run_PID_forward(float x);
void Run_PID_left(float y);
void Grab_ZhuanPan_to_Storage(int color, int targetPos);
void Grab_Storage_to_Place(int targetPos);
void Grab_Ground_to_Storage();
void rotateBase(float degrees);

// 基础函数实现
void update_imu_angle() {
    if (Serial_WTIMU.available()) {
        JY901.CopeSerialData(Serial_WTIMU.read());
        float new_yaw = (float)JY901.stcAngle.Angle[2] / 32768 * 180;
        
        // 计算角度增量
        float delta_yaw = new_yaw - last_yaw;
        if (delta_yaw > 180) delta_yaw -= 360;
        if (delta_yaw < -180) delta_yaw += 360;
        accumulated_yaw += delta_yaw;
        last_yaw = new_yaw;
        yaw = new_yaw;
        
        // cur_rot 表示相对起始位置的旋转（单位：圈数，1圈=90°）
        cur_rot = accumulated_yaw / 90.0f;
        yaw100 = (int)(yaw * 100);
    }
}

void parse_task_code() {
    // 格式: "156+123+516+231"
    // 索引: 0 1 2 | 3 | 4 5 6 | 7 | 8 9 10 | 11 | 12 13 14
    //       第一组  '+'  第二组  '+'  第三组   '+'  第四组
    
    // 检查格式
    if (receivedData[3] != '+' || receivedData[7] != '+' || receivedData[11] != '+') {
        taskCodeValid = false;
        return;
    }
    
    // 第一组：第一批搬运顺序（颜色）
    for (int i = 0; i < 3; i++) {
        if (receivedData[i] >= '1' && receivedData[i] <= '6') {
            currentTask.batch1_colors[i] = receivedData[i] - '0';
        } else {
            taskCodeValid = false;
            return;
        }
    }
    
    // 第二组：第一批放置位置
    for (int i = 0; i < 3; i++) {
        if (receivedData[i + 4] >= '1' && receivedData[i + 4] <= '3') {
            currentTask.batch1_positions[i] = receivedData[i + 4] - '0';
        } else {
            taskCodeValid = false;
            return;
        }
    }
    
    // 第三组：第二批搬运顺序（颜色）
    for (int i = 0; i < 3; i++) {
        if (receivedData[i + 8] >= '1' && receivedData[i + 8] <= '6') {
            currentTask.batch2_colors[i] = receivedData[i + 8] - '0';
        } else {
            taskCodeValid = false;
            return;
        }
    }
    
    // 第四组：第二批放置位置
    for (int i = 0; i < 3; i++) {
        if (receivedData[i + 12] >= '1' && receivedData[i + 12] <= '3') {
            currentTask.batch2_positions[i] = receivedData[i + 12] - '0';
        } else {
            taskCodeValid = false;
            return;
        }
    }
    
    taskCodeValid = true;
}

void generate_execution_plan() {
    // 第一批：抓取并放置
    for (int i = 0; i < 3; i++) {
        executionPlan[i].color = currentTask.batch1_colors[i];
        executionPlan[i].targetPos = currentTask.batch1_positions[i];
        executionPlan[i].isBatch1 = true;
        executionPlan[i].isGrab = true;
    }
    
    // 第二批：抓取并放置
    for (int i = 0; i < 3; i++) {
        executionPlan[i + 3].color = currentTask.batch2_colors[i];
        executionPlan[i + 3].targetPos = currentTask.batch2_positions[i];
        executionPlan[i + 3].isBatch1 = false;
        executionPlan[i + 3].isGrab = true;
    }
    
    planIndex = 0;
}

void display_task_info() {
    char buf[64];
    
    // 显示完整任务码
    sprintf(buf, "t0.txt=\"%s\"\xff\xff\xff", receivedData);
    Serial_TJCHMI.print(buf);
    
    // 显示进度
    sprintf(buf, "t2.txt=\"%d/6\"\xff\xff\xff", planIndex);
    Serial_TJCHMI.print(buf);
    
    // 显示当前角度
    sprintf(buf, "x0.val=%d\xff\xff\xff", yaw100);
    Serial_TJCHMI.print(buf);
    
    // 显示当前任务
    if (planIndex < 6 && taskCodeValid) {
        sprintf(buf, "t4.txt=\"C:%d P:%d\"\xff\xff\xff", 
                executionPlan[planIndex].color, 
                executionPlan[planIndex].targetPos);
        Serial_TJCHMI.print(buf);
    }
}

void process_vision_data() {
    while (MaixSerial.available() >= 7) {
        uint8_t header = MaixSerial.read();
        if (header == 0xAA) {
            uint8_t color = MaixSerial.read();
            int16_t cx = MaixSerial.read() | (MaixSerial.read() << 8);
            int16_t cy = MaixSerial.read() | (MaixSerial.read() << 8);
            uint8_t tail = MaixSerial.read();
            
            if (tail == 0xBB) {
                dx = (int8_t)(cx - 160);  // 假设图像中心是160
                dy = (int8_t)(cy - 120);  // 假设图像中心是120
                vision_updated = true;
            }
        }
    }
}

bool allMotorsStopped() {
    return (M1_stepper.distanceToGo() == 0 &&
            M2_stepper.distanceToGo() == 0 &&
            M3_stepper.distanceToGo() == 0 &&
            M4_stepper.distanceToGo() == 0);
}

void Motor_Setup(float Vm, float Accel_Time) {
    float MaxSpeed = Vm * PULSES_PER_METER;
    float Acceleration = Vm * PULSES_PER_METER / Accel_Time;
    
    M1_stepper.setMaxSpeed(MaxSpeed);
    M1_stepper.setAcceleration(Acceleration);
    M2_stepper.setMaxSpeed(MaxSpeed);
    M2_stepper.setAcceleration(Acceleration);
    M3_stepper.setMaxSpeed(MaxSpeed);
    M3_stepper.setAcceleration(Acceleration);
    M4_stepper.setMaxSpeed(MaxSpeed);
    M4_stepper.setAcceleration(Acceleration);
}

void RunMotors() {
    M1_stepper.run();
    M2_stepper.run();
    M3_stepper.run();
    M4_stepper.run();
}

void Run_forward(float distance) {
    if (fabs(distance) <= 100) {
        Motor_Setup(distance / 70, 1);
    } else if (fabs(distance) < 1000) {
        Motor_Setup(distance / 500, 1);
    } else {
        Motor_Setup(distance / 1000, 1);
    }
    
    long pulses = (long)(distance / 1000 * PULSES_PER_METER);
    M1_stepper.move(pulses * motorPolarity[0]);
    M2_stepper.move(pulses * motorPolarity[1]);
    M3_stepper.move(pulses * motorPolarity[2]);
    M4_stepper.move(pulses * motorPolarity[3]);
    RunMotors();
}

void RotateCar_toTarget(float targetRot, float Rpm, float Accel_Time) {
    Motor_Setup(Rpm, Accel_Time);
    long pulses = (long)(targetRot * turn_Pulses);
    
    M1_stepper.move(motorPolarity[0] * pulses);
    M2_stepper.move(motorPolarity[1] * (-pulses));
    M3_stepper.move(motorPolarity[2] * pulses);
    M4_stepper.move(motorPolarity[3] * (-pulses));
    RunMotors();
}

void Run_PID_forward(float x) {
    float Vm = fabs(x) / 100;
    Motor_Setup(Vm, 1);
    
    // 根据任务阶段调整速度
    float factor = 0.6;
    if (taskIndex == 1 || taskIndex == 4) factor = 0.5;
    else if (taskIndex == 6) factor = 0.4;
    
    x = factor * x;
    long pulses = (long)(x / 1000 * PULSES_PER_METER);
    M1_stepper.move(pulses * motorPolarity[0]);
    M2_stepper.move(pulses * motorPolarity[1]);
    M3_stepper.move(pulses * motorPolarity[2]);
    M4_stepper.move(pulses * motorPolarity[3]);
    RunMotors();
}

void Run_PID_left(float y) {
    float Vm = fabs(y) / 100;
    Motor_Setup(Vm, 1);
    
    float factor = 0.6;
    if (taskIndex == 1 || taskIndex == 4 || taskIndex == 6) factor = 0.5;
    
    y = factor * y;
    long pulses = (long)(y / 1000 * PULSES_PER_METER);
    M1_stepper.move(-pulses * motorPolarity[0]);
    M2_stepper.move(pulses * motorPolarity[1]);
    M3_stepper.move(pulses * motorPolarity[2]);
    M4_stepper.move(-pulses * motorPolarity[3]);
    RunMotors();
}

void Grab_ZhuanPan_to_Storage(int color, int targetPos) {
    time1 = 0;
    cishu = 0;
    
    gripperServo.setAngle(0);
    rotateBase(-89);
    storageServo.setAngle(storage[color]);
    
    armStepper.runToNewPosition(0);
    gripperStepper.runToNewPosition(0);
    
    // 视觉定位循环
    int vision_wait_count = 0;
    do {
        uint8_t cmd[] = {0xAA, 0xEE, (uint8_t)color, 0xBB};
        MaixSerial.write(cmd, sizeof(cmd));  // 发送颜色查询
        MaixSerial.flush();
        
        while (!vision_updated && vision_wait_count < 50) {
            delay(10);
            vision_wait_count++;
            process_vision_data();
        }
        vision_updated = false;
        
        if (vision_wait_count >= 50) break;  // 超时退出
        
        delay(100);
        
        // 根据视觉误差调整位置
        if (dx > 80 && dy < 20 && dx != 160 && dy != -120) {
            Run_forward(100);
            while (!allMotorsStopped()) RunMotors();
            cishu++;
        } else if (dx < -80 && dy < 20) {
            Run_forward(-80);
            while (!allMotorsStopped()) RunMotors();
            cishu--;
        }
        
        if (cishu == 2 || cishu == -2 || time1 == 50) {
            Run_forward(-cishu * 80);
            while (!allMotorsStopped()) RunMotors();
            cishu = 0;
            time1 = 0;
        }
        
        delay(100);
        time1++;
        vision_wait_count = 0;
        
    } while ((dx > 80 || dx < -80 || dy > 80 || dy < -80) && time1 < 50);
    
    // 精确调整
    uint8_t cmd[] = {0xAA, 0xEE, (uint8_t)color, 0xBB};
    MaixSerial.write(cmd, sizeof(cmd));
    MaixSerial.flush();
    vision_wait_count = 0;
    while (!vision_updated && vision_wait_count < 50) {
        delay(10);
        vision_wait_count++;
        process_vision_data();
    }
    vision_updated = false;
    
    if (dy < -20) {
        Run_PID_left(dy * 0.6);
        while (!allMotorsStopped()) RunMotors();
    }
    
    // 抓取动作
    gripperStepper.runToNewPosition(0);
    armStepper.runToNewPosition(STEPPER_ZHUANPAN + 400);
    delay(200);
    gripperServo.setAngle(GRIPPER_CLOSE_ANGLE);
    gripperServo.wait();
    armStepper.runToNewPosition(STEPPER_ZERO);
    delay(100);
    
    rotateBase(15);
    gripperStepper.runToNewPosition(STEPPER_GRIPPER_ZHUANPAN_CENTER);
    delay(150);
    armStepper.runToNewPosition(STEPPER_STORAGE + 400);
    
    Run_forward(-cishu * 80);
    while (!allMotorsStopped()) RunMotors();
    cishu = 0;
    
    gripperServo.setAngle(-35);
    delay(100);
    armStepper.runToNewPosition(STEPPER_ZERO);
    delay(200);
}

void Grab_Storage_to_Place(int targetPos) {
    armStepper.runToNewPosition(STEPPER_ZERO);
    gripperStepper.runToNewPosition(STEPPER_GRIPPER_ZHUANPAN_CENTER);
    storageServo.setAngle(storage[currentTask.batch1_colors[targetPos - 1]]);
    gripperServo.setAngle(-35);
    delay(300);
    rotateBase(15);
    delay(300);
    armStepper.runToNewPosition(STEPPER_STORAGE);
    delay(300);
    gripperServo.setAngle(GRIPPER_CLOSE_ANGLE);
    delay(300);
    armStepper.runToNewPosition(STEPPER_ZERO);
    delay(300);
    
    rotateBase(ARM_BASE_STEPPER_ANGLE[currentTask.batch1_colors[targetPos - 1]]);
    gripperStepper.runToNewPosition(STEPPER_GRIPPER[currentTask.batch1_colors[targetPos - 1]]);
    armStepper.runToNewPosition(STEPPER_GROUND - (rounds - 1) * MATERIAL_HEIGHT);
    delay(1000);
    gripperServo.setAngle(0);
    delay(300);
    
    armStepper.runToNewPosition(STEPPER_ZERO);
    delay(500);
    gripperStepper.runToNewPosition(70);
    delay(300);
    rotateBase(ARM_BASE_STEPPER_ANGLE[0]);
}

void Grab_Ground_to_Storage() {
    storageServo.setAngle(storage[1]);
    rotateBase(ARM_BASE_STEPPER_ANGLE[currentTask.batch2_colors[planIndex - 3]]);
    gripperServo.setAngle(-35);
    gripperStepper.runToNewPosition(STEPPER_GRIPPER[currentTask.batch2_colors[planIndex - 3]]);
    armStepper.runToNewPosition(STEPPER_GROUND);
    delay(400);
    storageServo.setAngle(storage[currentTask.batch2_colors[planIndex - 3]]);
    delay(300);
    gripperServo.setAngle(GRIPPER_CLOSE_ANGLE);
    delay(500);
    armStepper.runToNewPosition(STEPPER_ZERO);
    gripperStepper.runToNewPosition(STEPPER_GRIPPER_ZHUANPAN_CENTER);
    delay(500);
    rotateBase(ARM_BASE_STEPPER_ANGLE[0]);
    delay(200);
    armStepper.runToNewPosition(STEPPER_STORAGE);
    delay(200);
    gripperServo.setAngle(-35);
    delay(500);
    armStepper.runToNewPosition(STEPPER_ZERO);
}

void rotateBase(float degrees) {
    long targetPulses = degrees * ROTATION_PULSES_PER_DEG;
    rotationStepper.moveTo(targetPulses);
    while (rotationStepper.distanceToGo() != 0) {
        rotationStepper.run();
    }
}

void rotateRelativeBase(float degree) {
    long movePulses = degree * ROTATION_PULSES_PER_DEG;
    rotationStepper.move(movePulses);
    while (rotationStepper.distanceToGo() != 0) {
        rotationStepper.run();
    }
}

void setup() {
    // 串口初始化
    Serial_TJCHMI.begin(TJCHMI_BAUDRATE);
    sprintf(str, "rest\xff\xff\xff");
    Serial_TJCHMI.print(str);
    
    while (Serial_TJCHMI.read() >= 0) {
        Serial_TJCHMI.print("page main\xff\xff\xff");
    }
    
    // 按钮初始化
    start_btn.reset();
    start_btn.attachClick([]() { startFlag = true; });
    
    // 电机初始化
    M1_stepper.setCurrentPosition(0);
    M2_stepper.setCurrentPosition(0);
    M3_stepper.setCurrentPosition(0);
    M4_stepper.setCurrentPosition(0);
    
    pinMode(M5_EN_PIN, OUTPUT);
    pinMode(M1_4_EN_PIN, OUTPUT);
    
    // IMU初始化
    IMU_Init();
    delay(100);
    
    // 扫码初始化
    Serial_QR.begin(QR_BAUDRATE);
    
    // 视觉初始化
    MaixSerial.begin(115200);
    uint8_t sendBuffer[] = {0xAA, 0xEE, 0x02, 0xBB};
    MaixSerial.write(sendBuffer, sizeof(sendBuffer));
    MaixSerial.flush();
    
    // 串口步进电机初始化
    Stepper_protocol.init(&Serial_Stepper, STEPPER_BAUDRATE);
    armStepper.init();
    gripperStepper.init();
    armStepper.set(5000, 0, 0, LUOGAN, 16);
    gripperStepper.set(1000, 1000, 1, CHILUN, 256);
    gripperStepper.runToNewPosition(STEPPER_GRIPPER_ZHUANPAN_CENTER);
    
    // 底部旋转轴
    rotationStepper.setMaxSpeed(60000);
    rotationStepper.setAcceleration(40000);
    
    // 舵机初始化
    protocol.init(&Serial_SERVO, SERVO_BAUDRATE);
    storageServo.init();
    gripperServo.init();
    storageServo.setAngleRange(-180, 180);
    gripperServo.setAngle(-35);
    storageServo.setSpeed(300);
    gripperServo.setSpeed(500);
    
    // 使能电机
    digitalWrite(M5_EN_PIN, LOW);
    digitalWrite(M1_4_EN_PIN, LOW);
    delay(300);
    
    nowtime = millis();
    currentState = TASK_WAIT_QR;
}

void loop() {
    start_btn.tick();
    
    // 更新IMU角度
    update_imu_angle();
    
    // 处理视觉数据
    process_vision_data();
    
    // 处理二维码
    if (scanFlag == false && scanflag1) {
        while (Serial_QR.available()) {
            char incomingByte = Serial_QR.read();
            
            if (incomingByte == 0x0A) {
                dataIndex = 0;
            } else if (incomingByte == 0x0D) {
                receivedData[dataIndex] = '\0';
                dataIndex = 0;
                scanFlag = true;
                parse_task_code();
                
                if (taskCodeValid) {
                    Serial_TJCHMI.print("t1.txt=\"QROK\"\xff\xff\xff");
                    char strTemp[30];
                    sprintf(strTemp, "t3.txt=\"%s\"\xff\xff\xff", receivedData);
                    Serial_TJCHMI.print(strTemp);
                    scanflag1 = 0;
                    generate_execution_plan();
                    currentState = TASK_DEPARTURE;
                } else {
                    scanFlag = false;
                    Serial_TJCHMI.print("t1.txt=\"QRERR\"\xff\xff\xff");
                }
            } else if (dataIndex < QR_BUFFER_SIZE - 1) {
                receivedData[dataIndex++] = incomingByte;
            }
        }
    }
    
    // 更新显示
    if (millis() >= nowtime + 100) {
        nowtime = millis();
        display_task_info();
    }
    
    // 状态机
    switch (currentState) {
        case TASK_IDLE:
            if (startFlag) {
                currentState = TASK_WAIT_QR;
                startFlag = false;
            }
            break;
            
        case TASK_WAIT_QR:
            // 等待二维码解析完成
            if (taskCodeValid) {
                currentState = TASK_DEPARTURE;
            }
            break;
            
        case TASK_DEPARTURE:
            if (runFlag) {
                stateStartTime = millis();
                Motor_Setup(300, 2);
                long pulses = (long)(200.0 / 1000 * PULSES_PER_METER);
                M1_stepper.move(pulses * motorPolarity[0]);
                M4_stepper.move(pulses * motorPolarity[3]);
                RunMotors();
                runFlag = false;
            }
            
            if (allMotorsStopped() || millis() - stateStartTime > STATE_TIMEOUT) {
                runFlag = true;
                currentState = TASK_GRAB_FROM_TABLE;
            }
            break;
            
        case TASK_GRAB_FROM_TABLE:
            if (planIndex < 6) {
                if (runFlag) {
                    stateStartTime = millis();
                    Grab_ZhuanPan_to_Storage(
                        executionPlan[planIndex].color,
                        executionPlan[planIndex].targetPos
                    );
                    runFlag = false;
                }
                
                if (millis() - stateStartTime > STATE_TIMEOUT) {
                    runFlag = true;
                    planIndex++;
                    currentState = TASK_GRAB_FROM_TABLE;
                } else if (allMotorsStopped()) {
                    runFlag = true;
                    planIndex++;
                    currentState = TASK_GRAB_FROM_TABLE;
                }
            } else {
                currentState = TASK_RETURN_HOME;
            }
            break;
            
        case TASK_RETURN_HOME:
            if (runFlag) {
                stateStartTime = millis();
                rotateBase(0);
                armStepper.runToNewPosition(STEPPER_ZERO);
                gripperStepper.runToNewPosition(STEPPER_GRIPPER_ZHUANPAN_CENTER);
                runFlag = false;
            }
            
            if (allMotorsStopped() && 
                rotationStepper.distanceToGo() == 0 &&
                millis() - stateStartTime > 1000) {
                currentState = TASK_COMPLETE;
            }
            break;
            
        case TASK_COMPLETE:
            // 任务完成，等待
            break;
            
        default:
            break;
    }
    
    // 运行所有电机
    M1_stepper.run();
    M2_stepper.run();
    M3_stepper.run();
    M4_stepper.run();
    rotationStepper.run();
}
