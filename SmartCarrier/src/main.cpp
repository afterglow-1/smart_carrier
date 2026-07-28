#include <Arduino.h>
#include <AccelStepper.h>
#include <FashionStar_SmartGripper.h>
#include <JY901.h>
#include <OneButton.h>

#ifndef CARRIER_START_ZONE
#define CARRIER_START_ZONE 1
#endif

static_assert(CARRIER_START_ZONE == 1 || CARRIER_START_ZONE == 2,
              "CARRIER_START_ZONE must be 1 or 2");

/*
 * SmartCarrier 两批物料全自动底盘路线
 * ================================================================
 * 默认路线：
 * 启停区 -> 二维码板 -> 原料区 -> 粗加工区 -> 暂存区
 *        -> 原料区 -> 粗加工区 -> 暂存区 -> 原启停区
 *
 * 一、坐标与方向约定
 * ---------------------------------------------------------------
 * FORWARD/BACKWARD/LEFT/RIGHT均为“车体坐标方向”，会随车头旋转。
 * TURN_CCW/TURN_CW表示从上往下看逆/顺时针原地旋转，数值单位为度。
 *
 *   Direction::FORWARD  ：沿当前车头方向前进
 *   Direction::BACKWARD ：沿当前车尾方向后退
 *   Direction::LEFT     ：相对当前车头向左横移
 *   Direction::RIGHT    ：相对当前车头向右横移
 *   Direction::TURN_CCW ：原地逆时针旋转指定角度
 *   Direction::TURN_CW  ：原地顺时针旋转指定角度
 *
 * 场地左上角可视为坐标原点(0,0)，向右为X正方向，向下为Y正方向。
 * 规划路线使用的中心线坐标为150、1200、2250 mm。
 *
 * 二、如何修改路线
 * ---------------------------------------------------------------
 * 实际运动路径全部写在下方的 ROUTE[] 数组中。每一行代表一段运动：
 *
 *   {运动方向, 距离(mm)/角度(deg), 到达点类型, "串口显示名称"}
 *
 * 示例：
 *   {Direction::LEFT, 800, StopAction::NONE, "new waypoint"}
 * 表示小车向左横移800 mm，到达后仅作为普通路径点继续运行。
 *
 * 修改距离或旋转角度：只修改数组中的第二个数字。
 * 修改方向：使用FORWARD/BACKWARD/LEFT/RIGHT/TURN_CCW/TURN_CW之一。
 * 增加路线：在ROUTE[]中需要的位置插入一行。
 * 旋转示例：
 *   {Direction::TURN_CCW, 90, StopAction::NONE, "turn CCW 90 deg"}
 *   {Direction::TURN_CW, 90, StopAction::NONE, "turn CW 90 deg"}
 * 删除路线：删除对应的一整行。
 * 调整停顿时间：修改CHECKPOINT_DWELL_MS。
 *
 * StopAction目前只用于串口说明和识别终点，不控制二维码或机械臂：
 *   NONE   = 普通路径点
 *   FINISH = 最后一段，抵达后关闭底盘驱动器
 *   其他值 = 作业区标记；底盘仍只停顿CHECKPOINT_DWELL_MS后继续
 *
 * 三、启动与安全
 * ---------------------------------------------------------------
 * 上电后程序只初始化电机和HWT101，四轮驱动器保持关闭并等待启动按钮。
 * 单击PB9启动按钮后从路线第1段开始；运行中单击则受控减速停车。
 * 安全停车后再次单击，会将路线状态清零并从第1段重新开始。
 * 调试串口：PB12(RX)、PB13(TX)、115200波特率。
 * 运行过程中发送字符x或X会立即拉高PE13，关闭四轮驱动器。
 */

constexpr uint32_t DEBUG_RX = PB12;
constexpr uint32_t DEBUG_TX = PB13;
constexpr uint32_t DEBUG_BAUD = 115200;

// HWT101串口设置，与examples/HWT101.ino保持一致。
constexpr uint32_t HWT_UART_RX_PIN = PD9;
constexpr uint32_t HWT_UART_TX_PIN = PD8;
constexpr uint32_t HWT_UART_BAUD = 115200;

// 主夹爪串口与绝对角度配置。
constexpr uint32_t GRIPPER_UART_RX_PIN = PC7;
constexpr uint32_t GRIPPER_UART_TX_PIN = PC6;
constexpr uint32_t GRIPPER_UART_BAUD = 115200;
constexpr uint8_t GRIPPER_SERVO_ID = 4;
// 根据当前实车安装方向标定：-90°为张开，0°为夹紧。
// 舵机到达夹紧角度后停止位移，由内部位置环和功率限制维持夹持力。
constexpr float GRIPPER_OPEN_ANGLE_DEG = -90.0F;
constexpr float GRIPPER_CLOSE_ANGLE_DEG = 0.0F;
constexpr uint16_t GRIPPER_MOVE_INTERVAL_MS = 800;
constexpr uint16_t GRIPPER_MAX_POWER_MW = 400;
constexpr uint32_t GRIPPER_ACTION_WAIT_MS = 1000;

constexpr uint32_t DRIVE_ENABLE_PIN = PE13;
// 机械臂底部M5旋转轴：PE10低电平使能，PE15方向，PB11脉冲。
constexpr uint32_t ROTATE_ENABLE_PIN = PE10;
constexpr uint32_t ARM_BASE_DIR_PIN = PE15;
constexpr uint32_t ARM_BASE_STEP_PIN = PB11;
// 与Car_2和final原车程序一致：V2/V3控制板启动按钮使用PB9，按下接GND。
// 早期V1资料曾使用PB8；如果你的实际控制板确为V1，只需把PB9改回PB8。
constexpr uint32_t START_BUTTON_PIN = PB9;

constexpr uint32_t M1_DIR_PIN = PD6;
constexpr uint32_t M1_STEP_PIN = PD4;
constexpr uint32_t M2_DIR_PIN = PE9;
constexpr uint32_t M2_STEP_PIN = PE11;
constexpr uint32_t M3_DIR_PIN = PD14;
constexpr uint32_t M3_STEP_PIN = PD15;
constexpr uint32_t M4_DIR_PIN = PC3_C;
constexpr uint32_t M4_STEP_PIN = PA1;

constexpr uint8_t MOTOR_INTERFACE_TYPE = AccelStepper::DRIVER;
// 四个数分别对应M1、M2、M3、M4。负号用于补偿左右电机安装方向相反。
// 如果某个轮子单独反转，只修改对应位置的1/-1，不要改路线方向。
constexpr int8_t MOTOR_POLARITY[4] = {-1, 1, -1, 1};

// 距离标定：沿用FSM_SQUARE.ino中的10000脉冲/米。
// 如果命令走1000 mm但实际只走900 mm：
// 新值 = 旧值 * 1000 / 900。
// 修改后应分别标定前后移动和左右横移；若两者误差不同，建议拆成两个系数。
constexpr float PULSES_PER_METER = 10000.0F;
constexpr float PULSES_PER_MM = PULSES_PER_METER / 1000.0F;

// 机械臂底部旋转轴：200步/圈、16细分、最新减速比5:1。
// 输出轴90°所需脉冲 = 200 * 16 * 5 * 90 / 360 = 4000。
constexpr float ARM_BASE_GEAR_RATIO = 5.0F;
constexpr long ARM_BASE_MOTOR_PULSES_PER_REV = 200L * 16L;
constexpr long ARM_BASE_90_DEG_PULSES =
    static_cast<long>(ARM_BASE_MOTOR_PULSES_PER_REV *
                      ARM_BASE_GEAR_RATIO * 90.0F / 360.0F);
constexpr float ARM_BASE_MAX_SPEED_PPS = 1000.0F;
constexpr float ARM_BASE_ACCELERATION_PPS2 = 500.0F;
// 若首次测试发现正脉冲实际为逆时针，将1改成-1。
constexpr int8_t ARM_BASE_CW_SIGN = -1;

// AccelStepper使用的单位是“脉冲/秒”和“脉冲/秒²”，不是RPM。
// MAX_SPEED越大，最高速度越高；ACCELERATION越大，加减速越急。
constexpr float MAX_SPEED_PULSES_PER_SECOND = 5000.0F;
constexpr float ACCELERATION_PULSES_PER_SECOND2 = 2000.0F;
// 按钮停车使用更大的受控减速度，使最高速下停车距离约为0.21 m。
constexpr float SAFE_STOP_DECELERATION_PPS2 = 6000.0F;

// 航向闭环参数：纠偏量的单位为“轮脉冲/秒”。
// 如果调试串口显示纠偏后|error|持续增大，将SIGN改成+1，或测试时发送字符i。
constexpr float YAW_KP = 45.0F;
constexpr float YAW_DEADBAND_DEG = 0.30F;
constexpr float YAW_MAX_CORRECTION_PPS = 900.0F;
constexpr float YAW_CORRECTION_SIGN = -1.0F;
// HWT101正角度对应从上往下看逆时针时保持+1；若实际总是顺时针转，改为-1。
constexpr float CCW_YAW_SIGN = 1.0F;
// 原地旋转闭环参数。角度误差越大转速越高，接近目标时自动减速。
constexpr float TURN_KP_PPS_PER_DEG = 28.0F;
constexpr float TURN_MIN_SPEED_PPS = 300.0F;
constexpr float TURN_MAX_SPEED_PPS = 1500.0F;
constexpr float TURN_TOLERANCE_DEG = 1.0F;
constexpr uint32_t IMU_STARTUP_WAIT_MS = 2000;
constexpr uint32_t IMU_TIMEOUT_MS = 500;
constexpr uint32_t IMU_DEBUG_INTERVAL_MS = 200;
// 每完成一段路线后的停车时间。500表示停车0.5秒再开始下一段。
constexpr uint32_t CHECKPOINT_DWELL_MS = 500;

HardwareSerial Serial_DEBUG(DEBUG_RX, DEBUG_TX);
HardwareSerial Serial_GRIPPER(GRIPPER_UART_RX_PIN, GRIPPER_UART_TX_PIN);
FSUS_Protocol gripperProtocol(&Serial_GRIPPER, GRIPPER_UART_BAUD);
FSUS_Servo gripperServo(GRIPPER_SERVO_ID, &gripperProtocol);
OneButton startButton;

AccelStepper motor1(MOTOR_INTERFACE_TYPE, M1_STEP_PIN, M1_DIR_PIN);
AccelStepper motor2(MOTOR_INTERFACE_TYPE, M2_STEP_PIN, M2_DIR_PIN);
AccelStepper motor3(MOTOR_INTERFACE_TYPE, M3_STEP_PIN, M3_DIR_PIN);
AccelStepper motor4(MOTOR_INTERFACE_TYPE, M4_STEP_PIN, M4_DIR_PIN);
AccelStepper armBaseStepper(
    MOTOR_INTERFACE_TYPE, ARM_BASE_STEP_PIN, ARM_BASE_DIR_PIN);

enum class Direction : uint8_t {
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
  TURN_CCW,
  TURN_CW
};

enum class StopAction : uint8_t {
  NONE,                      // 普通路径点
  QR_SCAN,                   // 二维码板位置标记
  PICK_BATCH_1,              // 第一批原料区位置标记
  PROCESS_AND_LOAD_BATCH_1,  // 第一批粗加工区位置标记
  STORE_BATCH_1,             // 第一批暂存区位置标记
  PICK_BATCH_2,              // 第二批原料区位置标记
  PROCESS_AND_LOAD_BATCH_2,  // 第二批粗加工区位置标记
  STORE_BATCH_2,             // 第二批暂存区位置标记
  ARM_BASE_RETURN_HOME,      // 小车在暂存区完成转向后，机械臂底座逆时针回零
  FINISH                     // 整条路线终点
};

// 一段路线的数据结构。ROUTE[]中的每一行都按这个顺序填写。
struct RouteSegment {
  Direction direction;       // 本段运动方向
  uint16_t amount;           // 平移段单位mm；TURN_CCW/TURN_CW段单位为度
  StopAction actionAtEnd;    // 到达后的点位类型
  const char *destination;   // 串口监视器显示的目的地名称
};

// 启停区1位于二维码板上方，启停区2位于二维码板下方。
// CARRIER_ROUTE.ino默认CARRIER_START_ZONE=1。
// CARRIER_ROUTE_START2.ino在包含本文件前把它定义为2。
#if CARRIER_START_ZONE == 1
constexpr Direction START_TO_QR_DIRECTION = Direction::BACKWARD;
// 当前旋转组合结束后车头朝图纸下方：返回上方启停区1应使用车体BACKWARD。
constexpr Direction RIGHT_LANE_TO_START_DIRECTION = Direction::BACKWARD;
constexpr const char *START_ZONE_NAME = "start zone 1";
#else
constexpr Direction START_TO_QR_DIRECTION = Direction::FORWARD;
// 当前旋转组合结束后车头朝图纸下方：返回下方启停区2应使用车体FORWARD。
constexpr Direction RIGHT_LANE_TO_START_DIRECTION = Direction::FORWARD;
constexpr const char *START_ZONE_NAME = "start zone 2";
#endif

/*
 * 路线数组：这是修改运动路径时最主要的区域。
 *
 * 小车中心沿x/y=150、1200、2250 mm的中心线运动。
 * 如果车体投影为300×300 mm，在400 mm宽的中央通道中理论上两侧各余50 mm。
 *
 * 路线段之间均为相对运动：每一段距离都从上一段的终点开始计算。
 * 例如连续写两段FORWARD 500 mm，效果等同于总共前进1000 mm，
 * 但中间会按照CHECKPOINT_DWELL_MS停车一次。
 */
constexpr RouteSegment ROUTE[] = {
    // 1. 从启停区向车体右侧横移50 mm
    //    RIGHT是麦克纳姆轮横移，不是向前或向后运动
    {Direction::LEFT, 100, StopAction::NONE, "start right shift"},

    // 2. 横移完成后，沿原方向驶向二维码板
    //    前一段与本段垂直，因此到二维码板的纵向距离仍为原来的1050 mm
    {START_TO_QR_DIRECTION, 1050, StopAction::QR_SCAN, "QR board"},

    // 3. 从右侧二维码板沿中央横向通道左移到场地中心
    {Direction::LEFT, 960, StopAction::NONE, "field center"},

    // 4. 第一批到达原料区后，从上往下看逆时针旋转90°
    {Direction::TURN_CCW, 90, StopAction::NONE, "raw area CCW 90 deg (batch 1)"},

    // 5. 从场地中心沿中央纵向通道向上，到第一批原料区
    {Direction::RIGHT, 990, StopAction::PICK_BATCH_1, "raw material area (batch 1)"},

    // 7.1 车头已朝向图纸右侧；车体RIGHT对应场地图纸向上，到MID
    {Direction::LEFT, 30, StopAction::NONE,"MID"},

    // 6. 原料区停留后，继续逆时针旋转180°
    {Direction::TURN_CCW, 180, StopAction::NONE, "rough area CCW 180 deg (batch 1)"},

    // 7.2 车头已朝向图纸右侧；车体RIGHT对应场地图纸向上，到粗加工区
    {Direction::RIGHT, 1930, StopAction::PROCESS_AND_LOAD_BATCH_1,
     "rough processing area (batch 1)"},

    // 8. 车头已朝向图纸右侧；车体LEFT对应图纸向下，回到场地中心
    {Direction::LEFT, 960, StopAction::NONE, "field center"},

    // 9. 第一批到达中心后，从上往下看顺时针旋转90°，车头朝图纸下方
    {Direction::TURN_CW, 90, StopAction::NONE, "storage CW 90 deg (batch 1)"},

    // 10. 车体BACKWARD对应图纸向左，到第一批暂存区
    {Direction::RIGHT, 950, StopAction::STORE_BATCH_1,
     "temporary storage (batch 1)"},

    // 11. 车头朝下时，车体LEFT对应图纸向右，回到场地中心
    {Direction::LEFT, 900, StopAction::NONE, "field center"},

    // 12. 第二批到达原料区后，逆时针旋转90°
    {Direction::TURN_CW, 90, StopAction::NONE, "raw area CCW 90 deg (batch 2)"},

    // 13. 车头朝下时，车体BACKWARD对应图纸向上，第二次到原料区
    {Direction::RIGHT, 900, StopAction::PICK_BATCH_2,
     "raw material area (batch 2)"},

    // 14. 第二批到达粗加工区后，逆时针旋转180°
    {Direction::TURN_CCW, 180, StopAction::NONE, "rough area CCW 180 deg (batch 2)"},

    // 15. 此时车头朝图纸右侧；车体RIGHT对应图纸向下，到粗加工区
    {Direction::RIGHT, 1800, StopAction::PROCESS_AND_LOAD_BATCH_2,
     "rough processing area (batch 2)"},

    // 16. 此时车头朝图纸左侧；车体RIGHT对应图纸向上，回到场地中心
    {Direction::LEFT, 900, StopAction::NONE, "field center"},

    // 17. 第二批到达暂存区后，先让小车完成原地转向；
    //     本段结束时再触发机械臂底座逆时针90°回零
    {Direction::TURN_CW, 90, StopAction::NONE,
     "storage CW 90 deg (batch 2)"},

    // 18. 车头朝左时，车体FORWARD对应图纸向左，到第二批暂存区
    {Direction::RIGHT, 900, StopAction::STORE_BATCH_2,
     "temporary storage (batch 2)"},

    // 19. 两批完成，从左侧暂存区横穿场地到右侧通道
    {Direction::LEFT, 2000, StopAction::ARM_BASE_RETURN_HOME, "right-side lane"},

    // 20. 此时车头朝图纸下方：启停区1用BACKWARD向上返回，
    //     启停区2用FORWARD向下返回。FINISH会关闭四轮驱动器
    {RIGHT_LANE_TO_START_DIRECTION, 1000, StopAction::FINISH, START_ZONE_NAME},
};

// 自动计算路线段数量。增删ROUTE[]内容时，不需要手工修改这个数。
constexpr size_t ROUTE_COUNT = sizeof(ROUTE) / sizeof(ROUTE[0]);

enum class ProgramState : uint8_t {
  GRIPPER_OPENING,   // 上电或安全停车后，夹爪正在相对张开90°
  WAITING_TO_CLAMP, // 夹爪已张开，等待第一次单击执行夹紧
  GRIPPER_CLOSING,  // 第一次单击后，夹爪正在回到上电时记录的夹紧原点
  WAITING_TO_START, // 夹爪已夹紧，等待第二次单击启动小车
  MOVING,           // 四个电机正在执行当前路线段
  ARM_BASE_MOVING,  // 底盘停车，机械臂底部旋转轴正在执行90°动作
  CHECKPOINT_DWELL, // 已到达路线点，停车等待CHECKPOINT_DWELL_MS
  SAFE_STOPPING,    // 收到按钮单击，正在受控减速停车
  FINISHED,         // 整条路线完成，驱动器已关闭
  EMERGENCY_STOP    // 收到x，紧急关闭驱动器
};

ProgramState programState = ProgramState::WAITING_TO_CLAMP;
size_t routeIndex = 0;
uint32_t checkpointStartTime = 0;
bool armBaseStopRequested = false;
bool gripperReady = false;
uint32_t gripperActionStartTime = 0;

void beginGripperOpenCycle();

// 以下方向均为乘MOTOR_POLARITY之前的“逻辑轮方向”。
constexpr int8_t ROTATION_SIGNS[4] = {1, -1, 1, -1};
int8_t translationSigns[4] = {0, 0, 0, 0};
long segmentStartLogicalPosition[4] = {0, 0, 0, 0};
long segmentTargetPulses = 0;
float segmentBaseSpeedPps = 0.0F;
uint32_t lastMotionUpdateUs = 0;

float currentYawDeg = 0.0F;
float targetYawDeg = 0.0F;
float unwrappedYawDeg = 0.0F;
float lastWrappedYawDeg = 0.0F;
float turnTargetUnwrappedYawDeg = 0.0F;
float headingErrorDeg = 0.0F;
float headingCorrectionPps = 0.0F;
float correctionSign = YAW_CORRECTION_SIGN;
float rotationDriveSign = 0.0F;
bool yawUnwrapInitialized = false;
bool segmentIsRotation = false;
bool imuReady = false;
uint32_t lastImuDataMs = 0;
uint32_t lastImuDebugMs = 0;
int lastRawButtonLevel = HIGH;

const char *directionName(Direction direction) {
  switch (direction) {
    case Direction::FORWARD:
      return "FORWARD";
    case Direction::BACKWARD:
      return "BACKWARD";
    case Direction::LEFT:
      return "LEFT";
    case Direction::RIGHT:
      return "RIGHT";
    case Direction::TURN_CCW:
      return "TURN_CCW";
    case Direction::TURN_CW:
      return "TURN_CW";
  }
  return "UNKNOWN";
}

const char *checkpointName(StopAction action) {
  switch (action) {
    case StopAction::QR_SCAN:
      return "QR board";
    case StopAction::PICK_BATCH_1:
      return "raw material area (batch 1)";
    case StopAction::PROCESS_AND_LOAD_BATCH_1:
      return "rough processing area (batch 1)";
    case StopAction::STORE_BATCH_1:
      return "temporary storage (batch 1)";
    case StopAction::PICK_BATCH_2:
      return "raw material area (batch 2)";
    case StopAction::PROCESS_AND_LOAD_BATCH_2:
      return "rough processing area (batch 2)";
    case StopAction::STORE_BATCH_2:
      return "temporary storage (batch 2)";
    case StopAction::ARM_BASE_RETURN_HOME:
      return "arm base return home";
    case StopAction::FINISH:
      return "Route complete";
    case StopAction::NONE:
      return "";
  }
  return "";
}

// 控制四轮驱动器公共使能脚。
// 本项目PE13为低电平使能、高电平关闭。
void setDriverEnabled(bool enabled) {
  digitalWrite(DRIVE_ENABLE_PIN, enabled ? LOW : HIGH);
}

// 机械臂底部M5驱动器同样为低电平使能。
void setArmBaseEnabled(bool enabled) {
  digitalWrite(ROTATE_ENABLE_PIN, enabled ? LOW : HIGH);
}

// 给每个AccelStepper对象设置相同的最大速度、加速度和软件零点。
void configureMotor(AccelStepper &motor) {
  motor.setMaxSpeed(MAX_SPEED_PULSES_PER_SECOND);
  motor.setAcceleration(ACCELERATION_PULSES_PER_SECOND2);
  motor.setCurrentPosition(0);
}

// AccelStepper::runSpeed()每次最多产生一个步进脉冲，必须在loop()中持续调用。
// 不能在MOVING状态中加入长时间delay，否则步进脉冲会中断、速度会异常。
// 把任意角度限制到[-180, 180)，用于正确处理179度到-179度的跨界。
float wrapAngleDeg(float angle) {
  while (angle >= 180.0F) {
    angle -= 360.0F;
  }
  while (angle < -180.0F) {
    angle += 360.0F;
  }
  return angle;
}

// 非阻塞读取HWT101。只要串口有数据就立刻交给JY901解析器。
// 本函数必须在loop()中高频调用，不能在其前后加入长时间delay。
void updateImu() {
  bool received = false;
  while (Serial_WTIMU.available() > 0) {
    JY901.CopeSerialData(Serial_WTIMU.read());
    received = true;
  }

  if (received) {
    const float newYawDeg =
        static_cast<float>(JY901.stcAngle.Angle[2]) / 32768.0F * 180.0F;
    if (!yawUnwrapInitialized) {
      unwrappedYawDeg = newYawDeg;
      lastWrappedYawDeg = newYawDeg;
      yawUnwrapInitialized = true;
    } else {
      unwrappedYawDeg += wrapAngleDeg(newYawDeg - lastWrappedYawDeg);
      lastWrappedYawDeg = newYawDeg;
    }
    currentYawDeg = newYawDeg;
    lastImuDataMs = millis();
    imuReady = true;
  }
}

bool imuTimedOut() {
  return !imuReady || (millis() - lastImuDataMs > IMU_TIMEOUT_MS);
}

// 返回某个电机在“逻辑轮方向”中的累计位置。
long logicalMotorPosition(uint8_t index) {
  switch (index) {
    case 0:
      return motor1.currentPosition() * MOTOR_POLARITY[0];
    case 1:
      return motor2.currentPosition() * MOTOR_POLARITY[1];
    case 2:
      return motor3.currentPosition() * MOTOR_POLARITY[2];
    default:
      return motor4.currentPosition() * MOTOR_POLARITY[3];
  }
}

// 平移进度=四个轮位移在当前平移方向上的投影平均值。
// 旋转纠偏向量与四种平移向量正交，所以纠偏脉冲不会被计入行驶距离。
float segmentProgressPulses() {
  float sum = 0.0F;
  for (uint8_t i = 0; i < 4; ++i) {
    const long displacement =
        logicalMotorPosition(i) - segmentStartLogicalPosition[i];
    sum += static_cast<float>(displacement * translationSigns[i]);
  }
  return sum * 0.25F;
}

void setLogicalWheelSpeeds(float m1, float m2, float m3, float m4) {
  motor1.setSpeed(m1 * MOTOR_POLARITY[0]);
  motor2.setSpeed(m2 * MOTOR_POLARITY[1]);
  motor3.setSpeed(m3 * MOTOR_POLARITY[2]);
  motor4.setSpeed(m4 * MOTOR_POLARITY[3]);
}

void stopWheelPulses() {
  setLogicalWheelSpeeds(0.0F, 0.0F, 0.0F, 0.0F);
  segmentBaseSpeedPps = 0.0F;
  headingCorrectionPps = 0.0F;
}

// runSpeed()按当前实时速度产生脉冲，不会强制四轮回到相同终点，
// 因此已经产生的航向纠偏不会在路段末尾被抵消。
void runAllMotorsAtCurrentSpeed() {
  motor1.runSpeed();
  motor2.runSpeed();
  motor3.runSpeed();
  motor4.runSpeed();
}

// 计算P航向控制并同时执行加减速和四轮脉冲输出。
void updateClosedLoopMotion() {
  const uint32_t nowUs = micros();
  float dt = static_cast<float>(nowUs - lastMotionUpdateUs) * 0.000001F;
  lastMotionUpdateUs = nowUs;
  if (dt > 0.05F) {
    dt = 0.05F;
  }

  const float progress = segmentProgressPulses();
  const float remaining =
      max(0.0F, static_cast<float>(segmentTargetPulses) - progress);

  // 根据剩余距离计算可安全刹停的速度，并用软件斜坡限制加速度。
  const float brakingSpeed =
      sqrtf(2.0F * ACCELERATION_PULSES_PER_SECOND2 * remaining);
  const float desiredBaseSpeed =
      min(MAX_SPEED_PULSES_PER_SECOND, brakingSpeed);
  const float speedStep = ACCELERATION_PULSES_PER_SECOND2 * dt;
  if (segmentBaseSpeedPps < desiredBaseSpeed) {
    segmentBaseSpeedPps =
        min(desiredBaseSpeed, segmentBaseSpeedPps + speedStep);
  } else {
    segmentBaseSpeedPps =
        max(desiredBaseSpeed, segmentBaseSpeedPps - speedStep);
  }

  headingErrorDeg = wrapAngleDeg(targetYawDeg - currentYawDeg);
  if (fabsf(headingErrorDeg) <= YAW_DEADBAND_DEG) {
    headingCorrectionPps = 0.0F;
  } else {
    headingCorrectionPps =
        constrain(YAW_KP * headingErrorDeg * correctionSign,
                  -YAW_MAX_CORRECTION_PPS, YAW_MAX_CORRECTION_PPS);
  }

  float wheelSpeed[4];
  float largestMagnitude = 0.0F;
  for (uint8_t i = 0; i < 4; ++i) {
    wheelSpeed[i] = translationSigns[i] * segmentBaseSpeedPps +
                    ROTATION_SIGNS[i] * headingCorrectionPps;
    largestMagnitude = max(largestMagnitude, fabsf(wheelSpeed[i]));
  }

  // 保持四轮速度比例，同时保证没有轮子超过AccelStepper最大速度。
  if (largestMagnitude > MAX_SPEED_PULSES_PER_SECOND) {
    const float scale = MAX_SPEED_PULSES_PER_SECOND / largestMagnitude;
    for (uint8_t i = 0; i < 4; ++i) {
      wheelSpeed[i] *= scale;
    }
  }

  setLogicalWheelSpeeds(
      wheelSpeed[0], wheelSpeed[1], wheelSpeed[2], wheelSpeed[3]);
  runAllMotorsAtCurrentSpeed();

}

// 使用HWT101的连续yaw完成原地定角旋转。
// unwrappedYawDeg消除了179°跳到-179°的问题，因此顺/逆时针180°都能按指定方向执行。
bool updateRotationMotion() {
  const uint32_t nowUs = micros();
  float dt = static_cast<float>(nowUs - lastMotionUpdateUs) * 0.000001F;
  lastMotionUpdateUs = nowUs;
  if (dt > 0.05F) {
    dt = 0.05F;
  }

  const float turnErrorDeg = turnTargetUnwrappedYawDeg - unwrappedYawDeg;
  headingErrorDeg = turnErrorDeg;
  if (fabsf(turnErrorDeg) <= TURN_TOLERANCE_DEG) {
    stopWheelPulses();
    return true;
  }

  const float desiredSpeed =
      constrain(fabsf(turnErrorDeg) * TURN_KP_PPS_PER_DEG,
                TURN_MIN_SPEED_PPS, TURN_MAX_SPEED_PPS);
  const float speedStep = ACCELERATION_PULSES_PER_SECOND2 * dt;
  if (segmentBaseSpeedPps < desiredSpeed) {
    segmentBaseSpeedPps =
        min(desiredSpeed, segmentBaseSpeedPps + speedStep);
  } else {
    segmentBaseSpeedPps =
        max(desiredSpeed, segmentBaseSpeedPps - speedStep);
  }

  // correctionSign沿用直线纠偏已经验证过的电机旋转极性。
  const float requestedDriveSign =
      (turnErrorDeg >= 0.0F ? 1.0F : -1.0F) * correctionSign;
  if (rotationDriveSign != 0.0F &&
      requestedDriveSign != rotationDriveSign) {
    // 越过目标时先把速度降为零，再反向微调，避免高速反接。
    segmentBaseSpeedPps = 0.0F;
  }
  rotationDriveSign = requestedDriveSign;
  headingCorrectionPps = rotationDriveSign * segmentBaseSpeedPps;

  setLogicalWheelSpeeds(
      ROTATION_SIGNS[0] * headingCorrectionPps,
      ROTATION_SIGNS[1] * headingCorrectionPps,
      ROTATION_SIGNS[2] * headingCorrectionPps,
      ROTATION_SIGNS[3] * headingCorrectionPps);
  runAllMotorsAtCurrentSpeed();
  return false;
}

// 第一次到达原料区时转到顺时针90°绝对位置；
// 最后一次到达暂存区时回到软件零点，即逆时针90°。
// 使用绝对目标可避免一次安全停车后重新启动时重复累加90°。
void startArmBaseMove(bool moveClockwiseTo90Deg) {
  stopWheelPulses();
  armBaseStopRequested = false;
  setArmBaseEnabled(true);

  const long targetPulses =
      moveClockwiseTo90Deg
          ? static_cast<long>(ARM_BASE_CW_SIGN) * ARM_BASE_90_DEG_PULSES
          : 0L;
  armBaseStepper.moveTo(targetPulses);

  Serial_DEBUG.print("Arm base: ");
  Serial_DEBUG.print(moveClockwiseTo90Deg ? "CW 90 deg" : "CCW 90 deg to home");
  Serial_DEBUG.print(", target pulses=");
  Serial_DEBUG.println(targetPulses);
  programState = ProgramState::ARM_BASE_MOVING;
}

void completeSafeStop() {
  stopWheelPulses();
  segmentIsRotation = false;
  setArmBaseEnabled(false);
  armBaseStopRequested = false;
  setDriverEnabled(false);
  routeIndex = 0;
  Serial_DEBUG.println(
      "Safe stop complete. Route reset; reopening gripper for the two-click restart.");
  beginGripperOpenCycle();
}

void updateArmBaseMotion() {
  armBaseStepper.run();
  if (armBaseStepper.distanceToGo() != 0) {
    return;
  }

  setArmBaseEnabled(false);

  if (armBaseStopRequested) {
    completeSafeStop();
    return;
  }

  Serial_DEBUG.print("Arm base: motion complete, position=");
  Serial_DEBUG.println(armBaseStepper.currentPosition());
  checkpointStartTime = millis();
  programState = ProgramState::CHECKPOINT_DWELL;
}

// 按钮停车不直接切断驱动器，而是沿当前平移方向逐步降低四轮速度。
// 减速过程中保留有限的航向纠偏；纠偏上限随平移速度下降，避免停车末段原地旋转。
void updateSafeStopping() {
  const uint32_t nowUs = micros();
  float dt = static_cast<float>(nowUs - lastMotionUpdateUs) * 0.000001F;
  lastMotionUpdateUs = nowUs;
  if (dt > 0.05F) {
    dt = 0.05F;
  }

  segmentBaseSpeedPps =
      max(0.0F, segmentBaseSpeedPps - SAFE_STOP_DECELERATION_PPS2 * dt);

  if (segmentIsRotation) {
    headingCorrectionPps = rotationDriveSign * segmentBaseSpeedPps;
    setLogicalWheelSpeeds(
        ROTATION_SIGNS[0] * headingCorrectionPps,
        ROTATION_SIGNS[1] * headingCorrectionPps,
        ROTATION_SIGNS[2] * headingCorrectionPps,
        ROTATION_SIGNS[3] * headingCorrectionPps);
    runAllMotorsAtCurrentSpeed();
    if (segmentBaseSpeedPps <= 1.0F) {
      completeSafeStop();
    }
    return;
  }

  headingErrorDeg = wrapAngleDeg(targetYawDeg - currentYawDeg);
  const float stopCorrectionLimit =
      min(YAW_MAX_CORRECTION_PPS, segmentBaseSpeedPps * 0.30F);
  if (fabsf(headingErrorDeg) <= YAW_DEADBAND_DEG) {
    headingCorrectionPps = 0.0F;
  } else {
    headingCorrectionPps =
        constrain(YAW_KP * headingErrorDeg * correctionSign,
                  -stopCorrectionLimit, stopCorrectionLimit);
  }

  float wheelSpeed[4];
  for (uint8_t i = 0; i < 4; ++i) {
    wheelSpeed[i] = translationSigns[i] * segmentBaseSpeedPps +
                    ROTATION_SIGNS[i] * headingCorrectionPps;
  }
  setLogicalWheelSpeeds(
      wheelSpeed[0], wheelSpeed[1], wheelSpeed[2], wheelSpeed[3]);
  runAllMotorsAtCurrentSpeed();

  if (segmentBaseSpeedPps <= 1.0F) {
    completeSafeStop();
  }
}

const char *programStateName() {
  switch (programState) {
    case ProgramState::GRIPPER_OPENING:
      return "GRIPPER_OPEN";
    case ProgramState::WAITING_TO_CLAMP:
      return "WAIT_CLAMP";
    case ProgramState::WAITING_TO_START:
      return "WAIT_START";
    case ProgramState::GRIPPER_CLOSING:
      return "GRIPPER_CLOSE";
    case ProgramState::MOVING:
      return "MOVING";
    case ProgramState::ARM_BASE_MOVING:
      return "ARM_BASE";
    case ProgramState::CHECKPOINT_DWELL:
      return "DWELL";
    case ProgramState::SAFE_STOPPING:
      return "STOPPING";
    case ProgramState::FINISHED:
      return "FINISHED";
    case ProgramState::EMERGENCY_STOP:
      return "STOPPED";
  }
  return "UNKNOWN";
}

// 把HWT101数据同步发送到电脑调试串口。
// 该上报独立于运动控制，因此停车、等待和故障状态下仍然会持续输出。
void reportImuToComputer() {
  const uint32_t nowMs = millis();
  if (nowMs - lastImuDebugMs < IMU_DEBUG_INTERVAL_MS) {
    return;
  }
  lastImuDebugMs = nowMs;

  Serial_DEBUG.print("HWT101 yaw=");
  if (imuReady) {
    Serial_DEBUG.print(currentYawDeg, 2);
  } else {
    Serial_DEBUG.print("NA");
  }

  Serial_DEBUG.print(" target=");
  Serial_DEBUG.print(targetYawDeg, 2);
  Serial_DEBUG.print(" error=");
  if (imuReady) {
    Serial_DEBUG.print(
        segmentIsRotation
            ? (turnTargetUnwrappedYawDeg - unwrappedYawDeg)
            : wrapAngleDeg(targetYawDeg - currentYawDeg),
        2);
  } else {
    Serial_DEBUG.print("NA");
  }

  Serial_DEBUG.print(" correction=");
  Serial_DEBUG.print(headingCorrectionPps, 0);
  Serial_DEBUG.print(" age=");
  if (imuReady) {
    Serial_DEBUG.print(nowMs - lastImuDataMs);
  } else {
    Serial_DEBUG.print("NA");
  }
  Serial_DEBUG.print("ms imu=");
  Serial_DEBUG.print(imuTimedOut() ? "TIMEOUT" : "OK");
  Serial_DEBUG.print(" state=");
  Serial_DEBUG.print(programStateName());
  Serial_DEBUG.print(" button=");
  Serial_DEBUG.println(digitalRead(START_BUTTON_PIN) == LOW ? "LOW" : "HIGH");
}

// 输出PB9每一次原始电平变化，用于区分“硬件没有接到PB9”和“OneButton未确认单击”。
// 正常按下一次应依次看到raw=LOW和raw=HIGH，随后出现Start button消息。
void reportRawButtonEdge() {
  const int level = digitalRead(START_BUTTON_PIN);
  if (level == lastRawButtonLevel) {
    return;
  }

  lastRawButtonLevel = level;
  Serial_DEBUG.print("BUTTON PB9 raw=");
  Serial_DEBUG.println(level == LOW ? "LOW (pressed)" : "HIGH (released)");
}

/*
 * 启动一段路线：
 * 1. 将毫米转换为脉冲；
 * 2. 根据麦克纳姆轮运动方向组合四轮正反转；
 * 3. 记录四轮起点，用平移投影脉冲计算实际完成距离；
 * 4. 切换为MOVING状态，让loop()持续读取yaw并调用runSpeed()。
 *
 * 四轮逻辑组合（尚未乘MOTOR_POLARITY）：
 *   前进： + + + +
 *   后退： - - - -
 *   左移： - + + -
 *   右移： + - - +
 */
void startSegment(const RouteSegment &segment) {
  segmentIsRotation =
      (segment.direction == Direction::TURN_CCW ||
       segment.direction == Direction::TURN_CW);
  segmentBaseSpeedPps = 0.0F;
  headingCorrectionPps = 0.0F;
  rotationDriveSign = 0.0F;
  lastMotionUpdateUs = micros();

  if (segmentIsRotation) {
    segmentTargetPulses = 0;
    const float requestedYawSign =
        (segment.direction == Direction::TURN_CCW)
            ? CCW_YAW_SIGN
            : -CCW_YAW_SIGN;
    turnTargetUnwrappedYawDeg =
        unwrappedYawDeg + requestedYawSign * static_cast<float>(segment.amount);
    targetYawDeg = wrapAngleDeg(turnTargetUnwrappedYawDeg);
  } else {
    segmentTargetPulses = lroundf(segment.amount * PULSES_PER_MM);
  }

  switch (segment.direction) {
    case Direction::FORWARD:
      translationSigns[0] = 1;
      translationSigns[1] = 1;
      translationSigns[2] = 1;
      translationSigns[3] = 1;
      break;
    case Direction::BACKWARD:
      translationSigns[0] = -1;
      translationSigns[1] = -1;
      translationSigns[2] = -1;
      translationSigns[3] = -1;
      break;
    case Direction::LEFT:
      translationSigns[0] = -1;
      translationSigns[1] = 1;
      translationSigns[2] = 1;
      translationSigns[3] = -1;
      break;
    case Direction::RIGHT:
      translationSigns[0] = 1;
      translationSigns[1] = -1;
      translationSigns[2] = -1;
      translationSigns[3] = 1;
      break;
    case Direction::TURN_CCW:
    case Direction::TURN_CW:
      translationSigns[0] = 0;
      translationSigns[1] = 0;
      translationSigns[2] = 0;
      translationSigns[3] = 0;
      break;
  }

  for (uint8_t i = 0; i < 4; ++i) {
    segmentStartLogicalPosition[i] = logicalMotorPosition(i);
  }
  Serial_DEBUG.print("Segment ");
  Serial_DEBUG.print(routeIndex + 1);
  Serial_DEBUG.print('/');
  Serial_DEBUG.print(ROUTE_COUNT);
  Serial_DEBUG.print(": ");
  Serial_DEBUG.print(directionName(segment.direction));
  Serial_DEBUG.print(' ');
  Serial_DEBUG.print(segment.amount);
  Serial_DEBUG.print(segmentIsRotation ? " deg -> " : " mm -> ");
  Serial_DEBUG.println(segment.destination);

  programState = ProgramState::MOVING;
}

// 根据routeIndex启动ROUTE[]中的当前路线段。
// 正常情况下终点由FINISH处理；越界检查用于防止数组访问错误。
void startCurrentSegment() {
  if (routeIndex >= ROUTE_COUNT) {
    setDriverEnabled(false);
    routeIndex = 0;
    Serial_DEBUG.println(
        "Route index reached the end. Reopening gripper and waiting for two clicks.");
    beginGripperOpenCycle();
    return;
  }
  startSegment(ROUTE[routeIndex]);
}

// 每次按钮启动都重新锁定当前车头方向，并把路线索引清零。
// 注意：程序无法知道人工停车后的场地坐标；若中途停车，请先把车放回启停区，
// 再次单击后才会按完整路线正确到达各个区域。
void startRouteFromBeginning() {
  if (imuTimedOut()) {
    Serial_DEBUG.println(
        "Start rejected: HWT101 has no fresh data. Check IMU before starting.");
    return;
  }

  stopWheelPulses();
  targetYawDeg = currentYawDeg;
  segmentIsRotation = false;
  armBaseStopRequested = false;
  setArmBaseEnabled(false);
  routeIndex = 0;
  setDriverEnabled(true);

  Serial_DEBUG.print("Start button: route restarted from segment 1, target yaw=");
  Serial_DEBUG.println(targetYawDeg, 2);
  startCurrentSegment();
}

// 上电、路线完成或安全停车后，都命令夹爪回到已标定的-90°张开位置。
void beginGripperOpenCycle() {
  stopWheelPulses();
  setDriverEnabled(false);

  if (!gripperReady) {
    programState = ProgramState::WAITING_TO_CLAMP;
    Serial_DEBUG.println(
        "Gripper reset failed: servo ID 4 is offline on PC7/PC6.");
    return;
  }

  gripperServo.setAngle(
      GRIPPER_OPEN_ANGLE_DEG, GRIPPER_MOVE_INTERVAL_MS, 0);
  gripperActionStartTime = millis();
  programState = ProgramState::GRIPPER_OPENING;
  Serial_DEBUG.println("Gripper command: OPEN to absolute -90.0 deg.");
}

void updateGripperOpening() {
  if (millis() - gripperActionStartTime < GRIPPER_ACTION_WAIT_MS) {
    return;
  }

  programState = ProgramState::WAITING_TO_CLAMP;
  Serial_DEBUG.println(
      "Gripper open. Place material, then click PB9 once to clamp.");
}

// 第一次单击只让夹爪运动到已标定的0°夹紧位置，不启动小车。
void startInitialGripperClamp() {
  if (!gripperReady) {
    Serial_DEBUG.println(
        "Start rejected: gripper servo ID 4 is offline on PC7/PC6.");
    return;
  }

  stopWheelPulses();
  setDriverEnabled(false);
  gripperServo.setAngle(
      GRIPPER_CLOSE_ANGLE_DEG,
      GRIPPER_MOVE_INTERVAL_MS,
      GRIPPER_MAX_POWER_MW);
  gripperActionStartTime = millis();
  programState = ProgramState::GRIPPER_CLOSING;
  Serial_DEBUG.println(
      "Gripper command: first click, CLOSE to absolute 0.0 deg; holding force enabled.");
}

void updateInitialGripperClamp() {
  if (millis() - gripperActionStartTime < GRIPPER_ACTION_WAIT_MS) {
    return;
  }

  programState = ProgramState::WAITING_TO_START;
  Serial_DEBUG.println(
      "Gripper clamped and holding. Click PB9 a second time to start the route.");
}

void requestSafeStop() {
  if (programState == ProgramState::ARM_BASE_MOVING) {
    armBaseStopRequested = true;
    if (fabsf(armBaseStepper.speed()) > 0.0F) {
      armBaseStepper.stop();
    } else {
      // 刚进入动作但尚未产生第一步时，stop()不会改变目标，因此显式取消。
      armBaseStepper.moveTo(armBaseStepper.currentPosition());
    }
    Serial_DEBUG.println(
        "Start button: arm base controlled stop requested.");
    return;
  }

  if (programState == ProgramState::MOVING) {
    lastMotionUpdateUs = micros();
    programState = ProgramState::SAFE_STOPPING;
    Serial_DEBUG.println("Start button: controlled deceleration requested.");
    return;
  }

  // 路径点停顿期间轮速本来就是零，直接完成安全停车。
  if (programState == ProgramState::CHECKPOINT_DWELL) {
    completeSafeStop();
  }
}

// 当前路线段完成后的处理：
// FINISH点关闭驱动器；其他点将routeIndex加1并进入短暂停车状态。
void handleArrival() {
  const RouteSegment &segment = ROUTE[routeIndex];
  stopWheelPulses();
  segmentIsRotation = false;

  Serial_DEBUG.print("Arrived: ");
  Serial_DEBUG.println(segment.destination);

  if (segment.actionAtEnd == StopAction::FINISH) {
    Serial_DEBUG.println(
        "Route complete. Wheel drivers disabled; reopening gripper.");
    setArmBaseEnabled(false);
    setDriverEnabled(false);
    routeIndex = 0;
    beginGripperOpenCycle();
    return;
  }

  routeIndex++;

  if (segment.actionAtEnd != StopAction::NONE) {
    Serial_DEBUG.print("Checkpoint: ");
    Serial_DEBUG.println(checkpointName(segment.actionAtEnd));
  }

  // 第一批首次到达原料区：机械臂底座从软件零点顺时针转到+90°。
  // 第二批到达暂存区后先执行下一段小车转向；该转向段完成时，
  // ARM_BASE_RETURN_HOME再触发底座逆时针90°返回软件零点。
  // 两个动作均为非阻塞状态；完成前不会启动下一段底盘路线。
  if (segment.actionAtEnd == StopAction::PICK_BATCH_1) {
    startArmBaseMove(true);
    return;
  }
  if (segment.actionAtEnd == StopAction::ARM_BASE_RETURN_HOME) {
    startArmBaseMove(false);
    return;
  }

  // 每个路径点短暂停车，然后loop()自动启动下一段，不需要人工确认。
  checkpointStartTime = millis();
  programState = ProgramState::CHECKPOINT_DWELL;
}

// 紧急停止采用直接关闭驱动器的方式，不等待减速完成。
// 触发后必须复位主控板才能重新开始路线。
void emergencyStop() {
  stopWheelPulses();
  segmentIsRotation = false;
  setArmBaseEnabled(false);
  armBaseStopRequested = false;
  setDriverEnabled(false);
  programState = ProgramState::EMERGENCY_STOP;
  Serial_DEBUG.println("EMERGENCY STOP: wheel drivers disabled. Reset to restart.");
}

void imuFaultStop() {
  stopWheelPulses();
  segmentIsRotation = false;
  setArmBaseEnabled(false);
  armBaseStopRequested = false;
  setDriverEnabled(false);
  programState = ProgramState::EMERGENCY_STOP;
  Serial_DEBUG.println(
      "IMU TIMEOUT: no fresh HWT101 data. Wheel drivers disabled.");
}

// OneButton确认一次完整单击后调用。回调内只改变状态，不做阻塞等待。
void onStartButtonClick() {
  switch (programState) {
    case ProgramState::WAITING_TO_CLAMP:
      startInitialGripperClamp();
      break;

    case ProgramState::WAITING_TO_START:
      startRouteFromBeginning();
      break;

    case ProgramState::FINISHED:
      beginGripperOpenCycle();
      break;

    case ProgramState::GRIPPER_OPENING:
      Serial_DEBUG.println(
          "Button ignored: wait until the gripper is fully open.");
      break;

    case ProgramState::GRIPPER_CLOSING:
      Serial_DEBUG.println(
          "Button ignored: wait until the material is clamped.");
      break;

    case ProgramState::MOVING:
    case ProgramState::ARM_BASE_MOVING:
    case ProgramState::CHECKPOINT_DWELL:
      requestSafeStop();
      break;

    case ProgramState::SAFE_STOPPING:
      Serial_DEBUG.println(
          "Button ignored: wait until controlled stopping is complete.");
      break;

    case ProgramState::EMERGENCY_STOP:
      Serial_DEBUG.println(
          "Button ignored after emergency/IMU stop. Reset the controller.");
      break;
  }
}

// 非阻塞读取调试串口。目前只处理x/X紧急停止命令。
void handleSerialCommands() {
  while (Serial_DEBUG.available()) {
    const char command = static_cast<char>(Serial_DEBUG.read());

    if (command == 'x' || command == 'X') {
      emergencyStop();
      return;
    }

    if (command == 'i' || command == 'I') {
      correctionSign = -correctionSign;
      Serial_DEBUG.print("Yaw correction direction inverted. sign=");
      Serial_DEBUG.println(correctionSign, 0);
    }

  }
}

void setup() {
  Serial_DEBUG.begin(DEBUG_BAUD);
  // 最早启动标记：不依赖HWT101、按钮、电机或路线状态。
  // 如果连这一行都看不到，说明当前烧录固件/COM端口/UART接线至少一项不正确。
  delay(100);
  Serial_DEBUG.println();
  Serial_DEBUG.println("[BOOT 1] PB13 debug TX started at 115200.");
  Serial_DEBUG.flush();

  // 在任何可能等待通信设备的初始化之前，先可靠关闭底盘和机械臂底座驱动器。
  pinMode(DRIVE_ENABLE_PIN, OUTPUT);
  pinMode(ROTATE_ENABLE_PIN, OUTPUT);
  digitalWrite(DRIVE_ENABLE_PIN, HIGH);
  digitalWrite(ROTATE_ENABLE_PIN, HIGH);

  Serial_WTIMU.begin(HWT_UART_BAUD);
  Serial_DEBUG.println("[BOOT 2] HWT101 UART PD9/PD8 started.");
  Serial_DEBUG.flush();

  // 只初始化舵机通信，不调用会阻塞等待的gripper.init()。
  // 无论上电时反馈角度是多少，都直接发送已标定的-90°张开命令。
  gripperProtocol.init(&Serial_GRIPPER, GRIPPER_UART_BAUD);
  gripperServo.init();
  gripperReady = gripperServo.isOnline;
  if (gripperReady) {
    const float startupGripperAngleDeg = gripperServo.queryAngle();
    Serial_DEBUG.print("[BOOT 3] Gripper ID 4 online, startup angle=");
    Serial_DEBUG.print(startupGripperAngleDeg, 1);
    Serial_DEBUG.println(" deg; commanding OPEN at -90.0 deg.");
    beginGripperOpenCycle();
  } else {
    Serial_DEBUG.println(
        "[BOOT 3] ERROR: gripper ID 4 offline on PC7/PC6.");
  }
  Serial_DEBUG.flush();

  pinMode(DRIVE_ENABLE_PIN, OUTPUT);
  pinMode(ROTATE_ENABLE_PIN, OUTPUT);

  // PE13和PE10都为低电平使能。上电时先保持两个驱动器关闭，
  // 底盘等待PB9启动；机械臂底座只在两个指定检查点短暂使能。
  digitalWrite(ROTATE_ENABLE_PIN, HIGH);
  // 上电时不使能底盘，必须等待PB9按钮单击。
  digitalWrite(DRIVE_ENABLE_PIN, HIGH);

  configureMotor(motor1);
  configureMotor(motor2);
  configureMotor(motor3);
  configureMotor(motor4);
  armBaseStepper.setMaxSpeed(ARM_BASE_MAX_SPEED_PPS);
  armBaseStepper.setAcceleration(ARM_BASE_ACCELERATION_PPS2);
  // 软件零点对应机械臂初始朝向。上电前必须人工确认底座位于该位置。
  armBaseStepper.setCurrentPosition(0);

  // PB9使用内部上拉：松开为HIGH，按钮按下接GND后为LOW。
  startButton.setup(START_BUTTON_PIN, INPUT_PULLUP, true);
  startButton.setDebounceMs(30);
  startButton.setClickMs(150);
  startButton.reset();
  startButton.attachClick(onStartButtonClick);
  lastRawButtonLevel = digitalRead(START_BUTTON_PIN);
  Serial_DEBUG.print("[BOOT 4] PB9 button initialized, raw=");
  Serial_DEBUG.println(lastRawButtonLevel == LOW ? "LOW" : "HIGH");
  Serial_DEBUG.flush();

  delay(300);
  Serial_DEBUG.println();
  Serial_DEBUG.println("=== SmartCarrier two-batch route ===");
  Serial_DEBUG.print("Start and finish: ");
  Serial_DEBUG.println(START_ZONE_NAME);
  Serial_DEBUG.println(
      "Place the initial chassis heading toward the top; route turns are automatic.");
  Serial_DEBUG.println(
      "Commands: x=emergency stop, i=invert yaw correction direction");
  Serial_DEBUG.println(
      "PB9 button: first click=clamp only; second click=start; running click=safe stop.");
  Serial_DEBUG.println("WARNING: verify wheel directions and distance calibration with the chassis raised.");

  // 等待HWT101持续输出数据，并把上电时的实际车头方向设为初始目标航向。
  // 每个旋转段完成后，程序会把旋转后的角度设为后续直线段的新目标航向。
  // 等待期间只读传感器、不驱动车轮，因此不需要人工测量绝对yaw值。
  const uint32_t imuWaitStart = millis();
  uint32_t lastImuBootReportMs = imuWaitStart;
  while (millis() - imuWaitStart < IMU_STARTUP_WAIT_MS) {
    updateImu();
    if (millis() - lastImuBootReportMs >= 500) {
      lastImuBootReportMs = millis();
      Serial_DEBUG.print("[BOOT 5] Waiting HWT101, received=");
      Serial_DEBUG.println(imuReady ? "YES" : "NO");
      Serial_DEBUG.flush();
    }
    delay(1);
  }

  if (imuTimedOut()) {
    imuFaultStop();
    Serial_DEBUG.println(
        "Check HWT101 power, 115200 baud, PD9(RX) and PD8(TX), then reset.");
    return;
  }

  targetYawDeg = currentYawDeg;
  Serial_DEBUG.print("HWT101 ready. Locked target yaw=");
  Serial_DEBUG.println(targetYawDeg, 2);
  Serial_DEBUG.print("Yaw correction sign=");
  Serial_DEBUG.println(correctionSign, 0);

  routeIndex = 0;
  if (gripperReady) {
    // 张开动作与HWT101启动等待并行进行；loop()会在计时完成后进入等待夹紧状态。
    updateGripperOpening();
  } else {
    programState = ProgramState::WAITING_TO_CLAMP;
    Serial_DEBUG.println(
        "NOT READY: gripper ID 4 is offline; PB9 start will be rejected.");
  }
}

void loop() {
  // HWT101必须在所有程序状态中持续读取，避免UART接收缓存溢出。
  updateImu();

  // OneButton依靠高频tick()完成消抖和单击识别，不能用长delay阻塞。
  startButton.tick();
  reportRawButtonEdge();

  // 无论当前处于什么状态，都优先检查紧急停止命令。
  handleSerialCommands();

  // 同步反馈到电脑串口；即使下面的状态分支提前return，也不会停止上报。
  reportImuToComputer();

  // 张开和夹紧均采用非阻塞计时，期间按钮、HWT101和串口仍持续工作。
  if (programState == ProgramState::GRIPPER_OPENING) {
    updateGripperOpening();
    delay(1);
    return;
  }

  if (programState == ProgramState::GRIPPER_CLOSING) {
    updateInitialGripperClamp();
    delay(1);
    return;
  }

  // 机械臂底座动作期间底盘保持停止，只运行M5的AccelStepper脉冲。
  if (programState == ProgramState::ARM_BASE_MOVING) {
    updateArmBaseMotion();
    return;
  }

  // 按钮触发的安全停车状态：继续产生逐步降低的脉冲，直到速度为零。
  if (programState == ProgramState::SAFE_STOPPING) {
    if (imuTimedOut()) {
      imuFaultStop();
      return;
    }
    updateSafeStopping();
    return;
  }

  // 路线点停车状态：计时结束后自动启动下一段。
  if (programState == ProgramState::CHECKPOINT_DWELL) {
    if (millis() - checkpointStartTime >= CHECKPOINT_DWELL_MS) {
      startCurrentSegment();
    }
    delay(1);
    return;
  }

  // FINISHED或EMERGENCY_STOP状态不再产生步进脉冲。
  if (programState != ProgramState::MOVING) {
    delay(1);
    return;
  }

  // 运动中如果500 ms没有收到HWT101串口数据，立即停止，避免开环跑偏。
  if (imuTimedOut()) {
    imuFaultStop();
    return;
  }

  if (segmentIsRotation) {
    // 旋转段由HWT101连续yaw闭环判断角度完成，不使用轮脉冲估算角度。
    if (updateRotationMotion()) {
      handleArrival();
    }
  } else {
    // 平移段实时纠正航向，并根据四轮投影脉冲计算距离。
    updateClosedLoopMotion();
    if (segmentProgressPulses() >= static_cast<float>(segmentTargetPulses)) {
      handleArrival();
    }
  }
}
