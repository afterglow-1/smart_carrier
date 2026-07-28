# 机械臂标定测试程序

该程序独立于正式 `SmartCarrier/src/main.cpp`，用于获取 M5、ID6、ID7 的回零、位置、方向和机械臂姿态标定数据。它不会修改 ID6/ID7 已经通过上位机保存的回零参数。

## 接线和参考位置

- 调试串口：PB12(RX)、PB13(TX)，115200 波特率。
- ID6/ID7 总线：PA3(RX)、PA2(TX)，115200 波特率。
- M5：PE10 低电平使能、PE15 方向、PB11 脉冲。
- M5 参数与正式主程序一致：200 步/圈、16 细分、5:1 减速比，输出轴顺时针 90 度为 4000 脉冲。
- 上电前把 M5 摆到与正式主程序相同的初始姿态。程序启动时直接将该姿态定义为 M5 软件零点，不会自动转动 M5。

该测试固件不初始化底盘、夹爪、扫码、屏幕、IMU 或相机，因此不会占用这些外设的引脚。

## 编译和烧录

在 `SmartCarrier/three_circle_feature` 目录执行：

```powershell
platformio run -c platformio-arm-calibration.ini -e genericSTM32H750VB
platformio run -c platformio-arm-calibration.ini -e genericSTM32H750VB -t upload
platformio device monitor -b 115200 -p COM10
```

如果串口不是 COM10，请修改配置文件或监视命令中的端口。

## 安全要求

1. 首次测试只连接并测试一个运动轴，确保随时可以切断机械臂电源。
2. 无限位碰撞回零会让机构接触机械端点。必须先在上位机确认回零方向、速度、超时、检测电流和检测时间正确。
3. 说明书要求使用固定负载标定碰撞电流。检测电流应略高于正常回零速度下的运行电流，不能直接照抄说明书示例值。
4. 首次点动使用较小脉冲数，例如 `jog 6 cw 50 30 100`。
5. 串口发送单个 `!` 会立即禁用 M5，并向 ID6、ID7发送回零中断和立即停止命令。
6. M5 被禁用后如果用手转动，软件位置将失效。重新上电摆正，或在正确姿态执行 `m5 zero`。

## 回零测试

先读取上位机保存的回零参数：

```text
params 6
params 7
```

输出中的 `mode` 应为 `2`，即多圈无限位碰撞回零。确认方向和阈值后，一次只测试一个轴：

```text
home 7
home 6
```

程序每 250 ms 读取一次回零状态：

- `homing=1`：正在回零。
- `homing_failed=1`：回零失败。
- 出现 `HOME_COMPLETE`：检测到回零过程已正常结束，并自动打印实时位置。
- `abort 6` 或 `abort 7`：中断对应轴的回零。

测试程序只发送模式号 `2` 的触发命令，不调用“修改回零参数”命令，因此不会覆盖上位机保存的方向、速度、电流、时间和上电回零设置。

## 常用命令

```text
help
params 6
home 6
origin 6
abort 6
enable 6
disable 6
jog 6 cw 50 30 100
jog 6 ccw 50 30 100
pos all
state all

m5 status
m5 goto 90
m5 jog -2
m5 stop
m5 disable
m5 zero

csv
mark rough1_safe
mark rough1_tray_hover
mark rough1_tray_grab
mark rough1_circle1
mark rough1_circle2
mark rough1_circle3
mark rough1_tray_release

stop all
!
```

`jog` 的方向是电机轴的 CW/CCW，不预先假定它对应机构伸出、缩回、上升或下降。请通过小距离点动确定实际方向并记录。

每条 `jog` 命令最多发送 3200 个脉冲，默认速度 60 RPM、默认加速度档位 100。位置输出中的 `raw` 是驱动器实时编码器位置，65536 对应电机一圈；`nominal_mm` 仅按现有机械参数估算：ID6 为模数 1、36 齿，ID7 为 12 mm 导程。最终标定应以实测位移和 `raw` 为准。

## 采集标定数据

机械臂到达目标姿态并停止后执行 `mark <标签>`。输出示例：

```text
MARK,123456,rough1_circle1,-4000,90.000,0,0.0000,0.0000,0x03,0,0.0000,0.0000,0x03
```

请分别采集粗加工区第 1/2 次和暂存区第 1/2 次的安全位、托盘悬停位、夹取位、抬升位、三个圆位和放回位。每个姿态重复到达三次并各执行一次 `mark`，最后把完整串口日志发回即可。

## 协议依据

根据《Emm_V5.0 步进闭环驱动说明书 Rev1.3》：

- `0x9A`：触发回零，模式 `0x02` 为多圈无限位碰撞回零。
- `0x9C 0x48`：强制中断并退出回零。
- `0x3B`：读取回零状态标志。
- `0x36`：读取电机实时位置，65536 为电机一圈。
- `0x3A`：读取使能、到位、堵转和堵转保护状态。
- `0x22`：读取当前保存的原点回零参数。
