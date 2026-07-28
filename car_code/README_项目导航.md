# 搬运小车代码导航

更新时间：2026-07-22

## 当前目录

| 目录 | 用途 | 当前处理方式 |
| --- | --- | --- |
| `reference_code/final` | 学长完成版工程，含移动、二维码、IMU、视觉、机械臂和任务状态机 | 建议作为当前主线参考 |
| `reference_code/Car_2` | 另一版完整工程，含颜色跟踪和另一套路线、机械臂参数 | 用于比对参数和功能，不直接与 `final` 混用 |
| `DDSM210` | DDSM210 电机及 STM32/ESP32 示例库 | 按实际电机型号选择性参考 |
| `ESPBot` | ESP32 通用外设与电机示例库 | 非 H750 主线，不作为当前编译工程 |
| `STM32BOT` | STM32H750 机器人相关底板/Arduino 支持资料 | H750 平台资料 |
| `RPGripper`、`SunnybotARM`、`SunSTP23`、`SunUPFLOW`、`SunOpenMV` | 夹爪、机械臂、步进和视觉模块的独立库/示例 | 仅在确认对应硬件后引用 |
| `electronics`、`vision` | 为本项目预留的资料目录 | 当前没有可用源文件 |

原始压缩包 `Car_2.zip` 和 `final.zip` 已保留；其内容分别解压到 `reference_code/Car_2`、`reference_code/final`，不会覆盖其他目录。

## 两份参考工程

两者都是 PlatformIO 工程，目标为 `genericSTM32H750VB`，使用 Arduino 框架和 ST-Link 下载。

- `final/src/main.cpp`：基于任务状态机，包含四轮步进驱动、底座旋转、二维码、IMU、MaixCam 视觉、夹爪、储物盘和两路串口步进电机。主要流程在 `setup()`、`loop()` 与 `task[8]` 中。
- `Car_2/src/main.cpp`：同样控制 H750 小车与机械臂，但路线点、机械臂行程和颜色跟踪逻辑不同。其 `Follow_Color*` 函数和 `move_map` 可用来理解或核对路线方案。

不要把两份 `main.cpp` 的参数直接拼接：引脚、物料高度、脉冲当量、舵机角度和路线坐标均可能对应不同届的机械结构。

## 当前建议

1. 先确认主控确为 STM32H750VB，并核对线束与 `final/src/main.cpp` 的引脚定义：四个底盘电机、底座旋转电机、二维码模块、串口屏、IMU、MaixCam、夹爪/储物盘舵机、两路 TTL 步进电机。
2. 将 `reference_code/final` 复制为本届独立工作工程后再修改；不要在学长原工程中直接试错。复制前应先清理生成目录 `.pio`，以避免携带无关的编译产物。
3. 在 PlatformIO 中打开新的工作工程，安装 `platformio.ini` 所列依赖并执行一次编译。当前电脑的命令行未检测到 PlatformIO，需要先安装 PlatformIO IDE 或 CLI。
4. 在车轮悬空、机械臂留出安全空间的条件下，依次验证串口屏、二维码、IMU、视觉、夹爪、TTL 步进、底盘单电机；每验证一项就记录实际串口、方向、ID 和零位。
5. 最后再校准 `PULSES_PER_METER`、旋转脉冲、各工位坐标、视觉比例和夹爪/储物盘角度，并用单工件完成全流程测试。

## 注意

参考工程的源文件注释显示为中文乱码，说明文件很可能使用 GBK/ANSI 编码。编辑时先在编辑器中以 GBK 尝试打开并确认中文显示正常；不要在未确认编码时批量保存或格式化，否则注释会被永久破坏。
