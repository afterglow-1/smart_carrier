# SunnybotARM
 SunnybotARM ESP32总库

更新内容
1. 

 一键标定
 todo
 添加舵机原始角度超限检查  已添加
 添加舵机uuid查询  已添加querySN，但内部编号都一样
 添加文本数据保存

切换littlefs https://randomnerdtutorials.com/esp8266-nodemcu-vs-code-platformio-littlefs/

 添加舵机在线检查
https://github.com/platformio/platform-espressif32
测试已支持2.0.3 S3
 PING用2.0.3不会出错
 gripper也不会出错

Backtrace:0x40142a44:0x3ffb27400x40142a41:0x3ffb2760 0x400df7e1:0x3ffb2780 0x400e017b:0x3ffb27a0 0x400d3862:0x3ffb27c0 0x400e29aa:0x3ffb2820 




ELF file SHA256: 0000000000000000

只要arm.init()就重启
Rebooting...
ets Jun  8 2016 00:22:57

Guru Meditation Error: Core  1 panic'ed (LoadProhibited). Exception was unhandled.

Core  1 register dump:
PC      : 0x400d117b  PS      : 0x00060030  A0      : 0x800d1178  A1      : 0x3ffb2780  
A2      : 0x00000000  A3      : 0x00000000  A4      : 0x00000000  A5      : 0x00000000  
A6      : 0x3ffb7f10  A7      : 0x00000000  A8      : 0x800d1145  A9      : 0x3ffb2760  
A10     : 0x3ffc1240  A11     : 0x00000000  A12     : 0x00000000  A13     : 0x00000001  
A14     : 0x00060020  A15     : 0x00000001  SAR     : 0x0000000a  EXCCAUSE: 0x0000001c  
EXCVADDR: 0x00000000  LBEG    : 0x40083be5  LEND    : 0x40083bed  LCOUNT  : 0x00000027  


Backtrace:0x400d1178:0x3ffb27800x400d1175:0x3ffb27a0 0x400d1021:0x3ffb27c0 0x400d111f:0x3ffb27e0 0x400d0fa2:0x3ffb2800 0x400d1efa:0x3ffb2820 


使用https://github.com/me-no-dev/EspExceptionDecoder 工具 没有成功？目录不对？
Backtrace:0x400d1084:0x3ffb2770 0x400d1081:0x3ffb27a0 0x400d1025:0x3ffb27c0 0x400d1043:0x3ffb27e0 0x400d0f7a:0x3ffb2800 0x400d1e4e:0x3ffb2820

Backtrace:0x400d1084:0x3ffb2770 0x400d1081:0x3ffb27a0 0x400d1025:0x3ffb27c0 0x400d1043:0x3ffb27e0 0x400d0f7a:0x3ffb2800 0x400d1e4e:0x3ffb2820 

xtensa-esp32-elf-addr2line -pfiaC -e gripper_control.ino.elf 0x400d1084:0x3ffb2770
xtensa-esp32-elf-addr2line -pfiaC -e gripper_control.ino.elf 0x400d1081:0x3ffb27a0
xtensa-esp32-elf-addr2line -pfiaC -e gripper_control.ino.elf 0x400d1025:0x3ffb27c0
xtensa-esp32-elf-addr2line -pfiaC -e gripper_control.ino.elf 0x400d1043:0x3ffb27e0
xtensa-esp32-elf-addr2line -pfiaC -e gripper_control.ino.elf 0x400d0f7a:0x3ffb2800
xtensa-esp32-elf-addr2line -pfiaC -e gripper_control.ino.elf 0x400d1e4e:0x3ffb2820

C:\Espressif\frameworks\esp-idf-v4.4.1>cd C:\Users\zjusu\AppData\Local\Temp\arduino_build_501938

C:\Users\zjusu\AppData\Local\Temp\arduino_build_501938>xtensa-esp32-elf-addr2line -pfiaC -e gripper_control.ino.elf 0x400d1084:0x3ffb2770
0x400d1084: FSUS_Servo::ping() at C:\Users\zjusu\Documents\Arduino\libraries\fashionstar-uart-servo-arduino/FashionStar_UartServo.cpp:50

C:\Users\zjusu\AppData\Local\Temp\arduino_build_501938>xtensa-esp32-elf-addr2line -pfiaC -e gripper_control.ino.elf 0x400d1081:0x3ffb27a0
0x400d1081: FSGP_Gripper::init(FSUS_Servo*, float, float) at C:\Users\zjusu\Documents\Arduino\libraries\fashionstar-gripper-arduino/FashionStar_SmartGripper.cpp:58

C:\Users\zjusu\AppData\Local\Temp\arduino_build_501938>xtensa-esp32-elf-addr2line -pfiaC -e gripper_control.ino.elf 0x400d1025:0x3ffb27c0
0x400d1025: FSARM_ARM5DoF::initServos() at C:\Users\zjusu\Documents\Arduino\libraries\fashionstar-arm-5dof-arduino/FashionStar_Arm5DoF.cpp:43

C:\Users\zjusu\AppData\Local\Temp\arduino_build_501938>xtensa-esp32-elf-addr2line -pfiaC -e gripper_control.ino.elf 0x400d1043:0x3ffb27e0
0x400d1043: FSARM_ARM5DoF::init() at C:\Users\zjusu\Documents\Arduino\libraries\fashionstar-arm-5dof-arduino/FashionStar_Arm5DoF.cpp:19

C:\Users\zjusu\AppData\Local\Temp\arduino_build_501938>xtensa-esp32-elf-addr2line -pfiaC -e gripper_control.ino.elf 0x400d0f7a:0x3ffb2800
0x400d0f7a: setup() at C:\Users\zjusu\AppData\Local\Temp\arduino_modified_sketch_45039/gripper_control.ino:13

C:\Users\zjusu\AppData\Local\Temp\arduino_build_501938>xtensa-esp32-elf-addr2line -pfiaC -e gripper_control.ino.elf 0x400d1e4e:0x3ffb2820
0x400d1e4e: loopTask(void*) at C:\Users\zjusu\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.3\cores\esp32/main.cpp:42

Guru Meditation Error: Core  1 panic'ed (LoadProhibited). Exception was unhandled.

Core  1 register dump:
PC      : 0x400d1087  PS      : 0x00060030  A0      : 0x800d1084  A1      : 0x3ffb2770  
A2      : 0x00000001  A3      : 0x00000001  A4      : 0x0000000a  A5      : 0x00000000  
A6      : 0x3ffb7f10  A7      : 0x00000000  A8      : 0x800d1064  A9      : 0x3ffb2750  
A10     : 0x3ffc1240  A11     : 0x00000046  A12     : 0xcf6f62e8  A13     : 0x3ffc1514  
A14     : 0x00ff0000  A15     : 0xff000000  SAR     : 0x0000001d  EXCCAUSE: 0x0000001c  
EXCVADDR: 0x00000001  LBEG    : 0x400859fc  LEND    : 0x40085a06  LCOUNT  : 0x00000000  


Backtrace:0x400d1084:0x3ffb27700x400d1081:0x3ffb27a0 0x400d1025:0x3ffb27c0 0x400d1043:0x3ffb27e0 0x400d0f7a:0x3ffb2800 0x400d1e4e:0x3ffb2820 


S3
PIO Core Call Error: "The current working directory C:\\Users\\zjusu\\Documents\\PlatformIO\\Projects\\S3TEST will be used for the project.\r\n\r\nThe next files/directories have been created in C:\\Users\\zjusu\\Documents\\PlatformIO\\Projects\\S3TEST\r\ninclude - Put project header files here\r\nlib - Put here project specific (private) libraries\r\nsrc - Put project source files here\r\nplatformio.ini - Project Configuration File\r\n\n\nError: Unknown board ID 'esp32-s3-devkitc-1'"



