/******************************************************************************
                        参数配置库
版本: V1.0
V1.0记录: 1、增加M0603C111配置

*******************************************************************************/
#pragma once


#define BAUDRATE 115200
#define PWM_MAX 255
#define PWM_MIN -PWM_MAX
#define USE_DDSM210
#ifdef USE_DDSM210
#include "SunMotorDriver/DDSM210/DDSM210.h"
#endif
