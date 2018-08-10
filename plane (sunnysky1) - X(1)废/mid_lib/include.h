/*********include。h
用于包含所有需要使用的头文件
需要在每个c文件中引用
李锐戈
2014.11.16
2014 12 30 加入interaction.h
******
*****
****/
#ifndef _INCLUDE_H_
#define _INCLUDE_H_

#include "common.h"

#include "main.h"
#include "mid_lib.h"
#include "mid_NVIC.h"
#include "arm_math.h"
#include "all_init.h"

#include "freecars.h"
#include "safty.h"

#include "I2C.h"
#include "MPU_6050.h"
#include "BMP180.h"
#include "HMC5883.h"
#include "IMU.h"
#include "control.h"
#include "DEV_Nrf24L01.h"
#include "RCpin.h"
#include "OLED.h"
#include "sonar.h"

#include "ANO_DT.h"

#endif