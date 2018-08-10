#ifndef _DMP_H_
#define _DMP_H_

#include "MPU6050.h"

extern float Pitch ;
extern float Roll ;
extern float Yaw ;

extern short gyro[3];

extern float roll_quiet ;
extern float pitch_quiet ;
extern float yaw_quiet ;

void IMU_init();
void MIU_getdata();



#endif