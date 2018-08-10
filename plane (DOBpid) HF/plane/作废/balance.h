#ifndef _BALANCE_H_
#define _BALANCE_H_

extern float GyroX_ratio;
extern float GRAVITY_ADJUST_TIME_CONSTANT_X;

extern float GyroY_ratio;
extern float GRAVITY_ADJUST_TIME_CONSTANT_Y;

extern vint16 X_OUT ;       //加速度计输出
extern vint16 X_Gyro ;      //陀螺仪输出
extern vint16 Y_OUT ;
extern vint16 Y_Gyro ;


void get_quaternion();
extern int16 g_fXAngle;
extern int16 g_fYAngle;

void get_quaternion(); //获取并处理四元数
void AD_Calculate(void);

#endif
