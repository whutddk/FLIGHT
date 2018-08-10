#ifndef _BALANCE_H_
#define _BALANCE_H_

extern float GyroX_ratio;
extern float GRAVITY_ADJUST_TIME_CONSTANT_X;

extern float GyroY_ratio;
extern float GRAVITY_ADJUST_TIME_CONSTANT_Y;

extern vint16 X_OUT ;       //���ٶȼ����
extern vint16 X_Gyro ;      //���������
extern vint16 Y_OUT ;
extern vint16 Y_Gyro ;


void get_quaternion();
extern int16 g_fXAngle;
extern int16 g_fYAngle;

void get_quaternion(); //��ȡ��������Ԫ��
void AD_Calculate(void);

#endif
