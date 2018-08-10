#include "common.h"
#include "include.h"

vint16 X_OUT = 0;       //���ٶȼ����
vint16 X_Gyro = 0;      //���������
vint16 Y_OUT = 0;
vint16 Y_Gyro = 0;

/***********************ƽ����Ʋ���********************/
int16 OUTX_VAL = -780;   //ƽ��ֵ
int16 GYROX_VAL = -20;   //��������ֵ        //�Ӵ���󣬼�����ǰ
float GyroX_ratio =36.7; //13.9;    //�ں�ϵ��
float GRAVITY_ADJUST_TIME_CONSTANT_X = 1.3;

int16 OUTY_VAL = -100;   //ƽ��ֵ
int16 GYROY_VAL = 0;   //��������ֵ        //�Ӵ���󣬼�����ǰ
float GyroY_ratio = -36.7;//-13.9;    //�ں�ϵ��
float GRAVITY_ADJUST_TIME_CONSTANT_Y = 1.3;

float DT = 0.01;

/******************�м����****************/
int16 GyroX_Now = 0,angleX_offset_vertical = 0;  //������ת����Ľ��ٶȣ�ת����ļ��ٶȽǶ�
int16 GyroY_Now = 0,angleY_offset_vertical = 0;  //������ת����Ľ��ٶȣ�ת����ļ��ٶȽǶ�
/*****************���******************/
int16 g_fXAngle = 0,g_fGyroscopeAngleIntegral_X = 0; //�ںϺ�ĽǶ�
int16 g_fYAngle = 0,g_fGyroscopeAngleIntegral_Y = 0; //�ںϺ�ĽǶ�

//������������5���������Լ���е�ṹ


void get_quaternion() //��ȡ��������Ԫ��
{
  X_OUT = GetData(ACCEL_XOUT_H) ;
  X_Gyro = GetData(GYRO_YOUT_H) ; 
  
  Y_OUT =  GetData(ACCEL_YOUT_H) ;
  Y_Gyro = GetData(GYRO_XOUT_H) ;  
  
  AD_Calculate();
  push(0,X_OUT);
  push(1,Y_OUT);
  
  push(2,g_fXAngle);
  push(3,g_fYAngle);
}


//**************************************************************************
//   �廪�Ƕ��˲�����
//*************************************************************************

//**************************************************************************

void AD_Calculate(void)
{
  float fDeltaValue;
  

  
  GyroX_Now = (GYROX_VAL - X_Gyro) ;                            //�����ǲɼ����Ľ��ٶȹ�һ��
  GyroY_Now = (GYROY_VAL - Y_Gyro) ;                            //�����ǲɼ����Ľ��ٶȹ�һ��
  
  angleX_offset_vertical = (OUTX_VAL - X_OUT) /** MMA7361_ratio */;  //�����ٶȼƲɼ����ĽǶȹ�һ����
  angleY_offset_vertical = (OUTY_VAL - Y_OUT) /** MMA7361_ratio */;  //�����ٶȼƲɼ����ĽǶȹ�һ����
  
  /************************************************************/
//  if(angleX_offset_vertical > 2000)angleX_offset_vertical = 2000;               //��ֹ���ٶȽǶ����
//  if(angleX_offset_vertical < -2000)angleX_offset_vertical = -2000;
//  if(angleY_offset_vertical > 2000)angleY_offset_vertical = 2000;               //��ֹ���ٶȽǶ����
//  if(angleY_offset_vertical < -2000)angleY_offset_vertical = -2000;
//  //�����ںϺ�ĽǶ�
            //�廪�Ƕ��˲�����
  
  
  fDeltaValue = ( angleX_offset_vertical - g_fXAngle) / GRAVITY_ADJUST_TIME_CONSTANT_X;  //ʱ��ϵ������
  g_fGyroscopeAngleIntegral_X += (GyroX_ratio * X_Gyro + fDeltaValue) * DT;   //����������              //�ںϽǶ�
  g_fXAngle = g_fGyroscopeAngleIntegral_X;   //�����ںϽǶ�
  
  fDeltaValue = ( angleY_offset_vertical - g_fYAngle) / GRAVITY_ADJUST_TIME_CONSTANT_Y;  //ʱ��ϵ������
  g_fGyroscopeAngleIntegral_Y += (GyroY_ratio * Y_Gyro + fDeltaValue) * DT;   //����������              //�ںϽǶ�
  g_fYAngle = g_fGyroscopeAngleIntegral_Y;   //�����ںϽǶ�
  
//  push(6,X_OUT);
//  push(7,X_Gyro);
//  push(8,angleX_offset_vertical);
//  push(9,g_fXAngle);
//  push(10,g_fYAngle);
}



