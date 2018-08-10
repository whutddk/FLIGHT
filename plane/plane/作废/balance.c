#include "common.h"
#include "include.h"

vint16 X_OUT = 0;       //加速度计输出
vint16 X_Gyro = 0;      //陀螺仪输出
vint16 Y_OUT = 0;
vint16 Y_Gyro = 0;

/***********************平衡控制参数********************/
int16 OUTX_VAL = -780;   //平衡值
int16 GYROX_VAL = -20;   //陀螺仪中值        //加大向后，减少向前
float GyroX_ratio =36.7; //13.9;    //融合系数
float GRAVITY_ADJUST_TIME_CONSTANT_X = 1.3;

int16 OUTY_VAL = -100;   //平衡值
int16 GYROY_VAL = 0;   //陀螺仪中值        //加大向后，减少向前
float GyroY_ratio = -36.7;//-13.9;    //融合系数
float GRAVITY_ADJUST_TIME_CONSTANT_Y = 1.3;

float DT = 0.01;

/******************中间变量****************/
int16 GyroX_Now = 0,angleX_offset_vertical = 0;  //陀螺仪转化后的角速度，转化后的加速度角度
int16 GyroY_Now = 0,angleY_offset_vertical = 0;  //陀螺仪转化后的角速度，转化后的加速度角度
/*****************结果******************/
int16 g_fXAngle = 0,g_fGyroscopeAngleIntegral_X = 0; //融合后的角度
int16 g_fYAngle = 0,g_fGyroscopeAngleIntegral_Y = 0; //融合后的角度

//反复调整以上5个参数，以及机械结构


void get_quaternion() //获取并处理四元数
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
//   清华角度滤波方案
//*************************************************************************

//**************************************************************************

void AD_Calculate(void)
{
  float fDeltaValue;
  

  
  GyroX_Now = (GYROX_VAL - X_Gyro) ;                            //陀螺仪采集到的角速度归一化
  GyroY_Now = (GYROY_VAL - Y_Gyro) ;                            //陀螺仪采集到的角速度归一化
  
  angleX_offset_vertical = (OUTX_VAL - X_OUT) /** MMA7361_ratio */;  //将加速度计采集到的角度归一化，
  angleY_offset_vertical = (OUTY_VAL - Y_OUT) /** MMA7361_ratio */;  //将加速度计采集到的角度归一化，
  
  /************************************************************/
//  if(angleX_offset_vertical > 2000)angleX_offset_vertical = 2000;               //防止加速度角度溢出
//  if(angleX_offset_vertical < -2000)angleX_offset_vertical = -2000;
//  if(angleY_offset_vertical > 2000)angleY_offset_vertical = 2000;               //防止加速度角度溢出
//  if(angleY_offset_vertical < -2000)angleY_offset_vertical = -2000;
//  //计算融合后的角度
            //清华角度滤波方案
  
  
  fDeltaValue = ( angleX_offset_vertical - g_fXAngle) / GRAVITY_ADJUST_TIME_CONSTANT_X;  //时间系数矫正
  g_fGyroscopeAngleIntegral_X += (GyroX_ratio * X_Gyro + fDeltaValue) * DT;   //卡尔曼比例              //融合角度
  g_fXAngle = g_fGyroscopeAngleIntegral_X;   //最终融合角度
  
  fDeltaValue = ( angleY_offset_vertical - g_fYAngle) / GRAVITY_ADJUST_TIME_CONSTANT_Y;  //时间系数矫正
  g_fGyroscopeAngleIntegral_Y += (GyroY_ratio * Y_Gyro + fDeltaValue) * DT;   //卡尔曼比例              //融合角度
  g_fYAngle = g_fGyroscopeAngleIntegral_Y;   //最终融合角度
  
//  push(6,X_OUT);
//  push(7,X_Gyro);
//  push(8,angleX_offset_vertical);
//  push(9,g_fXAngle);
//  push(10,g_fYAngle);
}



