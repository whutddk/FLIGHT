//������̬����

#include "common.h"
#include "include.h"

struct _ctrl ctrl; //���Ƹ��Ͻṹ��


uint32 throttle = 0; //������
float pitch_rc = 0.;


void ctrl_init()
{
  //������
  ctrl.pitch.shell.kp = 6;
  ctrl.pitch.shell.ki = 0.1;
  ctrl.pitch.shell.kd = 0.2;
  ctrl.pitch.shell.kt = 0.005;
  ctrl.pitch.shell.increment = 0.;
  ctrl.pitch.shell.increment_max = 100;
  ctrl.pitch.shell.kp_out = 0.;
  ctrl.pitch.shell.ki_out = 0.;
  ctrl.pitch.shell.kd_out = 0.;
  ctrl.pitch.shell.error[1] = 0;
  ctrl.pitch.shell.aim= 0;
  ctrl.pitch.shell.pid_out = 0;
  
  //�����
  ctrl.roll.shell.kp = 6;
  ctrl.roll.shell.ki = 0.1;
  ctrl.roll.shell.kd = 0.2;
  ctrl.roll.shell.kt = 0.005;
  ctrl.roll.shell.increment = 0.;
  ctrl.roll.shell.increment_max = 100;
  ctrl.roll.shell.kp_out = 0.;
  ctrl.roll.shell.ki_out = 0.;
  ctrl.roll.shell.kd_out = 0.;
  ctrl.roll.shell.error[1] = 0;
  ctrl.roll.shell.aim = 0;
  ctrl.roll.shell.pid_out = 0;
  
  //ƫ����
  ctrl.yaw.shell.kp = 0.;
  ctrl.yaw.shell.ki = 0.;
  ctrl.yaw.shell.kd = 0;
  ctrl.yaw.shell.kt = 0.005;
  ctrl.yaw.shell.increment = 0.;
  ctrl.yaw.shell.increment_max = 300;
  ctrl.yaw.shell.kp_out = 0.;
  ctrl.yaw.shell.ki_out = 0.;
  ctrl.yaw.shell.kd_out = 0.;
  ctrl.yaw.shell.error[1] = 0;  
  ctrl.yaw.shell.aim = 0;
  ctrl.yaw.shell.pid_out = 0;
  
  ctrl.pitch.core.kp = 8.;
  ctrl.pitch.core.ki = 0.;
  ctrl.pitch.core.kd = 0.5;
  ctrl.pitch.core.kt = 0.005;
  ctrl.pitch.core.increment = 0.;
  ctrl.pitch.core.increment_max = 300;
  ctrl.pitch.core.kp_out = 0.;
  ctrl.pitch.core.ki_out = 0.;
  ctrl.pitch.core.kd_out = 0.;
  ctrl.pitch.core.error[1] = 0;
  ctrl.pitch.core.aim= 0;
  ctrl.pitch.core.pid_out = 0;
  
  //�����
  ctrl.roll.core.kp = 8;
  ctrl.roll.core.ki = 0.;
  ctrl.roll.core.kd = 0.5;
  ctrl.roll.core.kt = 0.005;
  ctrl.roll.core.increment = 0.;
  ctrl.roll.core.increment_max = 300;
  ctrl.roll.core.kp_out = 0.;
  ctrl.roll.core.ki_out = 0.;
  ctrl.roll.core.kd_out = 0.;
  ctrl.roll.core.error[1] = 0;
  ctrl.roll.core.aim = 0;
  ctrl.roll.core.pid_out = 0;
  
  //ƫ����
  ctrl.yaw.core.kp = 1.5;
  ctrl.yaw.core.ki = 0.;
  ctrl.yaw.core.kd = 0;
  ctrl.yaw.core.kt = 0.005;
  ctrl.yaw.core.increment = 0.;
  ctrl.yaw.core.increment_max = 300;
  ctrl.yaw.core.kp_out = 0.;
  ctrl.yaw.core.ki_out = 0.;
  ctrl.yaw.core.kd_out = 0.;
  ctrl.yaw.core.error[1] = 0;  
  ctrl.yaw.core.aim = 0;
  ctrl.yaw.core.pid_out = 0;
}


struct _ctrl ctrl;
int16_t Moto_duty[4];

extern float GY;

/***************************************************/
/*void CONTROL(float rol, float pit, float yaw)    */
/*���룺rol   �����                               */
/*      pit   ������                               */
/*			yaw   ����                                 */
/*�����                                           */
/*��ע������PID ����   �⻷���ǶȻ�������PID����    */
/*                     �ڻ������ٶȻ�������PD����  */
/***************************************************/
void CONTROL(float rol, float pit, float yaw)   
{
  if(ctrl.ctrlRate >= 2)
  {
    //*****************�⻷(�ǶȻ�)PID**************************//
    //��������///////////////
    ctrl.pitch.shell.error[1] = pit +  ctrl.pitch.shell.aim;
    
    if ( ctrl.pitch.shell.error[1] < 5 && ctrl.pitch.shell.error[1] > -5 )
    {
      ctrl.pitch.shell.increment += ctrl.pitch.shell.error[1];
    }
    else
    {
      ctrl.pitch.shell.increment = 0;
    }
    
    //limit for the max increment
    if(ctrl.pitch.shell.increment > ctrl.pitch.shell.increment_max)  	
    { ctrl.pitch.shell.increment = ctrl.pitch.shell.increment_max; }
    
    else if(ctrl.pitch.shell.increment < -ctrl.pitch.shell.increment_max)		
    { ctrl.pitch.shell.increment = -ctrl.pitch.shell.increment_max; }
    
    ctrl.pitch.shell.pid_out = ctrl.pitch.shell.kp * ctrl.pitch.shell.error[1] 
                                + ctrl.pitch.shell.ki * ctrl.pitch.shell.increment 
                                  + ctrl.pitch.shell.kd * (ctrl.pitch.shell.error[1] - ctrl.pitch.shell.error[0]);
    ctrl.pitch.shell.error[0] = ctrl.pitch.shell.error[1];
    
    
    
    //�������//////////////
    ctrl.roll.shell.error[1] = rol + ctrl.roll.shell.aim ;
    
    if ( ctrl.roll.shell.error[1] < 5 && ctrl.roll.shell.error[1] > -5 )
    {
      ctrl.roll.shell.increment += ctrl.roll.shell.error[1] ;
    }
    else
    {
      ctrl.roll.shell.increment = 0;
    }
    
    //limit for the max increment
    if(ctrl.roll.shell.increment > ctrl.roll.shell.increment_max)  	
    { ctrl.roll.shell.increment = ctrl.roll.shell.increment_max; }
    
    else if(ctrl.roll.shell.increment < -ctrl.roll.shell.increment_max)
    {  ctrl.roll.shell.increment = -ctrl.roll.shell.increment_max; }
    
    ctrl.roll.shell.pid_out  = ctrl.roll.shell.kp * ctrl.roll.shell.error[1]  
                                + ctrl.roll.shell.ki * ctrl.roll.shell.increment 
                                  + ctrl.roll.shell.kd * (ctrl.roll.shell.error[1]  - ctrl.roll.shell.error[0] );
    ctrl.roll.shell.error[0]  = ctrl.roll.shell.error[1] ;
    
    
    //�������/////////////
    ctrl.yaw.shell.pid_out = ctrl.yaw.shell.kp * ctrl.yaw.shell.aim  + ctrl.yaw.shell.kd * MPU6050_GYRO_LAST.Z;
    ctrl.ctrlRate = 0; 
  }
  ctrl.ctrlRate ++;
  
  
  //********************�ڻ�(���ٶȻ�)PD*********************************//
  /**���ٶȷ�����Ҫ�궨***/
  //roll
  ctrl.roll.core.error[1] = MPU6050_GYRO_LAST.X ;
  
  ctrl.roll.core.kp_out = ctrl.roll.core.kp * (ctrl.roll.shell.pid_out + MPU6050_GYRO_LAST.X * Gyro_Gr * RtA);
  ctrl.roll.core.kd_out = ctrl.roll.core.kd * (ctrl.roll.core.error[1] - ctrl.roll.core.error[0]);
  
  ctrl.roll.core.error[0] = ctrl.roll.core.error[1];
  
  //pitch
  ctrl.pitch.core.error[1] = MPU6050_GYRO_LAST.Y ;
  
  ctrl.pitch.core.kp_out = ctrl.pitch.core.kp * (ctrl.pitch.shell.pid_out  + MPU6050_GYRO_LAST.Y * Gyro_Gr * RtA);
  ctrl.pitch.core.kd_out = ctrl.pitch.core.kd * (ctrl.pitch.core.error[1] - ctrl.pitch.core.error[0]);
  
  ctrl.pitch.core.error[0] = ctrl.pitch.core.error[1];
  
  //YAW
  ctrl.yaw.core.error[1] = MPU6050_GYRO_LAST.Z ;
  
  ctrl.yaw.core.kp_out = ctrl.yaw.core.kp * (ctrl.yaw.shell.pid_out + MPU6050_GYRO_LAST.Z * Gyro_Gr * RtA);
  ctrl.yaw.core.kd_out = ctrl.yaw.core.kd * (ctrl.yaw.core.error[1] - ctrl.yaw.core.error[0]);
  
  ctrl.yaw.core.error[0] = ctrl.yaw.core.error[1];
  
  
  ctrl.roll.core.pid_out = ctrl.roll.core.kp_out + ctrl.roll.core.kd_out;
  ctrl.pitch.core.pid_out = ctrl.pitch.core.kp_out + ctrl.pitch.core.kd_out;
  ctrl.yaw.core.pid_out = ctrl.yaw.core.kp_out + ctrl.yaw.core.kd_out;
  
  
}



uint32 pwm_duty[4] = {0};


//������̬����
void CTRL_OUT()
{
  uint8 i = 0;
  
  uint32 date_throttle	= throttle/cos(Q_ANGLE.X/RtA)/cos(Q_ANGLE.Y/RtA);
  
  pwm_duty[0] = throttle
    - (int32)(ctrl.roll.core.pid_out)
      + (int32)(ctrl.yaw.core.pid_out);
  
  pwm_duty[1] = throttle
    + (int32)(ctrl.pitch.core.pid_out) 
      - (int32)(ctrl.yaw.core.pid_out);

  pwm_duty[2] = throttle
    + (int32)(ctrl.roll.core.pid_out) 
     + (int32)(ctrl.yaw.core.pid_out);
  
  pwm_duty[3] = throttle 
                            - (int32)(ctrl.pitch.core.pid_out) 
                              - (int32)(ctrl.yaw.core.pid_out);
  for (i = 0;i< 4 ;i++)
  {
    if (pwm_duty[i] > 8000 )
    {
      pwm_duty[i] = 8000;
    }
      
  }
  //����ͣ��״̬
  if (!flag_lock && !flag_dump && throttle >= 5100  )
  {  
     
    LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch0,pwm_duty[0]);
    
//    LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch1,pwm_duty[1]);

    LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch2,pwm_duty[2]);
    
//    LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch3,pwm_duty[3]);   
    
  }
  else
  {
    LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch0,5000);//�ɿ�ֹͣ
    LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch1,5000);
    LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch2,5000);
    LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch3,5000);
  }
}





