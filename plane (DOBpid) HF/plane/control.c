//飞行姿态控制

#include "common.h"
#include "include.h"

struct _ctrl ctrl; //控制复合结构体


uint32 throttle = 0; //限流阀
//float pitch_rc = 0.;


void ctrl_init()
{
  //俯仰角
  ctrl.pitch.shell.kp = 3.9;
  ctrl.pitch.shell.ki = 0.0;
  ctrl.pitch.shell.kd = 0.0;
  ctrl.pitch.shell.kt = 0.005;
  ctrl.pitch.shell.increment = 0.;
  ctrl.pitch.shell.increment_max = 100;
  ctrl.pitch.shell.kp_out = 0.;
  ctrl.pitch.shell.ki_out = 0.;
  ctrl.pitch.shell.kd_out = 0.;
  ctrl.pitch.shell.error[1] = 0;
  ctrl.pitch.shell.aim= 0;
  ctrl.pitch.shell.pid_out = 0;
  
  //横滚角
  ctrl.roll.shell.kp = 3.9;
  ctrl.roll.shell.ki = 0.0;
  ctrl.roll.shell.kd = 0.0;
  ctrl.roll.shell.kt = 0.005;
  ctrl.roll.shell.increment = 0.;
  ctrl.roll.shell.increment_max = 100;
  ctrl.roll.shell.kp_out = 0.;
  ctrl.roll.shell.ki_out = 0.;
  ctrl.roll.shell.kd_out = 0.;
  ctrl.roll.shell.error[1] = 0;
  ctrl.roll.shell.aim = 0;
  ctrl.roll.shell.pid_out = 0;
  
  //偏航角
  ctrl.yaw.shell.kp = 0.;
  ctrl.yaw.shell.ki = 0.;
  ctrl.yaw.shell.kd = 0;
  ctrl.yaw.shell.kt = 0.005;
  ctrl.yaw.shell.increment = 0.;
  ctrl.yaw.shell.increment_max = 100;
  ctrl.yaw.shell.kp_out = 0.;
  ctrl.yaw.shell.ki_out = 0.;
  ctrl.yaw.shell.kd_out = 0.;
  ctrl.yaw.shell.error[1] = 0;  
  ctrl.yaw.shell.aim = 0;
  ctrl.yaw.shell.pid_out = 0;
  
  ctrl.pitch.core.kp = 10.6;
  ctrl.pitch.core.ki = 0.;
  ctrl.pitch.core.kd = 1.2;
  ctrl.pitch.core.kt = 0.005;
  ctrl.pitch.core.increment = 0.;
  ctrl.pitch.core.increment_max = 300;
  ctrl.pitch.core.kp_out = 0.;
  ctrl.pitch.core.ki_out = 0.;
  ctrl.pitch.core.kd_out = 0.;
  ctrl.pitch.core.error[1] = 0;
  ctrl.pitch.core.aim= 0;
  ctrl.pitch.core.pid_out = 0;
  
  //横滚角
  ctrl.roll.core.kp = 10.6;
  ctrl.roll.core.ki = 0.;
  ctrl.roll.core.kd = 1.2;
  ctrl.roll.core.kt = 0.005;
  ctrl.roll.core.increment = 0.;
  ctrl.roll.core.increment_max = 300;
  ctrl.roll.core.kp_out = 0.;
  ctrl.roll.core.ki_out = 0.;
  ctrl.roll.core.kd_out = 0.;
  ctrl.roll.core.error[1] = 0;
  ctrl.roll.core.aim = 0;
  ctrl.roll.core.pid_out = 0;
  
  //偏航角
  ctrl.yaw.core.kp = 0;
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
  
  
    //高度控制
//  ctrl.high.kp = 0;
//  ctrl.high.ki = 0;
//  ctrl.high.kd = 0.0;
//  ctrl.high.kt = 0.03;
//  ctrl.high.increment = 0.;
//  ctrl.high.increment_max = 50;
//  ctrl.high.kp_out = 0.;
//  ctrl.high.ki_out = 0.;
//  ctrl.high.kd_out = 0.;
//  ctrl.high.error[0] = 0;
//  ctrl.high.error[1] = 0;
//  ctrl.high.aim = 100;
//  ctrl.high.pid_out = 0;
}


struct _ctrl ctrl;
int16_t Moto_duty[4];

extern float GY;

/***************************************************/
/*void CONTROL(float rol, float pit, float yaw)    */
/*输入：rol   横滚角                               */
/*      pit   俯仰角                               */
/*			yaw   航向                                 */
/*输出：                                           */
/*备注：串级PID 控制   外环（角度环）采用PID调节    */
/*                     内环（角速度环）采用PD调节  */
/***************************************************/
void CONTROL(float rol, float pit, float yaw)   
{
  if(ctrl.ctrlRate >= 2)
  {
    //*****************外环(角度环)PID**************************//
    //俯仰计算///////////////
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
    
    
    
    //横滚计算//////////////
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
    
    
    //航向计算/////////////
    ctrl.yaw.shell.pid_out = ctrl.yaw.shell.kp * ctrl.yaw.shell.aim  + ctrl.yaw.shell.kd * MPU6050_GYRO_LAST.Z;
    ctrl.ctrlRate = 0; 
  }
  ctrl.ctrlRate ++;
  
  
  //********************内环(角速度环)PD*********************************//
  /**加速度方向需要标定***/
  //roll
  ctrl.roll.core.error[1] = MPU6050_GYRO_LAST.X ;
  
  ctrl.roll.core.kp_out = ctrl.roll.core.kp * (ctrl.roll.shell.pid_out + MPU6050_GYRO_LAST.X * Gyro_Gr * RtA);
  
  ctrl.roll.core.increment += ctrl.roll.core.error[1];
  
  if(ctrl.roll.core.increment > ctrl.roll.core.increment_max)  	
    { ctrl.roll.core.increment = ctrl.roll.core.increment_max; }
    
    else if(ctrl.roll.core.increment < -ctrl.roll.core.increment_max)		
    { ctrl.roll.core.increment = -ctrl.roll.core.increment_max; }

  ctrl.roll.core.ki_out = ctrl.roll.core.ki * ctrl.roll.core.increment;
  
  ctrl.roll.core.kd_out = ctrl.roll.core.kd * (ctrl.roll.core.error[1] - ctrl.roll.core.error[0]);
  
  ctrl.roll.core.error[0] = ctrl.roll.core.error[1];
  
  //pitch
  ctrl.pitch.core.error[1] = MPU6050_GYRO_LAST.Y ;
  
  ctrl.pitch.core.kp_out = ctrl.pitch.core.kp * (ctrl.pitch.shell.pid_out  + MPU6050_GYRO_LAST.Y * Gyro_Gr * RtA);
  
    ctrl.pitch.core.increment += ctrl.pitch.core.error[1];
  
  if(ctrl.pitch.core.increment > ctrl.pitch.core.increment_max)  	
    { ctrl.pitch.core.increment = ctrl.pitch.core.increment_max; }
    
    else if(ctrl.pitch.core.increment < -ctrl.pitch.core.increment_max)		
    { ctrl.pitch.core.increment = -ctrl.pitch.core.increment_max; }

  ctrl.pitch.core.ki_out = ctrl.pitch.core.ki * ctrl.pitch.core.increment;
  
  ctrl.pitch.core.kd_out = ctrl.pitch.core.kd * (ctrl.pitch.core.error[1] - ctrl.pitch.core.error[0]);
  
  ctrl.pitch.core.error[0] = ctrl.pitch.core.error[1];
  
  //YAW
  ctrl.yaw.core.error[1] = MPU6050_GYRO_LAST.Z ;
  
  ctrl.yaw.core.kp_out = ctrl.yaw.core.kp * (ctrl.yaw.shell.pid_out + MPU6050_GYRO_LAST.Z * Gyro_Gr * RtA);
  ctrl.yaw.core.kd_out = ctrl.yaw.core.kd * (ctrl.yaw.core.error[1] - ctrl.yaw.core.error[0]);
  
  ctrl.yaw.core.error[0] = ctrl.yaw.core.error[1];
  
  
  ctrl.roll.core.pid_out = ctrl.roll.core.kp_out + ctrl.roll.core.ki_out + ctrl.roll.core.kd_out;
  ctrl.pitch.core.pid_out = ctrl.pitch.core.kp_out + ctrl.pitch.core.ki_out + ctrl.pitch.core.kd_out;
  ctrl.yaw.core.pid_out = ctrl.yaw.core.kp_out + ctrl.yaw.core.kd_out;
  
  if ( ctrl.roll.core.pid_out > 600)
  {
    ctrl.roll.core.pid_out = 600;
  }
  if ( ctrl.roll.core.pid_out < -600 )
  {
    ctrl.roll.core.pid_out = -600;
  }
  
  if ( ctrl.pitch.core.pid_out > 600 )
  {
    ctrl.pitch.core.pid_out = 600;
  }
  if ( ctrl.pitch.core.pid_out < -600 )
  {
    ctrl.pitch.core.pid_out = -600;
  }
  
  if ( ctrl.yaw.core.pid_out > 600 )
  {
    ctrl.yaw.core.pid_out = 600;
  }
  if( ctrl.yaw.core.pid_out < -600 )
  {
    ctrl.yaw.core.pid_out = -600;
  }
  
  
}

//
//uint8 sonar_error = 0;
//void HIGH_PID()//设置角度PID算法，增量式
//{
//  if ( !flag_lock && !flag_dump && throttle >= 5100 )
//  {
//    ctrl.high.error[1] = ctrl.high.aim - distance_current;
//    
//
//    if ( ctrl.high.error[1] > -100 && ctrl.high.error[1] < 100 )
//    {
//      ctrl.high.increment += ctrl.high.ki * ctrl.high.error[1] * ctrl.high.kt;
//    }
//    else
//    {
//      ctrl.high.increment = 0;
//    }
//    
//        //抗积分饱和
//    if (ctrl.high.increment > ctrl.high.increment_max)
//    {
//      ctrl.high.increment = ctrl.high.increment_max;
//    }
//    if (ctrl.high.increment < -ctrl.high.increment_max)
//    {
//      ctrl.high.increment = -ctrl.high.increment_max;
//    }
//    
//    //pid
//    ctrl.high.pid_out = ctrl.high.kp * ctrl.high.error[1];
//    ctrl.high.pid_out += ctrl.high.increment;
//    ctrl.high.pid_out +=  ( ctrl.high.kd * ( ctrl.high.error[1] -  ctrl.high.error[0])/ctrl.high.kt );
//    
//    if ( ctrl.high.error[1] == ctrl.high.error[0] )
//    {
//      sonar_error ++;
//      if ( sonar_error >= 5 )
//      {
//        ctrl.high.pid_out = 0;
//      }
//    }
//    else
//    {
//      sonar_error = 0;
//    }
//      
//    
//    
//    
//    ctrl.high.error[0] = ctrl.high.error[1];
//    
//
//    push(13, (int16)ctrl.high.pid_out);
//
//    if ( ctrl.high.pid_out < 0 )
//    {
//      ctrl.high.pid_out = 0;
//    }
//    if ( ctrl.high.pid_out > 100 )
//    {
//      ctrl.high.pid_out = 100;
//    }
//  }
//  
//  //安全程序，误测则失效
//      if (ctrl.high.error[1] > 1000)
//    {
//      ctrl.high.pid_out = 0;
//    }
//}



uint32 pwm_duty[4] = {0};


//飞行姿态控制
void CTRL_OUT()
{
  uint8 i = 0;
  
  uint32 date_throttle	= (uint32)(throttle/cos(Q_ANGLE.X/RtA)/cos(Q_ANGLE.Y/RtA));
  
//  if (throttle > 10000)
//  {
//    throttle = 5000;
//  }
  push( 0,(int16)ctrl.pitch.core.pid_out );
  push( 1,(int16)ctrl.roll.core.pid_out );
  pwm_duty[0] = throttle /*+ (int32)(ctrl.high.pid_out)*/ - (int32)(ctrl.pitch.core.pid_out ) - (int32)(ctrl.roll.core.pid_out) /*+ (int32)(ctrl.yaw.core.pid_out)*/;
  
  pwm_duty[1] = throttle /* +(int32)(ctrl.high.pid_out)*/ + (int32)(ctrl.pitch.core.pid_out ) - (int32)(ctrl.roll.core.pid_out) /*- (int32)(ctrl.yaw.core.pid_out)*/;

  pwm_duty[2] = throttle /*+ (int32)(ctrl.high.pid_out)*/ + (int32)(ctrl.pitch.core.pid_out ) + (int32)(ctrl.roll.core.pid_out) /*+ (int32)(ctrl.yaw.core.pid_out)*/;
  
  pwm_duty[3] = throttle /*+ (int32)(ctrl.high.pid_out)*/ - (int32)(ctrl.pitch.core.pid_out ) + (int32)(ctrl.roll.core.pid_out) /*- (int32)(ctrl.yaw.core.pid_out)*/;
                              
  for (i = 0;i< 4 ;i++)
  {
    if (pwm_duty[i] > 9000 )
    {
      pwm_duty[0] = 5000;
      pwm_duty[1] = 5000;
      pwm_duty[2] = 5000;
      pwm_duty[3] = 5000;
    }
      
  }
  //不在停车状态
  if (!flag_lock && !flag_dump && throttle >= 5100  )
  {  
     
    LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch0,pwm_duty[0]);
    
    LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch1,pwm_duty[1]);

    LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch2,pwm_duty[2]);
    
    LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch3,pwm_duty[3]);   
    
  }
  else
  {
    LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch0,5000);//飞控停止
    LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch1,5000);
    LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch2,5000);
    LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch3,5000);
  }
}





