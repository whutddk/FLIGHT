//飞行姿态控制

#include "common.h"
#include "include.h"

struct _ctrl ctrl; //控制复合结构体

uint32 throttle = 0; //限流阀

void ctrl_init()
{
  //俯仰角
  ctrl.pitch.kp = 20.;
  ctrl.pitch.ki = 0.00;
  ctrl.pitch.kd = 0.03;
  ctrl.pitch.kt = 0.005;
  ctrl.pitch.increment = 0.;
  ctrl.pitch.increment_max = 100;
  ctrl.pitch.kp_out = 0.;
  ctrl.pitch.ki_out = 0.;
  ctrl.pitch.kd_out = 0.;
  ctrl.roll.error[0] = 0;
  ctrl.roll.error[1] = 0;
  ctrl.roll.error[2] = 0;
  ctrl.pitch.aim= 0;//-4;
  ctrl.pitch.pid_out = 0;
  
  //横滚角
  ctrl.roll.kp = 20;
  ctrl.roll.ki = 0.;
  ctrl.roll.kd = 0.03;
  ctrl.roll.kt = 0.005;
  ctrl.roll.increment = 0.;
  ctrl.roll.increment_max = 100;
  ctrl.roll.kp_out = 0.;
  ctrl.roll.ki_out = 0.;
  ctrl.roll.kd_out = 0.;
  ctrl.roll.error[0] = 0;
  ctrl.roll.error[1] = 0;
  ctrl.roll.error[2] = 0;
  ctrl.roll.aim = -3;//4.5;
  ctrl.roll.pid_out = 0;
  
  //偏航角
  ctrl.yaw.kp = 0.;
  ctrl.yaw.ki = 0.;
  ctrl.yaw.kd = 0;
  ctrl.yaw.kt = 0.005;
  ctrl.yaw.increment = 0.;
  ctrl.yaw.increment_max = 300;
  ctrl.yaw.kp_out = 0.;
  ctrl.yaw.ki_out = 0.;
  ctrl.yaw.kd_out = 0.;
  ctrl.roll.error[0] = 0;
  ctrl.roll.error[1] = 0;
  ctrl.roll.error[2] = 0;
  ctrl.yaw.aim = 0;
  ctrl.yaw.pid_out = 0;
  
  //高度控制
  ctrl.high.kp = 0.;
  ctrl.high.ki = 0.;
  ctrl.high.kd = 0;
  ctrl.high.kt = 0.02;
  ctrl.high.increment = 0.;
  ctrl.high.increment_max = 300;
  ctrl.high.kp_out = 0.;
  ctrl.high.ki_out = 0.;
  ctrl.high.kd_out = 0.;
  ctrl.roll.error[0] = 0;
  ctrl.roll.error[1] = 0;
  ctrl.roll.error[2] = 0;
  ctrl.high.aim = 500;
  ctrl.high.pid_out = 0;
}


void PITCH_PID()
{
  //不在停车状态
  if (!flag_dump && !flag_lock && throttle >= 100 ) 
  {
    ctrl.pitch.error[2] = Q_ANGLE.Y - ctrl.pitch.aim;
    
    ctrl.pitch.increment += ctrl.pitch.ki * ctrl.pitch.error[2] * ctrl.pitch.kt;
    
    //抗积分饱和
    if (ctrl.pitch.increment > ctrl.pitch.increment_max)
    {
      ctrl.pitch.increment = ctrl.pitch.increment_max;
    }
    if (ctrl.pitch.increment < -ctrl.pitch.increment_max)
    {
      ctrl.pitch.increment = -ctrl.pitch.increment_max;
    }
    
    //pid
    ctrl.pitch.pid_out = ctrl.pitch.kp * ctrl.pitch.error[2];
    ctrl.pitch.pid_out += ctrl.pitch.increment;  
    ctrl.pitch.pid_out += ctrl.pitch.kd * MPU6050_GYRO_LAST.Y;
    
    //最大值控制
    if ( ctrl.pitch.pid_out > 500 )
    {
      ctrl.pitch.pid_out = 500;
    }
    if ( ctrl.pitch.pid_out < -500 )
    {
      ctrl.pitch.pid_out = -500;
    }
    //    push(3, ctrl.pitch.increment);
    //    push(4,ctrl.pitch.pid_out );
  }
}

void ROLL_PID()
{
  //不在停车状态
  if ( !flag_lock && !flag_dump && throttle >= 100 )
  {
    ctrl.roll.error[2] = Q_ANGLE.X - ctrl.roll.aim;
    
    ctrl.roll.increment += ctrl.roll.ki * ctrl.roll.error[2] * ctrl.roll.kt;
    
    //抗积分饱和
    if (ctrl.roll.increment > ctrl.roll.increment_max)
    {
      ctrl.roll.increment = ctrl.roll.increment_max;
    }
    if (ctrl.roll.increment < -ctrl.roll.increment_max)
    {
      ctrl.roll.increment = -ctrl.roll.increment_max;
    }
    
    //pid
    ctrl.roll.pid_out = ctrl.roll.kp * ctrl.roll.error[2];
    ctrl.roll.pid_out += ctrl.roll.increment;  
    ctrl.roll.pid_out += ctrl.roll.kd * MPU6050_GYRO_LAST.X;
    
    //最大值控制
    if ( ctrl.roll.pid_out > 500 )
    {
      ctrl.roll.pid_out = 500;
    }
    if ( ctrl.roll.pid_out < -500 )
    {
      ctrl.roll.pid_out = -500;
    }
    //    push(3, ctrl.pitch.increment);
    //    push(4,ctrl.pitch.pid_out );
  }
}

void YAW_PID()
{
  //不在停车状态
  if ( !flag_lock && !flag_dump && throttle >= 100 )
  {
    ctrl.yaw.error[2] = north - ctrl.yaw.aim;
    
    ctrl.yaw.increment += ctrl.yaw.ki * ctrl.yaw.error[2] * ctrl.yaw.kt;
    
    //抗积分饱和
    if (ctrl.yaw.increment > ctrl.yaw.increment_max)
    {
      ctrl.yaw.increment = ctrl.yaw.increment_max;
    }
    if (ctrl.yaw.increment < -ctrl.yaw.increment_max)
    {
      ctrl.yaw.increment = -ctrl.yaw.increment_max;
    }
    
    //pid
    ctrl.yaw.pid_out = ctrl.yaw.kp * ctrl.yaw.error[2];
    ctrl.yaw.pid_out += ctrl.yaw.increment;  
    ctrl.yaw.pid_out += -ctrl.yaw.kd * MPU6050_GYRO_LAST.Z;
    
    //最大值控制
    if ( ctrl.yaw.pid_out > 200 )
    {
      ctrl.yaw.pid_out = 200;
    }
    if ( ctrl.yaw.pid_out < -200 )
    {
      ctrl.yaw.pid_out = -200;
    }
    
  }
  
}

void HIGH_PID()//设置角度PID算法，增量式
{

  
  ctrl.high.error[2] = ctrl.high.aim - distance_current;
  
  ctrl.high.pid_out +=   ( ctrl.high.kp * ( ctrl.high.error[2] - ctrl.high.error[1] ) );
  ctrl.high.pid_out +=  ( ctrl.high.ki * ctrl.high.error[2] * ctrl.high.kt);//时间系数统一处理
  ctrl.high.pid_out +=  ( ctrl.high.kd * ( ctrl.high.error[2] - 2 * ctrl.high.error[1] + ctrl.high.error[0] )/ctrl.high.kt );
 
  ctrl.high.error[0] = ctrl.high.error[1];
  ctrl.high.error[1] = ctrl.high.error[2];
  
  //pwm_duty1 += result ;
    push(13, ctrl.high.pid_out);
  if ( ctrl.high.pid_out > throttle )
  {
    ctrl.high.pid_out = throttle;
  }
  if ( ctrl.high.pid_out < 0 )
  {
    ctrl.high.pid_out = 0;
  }
  
  if ( ctrl.high.pid_out > 2000 )
  {
    ctrl.high.pid_out = 2000;
  }

  
}

//飞行姿态控制
void CTRL_OUT()
{
  //不在停车状态
  if (!flag_lock && !flag_dump && throttle >= 100  )
 {  
    LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch0,5000 + (int32)ctrl.high.pid_out
                            + (int32)(ctrl.pitch.pid_out)
                             + (int32)(ctrl.yaw.pid_out));
    
    LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch1,5000 + (int32)ctrl.high.pid_out
                            + (int32)(ctrl.roll.pid_out) 
                              - (int32)(ctrl.yaw.pid_out));
    
    LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch2,5000 + (int32)ctrl.high.pid_out
                            - (int32)(ctrl.pitch.pid_out) 
                              + (int32)(ctrl.yaw.pid_out));
    
    LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch3,5000 + (int32)ctrl.high.pid_out 
                            - (int32)(ctrl.roll.pid_out) 
                              - (int32)(ctrl.yaw.pid_out));
    
    
    
  }
  else
  {
    LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch0,5000);//飞控停止
    LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch1,5000);
    LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch2,5000);
    LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch3,5000);
  }
}





