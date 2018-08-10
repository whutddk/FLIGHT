/*头文件放在CTL_h_里面
*/

#include "include.h"

#define HOLD false



struct _hold hold;


//惯性定点算法
//对XX严格积分，得到结果PID后作为打舵输出
//建议对角度进行时间积分
void add_angle()//严格定时调用一次（积分）
{
  if ( !flag_dump && !flag_lock && throttle >= 1010 )
  {
    hold.pitch.error[1] += ( Pitch - pitch_quiet );
    hold.roll.error[1] += ( Roll - roll_quiet ) ; 
  }
  else 
  {
     hold.pitch.error[1] = 0;
     hold.roll.error[1] = 0;       
  }
}


//定点PID，输出目标角度值（限幅第一）
//注意积分的滞后效应
void hold_point()
{
  if ( !flag_dump && !flag_lock && throttle >= 1010 )
  {
    hold.pitch.kp_out = hold.pitch.kp * (hold.pitch.error[1]);
    hold.pitch.kd_out = hold.pitch.kd * (hold.pitch.error[1] - hold.pitch.error[0]); 
    hold.pitch.error[0] = hold.pitch.error[1];
    
    hold.roll.kp_out = hold.roll.kp * (hold.roll.error[1]);
    hold.roll.kd_out = hold.roll.kd * (hold.roll.error[1] - hold.roll.error[0]);
    hold.roll.error[0] = hold.roll.error[1];
    
    hold.pitch.pid_out = hold.pitch.kp_out + hold.pitch.kd_out;
    hold.roll.pid_out =  hold.roll.kp_out +  hold.roll.kd_out;
    
    
    ctrl.pitch.shell.aim = hold.pitch.pid_out;
    ctrl.roll.shell.aim = hold.roll.pid_out;
  }  
  else
  {
    ;
  }
}

