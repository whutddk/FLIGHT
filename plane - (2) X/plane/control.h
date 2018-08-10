#ifndef _CONTROL_H_
#define _CONTROL_H_

extern uint32 throttle;  //ÏÞÁ÷·§

struct _pid{
  float kp;
  float ki;
  float kd;
  float kt;
  float increment;
  float increment_max;
  float kp_out;
  float ki_out;
  float kd_out;
  float error;
  float aim;
  float pid_out;
};

struct _ctrl{
  struct _pid pitch;    
  struct _pid roll;  
  struct _pid yaw;  
  struct _pid high;
};

extern struct _ctrl ctrl;

void ctrl_init();
void PITCH_PID();
void YAW_PID();
void ROLL_PID();
void CTRL_OUT();

#endif
