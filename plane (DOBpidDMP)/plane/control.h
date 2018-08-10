#ifndef _CONTROL_H_
#define _CONTROL_H_

#define RtA 		57.324841		//����ʲô����

extern uint32 throttle;  //������
extern uint32 pwm_duty[4];
extern float pitch_rc;
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
  float error[2];
  float aim;
  float pid_out;
};



struct _tache{
  struct _pid shell;
  struct _pid core;	
};

struct _ctrl{
  uint8  ctrlRate;
  struct _tache pitch;    
  struct _tache roll;  
  struct _tache yaw;
  struct _pid high;
};

struct _hold{
  struct _pid pitch;
  struct _pid roll;
};

extern struct _ctrl ctrl;
extern struct _hold hold;

void ctrl_init();
void CONTROL(float rol, float pit, float yaw);

void CTRL_OUT();

//�ϸ�ʱ����һ�Σ����֣�
void add_angle();

//����PID�����Ŀ��Ƕ�ֵ���޷���һ��
//ע����ֵ��ͺ�ЧӦ
void hold_point();


#endif
