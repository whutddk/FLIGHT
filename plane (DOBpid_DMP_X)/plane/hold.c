/*ͷ�ļ�����CTL_h_����
*/

#include "include.h"

#define HOLD false



struct _hold hold;


//���Զ����㷨
//��XX�ϸ���֣��õ����PID����Ϊ������
//����ԽǶȽ���ʱ�����
void add_angle()//�ϸ�ʱ����һ�Σ����֣�
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


//����PID�����Ŀ��Ƕ�ֵ���޷���һ��
//ע����ֵ��ͺ�ЧӦ
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

