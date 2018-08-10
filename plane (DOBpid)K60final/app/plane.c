/**
* --------------����"��������K60�ײ��V3"�Ĺ��̣�plane��-----------------
* @file plane.c
* @version 0.0
* @date 2013-9-29
* @brief ���ڸù��̵�����
*
* ��Ȩ����:�����������µ��Ӽ������޹�˾
* http://www.lpld.cn
* mail:support@lpld.cn
* Ӳ��ƽ̨:  LPLD K60 Card / LPLD K60 Nano
*
* �����̻���"��������K60�ײ��V3"������
* ���п�Դ�������"lib"�ļ����£��û����ظ��ĸ�Ŀ¼�´��룬
* �����û������豣����"project"�ļ����£��Թ����������ļ�������
* �ײ��ʹ�÷���������ĵ��� 
*
*/
#include "common.h"
#include "include.h"

void main (void)
{
  
  //��ȫϵͳ
  flag_dump = 1;//�̵�����������
  flag_lock = 1;//�ɿ���������
  BEEP_init();
  _BZ_ON;
  delay(1000000);
  _BZ_OFF;
  all_init();
  while(1)
  {
    //��ȫϵͳ���ɻ�����
    if ( flag_dump )
    {
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch0,5000);//�ɿ�ֹͣ
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch1,5000);
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch2,5000);
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch3,5000);
      LPLD_GPIO_Output_b(PTD, 10, 0);//�ɻ�����
    }
    
    //��ȫϵͳ���������
    if( flag_lock )
    {
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch0,5000);//�ɿ�ֹͣ
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch1,5000);
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch2,5000);
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch3,5000);
      
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch0,5000);//�������
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch1,5000);
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch2,5000);
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch3,5000);
    }
    
  } 
}


