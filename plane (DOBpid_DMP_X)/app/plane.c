
/*����Ƶ���Ѿ���Ϊ100HZ,����1000~2000*/

#include "common.h"
#include "include.h"


//void IMU_init();
//void MIU_getdata();




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
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch0,1000);//�ɿ�ֹͣ
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch1,1000);
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch2,1000);
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch3,1000);
      //LPLD_GPIO_Output_b(PTD, 10, 0);//�ɻ�����
    }
    
    //��ȫϵͳ���������
    if( flag_lock )
    {
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch0,1000);//�ɿ�ֹͣ
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch1,1000);
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch2,1000);
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch3,1000);
      
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch0,1000);//�������
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch1,1000);
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch2,1000);
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch3,1000);
    }
    
  } 
}


