
/*控制频率已经将为100HZ,油门1000~2000*/

#include "common.h"
#include "include.h"


//void IMU_init();
//void MIU_getdata();




void main (void)
{
  
  //安全系统
  flag_dump = 1;//继电器掉电命令
  flag_lock = 1;//飞控锁定命令
  BEEP_init();
  _BZ_ON;
  delay(1000000);
  _BZ_OFF;
  all_init();

  while(1)
  {

    //安全系统，飞机掉电
    if ( flag_dump )
    {
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch0,1000);//飞控停止
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch1,1000);
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch2,1000);
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch3,1000);
      //LPLD_GPIO_Output_b(PTD, 10, 0);//飞机掉电
    }
    
    //安全系统，电调锁定
    if( flag_lock )
    {
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch0,1000);//飞控停止
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch1,1000);
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch2,1000);
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch3,1000);
      
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch0,1000);//电调锁定
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch1,1000);
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch2,1000);
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch3,1000);
    }
    
  } 
}


