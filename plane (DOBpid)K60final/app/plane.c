/**
* --------------基于"拉普兰德K60底层库V3"的工程（plane）-----------------
* @file plane.c
* @version 0.0
* @date 2013-9-29
* @brief 关于该工程的描述
*
* 版权所有:北京拉普兰德电子技术有限公司
* http://www.lpld.cn
* mail:support@lpld.cn
* 硬件平台:  LPLD K60 Card / LPLD K60 Nano
*
* 本工程基于"拉普兰德K60底层库V3"开发，
* 所有开源代码均在"lib"文件夹下，用户不必更改该目录下代码，
* 所有用户工程需保存在"project"文件夹下，以工程名定义文件夹名，
* 底层库使用方法见相关文档。 
*
*/
#include "common.h"
#include "include.h"

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
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch0,5000);//飞控停止
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch1,5000);
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch2,5000);
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch3,5000);
      LPLD_GPIO_Output_b(PTD, 10, 0);//飞机掉电
    }
    
    //安全系统，电调锁定
    if( flag_lock )
    {
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch0,5000);//飞控停止
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch1,5000);
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch2,5000);
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch3,5000);
      
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch0,5000);//电调锁定
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch1,5000);
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch2,5000);
      LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch3,5000);
    }
    
  } 
}


