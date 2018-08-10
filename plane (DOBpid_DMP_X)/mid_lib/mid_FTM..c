/***
***FTM模块中层库
李锐戈
2014 11 22

***
*/

#include "common.h"
#include "include.h"


/////////////20kHZ
void PWM0_init()                 //FTM0通道多，8通，用于控制PWM
{ 
  FTM_InitTypeDef ftm_init_struct;
  ftm_init_struct.FTM_Ftmx = FTM0;	//使能FTM0通道
  ftm_init_struct.FTM_Mode = FTM_MODE_PWM;	//使能PWM模式
  ftm_init_struct.FTM_PwmFreq = 100;	//PWM频率500Hz
  ftm_init_struct.FTM_PwmDeadtimeCfg = DEADTIME_CH23;
  ftm_init_struct.FTM_PwmDeadtimeVal=0;
  LPLD_FTM_Init(ftm_init_struct);
  
    LPLD_FTM_PWM_Enable(FTM0, //使用FTM0
                      FTM_Ch0, //使能Ch0通道
                      10, //初始化角度0度
                      PTC1, //使用Ch0通道的PTC1引脚
                      ALIGN_LEFT        //脉宽左对齐
                      );    
    LPLD_FTM_PWM_Enable(FTM0, //使用FTM0
                      FTM_Ch1, //使能Ch0通道
                      10, //初始化角度0度
                      PTC2, //使用Ch0通道的PTC1引脚
                      ALIGN_LEFT        //脉宽左对齐
                      ); 
    LPLD_FTM_PWM_Enable(FTM0, //使用FTM0
                      FTM_Ch2, //使能Ch0通道
                      10, //初始化角度0度
                      PTC3, //使用Ch0通道的PTC1引脚
                      ALIGN_LEFT        //脉宽左对齐
                      ); 
    LPLD_FTM_PWM_Enable(FTM0, //使用FTM0
                      FTM_Ch3, //使能Ch0通道
                      10, //初始化角度0度
                      PTC4, //使用Ch0通道的PTC1引脚
                      ALIGN_LEFT        //脉宽左对齐
                      ); 
}/*方PWM输出*/


