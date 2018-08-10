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
  ftm_init_struct.FTM_PwmFreq = 500;	//PWM频率500Hz
  ftm_init_struct.FTM_PwmDeadtimeCfg = DEADTIME_CH23;
  ftm_init_struct.FTM_PwmDeadtimeVal=0;
  LPLD_FTM_Init(ftm_init_struct);
  
    LPLD_FTM_PWM_Enable(FTM0, //使用FTM0
                      FTM_Ch0, //使能Ch0通道
                      2000, //初始化角度0度
                      PTC1, //使用Ch0通道的PTC1引脚
                      ALIGN_LEFT        //脉宽左对齐
                      );    
    LPLD_FTM_PWM_Enable(FTM0, //使用FTM0
                      FTM_Ch1, //使能Ch0通道
                      2000, //初始化角度0度
                      PTC2, //使用Ch0通道的PTC1引脚
                      ALIGN_LEFT        //脉宽左对齐
                      ); 
    LPLD_FTM_PWM_Enable(FTM0, //使用FTM0
                      FTM_Ch2, //使能Ch0通道
                      2000, //初始化角度0度
                      PTC3, //使用Ch0通道的PTC1引脚
                      ALIGN_LEFT        //脉宽左对齐
                      ); 
    LPLD_FTM_PWM_Enable(FTM0, //使用FTM0
                      FTM_Ch3, //使能Ch0通道
                      2000, //初始化角度0度
                      PTC4, //使用Ch0通道的PTC1引脚
                      ALIGN_LEFT        //脉宽左对齐
                      ); 
}/*方PWM输出*/


void qd1_init()                 //FTM1通道，2通，用于控制PWM
{ 
 //配置正交解码功能参数
  FTM_InitTypeDef ftm_init_struct;
  ftm_init_struct.FTM_Ftmx = FTM1;              //只有FTM1和FTM2有正交解码功能
  ftm_init_struct.FTM_Mode = FTM_MODE_QD;       //正交解码功能
  ftm_init_struct.FTM_QdMode = QD_MODE_PHAB;    //AB相输入模式
  //初始化FTM
  LPLD_FTM_Init(ftm_init_struct);
  //使能AB相输入通道
  //PTB0引脚接A相输入、PTB1引脚接B相输入
  LPLD_FTM_QD_Enable(FTM1, PTA12, PTA13);
}

void qd2_init(void) //正交解码AB相模式初始化
{
  //配置正交解码功能参数
  FTM_InitTypeDef ftm_init_struct;
  ftm_init_struct.FTM_Ftmx = FTM2;              //只有FTM1和FTM2有正交解码功能
  ftm_init_struct.FTM_Mode = FTM_MODE_QD;       //正交解码功能
  ftm_init_struct.FTM_QdMode = QD_MODE_PHAB;    //AB相输入模式
  //初始化FTM
  LPLD_FTM_Init(ftm_init_struct);
  //使能AB相输入通道
  //PTB0引脚接A相输入、PTB1引脚接B相输入
  LPLD_FTM_QD_Enable(FTM2, PTB18, PTB19);
}