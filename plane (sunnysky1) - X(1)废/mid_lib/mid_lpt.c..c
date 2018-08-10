/***脉冲累加器配置 C 文件
***李锐戈
***2014 11 28
2015 2 14 李锐戈 编码器重新映射
*/

#include "common.h"
#include "include.h"

void lpt_init()
{
  LPTMR_InitTypeDef lpt_inittype;
  lpt_inittype.LPTMR_Mode = LPTMR_MODE_PLACC;
  lpt_inittype.LPTMR_PluseAccInput = LPTMR_ALT2;
  lpt_inittype.LPTMR_IntEnable = FALSE ;
  lpt_inittype.LPTMR_Isr = NULL;
  LPLD_LPTMR_Init(lpt_inittype); 
}

//uint16 pluse_count_h = 0; //定义计数器高位
uint32 pluse_count_l = 0; //定义计数器低位

void get_count()//需要在主循环内不断调用
{
  pluse_count_l += LPLD_LPTMR_GetPulseAcc(); //可以直接拆了LPTMR的getpulse函数
  LPLD_LPTMR_ResetCounter() ;
    
    lpt_init();
    
    
}



