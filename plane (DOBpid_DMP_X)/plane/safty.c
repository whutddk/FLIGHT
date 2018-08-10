#include "common.h"
#include "include.h"

uint8 flag_dump = 1;//继电器掉电命令
uint8 flag_lock = 1;//飞控锁定命令


void MCU_Reset()//看门狗复位处理器
{
  LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch0,1000);//飞控停止
  LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch1,1000);
  LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch2,1000);
  LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch3,1000);
  LPLD_WDOG_Init(100);
  LPLD_WDOG_Enable();   //看门狗复位
  delay(300000); //强制卡死
}

void safty_lock()
{
  LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch0,1000);//飞控解锁
  LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch1,1000);
  LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch2,1000);
  LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch3,1000);
  
}

void LED_init()
{
  GPIO_InitTypeDef LED_gpio;
  LED_gpio.GPIO_PTx = PTA;
  LED_gpio.GPIO_Pins = GPIO_Pin17;
  LED_gpio.GPIO_Dir = DIR_OUTPUT;
  LED_gpio.GPIO_Output = OUTPUT_H;
  LED_gpio.GPIO_PinControl =  IRQC_DIS | OUTPUT_DSH;
  LPLD_GPIO_Init(LED_gpio);
  
  LPLD_GPIO_Output_b(PTA, 17, 1);
}

void BEEP_init()
{
  GPIO_InitTypeDef beep_gpio;
  beep_gpio.GPIO_PTx = PTE;
  beep_gpio.GPIO_Pins = GPIO_Pin12;
  beep_gpio.GPIO_Dir = DIR_OUTPUT;
  beep_gpio.GPIO_Output = OUTPUT_H;
  beep_gpio.GPIO_PinControl =  IRQC_DIS | OUTPUT_DSH;
  LPLD_GPIO_Init(beep_gpio);
  
  LPLD_GPIO_Output_b(PTE, 12, 1);
}

void check_unbalance()//检测飞机是否过度倾斜，是则复位
{
  if (( !flag_lock && !flag_dump && throttle >= 1010 )
      &&
      (Roll - roll_quiet > 10 ||Roll - roll_quiet  < -10 || Pitch - pitch_quiet > 10 || Pitch - pitch_quiet  < -10))//已经起飞，且xy方向过度倾斜
  {
    MCU_Reset();
  }
  _BZ_ON;
}

void quick_land()
{
  if ( !flag_lock && !flag_dump && throttle >= 1300 )
  {
    throttle = 1300;
  }
}