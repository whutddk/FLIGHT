#include "common.h"
#include "include.h"

uint8 flag_dump = 1;//�̵�����������
uint8 flag_lock = 1;//�ɿ���������


void MCU_Reset()//���Ź���λ������
{
  LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch0,5000);//�ɿ�ֹͣ
  LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch1,5000);
  LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch2,5000);
  LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch3,5000);
  LPLD_WDOG_Init(10);
  LPLD_WDOG_Enable();   //���Ź���λ
  delay(300000); //ǿ�ƿ���
}

void safty_lock()
{
  LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch0,5000);//�ɿؽ���
  LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch1,5000);
  LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch2,5000);
  LPLD_FTM_PWM_ChangeDuty(FTM0,FTM_Ch3,5000);
  
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
  
  LPLD_GPIO_Output_b(PTD, 10, 1);
}

void check_unbalance()//���ɻ��Ƿ������б������λ
{
  if (( !flag_lock && !flag_dump && throttle >= 5100 )
      &&
      (Q_ANGLE.X >10||Q_ANGLE.X < -10 || Q_ANGLE.Y > 10 || Q_ANGLE.Y < -10))//�Ѿ���ɣ���xy���������б
  {
    MCU_Reset();
  }
  _BZ_ON;
}

void quick_land()
{
  if ( !flag_lock && !flag_dump && throttle >= 6500 )
  {
    throttle = 6500;
  }
}