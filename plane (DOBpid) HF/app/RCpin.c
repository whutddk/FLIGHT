#include "include.h"


double chn[20] = {10.999999};
uint8 miss_cnt = 0;

void check_pin()
{
  
  
  uint8 RC_BUFF[32] = {0,0};
  LPLD_Nrf24L01_RecvFrame(RC_BUFF, 32);
  
  OLED_CLS();
  //�������ź�BUFF��16��
  if ( RC_BUFF[16] == 10 )
  {
      MCU_Reset();   
  }
  else if ( RC_BUFF[16] == 20 )
  {
    OLED_P8x16Str(1,0,"DA");
    miss_cnt = 0;
  }
  else
  {
//    miss_cnt ++ ;
//    if ( miss_cnt > 20 )
//    {
//      MCU_Reset();
//    }
  }
  
  
  //���ң���ź�
  if ( RC_BUFF[17] == 5 )
  {
    //ǰ�� PITCH
    OLED_P8x16Str(1,2,"AH");
    ctrl.pitch.shell.aim = 5;
  }
  else if (  RC_BUFF[17] == 6  )
  {
    //����PITCH
    OLED_P8x16Str(1,2,"BA");
    ctrl.pitch.shell.aim = -5;
  }
  else
  {
   //����
    OLED_P8x16Str(1,2,"HOLD");
    ctrl.pitch.shell.aim = 0;
  }
  
  if ( RC_BUFF[18] == 5 )
  {
    //��ROLL
    OLED_P8x16Str(1,4,"LE");
    ctrl.roll.shell.aim = -5;
  }
  else if ( RC_BUFF[18] == 6 )
  {
    //��ROLL
    OLED_P8x16Str(1,4,"RI");
    ctrl.roll.shell.aim = 5;
  }
  else
  {
    //����
    OLED_P8x16Str(1,4,"HOLD");
    ctrl.roll.shell.aim = 0;
  }
}


