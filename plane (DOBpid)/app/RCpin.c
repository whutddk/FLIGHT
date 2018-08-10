#include "include.h"


double chn[20] = {10.999999};
uint8 miss_cnt = 0;

void check_pin()
{
  uint8 RC_BUFF[32] = {0,0};
  LPLD_Nrf24L01_RecvFrame(RC_BUFF, 32);
  
  
  //¼ì²éµÆËþÐÅºÅBUFF¡¾16¡¿
  if ( RC_BUFF[16] != 121 )
  {
    miss_cnt ++;
    if (miss_cnt == 10)
    {
      MCU_Reset();
    }    
  }
  else
  {
    miss_cnt = 0;
  }
}


