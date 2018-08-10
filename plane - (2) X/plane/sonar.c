#include "include.h"

uint16 distance_current = 0;



void sonar()//测距并计算返回
{
  LPLD_UART_PutChar(UART4, 0x55);//请求距离 
  distance_current = LPLD_UART_GetChar(UART4);
  distance_current = distance_current << 8;
  distance_current |= LPLD_UART_GetChar(UART4);
  push(12,(int16)(distance_current) );
}