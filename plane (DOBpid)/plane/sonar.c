#include "include.h"

uint16 distance_current = 0;



void sonar()//��ಢ���㷵��
{
  LPLD_UART_PutChar(UART4, 0x55);//������� 
  distance_current = LPLD_UART_GetChar(UART4);
  distance_current = distance_current << 8;
  distance_current |= LPLD_UART_GetChar(UART4);
  push(12,(int16)(distance_current) );
}