#include "include.h"

uint16 distance_current = 0;
uint8 sonar_ON = 0;

void read_dis()//读取高度中断
{
  UART_InitTypeDef sonar_struct;/*串口号*/
  
  sonar_struct.UART_Uartx = UART4; //使用UARTx
  LPLD_UART_DisableIrq(sonar_struct);
  
  
  distance_current = LPLD_UART_GetChar(UART4);
  distance_current = distance_current << 8;
  distance_current |= LPLD_UART_GetChar(UART4);
  push(12,(int16)(distance_current) );
  HIGH_PID();
  sonar_ON = 0;
}

  
void uart4_init()//测距蓝牙
{
  UART_InitTypeDef sonar_struct;/*串口号*/
  
  sonar_struct.UART_Uartx = UART4; //使用UARTx
  sonar_struct.UART_BaudRate = 9600; /*设置波特率(9600)*/
  sonar_struct.UART_RxPin = PTE25;  //接收引脚为PTE1
  sonar_struct.UART_TxPin = PTE24;  //发送引脚为PTE0
  sonar_struct.UART_RxIntEnable = TRUE;
  sonar_struct.UART_RxIsr = read_dis;
  LPLD_UART_Init(sonar_struct);
  

  
}



void sonar()//测距并计算返回
{
  UART_InitTypeDef sonar_struct;
  
  if(sonar_ON == 0)
  {
    LPLD_UART_PutChar(UART4, 0x55);//请求距离 
    sonar_struct.UART_Uartx = UART4;
    LPLD_UART_EnableIrq(sonar_struct);
  }
  sonar_ON = 1;
  
//  distance_current = LPLD_UART_GetChar(UART4);
//  distance_current = distance_current << 8;
//  distance_current |= LPLD_UART_GetChar(UART4);
//  push(12,(int16)(distance_current) );
}