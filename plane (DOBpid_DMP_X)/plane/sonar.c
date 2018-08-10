#include "include.h"

uint16 distance_current = 0;
uint8 sonar_ON = 0;

void read_dis()//��ȡ�߶��ж�
{
  UART_InitTypeDef sonar_struct;/*���ں�*/
  
  sonar_struct.UART_Uartx = UART4; //ʹ��UARTx
  LPLD_UART_DisableIrq(sonar_struct);
  
  
  distance_current = LPLD_UART_GetChar(UART4);
  distance_current = distance_current << 8;
  distance_current |= LPLD_UART_GetChar(UART4);
  push(12,(int16)(distance_current) );
  HIGH_PID();
  sonar_ON = 0;
}

  
void uart4_init()//�������
{
  UART_InitTypeDef sonar_struct;/*���ں�*/
  
  sonar_struct.UART_Uartx = UART4; //ʹ��UARTx
  sonar_struct.UART_BaudRate = 9600; /*���ò�����(9600)*/
  sonar_struct.UART_RxPin = PTE25;  //��������ΪPTE1
  sonar_struct.UART_TxPin = PTE24;  //��������ΪPTE0
  sonar_struct.UART_RxIntEnable = TRUE;
  sonar_struct.UART_RxIsr = read_dis;
  LPLD_UART_Init(sonar_struct);
  

  
}



void sonar()//��ಢ���㷵��
{
  UART_InitTypeDef sonar_struct;
  
  if(sonar_ON == 0)
  {
    LPLD_UART_PutChar(UART4, 0x55);//������� 
    sonar_struct.UART_Uartx = UART4;
    LPLD_UART_EnableIrq(sonar_struct);
  }
  sonar_ON = 1;
  
//  distance_current = LPLD_UART_GetChar(UART4);
//  distance_current = distance_current << 8;
//  distance_current |= LPLD_UART_GetChar(UART4);
//  push(12,(int16)(distance_current) );
}