/*******
*******UART中层库，链接拉普兰德库，使其更易使用
******
******/
#include "common.h"
#include "include.h"

/*
    描述：
      发送引脚，每个UARTx对应不同的TxPin
    取值：
      UART0-PTA2、PTA14、*PTB17
      UART1-PTE0、*PTC4
      UART2-*PTD3
      UART3-PTE4、PTB11、*PTC17
      UART4-PTE24、*PTC15
      UART5-PTD9、*PTE8
    初始化：
      不必须初始化、默认值为带*的引脚
  */

/*
    描述：
      接收引脚，每个UARTx对应不同的RxPin
    取值：
      UART0-PTA1、PTA15、*PTB16
      UART1-PTE1、*PTC3
      UART2-*PTD2
      UART3-PTE5、PTB10、*PTC16
      UART4-PTE25、*PTC14
      UART5-PTD8、*PTE9
    初始化：
      不必须初始化、默认值为带*的引脚
  */

void uart0_init()
{
  UART_InitTypeDef uart_init_struct;/*串口号*/
  uart_init_struct.UART_Uartx = UART0; //使用UARTx
  uart_init_struct.UART_BaudRate = 115200; /*设置波特率(9600)*/
  uart_init_struct.UART_RxPin = PTA15;  //接收引脚为PTE9
  uart_init_struct.UART_TxPin = PTA14;  //发送引脚为PTE8
  uart_init_struct.UART_RxIntEnable = FALSE;
  //uart_init_struct.UART_RxPin=PTD6;
  //uart_init_struct.UART_TxPin=PTD7;
  //初始化UART
  LPLD_UART_Init(uart_init_struct);
}

void uart0_isr_init()
{
  UART_InitTypeDef uart_init_struct;/*串口号*/
  uart_init_struct.UART_Uartx = UART0; //使用UARTx
  uart_init_struct.UART_BaudRate = 115200; /*设置波特率(9600)*/
  uart_init_struct.UART_RxPin = PTA15;  //接收引脚为PT
  uart_init_struct.UART_TxPin = PTA14;  //发送引脚为PT
  uart_init_struct.UART_RxIntEnable = TRUE;
  uart_init_struct.UART_RxIsr = uart0_isr;
  //uart_init_struct.UART_RxPin=PTD6;
  //uart_init_struct.UART_TxPin=PTD7;
  //初始化UART
  LPLD_UART_Init(uart_init_struct);
  LPLD_UART_EnableIrq(uart_init_struct);
}

  UART_InitTypeDef sonar_struct;/*串口号*/
void uart1_init()//测距蓝牙
{

  sonar_struct.UART_Uartx = UART1; //使用UARTx
  sonar_struct.UART_BaudRate = 9600; /*设置波特率(9600)*/
  sonar_struct.UART_RxPin = PTE1;  //接收引脚为PTE1
  sonar_struct.UART_TxPin = PTE0;  //发送引脚为PTE0
  sonar_struct.UART_RxIntEnable = TRUE;
  LPLD_UART_Init(sonar_struct);

}

