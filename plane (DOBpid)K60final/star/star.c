#include "include.h"


int16 times_cnt = 0;

void get_star();



void uart4_init()//测距蓝牙
{
  UART_InitTypeDef star_struct;/*串口号*/
  
  star_struct.UART_Uartx = UART4; //使用UARTx
  star_struct.UART_BaudRate = 115200; /*设置波特率(9600)*/
  star_struct.UART_RxPin =PTE25;//PTD8;  //接收引脚为PTE1
  star_struct.UART_TxPin = PTE24; //PTD9;  //发送引脚为PTE0
  star_struct.UART_RxIntEnable = TRUE;
  star_struct.UART_RxIsr = get_star;
  LPLD_UART_Init(star_struct);
  LPLD_UART_EnableIrq(star_struct);
}

void get_star()
{
  
  UART_InitTypeDef star_struct;/*串口号*/
  uint8 data_temp;
  
  uint8 data[6];
  push(16,(int16)times_cnt );
  
  star_struct.UART_Uartx = UART4; //使用UARTx
   LPLD_UART_DisableIrq(star_struct);
  data_temp = LPLD_UART_GetChar(UART4);
  if ( data_temp == 0x55) //first
  {
    data_temp = LPLD_UART_GetChar(UART4);
    switch ( data_temp )
    {
    case (0x52)://角速度
      {
        data[0] = LPLD_UART_GetChar(UART4);
        data[1] = LPLD_UART_GetChar(UART4);
        data[2] = LPLD_UART_GetChar(UART4);
        data[3] = LPLD_UART_GetChar(UART4);
        data[4] = LPLD_UART_GetChar(UART4);
        data[5] = LPLD_UART_GetChar(UART4);
        
        MPU6050_GYRO_LAST.X = (int16)(
          data[0] | data[1] << 8
            )/32768.*2000; //~/S
          
        MPU6050_GYRO_LAST.Y = (int16)(
          data[2] | data[3] << 8
            )/32768.*2000;
        
        MPU6050_GYRO_LAST.Z = (int16)(
          data[4] | data[5] << 8
            )/32768.*2000;
        break;
      }
    case (0x53)://角度
      {
        data[0] = LPLD_UART_GetChar(UART4);
        data[1] = LPLD_UART_GetChar(UART4);
        data[2] = LPLD_UART_GetChar(UART4);
        data[3] = LPLD_UART_GetChar(UART4);
        data[4] = LPLD_UART_GetChar(UART4);
        data[5] = LPLD_UART_GetChar(UART4);
        
        Q_ANGLE.X = ((int16)(
          data[0] | data[1] << 8
            )/32768.*180) + 0.2 ;//(° );
        
        Q_ANGLE.Y = ((int16)(
          data[2] | data[3] << 8
            )/32768.*180 ) + 3.1;//(° );
        
        Q_ANGLE.Z = (int16)(
          data[4] | data[5] << 8
            )/32768.*180;//(° );
        break;
      }
    }

  }
  else
  {;}
  push(0,MPU6050_GYRO_LAST.X );
  push(1,MPU6050_GYRO_LAST.Y );
  push(2,MPU6050_GYRO_LAST.Z );
  push(3,Q_ANGLE.X * 100);
  push(4,Q_ANGLE.Y * 100);
  push(5,Q_ANGLE.Z * 100);
  times_cnt = 0;

  LPLD_UART_EnableIrq(star_struct);
  
}