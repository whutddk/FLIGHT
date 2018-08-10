/***
***spi配置库主要for24l01和SD卡
李锐戈
2014 11 22
***
*/

/********************************************
 * 说明: 
 * 硬件配置说明:
 * K60单片机引脚        NRF24L01引脚
 * 
 *     INT    -PTE6  -----> IRQ
 *     CE     -PTE5  -----> CE
 *     CS     -PTE4 -----> CSN
 *     SCK    -PTE2 -----> SCK
 *     MOSI   -PTE1 -----> MOSI
 *     MISO   -PTE3 <----- MISO
 *

 * 通过修改NRF24L01_WORKMODE改变发送和接收方式
 ********************************************/
#include "common.h"
#include "include.h"
   
/* 
//选择编译方式 决定当前例程中NRF24L01的工作方式
#define NRF24L01_RX_WORKMODE      (0) //发送程序
#define NRF24L01_TX_WORKMODE      (1) //接收程序
#define NRF24L01_WORKMODE         NRF24L01_TX_WORKMODE 

uint8 rx_buf[NRF24L01_PLOAD_LEN] = {0x00};     //接收缓冲区
uint8 tx_buf[NRF24L01_PLOAD_LEN] = {0x00};     //发送缓冲区





void nrf24l01_RX_init()
{
  GPIO_InitTypeDef gpio_init_struct;
   //中断函数声明
  gpio_init_struct.GPIO_PTx = PTE;        //PORTE
  gpio_init_struct.GPIO_Pins = GPIO_Pin6; //引脚6
  gpio_init_struct.GPIO_Dir = DIR_INPUT;  //输入
  gpio_init_struct.GPIO_PinControl = INPUT_PULL_DOWN|IRQC_FA;  //内部下拉|下降沿中断
  gpio_init_struct.GPIO_Isr = porte_handle;      //中断函数
    
  LPLD_GPIO_Init(gpio_init_struct);
  LPLD_GPIO_EnableIrq(gpio_init_struct); //内部下拉,使能PORTE6作为NRF24L01的IRQ中断
  LPLD_Nrf24L01_RxModeInit();
}   
uint8 Flag_Smitte = 0; //发送标志，

void nrf24l01_checksend()
{ int i; 
    if(Flag_Smitte)
    {
      for(i = 0 ;i <32 ;i++)//设置发送的32个字节数据
      {
        tx_buf[i] = i;
      }
      if(LPLD_Nrf24L01_SendFrame(tx_buf,NRF24L01_PLOAD_LEN))
      {
        
      }
      else
      {
       
      }
      Flag_Smitte = 0;
    }
   
}

//nrf24l01接收中断
void porte_handle()
{   
  LPLD_Nrf24L01_RecvFrame(rx_buf, NRF24L01_PLOAD_LEN);
}
*/
/********************************************************************/
