#include "common.h"
#include "include.h"

void NVIC_set()
{
  NVIC_InitTypeDef nvic_init_struct;
  
  nvic_init_struct.NVIC_IRQChannel = PIT3_IRQn;
  nvic_init_struct.NVIC_IRQChannelGroupPriority = NVIC_PriorityGroup_2;
  nvic_init_struct.NVIC_IRQChannelPreemptionPriority = 0;
  nvic_init_struct.NVIC_IRQChannelSubPriority = 0;  
  nvic_init_struct.NVIC_IRQChannelEnable = TRUE;
  LPLD_NVIC_Init(nvic_init_struct);
  
  nvic_init_struct.NVIC_IRQChannel = PIT0_IRQn;
  nvic_init_struct.NVIC_IRQChannelGroupPriority = NVIC_PriorityGroup_2;
  nvic_init_struct.NVIC_IRQChannelPreemptionPriority = 1;
  nvic_init_struct.NVIC_IRQChannelSubPriority = 0;
nvic_init_struct.NVIC_IRQChannelEnable = TRUE;  
  LPLD_NVIC_Init(nvic_init_struct);
  
  nvic_init_struct.NVIC_IRQChannel = PIT1_IRQn;
  nvic_init_struct.NVIC_IRQChannelGroupPriority = NVIC_PriorityGroup_2;
  nvic_init_struct.NVIC_IRQChannelPreemptionPriority = 2;
  nvic_init_struct.NVIC_IRQChannelSubPriority = 0;  
  nvic_init_struct.NVIC_IRQChannelEnable = TRUE;
  LPLD_NVIC_Init(nvic_init_struct);
//  
//  nvic_init_struct.NVIC_IRQChannel = UART0_RX_TX_IRQn;
//  nvic_init_struct.NVIC_IRQChannelGroupPriority = NVIC_PriorityGroup_2;
//  nvic_init_struct.NVIC_IRQChannelPreemptionPriority = 2;
//  nvic_init_struct.NVIC_IRQChannelSubPriority = 1;  
//  LPLD_NVIC_Init(nvic_init_struct);
//  
  nvic_init_struct.NVIC_IRQChannel = PIT2_IRQn;
  nvic_init_struct.NVIC_IRQChannelGroupPriority = NVIC_PriorityGroup_2;
  nvic_init_struct.NVIC_IRQChannelPreemptionPriority = 1;
  nvic_init_struct.NVIC_IRQChannelSubPriority = 0; 
  nvic_init_struct.NVIC_IRQChannelEnable = TRUE;
  LPLD_NVIC_Init(nvic_init_struct);
}
