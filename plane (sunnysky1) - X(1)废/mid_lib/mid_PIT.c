/*
*PIT配置中层库
李锐戈
2014 11 22
*
*/
#include "common.h"
#include "include.h"

void pit0us_init()//100us用于 高精度计时
{
  PIT_InitTypeDef pit_init_struct;
  //配置PIT0参数
  pit_init_struct.PIT_Pitx = PIT0;
  pit_init_struct.PIT_PeriodS = 0;
  pit_init_struct.PIT_PeriodMs = 0;
  pit_init_struct.PIT_PeriodUs = 100;     //定时周期
  pit_init_struct.PIT_Isr = pit0us_isr;  //设置中断函数
  //初始化PIT0
  LPLD_PIT_Init(pit_init_struct); 
  
  LPLD_PIT_EnableIrq(pit_init_struct);
}

void pit1ms_init()      //5ms
{
  PIT_InitTypeDef pit_init_struct;
  //配置PIT0参数
  pit_init_struct.PIT_Pitx = PIT1;
  pit_init_struct.PIT_PeriodS = 0;
  pit_init_struct.PIT_PeriodMs = 1;     //定时周期5ms
  pit_init_struct.PIT_PeriodUs = 0;
  pit_init_struct.PIT_Isr = pit1ms_isr;  //设置中断函数
  //初始化PIT0
  LPLD_PIT_Init(pit_init_struct); 
  
  LPLD_PIT_EnableIrq(pit_init_struct);
}

void pit2ms_init()       //运行时间计时1ms
{
  PIT_InitTypeDef pit_init_struct;
  //配置PIT0参数
  pit_init_struct.PIT_Pitx = PIT2;
  pit_init_struct.PIT_PeriodS = 0;
  pit_init_struct.PIT_PeriodMs = 1;     //定时周期
  pit_init_struct.PIT_PeriodUs = 0;
  pit_init_struct.PIT_Isr = pit2ms_isr;  //设置中断函数
  //初始化PIT2
  LPLD_PIT_Init(pit_init_struct); 
  
  LPLD_PIT_EnableIrq(pit_init_struct);
}

void pit3us_init(uint32 Periodus)
{
  PIT_InitTypeDef pit_init_struct;
  //配置PIT3参数
  pit_init_struct.PIT_Pitx = PIT3;
  pit_init_struct.PIT_PeriodS = 0;
  pit_init_struct.PIT_PeriodMs = 0;
  pit_init_struct.PIT_PeriodUs = Periodus;     //定时周期
  pit_init_struct.PIT_Isr = pit3us_isr;  //设置中断函数
  //初始化PIT0
  LPLD_PIT_Init(pit_init_struct);
  
  LPLD_PIT_EnableIrq(pit_init_struct);  
}

uint8 flag_100us = 0;
uint8 flag_200us = 0;
uint8 flag_300us = 0;
uint8 flag_400us = 0;
uint8 flag_500us = 0;
uint8 flag_600us = 0;
uint8 flag_700us = 0;
uint8 flag_800us = 0;
uint8 flag_900us = 0;
uint8 flag_1ms = 0;
uint8 us_count = 0;
uint16 us_cnt1 = 0;
uint8 flag_beep = 0;
void pit0us_isr()//超声波测距中断函数
{
  us_count ++;
  us_cnt1 ++;
  flag_100us = 1;
  
  
  
  if (us_count % 2 == 0 )//两个5ms即10ms
  {
    flag_200us = 1;
    
  }
  if ( us_count % 3 == 0 )
  {
    flag_300us = 1;
  }
  if ( us_count % 4 == 0 )
  {
    flag_400us = 1;
  }
  if ( us_count % 5 == 0 )
  {
    flag_500us = 1;
    
  }
  if ( us_count % 6 == 0 )
  {
    flag_600us = 1;
  }
  if ( us_count % 7 == 0 )
  {
    flag_700us = 1; 
    
  }
  if ( us_count % 8 == 0)
  {
    flag_800us = 1;
  }
  if ( us_count % 9 == 0)
  {
    flag_900us = 1;
  }
  if ( us_count % 10 == 0)
  {
    flag_1ms = 1;
    us_count = 0; 
    
  }
}


/********************************************/
uint8 flag_5ms = 0;
uint8 flag_10ms = 0;
uint8 flag_20ms = 0;
uint8 flag_30ms = 0;
uint8 flag_60ms = 0;
uint8 flag_70ms = 0;
uint8 flag_80ms = 0;
uint8 flag_90ms = 0;
uint8 flag_100ms = 0;
uint8 flag_250ms = 0;
uint8 flag_500ms = 0;
uint8 flag_1s = 0;
uint16 count = 0;

void pit1ms_isr()//主函数计时中断函数及电磁检测函数中断
{ 
  count ++;
  
  {
#if ANO
    ANO_DT_Data_Exchange();
#endif
  }
  if ( count % 2 == 0 )
  {
    Prepare_Data();
  }
  if ( count % 5 == 0 )
  {
    flag_5ms = 1;
    
    Get_Attitude();
    //Multiple_read_HMC5883();
    //BMP180Convert(&temp);
    
    if ( !flag_dump && !flag_lock )
    {
      PITCH_PID();
      //ROLL_PID();
      //YAW_PID();
      CTRL_OUT();
    }
    
  }
  if (count % 10 == 0 )//两个5ms即10ms
  {
    flag_10ms = 1;
    //    get_quaternion();
    
  }
  if ( count % 20 == 0 )
  {
    flag_20ms = 1;
  } 
  if ( count % 30 == 0 )
  {
    flag_30ms = 1;
    _BZ_OFF;
    //sonar();
    
    
  }
  if ( count % 60 == 0)
  {
    //check_pin();
#if !ANO
    sendDataToScope(); 
#endif
    LPLD_WDOG_Feed();
  }

     

  if ( count % 500 == 0 )
  {
    flag_500ms = 1;

    
  }
  if ( count % 1000 == 0 )
  {
    flag_1s = 1;
    count = 0;
    check_unbalance();
  }
}

uint16 time_count = 0 ;

void pit2ms_isr() //1ms
{
  time_count++;
  
}

void pit3us_isr()
{
  PIT_InitTypeDef pit_init_struct;//释放PIT3参数
  
  flag_delay_complete = 1;
  
  pit_init_struct.PIT_Pitx = PIT3;
  LPLD_PIT_Deinit(pit_init_struct);
}

uint8 flag_delay_complete = 0;
void delay(uint32 time_us)
{
  flag_delay_complete = 0;
  pit3us_init(time_us);
  while(!flag_delay_complete);
}

void disable_all_isr()//关闭所有中断
{
  PIT_InitTypeDef pit_init_struct;
  pit_init_struct.PIT_Pitx = PIT1;
  LPLD_PIT_DisableIrq(pit_init_struct);
  pit_init_struct.PIT_Pitx = PIT2;
  LPLD_PIT_DisableIrq(pit_init_struct);
}

void enbale_all_isr()//开总中断
{
  PIT_InitTypeDef pit_init_struct;
  pit_init_struct.PIT_Pitx = PIT1;
  LPLD_PIT_EnableIrq(pit_init_struct);
  pit_init_struct.PIT_Pitx = PIT2;
  LPLD_PIT_EnableIrq(pit_init_struct);
}

void I2Cdelay_1us(uint8_t us)                 // 1/15 us延时函数
{
  volatile uint8_t i ,j ;
  if(us < 1 )  us = 1 ;
  for(i=0;i<us;i++) 
  {
    for(j = 0 ;j < 1 ;j ++)
      ;    
  }  
}