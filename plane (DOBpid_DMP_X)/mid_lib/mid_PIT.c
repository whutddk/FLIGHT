/*
*PIT配置中层库
李锐戈
2014 11 22
*
*/
#include "common.h"
#include "include.h"

void pit0us_init()//5ms采样
{
  PIT_InitTypeDef pit_init_struct;
  //配置PIT0参数
  pit_init_struct.PIT_Pitx = PIT0;
  pit_init_struct.PIT_PeriodS = 0;
  pit_init_struct.PIT_PeriodMs = 5;//5ms给DMP采样用
  pit_init_struct.PIT_PeriodUs = 0;     //定时周期
  pit_init_struct.PIT_Isr = pit0us_isr;  //设置中断函数
  //初始化PIT0
  LPLD_PIT_Init(pit_init_struct); 
  
  LPLD_PIT_EnableIrq(pit_init_struct);
}

void pit1ms_init()      //1ms
{
  PIT_InitTypeDef pit_init_struct;
  //配置PIT0参数
  pit_init_struct.PIT_Pitx = PIT1;
  pit_init_struct.PIT_PeriodS = 0;
  pit_init_struct.PIT_PeriodMs = 1;     //定时周期1ms
  pit_init_struct.PIT_PeriodUs = 0;
  pit_init_struct.PIT_Isr = pit1ms_isr;  //设置中断函数
  //初始化PIT0
  LPLD_PIT_Init(pit_init_struct); 
  
  LPLD_PIT_EnableIrq(pit_init_struct);
}

void pit2ms_init()       //PID控制10ms
{
  PIT_InitTypeDef pit_init_struct;
  //配置PIT0参数
  pit_init_struct.PIT_Pitx = PIT2;
  pit_init_struct.PIT_PeriodS = 0;
  pit_init_struct.PIT_PeriodMs = 5;     //定时周期
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


int32 GYRO_avg[3] = {0};

//int32 GYRO_list[3][5] = {0};
//uint8 gyro_cnt = 0;//滑块

int32 GYRO_RC[3][2] = {0};
float aef = 0.;
void pit0us_isr()//DMP采样5ms
{
  uint8 i = 0;
  uint8 j = 0;
  
//  int32 sum = 0;
  
  MIU_getdata();
  
  if ( gyro[0] > 4500 )
  {
    gyro[0] = 4500;
  }
  if ( gyro[0] < -4500 ) 
  {
    gyro[0] = -4500;
  }
  
   if ( gyro[1] > 4500 )
  {
    gyro[1] = 4500;
  }
  if ( gyro[1] < -4500 ) 
  {
    gyro[1] = -4500;
  }
  
    if ( gyro[2] > 4500 )
  {
    gyro[2] = 4500;
  }
  if ( gyro[2] < -4500 ) 
  {
    gyro[2] = -4500;
  }
    
  for( i = 0;i < 3 ;i++ )
  {
    GYRO_RC[i][1] = (int32) (aef * gyro[i]/10 + ( 1 - aef) * GYRO_RC[i][0]);
    GYRO_avg[i] = GYRO_RC[i][1];
    GYRO_RC[i][0] = GYRO_RC[i][1];
  }
//  GYRO_list[0][gyro_cnt] = gyro[0];
//  GYRO_list[1][gyro_cnt] = gyro[1];
//  GYRO_list[2][gyro_cnt] = gyro[2];
//  
//  gyro_cnt ++;
//  if (gyro_cnt == 5)
//  {
//    gyro_cnt = 0;
//  }
//  
//  for ( i = 0; i < 3;i ++ )
//  {
//    sum = 0;
//    for ( j = 0; j < 5 ;j++ )
//    {
//      sum += GYRO_list[i][j];
//    }
//    GYRO_avg[i] = sum / 50;
//    
//  }
  push(3,GYRO_avg[0]);
  push(4,GYRO_avg[1]);
  push(5,GYRO_avg[2]);
  
}


/********************************************/

uint16 count = 0;

void pit1ms_isr()//主函数计时中断函数及中断1ms
{ 
  count ++; 
  {

  }
  if ( count % 5 == 0 )
  {    
    //LPLD_WDOG_Feed();   
#if HOLD
    add_angle();
#endif
  }

  if ( count % 20 == 0 )
  {    
#if HOLD
    hold_point();
#endif
  } 
  if ( count % 30 == 0 )
  {   
    //sonar();      
  }
  if ( count % 60 == 0)
  {
      _BZ_OFF;
      
      check_pin();
      sendDataToScope(); 
  } 
  
  if ( count % 1000 == 0 )
  {
    count = 0;
    check_unbalance();
  }
}


void pit2ms_isr() //10ms
{
  if ( !flag_dump && !flag_lock )
  {
    CONTROL(Roll - roll_quiet, Pitch - pitch_quiet, Yaw - yaw_quiet) ;
  }
  CTRL_OUT();
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