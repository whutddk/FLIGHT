/***初始化C文件
***李锐戈
***2015 3 24
***
*/

#include "common.h"
#include "include.h"





void all_init()//将所有需要使用的模块初始化
{
  uart0_isr_init();//调试串口初始化
  NVIC_set();
  
  LED_init();
  uart4_init();  
  
  
  I2C_CLK_INIT();
  I2C_DAT_OUT();
  

  { 
    MPU6050_Inital();
    IMU_init();
    MIU_getdata();
//    roll_quiet = Roll;
//    pitch_quiet = Pitch;
//    yaw_quiet = Yaw;
  }
  
  while(flag_dump);//等待上电指令   
  
  
  _BZ_ON;
  delay(300000);//延时5s让电调初始化
  _BZ_OFF; 
  while(flag_lock)//等待飞控解锁指令  
  //在freecars中发布解锁命令
   {
//    MIU_getdata();
//    roll_quiet = ( Roll + roll_quiet ) / 2 ;
//    pitch_quiet = ( Pitch + pitch_quiet ) / 2 ;
//    yaw_quiet =  ( Yaw + yaw_quiet ) / 2;   
  }
  _BZ_ON;
  delay(100000);//延时5s让电调初始化
  _BZ_OFF;
  delay(100000);
  _BZ_ON;
  delay(100000);//延时5s让电调初始化
  _BZ_OFF;


  
  pit1ms_init();//1ms定时器初始化
  ctrl_init();//控制初始化
  
  
  //  while(!LPLD_WDOG_Init(1000));//1s的看门狗，没放出来就不启动 
  //  LPLD_WDOG_Enable();   //使能看门狗
}