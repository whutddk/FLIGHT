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
  uart4_init();
  I2C_CLK_INIT();
  I2C_DAT_OUT();
  
  InitMPU6050();
  Init_HMC5883();
  //BMP180Init(&temp);
  //LPLD_Nrf24L01_Init();
  OLED_Init();
  Draw_BMP(0,0,66,7,mushroom);
  Draw_BMP(70,0,86,1,sky1);
  Draw_BMP(87,0,103,1,sky2);
  Draw_BMP(103,0,119,1,sky3);
  Draw_BMP(107,0,123,1,sky3);
  Draw_BMP(111,0,127,1,sky3);
  
  while(flag_dump);//等待上电指令   
  
  //在freecars中发布上电命令
  //在freecars中对PWM初始化
  //PWM0_init();//飞控启动自锁，占空比2000 
  _BZ_ON;
  delay(300000);//延时5s让电调初始化
  _BZ_OFF; 
  //LPLD_Nrf24L01_RxModeInit();
  while(flag_lock)//等待飞控解锁指令  
  //在freecars中发布解锁命令
  {
    mes_offset();
  }
  _BZ_ON;
  delay(100000);//延时5s让电调初始化
  _BZ_OFF;
  delay(100000);
  _BZ_ON;
  delay(100000);//延时5s让电调初始化
  _BZ_OFF;
  //pit0us_init();
  //  pit2ms_init();
  
  
  pit1ms_init();//1ms定时器初始化
  ctrl_init();//控制初始化
  //  while(!LPLD_WDOG_Init(1000));//1s的看门狗，没放出来就不启动 
  //  LPLD_WDOG_Enable();   //使能看门狗
}