/*****
******中层库总头文件，所有自己写的中层库的函数在此声明
李锐戈
2014 11 16
李锐戈 2015 2 14 、、计数器只有32位的低位，方便使用，理论可行 
*****/

#ifndef _MID_LIB_H_
#define _MID_LIB_H_


#include "common.h"
#include "include.h"


/*串口使用设置包含：串口号，波特率，是否使用接收中断
******
****
**
*
引脚全部默认*/
void uart0_init();
void uart0_isr_init();
void uart0_isr(void);
void uart1_init();
/*其余均在LPLD库*/

/*ADC初始化：软件触发 ADC号，位数***/
void ADC_Init_solf ( ADC_Type *Adcx , uint8 BitMode );
/*ADC初始化：硬件触发
******************
******************
*****************
暂时无法使用
****************
**************
*/

/*其余均在LPLD库*/

/*FTM_PWM使用设置：无
******
****
**
*
引脚可单独调
pin--FTMx通道对应的引脚
 *      FTM0
 *       FTM_Ch0-PTA3\PTC1
 *       FTM_Ch1-PTA4\PTC2
 *       FTM_Ch2-PTA5\PTC3
 *       FTM_Ch3-PTA6\PTC4
 *       FTM_Ch4-PTA7\PTD4
 *       FTM_Ch5-PTA0\PTD5
 *       FTM_Ch6-PTA1\PTD6
 *       FTM_Ch7-PTA2\PTD7*/

/*将所有FTM0的通道都初始化了5000居中占空比*/
void PWM0_init();                 //FTM0通道多，8通，用于控制PWM
void qd1_init();                 //FTM用于控制电机，可以考虑交换
void qd2_init();
/*
改变占空比用LPLD的库
*/

//void ic_init();//输入捕捉器
//void ic_isr();//不懂如何触发，如何计数
/*      FTM1
 *       FTM_Ch0-PTA8\PTA12\PTB0
 *       FTM_Ch1-PTA9\PTA13\PTB1
 *      FTM2
 *       FTM_Ch0-PTA10\PTB18
 *       FTM_Ch1-PTA11\PTB19
 *    capture_edge--捕获边缘设置
 *      |__CAPTURE_RI    --上升沿捕获
 *      |__CAPTURE_FA    --下降沿捕获
 *      |__CAPTURE_RIFA  --上升下降沿捕获
 *
*/

/*********************************
**PIT定时器设置
***********************************/
extern uint32 solfware_count;//软件总计时
extern uint8 flag_delay_complete;
extern uint8 sonar_count;


void pit0us_init();//100US用于 超声波测距
void pit1ms_init();//5ms用于电感采集和标准延迟
void pit2ms_init();//mS
void pit3us_init(uint32 Periodus);//US
void pit0us_isr();//用于超声波
void pit1ms_isr();//用于主程序定时
void pit2ms_isr();//用于软件总计时
void pit3us_isr();//
void delay(uint32 time_us);//标准延时函数，us
void disable_all_isr();
void enbale_all_isr();
void BFdelay_1us(uint8_t us);                 //1us延时函数

/*****
***24L01
***
*/

//extern uint8 Flag_Smitte;

//uint8 LPLD_Nrf24L01_Init();//初始化24L01

//uint8 LPLD_Nrf24L01_TxModeInit();//初始化发送模式

//void nrf24l01_RX_init();//初始化接收模式

//void nrf24l01_checksend();//检查发送标志位，如果为真，就发送32为buf中的数据

//void porte_handle(void);//接收中断



/***
****LPTMR
****
*/

//extern uint16 pluse_count_h ; //定义计数器高位
extern uint32 pluse_count_l ; //定义计数器

void lpt_init(void);//脉冲计数C5口
void get_count();//计数
#endif