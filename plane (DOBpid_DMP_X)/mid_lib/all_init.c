/***��ʼ��C�ļ�
***�����
***2015 3 24
***
*/

#include "common.h"
#include "include.h"





void all_init()//��������Ҫʹ�õ�ģ���ʼ��
{
  uart0_isr_init();//���Դ��ڳ�ʼ��
  NVIC_set();
  
  LED_init();
  uart4_init();  
  
  
  I2C_CLK_INIT();
  I2C_DAT_OUT();
  
  LPLD_Nrf24L01_Init();
  LPLD_Nrf24L01_RxModeInit();
  OLED_Init();
  Draw_BMP(0,0,66,7,mushroom);
  Draw_BMP(70,0,86,1,sky1);
  Draw_BMP(87,0,103,1,sky2);
  Draw_BMP(103,0,119,1,sky3);
  Draw_BMP(107,0,123,1,sky3);
  Draw_BMP(111,0,127,1,sky3);
  
  { 
    MPU6050_Inital();
    IMU_init();
    MIU_getdata();
//    roll_quiet = Roll;
//    pitch_quiet = Pitch;
//    yaw_quiet = Yaw;
  }
  
  while(flag_dump);//�ȴ��ϵ�ָ��   
  
  
  _BZ_ON;
  delay(300000);//��ʱ5s�õ����ʼ��
  _BZ_OFF; 
  while(flag_lock)//�ȴ��ɿؽ���ָ��  
  //��freecars�з�����������
   {
//    MIU_getdata();
//    roll_quiet = ( Roll + roll_quiet ) / 2 ;
//    pitch_quiet = ( Pitch + pitch_quiet ) / 2 ;
//    yaw_quiet =  ( Yaw + yaw_quiet ) / 2;   
  }
  _BZ_ON;
  delay(100000);//��ʱ5s�õ����ʼ��
  _BZ_OFF;
  delay(100000);
  _BZ_ON;
  delay(100000);//��ʱ5s�õ����ʼ��
  _BZ_OFF;


  ctrl_init();//���Ƴ�ʼ��
  pit1ms_init();//1ms��ʱ����ʼ��
  pit0us_init();//DMP5MS
  pit2ms_init();//CTL10MS
  
  
  //  while(!LPLD_WDOG_Init(1000));//1s�Ŀ��Ź���û�ų����Ͳ����� 
  //  LPLD_WDOG_Enable();   //ʹ�ܿ��Ź�
}