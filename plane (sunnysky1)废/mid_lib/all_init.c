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
  
  while(flag_dump);//�ȴ��ϵ�ָ��   
  
  //��freecars�з����ϵ�����
  //��freecars�ж�PWM��ʼ��
  //PWM0_init();//�ɿ�����������ռ�ձ�2000 
  _BZ_ON;
  delay(300000);//��ʱ5s�õ����ʼ��
  _BZ_OFF; 
  //LPLD_Nrf24L01_RxModeInit();
  while(flag_lock)//�ȴ��ɿؽ���ָ��  
  //��freecars�з�����������
  {
    mes_offset();
  }
  _BZ_ON;
  delay(100000);//��ʱ5s�õ����ʼ��
  _BZ_OFF;
  delay(100000);
  _BZ_ON;
  delay(100000);//��ʱ5s�õ����ʼ��
  _BZ_OFF;
  //pit0us_init();
  //  pit2ms_init();
  
  
  pit1ms_init();//1ms��ʱ����ʼ��
  ctrl_init();//���Ƴ�ʼ��
  //  while(!LPLD_WDOG_Init(1000));//1s�Ŀ��Ź���û�ų����Ͳ����� 
  //  LPLD_WDOG_Enable();   //ʹ�ܿ��Ź�
}