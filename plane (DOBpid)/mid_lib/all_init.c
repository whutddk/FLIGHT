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
  
  I2C_CLK_INIT();
  I2C_DAT_OUT();
  
  InitMPU6050();
  Init_HMC5883();
  //BMP180Init(&temp);
  //LPLD_Nrf24L01_Init();
 
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
  MPU6050_ACC_OFFSET.Z -= 8192;
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