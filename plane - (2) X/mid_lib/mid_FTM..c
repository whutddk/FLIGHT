/***
***FTMģ���в��
�����
2014 11 22

***
*/

#include "common.h"
#include "include.h"


/////////////20kHZ
void PWM0_init()                 //FTM0ͨ���࣬8ͨ�����ڿ���PWM
{ 
  FTM_InitTypeDef ftm_init_struct;
  ftm_init_struct.FTM_Ftmx = FTM0;	//ʹ��FTM0ͨ��
  ftm_init_struct.FTM_Mode = FTM_MODE_PWM;	//ʹ��PWMģʽ
  ftm_init_struct.FTM_PwmFreq = 500;	//PWMƵ��500Hz
  ftm_init_struct.FTM_PwmDeadtimeCfg = DEADTIME_CH23;
  ftm_init_struct.FTM_PwmDeadtimeVal=0;
  LPLD_FTM_Init(ftm_init_struct);
  
    LPLD_FTM_PWM_Enable(FTM0, //ʹ��FTM0
                      FTM_Ch0, //ʹ��Ch0ͨ��
                      2000, //��ʼ���Ƕ�0��
                      PTC1, //ʹ��Ch0ͨ����PTC1����
                      ALIGN_LEFT        //���������
                      );    
    LPLD_FTM_PWM_Enable(FTM0, //ʹ��FTM0
                      FTM_Ch1, //ʹ��Ch0ͨ��
                      2000, //��ʼ���Ƕ�0��
                      PTC2, //ʹ��Ch0ͨ����PTC1����
                      ALIGN_LEFT        //���������
                      ); 
    LPLD_FTM_PWM_Enable(FTM0, //ʹ��FTM0
                      FTM_Ch2, //ʹ��Ch0ͨ��
                      2000, //��ʼ���Ƕ�0��
                      PTC3, //ʹ��Ch0ͨ����PTC1����
                      ALIGN_LEFT        //���������
                      ); 
    LPLD_FTM_PWM_Enable(FTM0, //ʹ��FTM0
                      FTM_Ch3, //ʹ��Ch0ͨ��
                      2000, //��ʼ���Ƕ�0��
                      PTC4, //ʹ��Ch0ͨ����PTC1����
                      ALIGN_LEFT        //���������
                      ); 
}/*��PWM���*/


void qd1_init()                 //FTM1ͨ����2ͨ�����ڿ���PWM
{ 
 //�����������빦�ܲ���
  FTM_InitTypeDef ftm_init_struct;
  ftm_init_struct.FTM_Ftmx = FTM1;              //ֻ��FTM1��FTM2���������빦��
  ftm_init_struct.FTM_Mode = FTM_MODE_QD;       //�������빦��
  ftm_init_struct.FTM_QdMode = QD_MODE_PHAB;    //AB������ģʽ
  //��ʼ��FTM
  LPLD_FTM_Init(ftm_init_struct);
  //ʹ��AB������ͨ��
  //PTB0���Ž�A�����롢PTB1���Ž�B������
  LPLD_FTM_QD_Enable(FTM1, PTA12, PTA13);
}

void qd2_init(void) //��������AB��ģʽ��ʼ��
{
  //�����������빦�ܲ���
  FTM_InitTypeDef ftm_init_struct;
  ftm_init_struct.FTM_Ftmx = FTM2;              //ֻ��FTM1��FTM2���������빦��
  ftm_init_struct.FTM_Mode = FTM_MODE_QD;       //�������빦��
  ftm_init_struct.FTM_QdMode = QD_MODE_PHAB;    //AB������ģʽ
  //��ʼ��FTM
  LPLD_FTM_Init(ftm_init_struct);
  //ʹ��AB������ͨ��
  //PTB0���Ž�A�����롢PTB1���Ž�B������
  LPLD_FTM_QD_Enable(FTM2, PTB18, PTB19);
}