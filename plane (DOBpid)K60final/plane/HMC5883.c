#include "include.h"

uint8 HMC_BUF[6];
int16 north = 0;
void Single_Write_HMC5883(uint8 REG_Address,uint8 REG_data)
{
  I2C_start();                  //��ʼ�ź�
  I2CSendByte(HMCAddress);   //�����豸��ַ+д�ź�
  I2CSendByte(REG_Address);    //�ڲ��Ĵ�����ַ����ο�����pdf 
  I2CSendByte(REG_data);       //�ڲ��Ĵ������ݣ���ο�����pdf
  I2C_stop();                   //����ֹͣ�ź�
}

uint8 Single_Read_HMC5883(uint8 REG_Address)
{  
  uint8 REG_data;
  I2C_start();                          //��ʼ�ź�
  I2CSendByte(HMCAddress);           //�����豸��ַ+д�ź�
  I2CSendByte(REG_Address);                   //���ʹ洢��Ԫ��ַ����0��ʼ	
  I2C_start();                        //��ʼ�ź�
  I2CSendByte(HMCAddress+1);         //�����豸��ַ+���ź�
  REG_data=I2CreceiveByte();              //�����Ĵ�������
  I2Cslave_ACK();   
  I2C_stop();                           //ֹͣ�ź�
  return REG_data; 
}

//******************************************************
//
//��������HMC5883�ڲ��Ƕ����ݣ���ַ��Χ0x3~0x5
//
//******************************************************
void Multiple_read_HMC5883(void)
{   
  uint8 i;
  I2C_start();                          //��ʼ�ź�
  I2CSendByte(HMCAddress);           //�����豸��ַ+д�ź�
  I2CSendByte(0x03);                   //���ʹ洢��Ԫ��ַ����0x3��ʼ	
  I2C_start();                          //��ʼ�ź�
  I2CSendByte(HMCAddress+1);         //�����豸��ַ+���ź�
  for (i=0; i<7; i++)                      //������ȡ6����ַ���ݣ��洢��BUF
  {
    HMC_BUF[i] = I2CreceiveByte();          //BUF[0]�洢0x32��ַ�е�����
    if (i == 6)
    {
      I2Cslave_ACK();                //���һ��������Ҫ��NOACK
    }
    else
    {
      I2Cslave_NOACK();                //��ӦACK
    }
  }
  I2C_stop();                          //ֹͣ�ź�
  north = HMC_BUF[0]<<8 | HMC_BUF[1];//ָ��
  //push(9,north);
  //push(10,HMC_BUF[4]<<8 | HMC_BUF[5]);
  //push(11,HMC_BUF[2]<<8 | HMC_BUF[3]);
}

//��ʼ��HMC5883��������Ҫ��ο�pdf�����޸�****
void Init_HMC5883()
{
  Single_Write_HMC5883(0x02,0x00);  //
  Single_Write_HMC5883(0x01,0xE0);  //
}