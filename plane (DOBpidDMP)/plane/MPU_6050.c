//�������������Ǽ��ٶȼ�
//2015 5 4
//DDK

#include "common.h"
#include "include.h"

//**************************************
//��I2C�豸д��һ���ֽ�����
//**************************************
void Single_WriteI2C(uint8 REG_Address,uint8 REG_data)
{
  I2C_start();                  //��ʼ�ź�
  I2CSendByte(MUPAddress);   //�����豸��ַ+д�ź�
  I2CSendByte(REG_Address);    //�ڲ��Ĵ�����ַ��
  I2CSendByte(REG_data);       //�ڲ��Ĵ������ݣ�
  I2C_stop();                   //����ֹͣ�ź�
}
//**************************************
//��I2C�豸��ȡһ���ֽ�����
//**************************************
uint8 Single_ReadI2C(uint8 REG_Address)
{
  uint8 REG_data;
  I2C_start();                   //��ʼ�ź�
  I2CSendByte(MUPAddress);    //�����豸��ַ+д�ź�
  I2CSendByte(REG_Address);     //���ʹ洢��Ԫ��ַ����0��ʼ	
  I2C_start();                   //��ʼ�ź�
  I2CSendByte(MUPAddress+1);  //�����豸��ַ+���ź�
  REG_data=I2CreceiveByte();       //�����Ĵ�������
  I2Ccheck_ACK();                //����Ӧ���ź�
  I2C_stop();                    //ֹͣ�ź�
  return REG_data;
}



//**************************************
//��ʼ��MPU6050
//**************************************
void InitMPU6050()
{
  Single_WriteI2C(PWR_MGMT_1, 0x00);	//�������״̬
 Single_WriteI2C(SMPLRT_DIV, 0x07);
  Single_WriteI2C(CONFIG, 0x06);
  
  
  Single_WriteI2C(GYRO_CONFIG, 0x08);
  Single_WriteI2C(ACCEL_CONFIG, 0x08);
//  Single_WriteI2C(GYRO_CONFIG, 0x18);
//  Single_WriteI2C(ACCEL_CONFIG, 0x01);
  
  
  
  
  
  
  
  MPU6050_ACC_OFFSET.X = 269;//-780;
  MPU6050_ACC_OFFSET.Y = 269;//-100;
  MPU6050_ACC_OFFSET.Z = 0;
  
  MPU6050_GYRO_OFFSET.X = -459;
  MPU6050_GYRO_OFFSET.Y = 19;
  MPU6050_GYRO_OFFSET.Z = -28;
}
//**************************************
//�ϳ�����
//**************************************
int16 GetData(uint8 REG_Address)
{
  uint8 H = 0,L = 0;
  H = Single_ReadI2C(REG_Address);
  L = Single_ReadI2C(REG_Address+1);
  return (H<<8)+L;   //�ϳ�����
}
