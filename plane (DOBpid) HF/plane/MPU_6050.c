//数字三轴陀螺仪加速度计
//2015 5 4
//DDK

#include "common.h"
#include "include.h"

//**************************************
//向I2C设备写入一个字节数据
//**************************************
void Single_WriteI2C(uint8 REG_Address,uint8 REG_data)
{
  I2C_start();                  //起始信号
  I2CSendByte(MUPAddress);   //发送设备地址+写信号
  I2CSendByte(REG_Address);    //内部寄存器地址，
  I2CSendByte(REG_data);       //内部寄存器数据，
  I2C_stop();                   //发送停止信号
}
//**************************************
//从I2C设备读取一个字节数据
//**************************************
uint8 Single_ReadI2C(uint8 REG_Address)
{
  uint8 REG_data;
  I2C_start();                   //起始信号
  I2CSendByte(MUPAddress);    //发送设备地址+写信号
  I2CSendByte(REG_Address);     //发送存储单元地址，从0开始	
  I2C_start();                   //起始信号
  I2CSendByte(MUPAddress+1);  //发送设备地址+读信号
  REG_data=I2CreceiveByte();       //读出寄存器数据
  I2Ccheck_ACK();                //接收应答信号
  I2C_stop();                    //停止信号
  return REG_data;
}



//**************************************
//初始化MPU6050
//**************************************
void InitMPU6050()
{
  Single_WriteI2C(PWR_MGMT_1, 0x00);	//解除休眠状态
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
//合成数据
//**************************************
int16 GetData(uint8 REG_Address)
{
  uint8 H = 0,L = 0;
  H = Single_ReadI2C(REG_Address);
  L = Single_ReadI2C(REG_Address+1);
  return (H<<8)+L;   //合成数据
}
