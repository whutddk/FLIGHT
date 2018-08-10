#include "include.h"

uint8 HMC_BUF[6];
int16 north = 0;
void Single_Write_HMC5883(uint8 REG_Address,uint8 REG_data)
{
  I2C_start();                  //起始信号
  I2CSendByte(HMCAddress);   //发送设备地址+写信号
  I2CSendByte(REG_Address);    //内部寄存器地址，请参考中文pdf 
  I2CSendByte(REG_data);       //内部寄存器数据，请参考中文pdf
  I2C_stop();                   //发送停止信号
}

uint8 Single_Read_HMC5883(uint8 REG_Address)
{  
  uint8 REG_data;
  I2C_start();                          //起始信号
  I2CSendByte(HMCAddress);           //发送设备地址+写信号
  I2CSendByte(REG_Address);                   //发送存储单元地址，从0开始	
  I2C_start();                        //起始信号
  I2CSendByte(HMCAddress+1);         //发送设备地址+读信号
  REG_data=I2CreceiveByte();              //读出寄存器数据
  I2Cslave_ACK();   
  I2C_stop();                           //停止信号
  return REG_data; 
}

//******************************************************
//
//连续读出HMC5883内部角度数据，地址范围0x3~0x5
//
//******************************************************
void Multiple_read_HMC5883(void)
{   
  uint8 i;
  I2C_start();                          //起始信号
  I2CSendByte(HMCAddress);           //发送设备地址+写信号
  I2CSendByte(0x03);                   //发送存储单元地址，从0x3开始	
  I2C_start();                          //起始信号
  I2CSendByte(HMCAddress+1);         //发送设备地址+读信号
  for (i=0; i<7; i++)                      //连续读取6个地址数据，存储中BUF
  {
    HMC_BUF[i] = I2CreceiveByte();          //BUF[0]存储0x32地址中的数据
    if (i == 6)
    {
      I2Cslave_ACK();                //最后一个数据需要回NOACK
    }
    else
    {
      I2Cslave_NOACK();                //回应ACK
    }
  }
  I2C_stop();                          //停止信号
  north = HMC_BUF[0]<<8 | HMC_BUF[1];//指北
  //push(9,north);
  //push(10,HMC_BUF[4]<<8 | HMC_BUF[5]);
  //push(11,HMC_BUF[2]<<8 | HMC_BUF[3]);
}

//初始化HMC5883，根据需要请参考pdf进行修改****
void Init_HMC5883()
{
  Single_Write_HMC5883(0x02,0x00);  //
  Single_Write_HMC5883(0x01,0xE0);  //
}