#include "include.h"



BMP180_info temp;
void BMP180AddressWrite(uint8 addresss,uint8 dataCode)
{
  I2C_start();    
  I2CSendByte(BMP180_DEVICE_ADDRESS_BASE_VALUE );   
  I2CSendByte(addresss);   ;
  I2CSendByte(dataCode);        
  I2C_stop();                   
}

/******************************************************
Function	:BMP180AddressReadByte
Input		:address
Output		:N/A
Return		:data which read from bmp180's address
Description	:read byte-data from bmp180's address
Note		:不需要应答.
******************************************************/
uint8 BMP180AddressReadByte(uint8 address)
{  
  uint8 dataCode;
  
  I2C_start();                           
  I2CSendByte(BMP180_DEVICE_ADDRESS_BASE_VALUE );   
  I2CSendByte(address);           
  I2C_start();                          
  I2CSendByte(BMP180_DEVICE_ADDRESS_BASE_VALUE + 1);   
  dataCode=I2CreceiveByte();  
  I2Cslave_ACK() ;
  I2C_stop();  
  
  return dataCode; 
}

/******************************************************
Function	:BMP180AddressRead2Byte
Input		:address
Output		:N/A
Return		:long data
Description	:从连续地址读取数据，并"组装"为long型数据
Note		:N/A
******************************************************/
static int32 BMP180AddressRead2Byte(uint8 address)
{
  uint8 msb , lsb ;
  
  msb = BMP180AddressReadByte(address)   ;
  lsb = BMP180AddressReadByte(address+1) ;
  
  return ( ((int32)msb) << 8 | lsb) ;
}

/******************************************************
Function	:BMP180ReadUnsetTemperature
Input		:address
Output		:N/A
Return		:shour int byte-data
Description	:读取未校正的温度值
Note		:接收后面的一字节数据，主机不需要应答
******************************************************/
static int32 BMP180ReadUnsetTemperature(void)
{
  BMP180AddressWrite(0xf4,0x2e) ;
  
  return (BMP180AddressRead2Byte(0xf6));
}

/******************************************************
Function	:BMP180ReadUnsetPressure
Input		:N/A
Output		:N/A
Return		:未校正气压值
Description	:读取未校正的气压值
Note		:N/A
******************************************************/
static int32 BMP180ReadUnsetPressure(void)
{
  
  int32 pressure = 0;
  
  BMP180AddressWrite(0xf4,0x34 + (OSS<<6)) ;
  
//  delay5msForBMP180();
//  delay5msForBMP180();
  
  
  pressure = BMP180AddressRead2Byte(0xf6) ;
  pressure = (((int32)pressure <<8) + BMP180AddressReadByte(0xf8)) >>(8-OSS) ;
  
  return pressure;	
  
}

/******************************************************
Function	:BMP180ReadCalibrateParam
Input		:BMP180_info type point
Output		:AC1,AC3,AC3,AC4,AC5,AC6,B1,B2,MB,MC,MD
Return		:N/A
Description	:读取校正参数
Note		:N/A
******************************************************/
static void BMP180ReadCalibrateParam(BMP180_info *p)
{
  p->cal_param.AC1= BMP180AddressRead2Byte(0xAA);
  p->cal_param.AC2= BMP180AddressRead2Byte(0xAC);
  p->cal_param.AC3= BMP180AddressRead2Byte(0xAE);
  p->cal_param.AC4= BMP180AddressRead2Byte(0xB0);
  p->cal_param.AC5= BMP180AddressRead2Byte(0xB2);
  p->cal_param.AC6= BMP180AddressRead2Byte(0xB4);
  p->cal_param.B1=  BMP180AddressRead2Byte(0xB6);
  p->cal_param.B2=  BMP180AddressRead2Byte(0xB8);
  p->cal_param.MB=  BMP180AddressRead2Byte(0xBA);
  p->cal_param.MC_=  BMP180AddressRead2Byte(0xBC);
  p->cal_param.MD=  BMP180AddressRead2Byte(0xBE);
}

/******************************************************
Function	:Init_BMP180
Input		:AC1,AC2,AC3,AC4,AC5,AC6,B1,B2,MB,MC,MD
Output		:AC1,AC2,AC3,AC4,AC5,AC6,B1,B2,MB,MC,MD
Return		:N/A
Description	:初始化
Note		:N/A
******************************************************/
void BMP180Init(BMP180_info *p)
{
  while(BMP180AddressReadByte(BMP180_ID_REGISTER_ADDRESS)!= BMP180_ID_FIXED_VALUE);//存在

    
    BMP180ReadCalibrateParam(p);
    
    p->Version = BMP180AddressReadByte(BMP180_VERSION_REGISTER_ADDRESS);
  

}

void BMP180Convert(BMP180_info *temp)
{	
  int32 x1, x2, B5, B6, x3, B3, p;
  unsigned long b4, b7;
  
  //未校正的温度值
  temp->UnsetTemperature = BMP180ReadUnsetTemperature();
  //未校正的气压值
  temp->UnsetGasPress = BMP180ReadUnsetPressure();
  
  //温度校正
  x1 = ((temp->UnsetTemperature) - temp->cal_param.AC6) * (temp->cal_param.AC5) >> 15;
  x2 = ((int32)(temp->cal_param.MC_) << 11) / (x1 + temp->cal_param.MD);
  B5 = x1 + x2;
  temp->Temperature= (B5 + 8) >> 4;
  
  //气压校正
  B6 = B5- 4000;
  x1 = ((int32)(temp->cal_param.B2) * (B6 * B6 >> 12)) >> 11;
  x2 = ((int32)temp->cal_param.AC2) * B6 >> 11;
  x3 = x1 + x2;
  B3 = ((((int32)(temp->cal_param.AC1) * 4 + x3)<<OSS) + 2)/4;
  x1 = ((int32)temp->cal_param.AC3) * B6 >> 13;
  x2 = ((int32)(temp->cal_param.B1) * (B6 * B6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = ((int32)(temp->cal_param.AC4) * (unsigned long) (x3 + 32768)) >> 15;
  b7 = ((unsigned long)(temp->UnsetGasPress) - B3) * (50000 >> OSS);
  if( b7 < 0x80000000)
  {
    p = (b7 * 2) / b4 ;
  }
  else
  {
    p = (b7 / b4) * 2;
  }
  x1 = (p >> 8) * (p >> 8);
  x1 = ((int32)x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  temp->GasPress= p + ((x1 + x2 + 3791) >> 4);
  
  //海拔计算
  temp->Altitude =(44330.0 * (1.0-pow((float)(temp->GasPress) / 101325.0, 1.0/5.255)) );
  //push(12,(int16)(temp->GasPress/10.-10000));
}