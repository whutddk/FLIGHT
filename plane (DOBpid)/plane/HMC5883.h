#ifndef _HMC5883_H_
#define _HMC5883_H_

#define	HMCAddress   0x3C	  //��������5883��IIC�����еĴӵ�ַ

extern int16 north ;


void Single_Write_HMC5883(uint8 REG_Address,uint8 REG_data);
uint8 Single_Read_HMC5883(uint8 REG_Address);
void Multiple_read_HMC5883(void);
void Init_HMC5883();
#endif 
