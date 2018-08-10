#ifndef _BMP180_H_
#define _BMP180_H_


#define	BMP180_DEVICE_ADDRESS_BASE_VALUE   0xee	 //*器件地址基值              
//#define BMP180_CONTROL_REGISTER_ADDRESS_BASE_VALUE	0xf4 //*控制寄存器地址
#define BMP180_ID_REGISTER_ADDRESS	   0xd0 //*ID编号寄存器(0x55固定)
#define BMP180_VERSION_REGISTER_ADDRESS	   0XD1 //*版本编号
//#define BMP180_SOFT_RESET_REGISTER_BASE_VALUE	    0xe0 //软件复位寄存器,只写，设置0xb6
//id register 
#define BMP180_ID_FIXED_VALUE		0x55 /*id固定编号(0x55)*/
#define OSS  2
//BMP180校正参数(calibration param)
typedef struct {
	int16 AC1 ;
	int16 AC2 ;
	int16 AC3 ;
	uint16 AC4 ;
	uint16 AC5 ;
	uint16 AC6 ;
	int16 B1 ;
	int16 B2 ;
	int16 MB ;
	int16 MC_ ;
	int16 MD ;
}BMP180_cal_param;


typedef struct {
	uint8  ExistFlag ;  //存在标志

	BMP180_cal_param  cal_param;//修正系数

	uint8 Version ;				//版本

	int32 UnsetTemperature ;		//未校正的温度值
	int32 UnsetGasPress	  ;		//未校正的气压值

	int32 Temperature ;			/*校正后的温度值*/
	int32 GasPress ;				/*校正后的气压值*/

	float Altitude ;				/*海拔*/
	
}BMP180_info ;

extern BMP180_info temp;


void BMP180Convert(BMP180_info *temp);
void BMP180Init(BMP180_info *p);
#endif