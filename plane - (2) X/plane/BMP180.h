#ifndef _BMP180_H_
#define _BMP180_H_


#define	BMP180_DEVICE_ADDRESS_BASE_VALUE   0xee	 //*������ַ��ֵ              
//#define BMP180_CONTROL_REGISTER_ADDRESS_BASE_VALUE	0xf4 //*���ƼĴ�����ַ
#define BMP180_ID_REGISTER_ADDRESS	   0xd0 //*ID��żĴ���(0x55�̶�)
#define BMP180_VERSION_REGISTER_ADDRESS	   0XD1 //*�汾���
//#define BMP180_SOFT_RESET_REGISTER_BASE_VALUE	    0xe0 //�����λ�Ĵ���,ֻд������0xb6
//id register 
#define BMP180_ID_FIXED_VALUE		0x55 /*id�̶����(0x55)*/
#define OSS  2
//BMP180У������(calibration param)
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
	uint8  ExistFlag ;  //���ڱ�־

	BMP180_cal_param  cal_param;//����ϵ��

	uint8 Version ;				//�汾

	int32 UnsetTemperature ;		//δУ�����¶�ֵ
	int32 UnsetGasPress	  ;		//δУ������ѹֵ

	int32 Temperature ;			/*У������¶�ֵ*/
	int32 GasPress ;				/*У�������ѹֵ*/

	float Altitude ;				/*����*/
	
}BMP180_info ;

extern BMP180_info temp;


void BMP180Convert(BMP180_info *temp);
void BMP180Init(BMP180_info *p);
#endif