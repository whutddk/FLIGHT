#ifndef _IMU_H_
#define _IMU_H_

#define RtA 		57.324841				//���ȵ��Ƕ�
#define AtR    	0.0174533				//�ȵ��Ƕ�
#define Acc_G 	0.0011963				//���ٶȱ��G
#define Gyro_G 	0.0152672				//���ٶȱ�ɶ�
#define Gyro_Gr	0.0002663	
#define FILTER_NUM 20

typedef struct
{
  int16 X;
  int16 Y;
  int16 Z;
}S_INT16_XYZ;

typedef struct
{
  float X;
  float Y;
  float Z;
}S_FLOAT_XYZ;

extern S_FLOAT_XYZ Q_ANGLE;			//��Ԫ��������ĽǶ�
extern S_INT16_XYZ MPU6050_GYRO_LAST;  //�ɼ�����
extern S_INT16_XYZ MPU6050_ACC_OFFSET;
extern S_INT16_XYZ MPU6050_GYRO_OFFSET;
extern S_INT16_XYZ ACC_AVG;		//ƽ��ֵ�˲����ACC

void Prepare_Data(void);
void Get_Attitude(void);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
void mes_offset();
#endif
