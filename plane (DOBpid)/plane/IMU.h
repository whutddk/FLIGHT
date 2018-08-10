#ifndef _IMU_H_
#define _IMU_H_

#define RtA 		57.324841				//弧度到角度
#define AtR    	0.0174533				//度到角度
#define Acc_G 	0.0011963				//加速度变成G
#define Gyro_G 	0.0152672				//角速度变成度
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

extern S_FLOAT_XYZ Q_ANGLE;			//四元数计算出的角度
extern S_INT16_XYZ MPU6050_GYRO_LAST;  //采集数据
extern S_INT16_XYZ MPU6050_ACC_OFFSET;
extern S_INT16_XYZ MPU6050_GYRO_OFFSET;
extern S_INT16_XYZ ACC_AVG;		//平均值滤波后的ACC

void Prepare_Data(void);
void Get_Attitude(void);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
void mes_offset();
#endif
