//��̬����

#include "common.h"
#include "include.h"


S_INT16_XYZ ACC_AVG;		//ƽ��ֵ�˲����ACC
S_INT16_XYZ MPU6050_ACC_LAST;   //�ɼ�����
S_INT16_XYZ MPU6050_ACC_OFFSET; //��Ư

S_INT16_XYZ MPU6050_GYRO_LAST;  //�ɼ�����
S_INT16_XYZ MPU6050_GYRO_OFFSET;//��Ư
S_FLOAT_XYZ GYRO_I;		//�����ǻ���
S_FLOAT_XYZ EXP_ANGLE;		//�����Ƕ�
S_FLOAT_XYZ DIF_ANGLE;		//�����Ƕ���ʵ�ʽǶȲ�
S_FLOAT_XYZ Q_ANGLE;		//��Ԫ��������ĽǶ�

int16 ACC_X_BUF[FILTER_NUM],
ACC_Y_BUF[FILTER_NUM],
ACC_Z_BUF[FILTER_NUM];	//���ٶȻ��������˲�����

void mes_offset()//���֮ǰ���������������ֵ̬
{
  MPU6050_ACC_OFFSET.X += GetData(ACCEL_XOUT_H);
  MPU6050_ACC_OFFSET.X /= 2;
  
  MPU6050_ACC_OFFSET.Y += GetData(ACCEL_YOUT_H);
  MPU6050_ACC_OFFSET.Y /= 2;
  
  MPU6050_ACC_OFFSET.Z += GetData(ACCEL_ZOUT_H);
  MPU6050_ACC_OFFSET.Z /= 2;
  
  MPU6050_GYRO_OFFSET.X += GetData(GYRO_XOUT_H);
  MPU6050_GYRO_OFFSET.X /= 2;
  
  MPU6050_GYRO_OFFSET.Y += GetData(GYRO_YOUT_H);
  MPU6050_GYRO_OFFSET.Y /= 2;
  
  MPU6050_GYRO_OFFSET.Z += GetData(GYRO_ZOUT_H);
  MPU6050_GYRO_OFFSET.Z /= 2;
  
  
  //ָ�������ѹδ��
}


void Prepare_Data(void)
{
  static uint8 filter_cnt=0;
  int32_t temp1=0,temp2=0,temp3=0;
  uint8 i;
  
  MPU6050_ACC_LAST.X = GetData(ACCEL_XOUT_H) - MPU6050_ACC_OFFSET.X;
  MPU6050_ACC_LAST.Y = GetData(ACCEL_YOUT_H) - MPU6050_ACC_OFFSET.Y;
  MPU6050_ACC_LAST.Z = GetData(ACCEL_ZOUT_H) - MPU6050_ACC_OFFSET.Z ;
  
  MPU6050_GYRO_LAST.X = GetData(GYRO_XOUT_H) - MPU6050_GYRO_OFFSET.X;
  MPU6050_GYRO_LAST.Y = GetData(GYRO_YOUT_H) - MPU6050_GYRO_OFFSET.Y; 
  MPU6050_GYRO_LAST.Z = GetData(GYRO_ZOUT_H) - MPU6050_GYRO_OFFSET.Z;
  //  MPU6050_Read();
  //  MPU6050_Dataanl();
  //  
  ACC_X_BUF[filter_cnt] = MPU6050_ACC_LAST.X;//���»�����������
  ACC_Y_BUF[filter_cnt] = MPU6050_ACC_LAST.Y;
  ACC_Z_BUF[filter_cnt] = MPU6050_ACC_LAST.Z;
  for(i=0;i<FILTER_NUM;i++)
  {
    temp1 += ACC_X_BUF[i];
    temp2 += ACC_Y_BUF[i];
    temp3 += ACC_Z_BUF[i];
  }
  ACC_AVG.X = temp1 / FILTER_NUM;
  ACC_AVG.Y = temp2 / FILTER_NUM;
  ACC_AVG.Z = temp3 / FILTER_NUM;
  filter_cnt++;
  if(filter_cnt==FILTER_NUM)	filter_cnt=0;
  
  //����û�õ�
  //  GYRO_I.X += MPU6050_GYRO_LAST.X*Gyro_G*0.002;//0.0001��ʱ����,����prepare��ִ������
  //  GYRO_I.Y += MPU6050_GYRO_LAST.Y*Gyro_G*0.002;
  //  GYRO_I.Z += MPU6050_GYRO_LAST.Z*Gyro_G*0.002;
  
  push(0,ACC_AVG.X);
  push(1,ACC_AVG.Y);
  push(2,ACC_AVG.Z);

  
  push(3,MPU6050_GYRO_LAST.X);
  push(4,MPU6050_GYRO_LAST.Y);
  push(5,MPU6050_GYRO_LAST.Z);
}

void Get_Attitude(void)
{
  IMUupdate(MPU6050_GYRO_LAST.X*Gyro_Gr,
            MPU6050_GYRO_LAST.Y*Gyro_Gr,
            MPU6050_GYRO_LAST.Z*Gyro_Gr,
            ACC_AVG.X,ACC_AVG.Y,ACC_AVG.Z);	//*0.0174ת�ɻ���
}
////////////////////////////////////////////////////////////////////////////////
#define Kp 10.0f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.008f                          // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.0025f                   // half the sample period�������ڵ�һ��

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
  
  float DSP_TEMP1,DSP_TEMP2,DSP_TEMP3;
  float norm;
  //  float hx, hy, hz, bx, bz;
  float vx, vy, vz;// wx, wy, wz;
  float ex, ey, ez;
  
  // �Ȱ���Щ�õõ���ֵ���
  //DSP
  float q0q0;// = q0*q0;
  float q0q1;// = q0*q1;
  float q0q2;// = q0*q2;
  //  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  //  float q1q2 = q1*q2;
  float q1q3;// = q1*q3;
  float q2q2;// = q2*q2;
  float q2q3;// = q2*q3;
  float q3q3;// = q3*q3;  
  arm_mult_f32(&q0,&q0,&q0q0,1);
  arm_mult_f32(&q0,&q1,&q0q1,1);
  arm_mult_f32(&q0,&q2,&q0q2,1);
  //arm_mult_f32(&q0,&q3,&q0q3,1);
  arm_mult_f32(&q1,&q1,&q1q1,1);
  //arm_mult_f32(&q1,&q2,&q1q2,1);
  arm_mult_f32(&q1,&q3,&q1q3,1);
  arm_mult_f32(&q2,&q2,&q2q2,1);
  arm_mult_f32(&q2,&q3,&q2q3,1);
  arm_mult_f32(&q3,&q3,&q3q3,1);
  
  
  if(ax*ay*az==0)
    return;
  
  
  // DSP  norm = sqrt(ax*ax + ay*ay + az*az);       //acc���ݹ�һ��
  arm_mult_f32(&ax,&ax,&DSP_TEMP1,1);
  arm_mult_f32(&ay,&ay,&DSP_TEMP2,1);
  arm_mult_f32(&az,&az,&DSP_TEMP3,1);
  arm_sqrt_f32(DSP_TEMP1 + DSP_TEMP2 + DSP_TEMP3 ,&norm);
  
  //
  ax = ax /norm;
  ay = ay / norm;
  az = az / norm;
  
  // estimated direction of gravity and flux (v and w)              �����������������/��Ǩ
  //DSP
  //vx = 2*(q1q3 - q0q2);	 
  arm_sub_f32(&q1q3,&q0q2,&DSP_TEMP1,1);
  DSP_TEMP2 = 2.;
  arm_mult_f32(&DSP_TEMP1,&DSP_TEMP2,&vx,1);
  //��Ԫ����xyz�ı�ʾ
  
  //vy = 2*(q0q1 + q2q3);
  arm_add_f32(&q0q1,&q2q3,&DSP_TEMP1,1);
  DSP_TEMP2 = 2.;
  arm_mult_f32(&DSP_TEMP1,&DSP_TEMP2,&vy,1);
  
  //vz = q0q0 - q1q1 - q2q2 + q3q3 ;
  arm_sub_f32(&q0q0,&q1q1,&DSP_TEMP1,1);
  arm_sub_f32(&DSP_TEMP1,&q2q2,&DSP_TEMP2,1);
  arm_add_f32(&DSP_TEMP2,&q3q3,&vz,1);
  
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  //ex = (ay*vz - az*vy) ;                           					 //�������������õ���־������
  arm_mult_f32(&ay,&vz,&DSP_TEMP1,1);
  arm_mult_f32(&az,&vy,&DSP_TEMP2,1);
  arm_sub_f32(&DSP_TEMP1,&DSP_TEMP2,&ex,1);
  
  //ey = (az*vx - ax*vz) ;
  arm_mult_f32(&az,&vx,&DSP_TEMP1,1);
  arm_mult_f32(&ax,&vz,&DSP_TEMP2,1);
  arm_sub_f32(&DSP_TEMP1,&DSP_TEMP2,&ey,1);
  
  
  //ez = (ax*vy - ay*vx) ;
  arm_mult_f32(&ax,&vy,&DSP_TEMP1,1);
  arm_mult_f32(&ay,&vx,&DSP_TEMP2,1);
  arm_sub_f32(&DSP_TEMP1,&DSP_TEMP2,&ez,1);
  
  
  exInt = exInt + ex * Ki;								  //�������л���
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;
  
  // adjusted gyroscope measurements
  gx = gx + Kp * ex + exInt;					   							//�����PI�󲹳��������ǣ����������Ư��
  gy = gy + Kp * ey + eyInt;
  gz = gz + Kp * ez + ezInt;				   							//�����gz����û�й۲��߽��н��������Ư�ƣ����ֳ����ľ��ǻ����������Լ�
  
  // integrate quaternion rate and normalise						   //��Ԫ�ص�΢�ַ���
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;
  
  // normalise quaternion
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;
  
  //Q_ANGLE.Yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* 57.3; // yaw
  
  
  Q_ANGLE.Y = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; 
  Q_ANGLE.X = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; 
  
  push (7,Q_ANGLE.X * 100);
  push (8,Q_ANGLE.Y * 100);
}