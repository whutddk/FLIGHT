#include "include.h"
#include "My_IIC.h"
#include "MA.h"

int result;
unsigned char accel_fsr;
unsigned short gyro_rate, gyro_fsr;
struct int_param_s int_param;
unsigned char more;
short gyro[3], accel[3], sensors;

float roll_quiet = 0.;
float pitch_quiet = 0.;
float yaw_quiet = 0;


void IMU_init()
{
     /***********************************************************
  Warring:
  设置MPU6050的DMP库功能，直接输出四元数   
  ************************************************************/
  int_param.cb = gyro_data_ready_cb;
  int_param.active_low = 1;
  result = mpu_init(&int_param);
  
  /* Get/set hardware configuration. Start gyro. */
  /* Wake up all sensors. */
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  /* Push both gyro and accel data into the FIFO. */
  mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_set_sample_rate(DEFAULT_MPU_HZ);
  mpu_get_sample_rate(&gyro_rate);
  mpu_get_gyro_fsr(&gyro_fsr);
  mpu_get_accel_fsr(&accel_fsr);
  
  /* Initialize HAL state variables. */
  memset(&hal, 0, sizeof (hal));
  hal.sensors = ACCEL_ON | GYRO_ON;
  hal.report = PRINT_QUAT;
 
    result = dmp_load_motion_driver_firmware();
#ifdef DELUG_USE_TERMINAL
  if (0 == result)
    UART_send_str(">>> Load Firmware Successful!\n");
#endif
  dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
  dmp_register_tap_cb(tap_cb);
//  dmp_register_android_orient_cb(android_orient_cb);
  hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
  DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
  DMP_FEATURE_GYRO_CAL;
  dmp_enable_feature(hal.dmp_features);
  dmp_set_fifo_rate(DEFAULT_MPU_HZ);
  mpu_set_dmp_state(1);
  hal.dmp_on = 1;
}

void MIU_getdata()
{
  double temp_1[3] = 0.;
  
  
   unsigned long sensor_timestamp;
   
   LPLD_GPIO_Output_b(PTA, 17, 1);
   
   hal.new_gyro = 1;
      // 未使能传感器或未收到新数据
//      if (!hal.sensors || !hal.new_gyro) {
//        // 可以在此处休眠，以降低功耗
//       // continue;
//      }
      // 传感器数据准备好并且打开DMP功能
      if (hal.new_gyro && hal.dmp_on) {
       
        dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
//        acc_X = mpu6050_getdata('A', 'X');
//        acc_Y = mpu6050_getdata('A', 'Y');
//        acc_Z = mpu6050_getdata('A', 'Z');
//        HMC_SingleWrite(0x0A, 0x01);
//        
//        //DelayMs(10);
//       
//        mag_X = HMC_SingleRead(HXH);
//        mag_X = mag_X<<8;
//        mag_X += HMC_SingleRead(HXL);
//        mag_Y = HMC_SingleRead(HYH);
//        mag_Y = mag_Y<<8;
//        mag_Y += HMC_SingleRead(HYL);
//        mag_Z = HMC_SingleRead(HZH);
//        mag_Z = mag_Z<<8;
//        mag_Z += HMC_SingleRead(HZL);
      }      
       if (sensors & INV_WXYZ_QUAT && hal.report & PRINT_QUAT)
        {
          q0=quat[0] / q30;
          q1=quat[1] / q30;
          q2=quat[2] / q30;
          q3=quat[3] / q30;
          
          Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
          Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
          Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3; //yaw
        
        push(0,(int16)( ( Pitch - pitch_quiet ) * 100 ) );
        push(1,(int16)( ( Roll - roll_quiet ) * 100 ) );
        push(2,(int16)( ( Yaw - yaw_quiet ) * 100 ) );
        push(3,gyro[0]);
        push(4,gyro[1]);
        push(5,gyro[2]);
        
        }
       else if (hal.new_gyro)
      {
   
      }
   //    NRF_Send_AF();                     //发送给上位机显示波形或3D效果
  LPLD_GPIO_Output_b(PTA, 17, 0);
}