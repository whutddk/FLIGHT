#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"

unsigned char whoAmI = 0;
unsigned char gyo_XH = 0;
unsigned char gyo_XL = 0;
unsigned char gyo_YH = 0;
unsigned char gyo_YL = 0;
unsigned char gyo_ZH = 0;
unsigned char gyo_ZL = 0;
unsigned char acc_XH = 0;
unsigned char acc_XL = 0;
unsigned char acc_YH = 0;
unsigned char acc_YL = 0;
unsigned char acc_ZH = 0;
unsigned char acc_ZL = 0;

unsigned short gyo_X = 0;
unsigned short gyo_Y = 0;
unsigned short gyo_Z = 0;
unsigned short acc_X = 0;
unsigned short acc_Y = 0;
unsigned short acc_Z = 0;

short int mag_X = 0;
short int mag_Y = 0;
short int mag_Z = 0;

#define q30  1073741824.0f
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;

long int quat[4];
float Pitch = 0.0f;
float Roll = 0.0f;
float Yaw = 0.0f;
float motor1=0.0,motor2=0.0,motor3=0.0,motor4=0.0;
/* Data requested by client. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (100)

struct rx_s {
  unsigned char header[3];
  unsigned char cmd;
};
struct hal_s {
  unsigned char sensors;
  unsigned char dmp_on;
  unsigned char wait_for_tap;
  volatile unsigned char new_gyro;
  unsigned short report;
  unsigned short dmp_features;
  unsigned char motion_int_mode;
  struct rx_s rx;
};
static struct hal_s hal = {0};

/* The sensors can be mounted onto the board in any orientation. The mounting
* matrix seen below tells the MPL how to rotate the raw data from thei
* driver(s).
* TODO: The following matrices refer to the configuration on an internal test
* board at Invensense. If needed, please modify the matrices to match the
* chip-to-body matrix for your particular set up.
*/
static signed char gyro_orientation[9] = {-1,  0,  0,
0, -1,  0,
0,  0,  1};

enum packet_type_e {
  PACKET_TYPE_ACCEL,
  PACKET_TYPE_GYRO,
  PACKET_TYPE_QUAT,
  PACKET_TYPE_TAP,
  PACKET_TYPE_ANDROID_ORIENT,
  PACKET_TYPE_PEDO,
  PACKET_TYPE_MISC
};

void send_client(void)
{
  //send_packet(PACKET_TYPE_QUAT, quat);
}

/* These next two functions converts the orientation matrix (see
* gyro_orientation) to a scalar representation for use by the DMP.
* NOTE: These functions are borrowed from Invensense's MPL.
*/
static inline unsigned short inv_row_2_scale(const signed char *row) {
  unsigned short b;
  
  if (row[0] > 0)
    b = 0;
  else if (row[0] < 0)
    b = 4;
  else if (row[1] > 0)
    b = 1;
  else if (row[1] < 0)
    b = 5;
  else if (row[2] > 0)
    b = 2;
  else if (row[2] < 0)
    b = 6;
  else
    b = 7; // error
  return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx) {
  unsigned short scalar;
  /*
  XYZ  010_001_000 Identity Matrix
  XZY  001_010_000
  YXZ  010_000_001
  YZX  000_010_001
  ZXY  001_000_010
  ZYX  000_001_010
  */
  
  scalar = inv_row_2_scale(mtx);
  scalar |= inv_row_2_scale(mtx + 3) << 3;
  scalar |= inv_row_2_scale(mtx + 6) << 6;
  
  return scalar;
}

/* Handle sensor on/off combinations. */
static void setup_gyro(void) {
  unsigned char mask = 0;
  if (hal.sensors & ACCEL_ON)
    mask |= INV_XYZ_ACCEL;
  if (hal.sensors & GYRO_ON)
    mask |= INV_XYZ_GYRO;
  /* If you need a power transition, this function should be called with a
  * mask of the sensors still enabled. The driver turns off any sensors
  * excluded from this mask.
  */
  mpu_set_sensors(mask);
  if (!hal.dmp_on)
    mpu_configure_fifo(mask);
}

static void tap_cb(unsigned char direction, unsigned char count) {
  char data[2];
  data[0] = (char)direction;
  data[1] = (char)count;
}

static inline void run_self_test(void) {
  int result;
//  char test_packet[4] = {0};
  long gyro[3], accel[3];
  
  result = mpu_run_self_test(gyro, accel);
  if (result == 0x7) {
    /* Test passed. We can trust the gyro data here, so let's push it down
    * to the DMP.
    */
    float sens;
    unsigned short accel_sens;
    mpu_get_gyro_sens(&sens);
    gyro[0] = (long) (gyro[0] * sens);
    gyro[1] = (long) (gyro[1] * sens);
    gyro[2] = (long) (gyro[2] * sens);
    dmp_set_gyro_bias(gyro);
    mpu_get_accel_sens(&accel_sens);
    accel[0] *= accel_sens;
    accel[1] *= accel_sens;
    accel[2] *= accel_sens;
    dmp_set_accel_bias(accel);
  }
    /* Report results. */
 // test_packet[0] = 't';
 // test_packet[1] = result;
//  send_packet(PACKET_TYPE_MISC, test_packet);
}

static void gyro_data_ready_cb(void) {
  hal.new_gyro = 1;
}








