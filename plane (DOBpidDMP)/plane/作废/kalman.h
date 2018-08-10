#ifndef _KALMAN_H_
#define _KALMAN_H_
//
//#define KALMAN_Q        0.02
//#define KALMAN_R        8.0000

extern double KALMAN_Q;
extern double KALMAN_R;

double KalmanFilter_x(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
double KalmanFilter_y(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
double KalmanFilter_z(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);



#endif
