#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

#define Kalman_Filter_dt 0.001f

float Kalman_Filter_X(float angle_m, float gyro_m);
float Kalman_Filter_Y(float angle_m, float gyro_m);
float Kalman_Filter_Z(float angle_m, float gyro_m);

#endif
