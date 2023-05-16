#ifndef __PID_H__
#define __PID_H__

float PID_Balance(float Angle, float Gyro);
float PID_Velocity(int encoder_left,int encoder_right,int Speed);

#endif
