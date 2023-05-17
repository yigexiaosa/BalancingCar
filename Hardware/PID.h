#ifndef __PID_H__
#define __PID_H__

float PID_Vertical(float Angle, float Gyro);
float PID_Speed(int encoder_left, int encoder_right, int Speed);
float PID_Turn(int16_t left,int16_t right,int16_t turn,int16_t gyro);
float PID_Distance(int16_t left,int16_t right,int16_t targetDistance,int16_t nowDistance);
void PID_ClearI(void);

#endif
