#include "stm32f10x.h"
#include "stm32f10x_conf.h"

// 记得调整参数极性
#define PID_ZHONGZHI (4.0f)
// 直立环 PD
#define PID_Vertical_Kp (-200.0f / 0.4f)
#define PID_Vertical_Kd (-6.0f / 0.4f)
// 速度环 PI
#define PID_Speed_Kp (0.0f)
#define PID_Speed_Ki (PID_Speed_Kp / 200.0f)
// 转向环 PD
#define PID_Turn_Kp (-0.0f / 0.2f)
#define PID_Turn_Kd (-0.0f / 0.2f)
// 距离环 Pi
#define PID_Distance_Kp (-0.0f / 0.2f)
#define PID_Distance_Ki (-0.0f / 0.2f)

float Encoder_Integral;

/**
 * @brief 直立环 PD
 * @param Angle 当前俯仰角度
 * @param Gyro 俯仰角速度
 * @return 电机速度系数
 */
float PID_Vertical(float Angle, float Gyro) {
    //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数
    return PID_Vertical_Kp * (Angle - PID_ZHONGZHI) + Gyro * PID_Vertical_Kd;
}

/**
 * @brief 速度环 PI
 * @param encoder_left
 * @param encoder_right
 * @param Speed 目标速度,0为直立
 * @return 电机速度系数
 */
float PID_Speed(int encoder_left, int encoder_right, int speed) {
    static double Velocity, Encoder_Least, Encoder;
    static double Encoder_Integral;
    Encoder_Least = (encoder_left + encoder_right) * 0.5f - speed;
    Encoder *= 0.7f;
    Encoder += Encoder_Least*0.3f;
    if(Encoder * Encoder_Integral < 0){
        Encoder_Integral = 0;
    }
    Encoder_Integral += Encoder;
    // 积分限幅
    if(Encoder_Integral >= 100000) Encoder_Integral = 100000;
    if(Encoder_Integral <= -100000) Encoder_Integral = -100000;
    Velocity = Encoder * PID_Speed_Kp + Encoder_Integral * PID_Speed_Ki;
    return Velocity;
}

/**
 * @brief 转向环 PD
 * @param left 左电机编码器速度
 * @param right 右电机编码器速度
 * @param turn 目标转向速度
 * @param gyro 当前偏航角速度
 * @return 电机速度差系数
 */
float PID_Turn(int16_t left, int16_t right, int16_t turn, int16_t gyro) {
    return (turn - (left - right)) * PID_Turn_Kp + gyro * PID_Turn_Kd;
}

/**
 * @brief 距离环 PD  距离环需要配合速度环使用
 * @param left 左电机编码器速度
 * @param right 右电机编码器速度
 * @param targetDistance 目标距离
 * @param nowDistance 当前距离
 * @return targetSpeed
 */
float PID_Distance(int16_t left, int16_t right, int16_t targetDistance, int16_t nowDistance) {
    return (targetDistance - nowDistance) * PID_Distance_Kp + (left + right) * PID_Distance_Ki;
}

/**
 * @brief 清楚积分累计
 */
void PID_ClearI(void) {
    Encoder_Integral = 0;
}
