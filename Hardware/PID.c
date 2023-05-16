#include "stm32f10x.h"
#include "stm32f10x_conf.h"

// 记得调整参数极性
#define PID_ZHONGZHI (0.03f)
#define PID_Balance_Kp 410.0f
#define PID_Balance_Kd 1.4f
#define PID_Velocity_Kp (-10.0f)
#define PID_Velocity_Ki (-0.05f)

float Encoder_Integral;

/**
 * @brief 直立环 PD
 * @param Angle 当前俯仰角度
 * @param Gyro 俯仰角速度
 * @return 电机速度系数
 */
float PID_Balance(float Angle, float Gyro) {
    //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数
    return -(PID_Balance_Kp * (Angle - PID_ZHONGZHI) + Gyro * PID_Balance_Kd);
}

/**
 * @brief 速度环 PI
 * @param encoder_left
 * @param encoder_right
 * @param Speed 目标速度,0为直立
 * @return
 */
float PID_Velocity(int encoder_left,int encoder_right,int Speed)
{
    static float Velocity,Encoder_Least,Encoder,Movement;

//    Movement = Speed;

    //=============速度PI控制器=======================//
    Encoder_Least =(encoder_left + encoder_right)*0.5 - Speed;        //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零）
    Encoder *= 0.8f;		                                          //===一阶低通滤波器
    Encoder += Encoder_Least*0.2f;	                                  //===一阶低通滤波器
    Encoder_Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
    Encoder_Integral=Encoder_Integral+Movement;                       //===接收遥控器数据，控制前进后退
    if(Encoder_Integral>100000) Encoder_Integral=100000;                //===积分限幅
    if(Encoder_Integral<-100000)	Encoder_Integral=-100000;              //===积分限幅
    Velocity=Encoder*PID_Velocity_Kp+Encoder_Integral*PID_Velocity_Ki;//===速度控制
    return Velocity;
}

/**
 * @brief 清楚积分累计
 */
void PID_ClearI(void){
    Encoder_Integral = 0;
}
