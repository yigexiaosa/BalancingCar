#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "OLED.h"
#include "EncoderSpeed.h"
#include "Key.h"
#include "Motor.h"
#include "Timer.h"
#include "MPU6050.h"
#include "KalmanFilter.h"
#include "Delay.h"
#include "math.h"
#include "PID.h"
#include "HC_RS04.h"
#include "Bluetooth.h"

int16_t TargetSpeed,leftTargetSpeed,rightTargetSpeed,leftEncoderSpeed,rightEncoderSpeed,speedState = 200;
int16_t leftSpeed, rightSpeed;
uint16_t time;
float angleX,angleY,angleZ;
uint16_t TimerCount;
uint8_t bluetooth_ReceiveData,bluetooth_SendData;

//int TestCount;

MPU6050_GetDataTypeDef Data;

int main(){
    Delay_ms(100);// 上电延时

    OLED_Init();
    EncoderSpeed_Init();
    Key_Init();
    Motor_Init();
    MPU6050_Init();
    HC_RS04_Init();
    Bluetooth_Init();

    // 定时器最后初始化
    Timer_Init();

    while(1){
        if(Key_GetNum() == 1){
            leftTargetSpeed += speedState;
            if(leftTargetSpeed >= 2200 || leftTargetSpeed <= -2200) {
                leftTargetSpeed = 0;
                speedState = -speedState;
            }
        }

//        // 偏航角
//        OLED_ShowSignedNum(1, 1, (int16_t)angleZ,3);
//        OLED_ShowString(1, 5, ".");
//        OLED_ShowNum(1, 6, ((int16_t)((angleZ > 0 ? angleZ: -angleZ)*1000))%1000,3);
//
//        // 俯仰角
//        OLED_ShowSignedNum(2, 1, (int16_t)angleY,3);
//        OLED_ShowString(2, 5, ".");
//        OLED_ShowNum(2, 6, ((int16_t)((angleY > 0 ? angleY: -angleY)*1000))%1000,3);

        OLED_ShowNum(3, 1, time, 5);

        // 速度
        OLED_ShowSignedNum(4, 1, leftTargetSpeed, 5);
        OLED_ShowSignedNum(4, 10, leftEncoderSpeed, 5);
        OLED_ShowSignedNum(3, 10, leftSpeed, 5);



//        // 距离
//        OLED_ShowNum(1, 1, HC_RS04_Time_Small_Old, 5);
//        OLED_ShowNum(1, 10, HC_RS04_Time_Small_New, 5);
//        OLED_ShowNum(2, 1, HC_RS04_Time_Big_New, 5);
//        OLED_ShowNum(2,10,HC_RS04_Distance,5);

        // 蓝牙
        if(Bluetooth_GetRxFlag()){
            bluetooth_ReceiveData = Bluetooth_GetRxData();
            bluetooth_SendData = bluetooth_ReceiveData + 1;
            Bluetooth_SendByte(bluetooth_SendData);
        }

        OLED_ShowHexNum(1, 1, bluetooth_ReceiveData, 2);
        OLED_ShowHexNum(1, 4, bluetooth_SendData, 2);
    }
}

// 1ms运行一次
void TIM1_UP_IRQHandler(void){
    static float speedLoop,verticalLoop,steeringRing,leftSpeed_LPF,rightSpeed_LPF;

    {// 超声波时间
        HC_RS04_Time_Big++;
    }
    {// 电机测速
        leftEncoderSpeed = EncoderSpeed_Get() * 1000 / (4 * 34);
    }
    {// MPU6050
        if (MPU6050_GetState() != 0) {
            MPU6050_WaitState();
        } else {
            MPU6050_GetData(&Data);
        }
    }

    {// 滤波 + 速度环+直立环+转向环
        {// 滤波
            if (Data.GyroZ <= 2 && Data.GyroZ >= -2) Data.GyroZ = 0;
            // 偏航角
            // 偏航角速度
            float addAngleZ = ((float) Data.GyroZ) * MPU6050_AngleRange / 32768;
            angleZ += Kalman_Filter_dt * addAngleZ;
            angleZ = Kalman_Filter_Z(angleZ, addAngleZ);
            // 俯仰角
            // 俯仰角速度
            float addAngleY = ((float) Data.GyroY) * MPU6050_AngleRange / 32768;
            // 利用加速度计获取俯仰角
            angleY = -atanf((float) Data.AccX / (float) Data.AccZ) * 57.2974f;
            angleY = Kalman_Filter_Y(angleY, addAngleY);
        }

        {// PID 速度环PI + 直立环PD +
            // 平滑加速
            leftSpeed_LPF = leftSpeed_LPF * 0.8f + leftTargetSpeed * 0.2f;// 一阶低通滤波，为了平滑加速
            // 速度环
            speedLoop = PID_Velocity(leftEncoderSpeed, leftEncoderSpeed, leftSpeed_LPF);
            // 直立环
            verticalLoop = PID_Balance(angleY, ((float) Data.GyroY) * MPU6050_AngleRange / 32768);
            // 转向环
//       steeringRing = ;
            leftSpeed =
                    speedLoop
                    * 0.5 +
                    verticalLoop
                    * 0.5
                    ;
            Motor_SetLeftSpeed(leftSpeed);
        }
    }

//    TestCount++;
//    if(TestCount >= 5000){
//        MPU6050_Test();
//    }
    TimerCount ++;
    if(TimerCount >= 100){
        TimerCount = 0;
        HC_RS04_Open();
    }

    time = TimerCount_GetCount();

    TIM_ClearITPendingBit(Timer_TIMX, TIM_IT_Update);
}
