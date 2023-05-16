#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "OLED.h"
#include "EncoderSpeed.h"
#include "Motor.h"
#include "Timer.h"
#include "MPU6050.h"
#include "KalmanFilter.h"
#include "Delay.h"
#include "math.h"
#include "PID.h"
#include "HC_RS04.h"
#include "Bluetooth.h"

int16_t targetSpeed,leftTargetSpeed,rightTargetSpeed,leftEncoderSpeed,rightEncoderSpeed;
// 输入给电机的速度
int16_t leftSpeed, rightSpeed;
// 定时函数运行耗时计时
uint16_t time;
// 俯仰角之类的
float angleX,angleY,angleZ;
// 用于进行一次超声波测距的ms级计时
uint16_t TimerCount;
// 蓝牙读取、发送的信息
uint8_t bluetooth_ReceiveData,bluetooth_SendData;
// 状态机
uint8_t mainState_Now,mainState_Last,mainState_0;
// PID相关
float speedLoop,verticalLoop,turnLoop;
// 平滑变速
float leftSpeed_LPF,rightSpeed_LPF,Speed_LPF;

MPU6050_GetDataTypeDef Data;

void MPU6050_ReadDataAndFilter(void);
void MainStateTransition_DipToBig(void);
void MainStateTransition_DipNormal(void);
void MainStateTransition_TakeUp(void);
void MainStateTransition_LayDown(void);
void MainStateTransition_Bluetooth(void);
void Loop_Vertical(void);
void Loop_Speed(void);
void Loop_Turn(void);
void Loop_Distance(void);



int main(){
    Delay_ms(100);// 上电延时

    OLED_Init();
    EncoderSpeed_Init();
    Motor_Init();
    MPU6050_Init();
    HC_RS04_Init();
    Bluetooth_Init();

    // 定时器最后初始化
    Timer_Init();

    while(1){

        // 偏航角
        OLED_ShowSignedNum(1, 1, (int16_t)angleZ,3);
        OLED_ShowString(1, 5, ".");
        OLED_ShowNum(1, 6, ((int16_t)((angleZ > 0 ? angleZ: -angleZ)*1000))%1000,3);

        // 俯仰角
        OLED_ShowSignedNum(2, 1, (int16_t)angleY,3);
        OLED_ShowString(2, 5, ".");
        OLED_ShowNum(2, 6, ((int16_t)((angleY > 0 ? angleY: -angleY)*1000))%1000,3);

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

//        OLED_ShowHexNum(1, 1, bluetooth_Receive ta, 2);
    }
}

// 1ms运行一次
void TIM1_UP_IRQHandler(void){

    {// 超声波时间
        HC_RS04_Time_Big++;
    }
    {// 电机测速
        leftEncoderSpeed = EncoderSpeed_GetLeft() * 1000 / (4 * 34);
//        rightEncoderSpeed = EncoderSpeed_GetRight() * 1000 / (4 * 34);
        rightEncoderSpeed = leftEncoderSpeed;
    }

    MPU6050_ReadDataAndFilter();

    // 状态机
    switch (mainState_Now) {
        case 0:{// 正常运行状态
            if(mainState_Last != 0){// 如果是由其他状态转换而来的第一次需要清除积分累计
                PID_ClearI();
                mainState_Last = 0;
                break;
            }
            mainState_Last = 0;

            // 平滑加速
            Speed_LPF = Speed_LPF * 0.8f + targetSpeed * 0.2f;// 一阶低通滤波，为了平滑加速
            // PID
            {

            }
            // 串联
            leftSpeed = verticalLoop*0.5 +speedLoop*0.5;
            rightSpeed =verticalLoop*0.5 +speedLoop*0.5;
            // 速度设置
            Motor_SetLeftSpeed(leftSpeed);
            Motor_SetRightSpeed(rightSpeed);

            MainStateTransition_DipToBig();

            break;
        }
        case 1:{// 悬空状态(小车被拿起)
            mainState_Last = 1;

            break;
        }
        case 2:{// 无法自主调节,关闭电机
            mainState_Last = 1;

            break;
        }
        default:{// 不存在的情况

        }
    }

    // 每0.1s进行一次测距
    TimerCount ++;
    if(TimerCount >= 100){
        TimerCount = 0;
        HC_RS04_Open();
    }

    // 测量运行一次定时函数所消耗的时间
    time = TIM_GetCounter(TIM1);

    TIM_ClearITPendingBit(Timer_TIMX, TIM_IT_Update);
}

void MPU6050_ReadDataAndFilter(void){
    if (MPU6050_GetState() != 0) {
        MPU6050_WaitState();
    } else {
        MPU6050_GetData(&Data);
    }

    if (Data.GyroZ <= 2 && Data.GyroZ >= -2) Data.GyroZ = 0;
    // 偏航角
    // 偏航角速度
    float addAngleZ = ((float) Data.GyroZ) * MPU6050_AngleRange / 32768;
    angleZ += Kalman_Filter_dt * addAngleZ;
    // 俯仰角
    // 俯仰角速度
    float addAngleY = ((float) Data.GyroY) * MPU6050_AngleRange / 32768;
    // 利用加速度计获取俯仰角
    angleY = -atanf((float) Data.AccX / (float) Data.AccZ) * 57.2974f;
    // 卡尔曼滤波数据融合
    angleY = Kalman_Filter_Y(angleY, addAngleY);
}

/**
 * @brief 状态转换机1 倾角过大 0_To_2
 */
void MainStateTransition_DipToBig(void){
    static int16_t mainState_0_To_2_time;
    if (angleY > 50 || angleY < -50) {// 倾角过大计时
        mainState_0_To_2_time++;
    } else {// 逐步清零
        if (mainState_0_To_2_time >= 5) mainState_0_To_2_time -= 5;
        else if (mainState_0_To_2_time > 0) mainState_0_To_2_time = 0;
    }
    // 状态转换
    if (mainState_0_To_2_time >= 500) {// 持续0.5s倾角过大,判定为无法自主恢复

    }
}

/**
 * @brief 状态转换机2 倾角正常 2_To_0
 */
void MainStateTransition_DipNormal(void){

}

/**
 * @brief 状态转换机3 被拿起 0_To_1 2_To_1
 */
void MainStateTransition_TakeUp(void){

}

/**
 * @brief 状态转换机4 放下 1_To_2
 */
void MainStateTransition_LayDown(void){

}

/**
 * @brief 状态转换机5 正常状态内蓝牙转换
 */
void MainStateTransition_Bluetooth(void){

}

/**
 * @brief 直立环
 */
void Loop_Vertical(void){
    // 直立环
    verticalLoop = PID_Balance(angleY, ((float) Data.GyroY) * MPU6050_AngleRange / 32768);
    // 速度环
    speedLoop = PID_Velocity(leftEncoderSpeed, rightEncoderSpeed, Speed_LPF);
    // 转向环
//       turnLoop = ;
}

/**
 * @brief 速度环
 */
void Loop_Speed(void){

}

/**
 * @brief 转向环
 */
void Loop_Turn(void){

}

/**
 * @brief 距离环
 */
void Loop_Distance(void){

}
