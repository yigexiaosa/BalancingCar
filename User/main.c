#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "OLED.h"
#include "EncoderSpeed.h" //TIM3 TIM4
#include "Motor.h"// TIM2
#include "Timer.h"// TIM1
#include "MPU6050.h"
#include "KalmanFilter.h"
#include "Delay.h"
#include "math.h"
#include "PID.h"
#include "HC_RS04.h"// TIM1
#include "Bluetooth.h"
#include "Key.h"

int16_t targetSpeed,targetTurn,targetDistance,leftEncoderSpeed,rightEncoderSpeed;
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
float speedLoop,verticalLoop,turnLoop,distanceLoop;
// 平滑变速
float leftSpeed_LPF,rightSpeed_LPF,speed_LPF,turn;
// 电机测速时间计
int16_t TimeSpeedCount;

// key
int16_t key_Speed = 400;

MPU6050_GetDataTypeDef Data;

int32_t testTimeCount,testDataCount,testOldAccX,testNum;

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
    Key_Init();

    // 定时器最后初始化
    Timer_Init();

    while(1){
        if(Key_GetNum() == 1){
            targetSpeed = targetSpeed + key_Speed;
            if(targetSpeed >= 2400 || targetSpeed <= -2400){
                key_Speed = -key_Speed;
                targetSpeed = 0;
            }
        }

        // 偏航角
        OLED_ShowSignedNum(1, 1, (int16_t)angleZ,3);
        OLED_ShowString(1, 5, ".");
        OLED_ShowNum(1, 6, ((int16_t)((angleZ > 0 ? angleZ: -angleZ)*1000))%1000,3);

        // 俯仰角
        OLED_ShowSignedNum(2, 1, (int16_t)angleY,3);
        OLED_ShowString(2, 5, ".");
        OLED_ShowNum(2, 6, ((int16_t)((angleY > 0 ? angleY: -angleY)*1000))%1000,3);

//        // MPU6050更新频率
//        OLED_ShowNum(1, 10, testNum,5);

        // 运行一次函数所需时间
        OLED_ShowNum(3, 9, time/100, 2);

        // 速度
        OLED_ShowSignedNum(3, 1, leftSpeed, 5);
        OLED_ShowSignedNum(3, 11, leftEncoderSpeed, 4);
        OLED_ShowSignedNum(4, 1, rightSpeed, 5);
        OLED_ShowSignedNum(4, 11, rightEncoderSpeed, 4);

        // 距离
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

//        OLED_ShowNum(1, 1, time, 5);
//        OLED_ShowSignedNum(2, 1, Data.AccX, 5);
//        OLED_ShowSignedNum(3, 1, Data.AccY, 5);
//        OLED_ShowSignedNum(4, 1, Data.AccZ, 5);
//        OLED_ShowSignedNum(2, 8, Data.GyroX, 5);
//        OLED_ShowSignedNum(3, 8, Data.GyroY, 5);
//        OLED_ShowSignedNum(4, 8, Data.GyroZ, 5);
//        static int16_t max;
//        if(max <  Data.GyroX)max =  Data.GyroX;
//        if(max < Data.GyroY) max = Data.GyroY;
//        if(max < Data.GyroZ) max= Data.GyroZ;
//        OLED_ShowSignedNum(1, 8, max, 5);
    }
}

// 1ms运行一次
void TIM1_UP_IRQHandler(void){

    {// 超声波时间
        HC_RS04_Time_Big++;
    }

    static int16_t T_S_C_K = 10;
    TimeSpeedCount++;
    if(TimeSpeedCount >= T_S_C_K){// 电机测速 200Hz
        TimeSpeedCount = 0;
        leftEncoderSpeed = EncoderSpeed_GetLeft() * (1000 / T_S_C_K) / (4 * 34);
        rightEncoderSpeed = EncoderSpeed_GetRight() * (1000 / T_S_C_K) / (4 * 34);
    }

    MPU6050_ReadDataAndFilter();

    // 状态机
    switch (mainState_Now) {
        case 0:{// 正常运行状态
            if(mainState_Last != 0){// 如果是由其他状态转换而来的第一次需要清除积分累计
                PID_ClearI();
                mainState_Last = 0;
            }

            // 平滑加速
            speed_LPF = speed_LPF * 0.8f + targetSpeed * 0.2f;// 一阶低通滤波，为了平滑加速
            // 平滑转向

            // PID
            {
                Loop_Vertical();
                Loop_Speed();
//                Loop_Turn();
            }
            // 串联
            leftSpeed = verticalLoop + speedLoop + turnLoop;
            rightSpeed =verticalLoop + speedLoop - turnLoop;
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
            mainState_Last = 2;

            Motor_SetLeftSpeed(0);
            Motor_SetRightSpeed(0);
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

    // 每1s进行一次更新次数统计
    testTimeCount++;
    if(testOldAccX != Data.AccX){
        testDataCount++;
    }
    if(testTimeCount >= 1000){
        testNum = testDataCount;
        testTimeCount = 0;
        testDataCount = 0;
    }

    // 测量运行一次定时函数所消耗的时间
    time = TIM_GetCounter(TIM1);
    if(time/100 <= 4 || time/100 >= 8){// 如果超时就发送一次警告
        Bluetooth_SendByte(0xff);
    }

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
    float angleY_m = -atanf((float) Data.AccX / (float) Data.AccZ) * 57.2974f;
    // 卡尔曼滤波数据融合
//    angleY = Kalman_Filter_Y(angleY, addAngleY);
    // 一阶互补滤波数据融合
    static float k1 = 0.001f;
    angleY = k1 * angleY_m + (1-k1) * (angleY + addAngleY * Kalman_Filter_dt);
}

/**
 * @brief 状态转换机1 倾角过大 0_To_2
 */
void MainStateTransition_DipToBig(void){
    static int16_t mainState_0_To_2_time;
    if (angleY > 30 || angleY < -30) {// 倾角过大计时
        mainState_0_To_2_time++;
    } else {// 逐步清零
        if (mainState_0_To_2_time >= 5) mainState_0_To_2_time -= 5;
        else if (mainState_0_To_2_time > 0) mainState_0_To_2_time = 0;
    }
    // 状态转换
    if (mainState_0_To_2_time >= 200) {// 持续0.5s倾角过大,判定为无法自主恢复
        mainState_Now = 2;
        mainState_0_To_2_time = 0;
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
 * @brief 直立环 0.4
 */
void Loop_Vertical(void){
    verticalLoop = PID_Vertical(angleY, ((float) Data.GyroY) * MPU6050_AngleRange / 32768);
}

/**
 * @brief 速度环 0.2
 */
void Loop_Speed(void){
    speedLoop = PID_Speed(leftEncoderSpeed, rightEncoderSpeed, speed_LPF);
}

/**
 * @brief 转向环 0.2
 */
void Loop_Turn(void){
    turnLoop = PID_Turn(leftEncoderSpeed, rightEncoderSpeed, targetTurn,
                        ((float) Data.GyroZ) * MPU6050_AngleRange / 32768);
}

/**
 * @brief 距离环 0.2
 */
void Loop_Distance(void){
    distanceLoop = PID_Distance(leftEncoderSpeed, rightEncoderSpeed, targetDistance,HC_RS04_Distance);
}
