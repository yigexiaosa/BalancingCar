#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "math.h"

#define Motor_PWM_RCC_APBxPeriph_GPIOx RCC_APB2Periph_GPIOA
#define Motor_PWM_GPIO_Pin_Left GPIO_Pin_0
#define Motor_PWM_GPIO_Pin_Right GPIO_Pin_1
#define Motor_PWM_GPIOx GPIOA
#define Motor_PWM_TIMx TIM2

#define Motor_GPIOx GPIOB
#define Motor_RCC_APBxPeriph_GPIOx RCC_APB2Periph_GPIOB
#define Motor_GPIO_Pin_Left_1 GPIO_Pin_12
#define Motor_GPIO_Pin_Left_2 GPIO_Pin_13
#define Motor_GPIO_Pin_Right_1 GPIO_Pin_14
#define Motor_GPIO_Pin_Right_2 GPIO_Pin_15

#define Motor_TIM_OCLeftInit(a,b) TIM_OC1Init(a,b)
#define Motor_TIM_OCRightInit(a,b) TIM_OC2Init(a, b)
#define Motor_TIM_SetCompareLeft(a,b) TIM_SetCompare1(a, b)
#define Motor_TIM_SetCompareRight(a,b) TIM_SetCompare2(a, b)

#define Range 2000

void Motor_PWM_Init(void){
    // 1 RCC开启时钟,打开TIM外设和GPIO外设的时钟
    RCC_APB2PeriphClockCmd(Motor_PWM_RCC_APBxPeriph_GPIOx, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = Motor_PWM_GPIO_Pin_Left | Motor_PWM_GPIO_Pin_Right;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(Motor_PWM_GPIOx, &GPIO_InitStructure);
    GPIO_SetBits(Motor_PWM_GPIOx, Motor_PWM_GPIO_Pin_Left | Motor_PWM_GPIO_Pin_Right);
    // 2 配置时基单元
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_InternalClockConfig(Motor_PWM_TIMx);// 默认使用内部时钟,没有这行也可以
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;// 滤波器的内部时钟源分频倍数
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;// 向上计数
    TIM_TimeBaseInitStructure.TIM_Period = Range - 1;// 自动重装 72MHz/1/Range =  36KHz
    TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;// 预分频 PSC
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;// 重复计数器 高级计时器才有的
    TIM_TimeBaseInit(Motor_PWM_TIMx, &TIM_TimeBaseInitStructure);
    // 3 配置输出比较单元(CCR的值、输出比较模式、极性选择、输出使能)
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);// 给结构体赋初始值
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;// 输出比较模式设置为PWM1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;// 极性选择,不反转
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;// 使能
    TIM_OCInitStructure.TIM_Pulse = 0;// 设置CCR寄存器值
    Motor_TIM_OCLeftInit(Motor_PWM_TIMx, &TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;// 极性选择,反转
    Motor_TIM_OCRightInit(Motor_PWM_TIMx, &TIM_OCInitStructure);
    // 4 配置GPIO

    // 运行控制,启动计时器
    TIM_Cmd(Motor_PWM_TIMx, ENABLE);
}

void Motor_PWM_SetCompareLeft(uint16_t Compare){
    Motor_TIM_SetCompareLeft(Motor_PWM_TIMx, Compare);
}

void Motor_PWM_SetCompareRight(uint16_t Compare){
    Motor_TIM_SetCompareRight(Motor_PWM_TIMx, Compare);
}

void Motor_Init(void){
    Motor_PWM_Init();

    // 电机方向控制两个引脚初始化
    RCC_APB2PeriphClockCmd(Motor_RCC_APBxPeriph_GPIOx, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = Motor_GPIO_Pin_Left_1 | Motor_GPIO_Pin_Left_2 | Motor_GPIO_Pin_Right_1 | Motor_GPIO_Pin_Right_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(Motor_GPIOx, &GPIO_InitStructure);
    GPIO_SetBits(Motor_GPIOx, Motor_GPIO_Pin_Left_1 | Motor_GPIO_Pin_Left_2 | Motor_GPIO_Pin_Right_1 | Motor_GPIO_Pin_Right_2);
}

/**
 * @brief 设置左电机转速
 * @param speed 转速 -Range~Range
 */
void Motor_SetLeftSpeed(int16_t speed){
    speed = (speed > Range) ? Range : speed;
    speed = (speed < -Range) ? -Range : speed;
    if(speed >= 0){
        GPIO_SetBits(Motor_GPIOx, Motor_GPIO_Pin_Left_1);
        GPIO_ResetBits(Motor_GPIOx, Motor_GPIO_Pin_Left_2);
        Motor_PWM_SetCompareLeft(speed);
    }else{
        GPIO_ResetBits(Motor_GPIOx, Motor_GPIO_Pin_Left_1);
        GPIO_SetBits(Motor_GPIOx, Motor_GPIO_Pin_Left_2);
        Motor_PWM_SetCompareLeft(-speed);
    }
}

/**
 * @brief 设置右电机转速
 * @param speed 转速 -Range~Range
 */
void Motor_SetRightSpeed(int16_t speed){
    speed = (speed > Range) ? Range : speed;
    speed = (speed < -Range) ? -Range : speed;
    if(speed >= 0){
        GPIO_SetBits(Motor_GPIOx, Motor_GPIO_Pin_Right_2);
        GPIO_ResetBits(Motor_GPIOx, Motor_GPIO_Pin_Right_1);
        Motor_PWM_SetCompareRight(speed);
    }else{
        GPIO_ResetBits(Motor_GPIOx, Motor_GPIO_Pin_Right_2);
        GPIO_SetBits(Motor_GPIOx, Motor_GPIO_Pin_Right_1);
        Motor_PWM_SetCompareRight(-speed);
    }
}
