#include "stm32f10x.h"
#include "stm32f10x_conf.h"

void IC_Init(void){
    // 1 RCC开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    // 2 GPIO初始化,配置为输入模式
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // 3 时基计数单元配置
    TIM_InternalClockConfig(TIM3);// 默认使用内部时钟,没有这行也可以
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;// 滤波器的内部时钟源分频倍数
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;// 向上计数
    TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;// 自动重装 72MHz/7200/10000 = 1Hz ARR
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;// 预分频 PSC
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;// 重复计数器 高级计时器才有的
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
    // 4 输入捕获通道配置
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;// 输入通道选择 1
    TIM_ICInitStructure.TIM_ICFilter =  0xF;// 滤波器
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;// 上升沿触发
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;// 不分频
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;// 直连通道
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
//    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;// 输入通道选择 2
//    TIM_ICInitStructure.TIM_ICFilter =  0xF;// 滤波器
//    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;// 下降沿触发
//    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;// 不分频
//    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_IndirectTI;// 交叉通道
//    TIM_ICInit(TIM3, &TIM_ICInitStructure);
    TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);// ST公司给你的封装函数
    // 5 选择从模式触发源,TIFP1
    TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1);
    // 6 选择触发之后执行的操作,Reset
    TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);// 清空CNT
    // 7 开启计时
    TIM_Cmd(TIM3, ENABLE);
}

/**
 * @brief PSC = 72, 探测频率为1Hz,返回PWM的频率(Hz)
 * @return
 */
uint32_t IC_GetFreq(void){
    return 1000000 / (TIM_GetCapture1(TIM3) + 1);
}

/**
 * @brief 返回PWM的占空比
 * @return
 */
uint32_t IC_GetDuty(void){
    return (TIM_GetCapture2(TIM3) + 1) * 100 / TIM_GetCapture1(TIM3);
}
