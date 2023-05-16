#include "stm32f10x.h"
#include "stm32f10x_conf.h"

void EncoderSpeed_Init(void) {
    // TIM3 左电机
    // 1 RCC开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    // 2 GPIO初始化,配置为输入模式
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // 3 时基计数单元配置
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;// 滤波器的内部时钟源分频倍数
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;// 向上计数 目前没有作用
    TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;// 最大计数，也方便计算
    TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;// 不分频
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;// 重复计数器 高级计时器才有的
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;// 输入通道选择 1
    TIM_ICInitStructure.TIM_ICFilter =  0xF;// 滤波器
    // (在这里指的是不反相，下降沿触发就是反相，编码器接口始终都是上升沿，下降沿都有效),在下面编码器也有配置
//    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;// 输入通道选择 2
    TIM_ICInitStructure.TIM_ICFilter =  0xF;// 滤波器
    // (在这里指的是不反相，下降沿触发就是反相，编码器接口始终都是上升沿，下降沿都有效),在下面编码器也有配置
//    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);

    // 4 配置编码器接口
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    // 5 开启定时器
//    TIM_Cmd(TIM3, ENABLE);

    // TIM4 右电机
    // 1 RCC开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    // 2 GPIO初始化,配置为输入模式
//    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    // 3 时基计数单元配置
//    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;// 滤波器的内部时钟源分频倍数
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;// 向上计数 目前没有作用
    TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;// 最大计数，也方便计算
    TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;// 不分频
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;// 重复计数器 高级计时器才有的
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

//    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;// 输入通道选择 1
    TIM_ICInitStructure.TIM_ICFilter =  0xF;// 滤波器
    // (在这里指的是不反相，下降沿触发就是反相，编码器接口始终都是上升沿，下降沿都有效),在下面编码器也有配置
//    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;// 输入通道选择 2
    TIM_ICInitStructure.TIM_ICFilter =  0xF;// 滤波器
    // (在这里指的是不反相，下降沿触发就是反相，编码器接口始终都是上升沿，下降沿都有效),在下面编码器也有配置
//    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);

    // 4 配置编码器接口
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    // 5 开启定时器
    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

/**
 * @brief 获取左电机编码数
 * @return
 */
int16_t EncoderSpeed_GetLeft(void){
//    return TIM_GetCounter(TIM3);

    int16_t Temp;
    Temp = TIM_GetCounter(TIM3);
    TIM_SetCounter(TIM3, 0);
    return Temp;

}

/**
 * @brief 获取右电机编码数
 * @return
 */
int16_t EncoderSpeed_GetRight(void){
//    return TIM_GetCounter(TIM4);

    int16_t Temp;
    Temp = TIM_GetCounter(TIM4);
    TIM_SetCounter(TIM4, 0);
    // 右电机的极性相反
    return -Temp;

}
