#include "stm32f10x.h"
#include "stm32f10x_conf.h"

#define TimerCount_RCC_APBXPeriph_TIMX RCC_APB1Periph_TIM4
#define TimerCount_TIMX TIM4

void TimerCount_Init(void){
    // 1 RCC开启时钟
    RCC_APB1PeriphClockCmd(TimerCount_RCC_APBXPeriph_TIMX, ENABLE);
    // 2 选择时基单元的时钟源
    TIM_InternalClockConfig(TimerCount_TIMX);// 默认使用内部时钟,没有这行也可以
    // 3 配置时基单元
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;// 滤波器的内部时钟源分频倍数
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;// 向上计数
    TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;// 自动重装
    TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;// 预分频 72MHz/7200 = 1000kHz
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;// 重复计数器 高级计时器才有的
    TIM_TimeBaseInit(TimerCount_TIMX, &TIM_TimeBaseInitStructure);
    // 在上面这个函数,为了立刻让设置生效,跳过缓冲寄存器直接手动生成了一个更新事件,而副作用是生成更新事件的同时会产生更新中断
    TIM_Cmd(TimerCount_TIMX, ENABLE);
}

uint16_t TimerCount_GetCount(void){
    uint16_t Temp = TIM_GetCounter(TimerCount_TIMX);
    TIM_SetCounter(TimerCount_TIMX, 0);
    return Temp;
}
