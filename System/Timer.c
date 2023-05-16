#include "stm32f10x.h"
#include "stm32f10x_conf.h"

#define Timer_RCC_APBXPeriph_TIMX RCC_APB2Periph_TIM1
#define Timer_TIMX TIM1

void Timer_Init(void){
    // 1 RCC开启时钟
    RCC_APB2PeriphClockCmd(Timer_RCC_APBXPeriph_TIMX, ENABLE);
    // 2 选择时基单元的时钟源
    TIM_InternalClockConfig(Timer_TIMX);// 默认使用内部时钟,没有这行也可以
    // 3 配置时基单元
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;// 滤波器的内部时钟源分频倍数
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;// 向上计数
    TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;// 自动重装 72MHz/72/1000 = 1000kHz
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;// 预分频
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;// 重复计数器 高级计时器才有的
    TIM_TimeBaseInit(Timer_TIMX, &TIM_TimeBaseInitStructure);
    // 在上面这个函数,为了立刻让设置生效,跳过缓冲寄存器直接手动生成了一个更新事件,而副作用是生成更新事件的同时会产生更新中断
    // 如果不想在初始化的时候就执行一次中断函数,我们需要在上面这个函数到开启中断之前将中断标记位清零
    TIM_ClearFlag(Timer_TIMX, TIM_FLAG_Update);
    // 4 配置输出终端控制,允许更新中断输出到NVIC
    TIM_ITConfig(Timer_TIMX, TIM_IT_Update, ENABLE);// 更新中断 计数器的值等于自动重装寄存器的值时触发中断
    // 5 配置NVIC,在NVIC打开定时器中断的一个通道,选择优先级
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
    // 6运行控制,使能计数器
    TIM_Cmd(Timer_TIMX, ENABLE);
}

/*
void TIM1_UP_IRQHandler(void){
    if(TIM_GetITStatus(Timer_TIMX,TIM_IT_Update) == SET){

        TIM_ClearITPendingBit(Timer_TIMX, TIM_IT_Update);
    }
}
 */
