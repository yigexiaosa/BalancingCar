#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "HC_RS04.h"

#define HC_RS04_RCC_APB2Periph_GPIOX    RCC_APB2Periph_GPIOA
#define HC_RS04_RCC_APB2Periph_AFIO     RCC_APB2Periph_AFIO
#define HC_RS04_GPIOX                   GPIOA
#define HC_RS04_GPIO_Pin_X              GPIO_Pin_11
#define HC_RS04_GPIO_PortSourceGPIOX    GPIO_PortSourceGPIOA
#define HC_RS04_GPIO_PinSourceX         GPIO_PinSource11
#define HC_RS04_EXTI_LineX              EXTI_Line11
// 启动信号引脚
#define HC_RS04_GPIO_Pin_Trig           GPIO_Pin_12

int16_t HC_RS04_Time_Small_Old,HC_RS04_Time_Small_New;
uint16_t HC_RS04_Distance;// 单位：mm,30~600
uint16_t HC_RS04_Time_Big;
uint16_t HC_RS04_Time_Big_New;
uint8_t HC_RS04_State;

void HC_RS04_Init(void){
    // 开启APB2时钟
    RCC_APB2PeriphClockCmd(HC_RS04_RCC_APB2Periph_GPIOX, ENABLE);
    // 2 配置GPIOx 在参考手册中找到外设的GPIO配置
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = HC_RS04_GPIO_Pin_X;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(HC_RS04_GPIOX, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = HC_RS04_GPIO_Pin_Trig;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(HC_RS04_GPIOX, &GPIO_InitStructure);
    // 默认为低电平
    GPIO_ResetBits(HC_RS04_GPIOX, HC_RS04_GPIO_Pin_Trig);
    // 3 配置AFIO
    RCC_APB2PeriphClockCmd(HC_RS04_RCC_APB2Periph_AFIO, ENABLE);// 开启AFIO时钟
    GPIO_EXTILineConfig(HC_RS04_GPIO_PortSourceGPIOX, HC_RS04_GPIO_PinSourceX);
    // 4 配置EXTI
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = HC_RS04_EXTI_LineX;// 选择的线路
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;// 开启中断
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;// 设置为中断模式(中断/事件模式)
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;// 上升和下降沿触发
    EXTI_Init(&EXTI_InitStructure);
    // 5 配置NVIC
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置分组方式,2位抢占,两位响应
    NVIC_InitTypeDef NIVC_InitStructure;
    NIVC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NIVC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NIVC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;// 所选通道的抢占优先级,要高于Timer定时中断
    NIVC_InitStructure.NVIC_IRQChannelSubPriority = 0;// 所选通道的响应优先级
    NVIC_Init(&NIVC_InitStructure);
}

/**
 * @brief 进行一次探测
 */
void HC_RS04_Open(void){
    if(HC_RS04_State == 1) return;
    GPIO_SetBits(HC_RS04_GPIOX, HC_RS04_GPIO_Pin_Trig);
    // 拉高10us
    int16_t i = 100;
    while(i--);
    GPIO_ResetBits(HC_RS04_GPIOX, HC_RS04_GPIO_Pin_Trig);
}

/**
 * @brief 获取状态
 * @return 0~1  0:空闲  1:忙
 */
uint8_t HC_RS04_GetState(void){
    return HC_RS04_State;
}

void EXTI15_10_IRQHandler(void){// 中断函数的名字不能写错,最好直接从启动文件复制过来
    if (EXTI_GetITStatus(HC_RS04_EXTI_LineX) == SET) {
        // 上升沿
        if(GPIO_ReadInputDataBit(HC_RS04_GPIOX,HC_RS04_GPIO_Pin_X) == SET){
            HC_RS04_Time_Small_Old = (int16_t)TIM_GetCounter(TIM1);
            HC_RS04_Time_Big = 0;
            HC_RS04_State = 1;
        }
        // 下降沿
        else{
            HC_RS04_Time_Small_New = (int16_t)TIM_GetCounter(TIM1);
            HC_RS04_Time_Big_New = HC_RS04_Time_Big;
            HC_RS04_Distance =
                    (170 * (HC_RS04_Time_Big_New + 1)) + (0.17f * (HC_RS04_Time_Small_New - HC_RS04_Time_Small_Old));
            HC_RS04_State = 0;
            HC_RS04_Time_Big = 0;
        }
        EXTI_ClearITPendingBit(HC_RS04_EXTI_LineX);
    }
}
