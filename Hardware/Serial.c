#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include <stdio.h>
#include <stdarg.h>

uint8_t Serial_RxData,Serial_RxFlag;

void Serial_Init(void){
    // 1 RCC开启时钟 USART，GPIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    // 2 GPIO初始化 TX配置为复用输出,RX配置为输入
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // 3 配置USART
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 9600;// 分频系数
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;// 不使用流控
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;// 全双工
    USART_InitStructure.USART_Parity = USART_Parity_No;// 没有校验
    USART_InitStructure.USART_StopBits = USART_StopBits_1;// 停止位
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;// 数据位字长
    USART_Init(USART1, &USART_InitStructure);

    // 开启中断
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);

    // 开启USART
    USART_Cmd(USART1, ENABLE);
}

/**
 * @brief 通过串口向电脑串口助手发送数据
 * @param Byte
 */
void Serial_SendByte(uint8_t Byte){
    // 感觉把判断移在前面会更好一点,丢进数据就干其他的事情,如果再要发送就先等待一下
    while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, Byte);
//    while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);// 硬件自动清零
}

void Serial_SendArray(uint8_t *Array,uint16_t Length){
    for (uint16_t i = 0; i < Length; i++) {
        Serial_SendByte(Array[i]);
    }
}

void Serial_SendString(char *String){
    while(*String != '\0'){
        Serial_SendByte(*String);
        String++;
    }
}

void Serial_SendNumber(uint32_t Number,uint8_t Length){
    uint32_t K = 1;
    while(--Length) K*=10;
    while(K){
        Serial_SendByte((Number / K) % 10 + '0');
        K/=10;
    }
}

// 封装函数,可以避免上一个方法由于直接重定向了printf导致printf只能向串口1发送数据
void Serial_Printf(char *format, ...){
    char String[100];
    va_list arg;
    va_start(arg,format);
    vsprintf(String, format, arg);
    va_end(arg);
    Serial_SendString(String);
}

uint8_t Serial_GetRxFlag(void){
    return Serial_RxFlag;
}

uint8_t Serial_GetRxData(void){
    Serial_RxFlag = 0;
    return Serial_RxData;
}

// 中断函数,用于自动接收串口数据
void USART1_IRQHandler(void){
    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET) {
        Serial_RxData = USART_ReceiveData(USART1);
        Serial_RxFlag = 1;
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}
