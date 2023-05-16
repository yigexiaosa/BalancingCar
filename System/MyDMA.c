#include "stm32f10x.h"
#include "stm32f10x_conf.h"

uint16_t MySMA_CountSize;

void MyDMA_Init(uint32_t AddrA,uint32_t AddrB,uint16_t CountSize){
    MySMA_CountSize = CountSize;
    // RCC开启DMA时钟
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    // 初始化DMA
    DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_PeripheralBaseAddr = AddrA;// 外设地址
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;// 数据宽度
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;// 是否自增
    DMA_InitStructure.DMA_MemoryBaseAddr = AddrB;// 存贮器地址
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_BufferSize = CountSize;// 缓存区大小(就是传输计数器)
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;// 传输方向 外设站点作为数据源
    DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;// 软件触发/硬件触发
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 是否使用自动重装
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;// 优先级(一个DMA内会有很多通道,同时工作时会有一个先后优先顺序)
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    // 这里不需要开启DMA,需要数据转运的时候再使能
}

void MyDMA_Transfer(void){
    DMA_Cmd(DMA1_Channel1, DISABLE);
    DMA_SetCurrDataCounter(DMA1_Channel1, MySMA_CountSize);
    DMA_Cmd(DMA1_Channel1, ENABLE);
    while(DMA_GetFlagStatus(DMA1_FLAG_TC1) == RESET);// 其实也没必要等待,因为转运的速度非常非常快
    DMA_ClearFlag(DMA1_FLAG_TC1);
}
