#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "Delay.h"

#define Key_RCC_APB2Periph_GPIO RCC_APB2Periph_GPIOA
#define Key_GPIO GPIOA
#define Key_P_1 GPIO_Pin_4

void Key_Init(){
    RCC_APB2PeriphClockCmd(Key_RCC_APB2Periph_GPIO, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = Key_P_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;// 这里的参数只对输出模式有效
    GPIO_Init(Key_GPIO, &GPIO_InitStructure);// 配置过后默认是低电平
    GPIO_SetBits(GPIOA, Key_P_1);
}

/**
 *@brief 返回按键的键码
 * @return 返回键码1~2
 */
uint8_t Key_GetNum(){
    uint8_t KeyNum = 0;

    if (!GPIO_ReadInputDataBit(Key_GPIO, Key_P_1)) {
        Delay_ms(20);
        while(!GPIO_ReadInputDataBit(Key_GPIO, Key_P_1));
        Delay_ms(20);
        KeyNum = 1;
    }

    return KeyNum;
}
