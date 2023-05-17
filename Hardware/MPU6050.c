#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "MPU6050_Reg.h"
#include "MPU6050.h"
#include "Delay.h"

#define MPU6050_AccXCorrect     0
#define MPU6050_AccYCorrect     0
#define MPU6050_AccZCorrect     0
#define MPU6050_GyroXCorrect    (-48)
#define MPU6050_GyroYCorrect    (+18)
#define MPU6050_GyroZCorrect    0
#define MPU6050_I2C_Speed       ((uint32_t)800*1e3)

#define MPU6050_ADDRESS 0xD0

#define MPU6050_SCL GPIO_Pin_10
#define MPU6050_SDA GPIO_Pin_11

uint16_t MPU6050_State;
uint16_t MPU6050_Count;

uint8_t MPU6050_GetState(void){
    return MPU6050_State;
}

void MPU6050_Test(void){
    MPU6050_State = 1;
    MPU6050_WaitState();
}

void MPU6050_WaitState(void){
    if(MPU6050_State == 0) return;
    MPU6050_State = 0;

    I2C_Cmd(I2C2, DISABLE);

    I2C_DeInit(I2C2);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = MPU6050_SCL | MPU6050_SDA;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);// 配置过后默认是低电平

    for(int i = 0;i<32;i++){
        GPIO_WriteBit(GPIOB, MPU6050_SCL, 0);
        Delay_us(1);
        GPIO_WriteBit(GPIOB, MPU6050_SDA, 0);
        Delay_us(1);
        GPIO_WriteBit(GPIOB, MPU6050_SCL, 1);
        Delay_us(1);
        GPIO_WriteBit(GPIOB, MPU6050_SDA, 1);
        Delay_us(1);
    }

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    I2C_InitTypeDef I2CInitStructure;
    I2CInitStructure.I2C_Mode = I2C_Mode_I2C;
    I2CInitStructure.I2C_ClockSpeed = MPU6050_I2C_Speed;
    // 由于SDA总线是弱上拉，上升沿时间较长，所以在高速模式下SCL需要给低电平多一点时间让SDA有足够的时间翻转电平
    I2CInitStructure.I2C_DutyCycle = I2C_DutyCycle_2;// 快速模式下时钟占空比 2:1
    I2CInitStructure.I2C_Ack = I2C_Ack_Enable;// 默认应答位
    I2CInitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;// STM32作为从机的地址模式（7/10）
    I2CInitStructure.I2C_OwnAddress1 = 0x00;// STM32自身地址
    I2C_Init(I2C2, &I2CInitStructure);

    I2C_Cmd(I2C2, ENABLE);
}

void MPU6050WaitCheckEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT){
    if(MPU6050_State != 0) return;
    uint32_t TimeOut = 200;
    // 超时退出，完整项目的话需要考虑对对应错误进行处理
    while(I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS){
        if(TimeOut-- <= 0){
            MPU6050_Count++;
            if(MPU6050_Count >= 10){
                MPU6050_Count = 0;
                MPU6050_State = 1;
            }
            break;
        }
    }
}

/**
 * @brief 指定地址写寄存器
 * @param RegAddress
 * @param Data
 */
void MPU6050_WriteReg(uint8_t RegAddress,uint8_t Data){
    if(MPU6050_State != 0) return;
    I2C_GenerateSTART(I2C2, ENABLE);
    MPU6050WaitCheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);// EV5

    I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);
    // 硬件会自动接收应答
    MPU6050WaitCheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);// EV6

    I2C_SendData(I2C2, RegAddress);
    MPU6050WaitCheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING);// EV8

    I2C_SendData(I2C2, Data);
    // 感觉这里可以直接Stop让硬件自己等待数据发送完后发Stop,再再函数开头加一个判断数据有没有发送完
    // 好像Start和Stop硬件都会等待上一个数据发送完成后再发来着
    MPU6050WaitCheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);// EV8_2

    I2C_GenerateSTOP(I2C2,ENABLE);
}

/**
 * @brief 指定地址读寄存器
 * @param RegAddress
 * @return
 */
uint8_t MPU6050_ReadReg(uint8_t RegAddress){
    if(MPU6050_State != 0) return 0;
    uint8_t Data;

    I2C_GenerateSTART(I2C2, ENABLE);
    MPU6050WaitCheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);// EV5

    I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);

    // 硬件会自动接收应答
    MPU6050WaitCheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);// EV6

    I2C_SendData(I2C2, RegAddress);
    // 等待这一位数据发送完了再Start，不过其实也不需要
    MPU6050WaitCheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);// EV8_2

    I2C_GenerateSTART(I2C2, ENABLE);
    MPU6050WaitCheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);// EV5

    I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Receiver);
    // 硬件会自动接收应答
    MPU6050WaitCheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);// EV6

    // 在接收最后一个数据之前，需要先设置Ack非应答和STOP请求
    I2C_AcknowledgeConfig(I2C2, DISABLE);
    I2C_GenerateSTOP(I2C2,ENABLE);

    MPU6050WaitCheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);// EV7
    Data = I2C_ReceiveData(I2C2);

    // 设置回Ack默认位
    I2C_AcknowledgeConfig(I2C2, ENABLE);

    return Data;
}

void MPU6050_Init(void){
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Pin = MPU6050_SCL | MPU6050_SDA;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);// 配置过后默认是低电平
    GPIO_SetBits(GPIOB, MPU6050_SCL | MPU6050_SDA);


    I2C_InitTypeDef I2CInitStructure;
    I2CInitStructure.I2C_Mode = I2C_Mode_I2C;
    I2CInitStructure.I2C_ClockSpeed = MPU6050_I2C_Speed;
    // 由于SDA总线是弱上拉，上升沿时间较长，所以在高速模式下SCL需要给低电平多一点时间让SDA有足够的时间翻转电平
    I2CInitStructure.I2C_DutyCycle = I2C_DutyCycle_2;// 快速模式下时钟占空比 2:1
    I2CInitStructure.I2C_Ack = I2C_Ack_Enable;// 默认应答位
    I2CInitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;// STM32作为从机的地址模式（7/10）
    I2CInitStructure.I2C_OwnAddress1 = 0x00;// STM32自身地址
    I2C_Init(I2C2, &I2CInitStructure);

    I2C_Cmd(I2C2, ENABLE);

    // 关闭休眠模式,采用陀螺仪时钟
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
    // 采样率分频,决定了数据输出的快慢  10分频
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);
    // 配置寄存器
    MPU6050_WriteReg(MPU6050_CONFIG, 0x06);
    // 陀螺仪设置寄存器
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
    // 加速度计寄存器
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);
}

uint8_t MPU6050_GetId(void){
    if(MPU6050_State != 0) return 0;
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

void MPU6050_GetData(MPU6050_GetDataTypeDef* Data){
    if(MPU6050_State != 0) return;
    //由于加速度计、陀螺仪的结果地址都是在连续的一片地址上，可以使用I2C连续读取多个数据，加快I2C通信效率
    uint8_t DataH, DataL;

    DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
    Data->AccX = ((DataH << 8) | DataL) + MPU6050_AccXCorrect;

    DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
    Data->AccY = ((DataH << 8) | DataL) + MPU6050_AccYCorrect;

    DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
    Data->AccZ = ((DataH << 8) | DataL) + MPU6050_AccZCorrect;

//    DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
//    DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
//    Data->GyroX = ((DataH << 8) | DataL) + MPU6050_GyroXCorrect;

    DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
    Data->GyroY = ((DataH << 8) | DataL) + MPU6050_GyroYCorrect;

    DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
    Data->GyroZ = ((DataH << 8) | DataL) + MPU6050_GyroZCorrect;
}
