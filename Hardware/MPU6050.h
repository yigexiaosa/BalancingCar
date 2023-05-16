#ifndef __MPU_6050_H__
#define __MPU_6050_H__

// 满量程
// +-2000°
#define MPU6050_AngleRange 2000.0f
// +-16G,+=16000N
#define MPU6050_AccelerationRange 16000.0f

typedef struct {
    // 加速度
    int16_t AccX;
    int16_t AccY;
    int16_t AccZ;
    // 陀螺仪
    int16_t GyroX;
    int16_t GyroY;
    int16_t GyroZ;
} MPU6050_GetDataTypeDef;

uint8_t MPU6050_GetState(void);
void MPU6050_WaitState(void);
void MPU6050_WriteReg(uint8_t RegAddress,uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);
void MPU6050_Init(void);
uint8_t MPU6050_GetId(void);
void MPU6050_GetData(MPU6050_GetDataTypeDef* Data);

void MPU6050_Test(void);

#endif
