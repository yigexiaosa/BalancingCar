#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__

void Bluetooth_Init(void);
void Bluetooth_SendByte(uint8_t Byte);
void Bluetooth_SendArray(uint8_t *Array,uint16_t Length);
void Bluetooth_SendString(char *String);
void Bluetooth_SendNumber(uint32_t Number,uint8_t Length);
void Bluetooth_Printf(char *format, ...);
uint8_t Bluetooth_GetRxFlag(void);
uint8_t Bluetooth_GetRxData(void);


#endif
