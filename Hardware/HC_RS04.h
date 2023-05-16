#ifndef __HC_RS04_H__
#define __HC_RS04_H__

extern uint16_t HC_RS04_Distance;// 单位：mm,30~600
extern uint16_t HC_RS04_Time_Big;
extern int16_t HC_RS04_Time_Small_Old,HC_RS04_Time_Small_New;
extern uint16_t HC_RS04_Time_Big_New;

void HC_RS04_Init(void);
void HC_RS04_Open(void);
uint8_t HC_RS04_GetState(void);


#endif
