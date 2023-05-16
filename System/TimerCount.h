#ifndef __TIMER_COUNT_H__
#define __TIMER_COUNT_H__

#define TimerCount_TIMX TIM1

void TimerCount_Init(void);
uint16_t TimerCount_GetCount(void);

#endif
