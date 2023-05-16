#ifndef __TIMER_H__
#define __TIMER_H__

#define TimerExternalClockSource_TIMX TIM2

uint16_t TimerExternalClockSource_GetCounter(void);
void TimerExternalClockSource_Init(void);

#endif
