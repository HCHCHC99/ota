#ifndef DEV_TIMER0_H
#define DEV_TIMER0_H
#include "hc32f46x_timer0.h"

#define TIMER0_ENABLE                 timer0_Init(1000, s_tickCount_NoReset)
#define TIMER0_DISABLE                Timer0_Disable(M4_TMR01, Tim0_ChannelB)


typedef enum
{
    s_tickCount_Reset,
    s_tickCount_NoReset
}ReCount;



void timer0_Init(uint32_t us, ReCount cmd);
void Timer0_Disable(M4_TMR0_TypeDef* timer, en_tim0_channel_t channel);


#endif
