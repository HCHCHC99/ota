#ifndef HARDWARE_H
#define HARDWARE_H

#include "hc32f46x.h"

#define PLV_PORT        PortB
#define PLV_PIN         Pin06
#define PLV_CHANNEL     TimeraCh1

#define PHV_PORT        PortB
#define PHV_PIN         Pin07
#define PHV_CHANNEL     TimeraCh2

#define PLU_PORT        PortB
#define PLU_PIN         Pin08
#define PLU_CHANNEL     TimeraCh3

#define PHU_PORT        PortB
#define PHU_PIN         Pin09
#define PHU_CHANNEL     TimeraCh4




void Hardware_Init(void);
void Motor_GPIO_Init(void);
void hz_gpio_PULLUP(void);
void Power_GPIO_Init(void);
void Hall_GPIO_Init(void);

// void Motor_PHU_IO_Init(void);
// void Motor_PHV_IO_Init(void);
void PWM_GPIO_Init(void);
void Test_An_In_Init(void);
// void PushPullOutput_Init(void);
#endif // HARDWARE_H
