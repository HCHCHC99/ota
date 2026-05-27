#ifndef HZ_NOPWM_H
#define HZ_NOPWM_H

#include "sys_def.h"


#define NOPWM_POSITIVE_PORT PortB
#define NOPWM_POSITIVE_PIN  Pin09

#define NOPWM_NEGATIVE_PORT PortB
#define NOPWM_NEGATIVE_PIN  Pin07

void Motor_noPWM_Init(void);

void MotorNoPWM_Negative_Up_Run(void);

void MotorNoPWM_Positive_Up_Run(void);

void MotorNoPWM_Negative_Up_Stop(void);

void MotorNoPWM_Positive_Up_Stop(void);

#endif
