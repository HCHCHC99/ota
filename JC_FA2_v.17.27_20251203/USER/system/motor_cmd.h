#ifndef MOTOR_CMD_H
#define MOTOR_CMD_H

#include "sys_def.h"
#include "hz_timer.h"

// 硬件IO定义（电机模块专用）
#define MOTOR_PWM_PORT   GPIO_PORT_C
#define MOTOR_PWM_PIN    GPIO_PIN_0  
#define MOTOR_DIR_PORT   GPIO_PORT_C
#define MOTOR_DIR_PIN    GPIO_PIN_1    

#define MOTOR_SLOWSTART_TIME        0
#define MOTOR_POSITIVE_TIME         0
#define MOTOR_NEGATIVE_TIME         0

#define MOTOR_MAX_DUTY      100.0f   // 运行最大占空比

extern NonBlockingDelay_t Motor_slowstart_Delay;
// 接口声明
void MotorCmd_Init(void);

SystemError MotorCmd_Execute(PowerPolarity power_input, RodPosition pos_input);  // 执行电机控制（含缓动）
MotorCmdState MotorCmd_GetState(bool *executed);
// void MotorCmd_Positive_Init(void);
// void MotorCmd_Negative_Init(void);

// void MotorCmd_StopImmediately(void);  // 立即停止电机
// void MotorCmd_Stop(void);
// void MotorCmd_Positive_SlowStart(void);
// void MotorCmd_Negative_SlowStart(void);


#endif // MOTOR_CMD_H
