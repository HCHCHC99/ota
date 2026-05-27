#ifndef HALL_POSITION_H
#define HALL_POSITION_H

#include "sys_def.h"

// 上限位霍尔传感器
#define HALL_UPPER_PORT     PortB
#define HALL_UPPER_PIN      Pin02
#define HALL_HIGH_LEVEL 1  
#define HALL_LOW_LEVEL  0  

// 下限位霍尔传感器
#define HALL_LOWER_PORT     PortB
#define HALL_LOWER_PIN      Pin10


// 接口声明
void HallPosition_Init(void);
void HallPosition_Reset(void);
SystemError HallPosition_Sample(void);  // 双霍尔独立采样+窗口滤波
RodPosition HallPosition_GetState(bool *valid);

#endif // HALL_POSITION_H
