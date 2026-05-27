#ifndef SYS_MAIN_STATE_H
#define SYS_MAIN_STATE_H

#include "sys_def.h"
#define PRIMARY_PROTECT_TIME    0
#define SECONDARY_PROTECT_TIME  0
#define FIRST_POWER_ON_TIME     0
#define USART1_USB_SEND_TIME    2
// 全局系统上下文（外部声明，在.c中定义）
extern SystemContext g_sys_ctx;

// 接口声明
void SysMainState_Init(void);
void SysMainState_Process(void);  // 处理状态流转

#endif // SYS_MAIN_STATE_H
