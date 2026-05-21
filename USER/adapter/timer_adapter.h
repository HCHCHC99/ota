/********************************文件说明*************************************
*文件名: timer_adapter.h

*作者: Yuchen Tan

*版本: V1.0.2

*功能简介:

*备注: 无

*修改履历:

*****************************************************************************/
#ifndef TIMER_ADAPTER_H_
#define TIMER_ADAPTER_H_

/*****************************文件包含(公开)**********************************
*
*备注: 可在此包含底层的库文件
*
*****************************************************************************/
#include "common.h"
/******************************宏定义(公开)***********************************
*
*备注: 需要被外部使用或可任意修改的宏在这里定义
*
*****************************************************************************/
/*定时器索引定义*/
#define TIMER_0             (0)
#define TIMER_1             (1)
#define TIMER_2             (2)
#define TIMER_3             (3)

/*外设定时器中断使能*/
#define TIMER_IT_UPDATE     (0x01)  //更新中断


/*软件定时器索引定义*/
#define SW_TIMER_0          (0)
#define SW_TIMER_1          (1)
#define SW_TIMER_2          (2)
#define SW_TIMER_3          (3)
#define SW_TIMER_NB         (4)         //定时器数量
/*软件定时器事件定义*/
#define SW_TIMER_EVT_ALARM  (0X01)
/**************************数据类型及结构定义(公开)***************************
*
*备注: 需要被外部使用的数据结构及类型在这里定义
*
*****************************************************************************/
/*与MCU平台相关的抽象类型定义*/
#if (MCU_TYPE == MCU_TYPE_STM32)
typedef TIM_HandleTypeDef*  TIMER_INSTANCE_t;
typedef uint32_t            TIMER_CHANNEL_t;

#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
typedef void*               TIMER_INSTANCE_t;   //tim3没有类型定义,用void*替代
typedef en_tim3_m23_ccrx_t  TIMER_CHANNEL_t;

#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
typedef void*				TIMER_INSTANCE_t;	//void*类型可兼容不同定时器(M4_TMRA_TypeDef* || M4_TMR4_TypeDef*)
typedef uint8_t				TIMER_CHANNEL_t;	//uint8_t类型可兼容不同定时器的通道类型

#endif

/*PWM通道定义*/
typedef struct
{
    TIMER_INSTANCE_t        TimerIns;       //定时器实例
    TIMER_CHANNEL_t         Channal;        //定时器通道
}TIMER_CHANNAL_t;

/*软件定时器事件回调函数*/
typedef void (*TimerEvtCb_t)(void*);

/*MCU外设定时器类型定义*/
typedef struct
{
    uint8_t         State;
}TIMER_t;

/*软件定时器类型定义*/
typedef struct
{
    uint8_t         TimerEn;
    uint16_t        TimerCnt;
    uint16_t        AlarmTime;      //TimerCnt >= AlarmTime会触发TIMER_EVT_ALARM
    uint8_t         TimerEvent;
    TimerEvtCb_t    AlarmCallback;
}SW_TIMER_t;
/*****************************函数声明(公开)**********************************
*
*备注: 需要被外部使用的接口函数在这里声明
*
*****************************************************************************/
/*MCU外设定时器-基本功能*/
void timer_IT_Enable(uint8_t Timer, uint8_t ITEnable);
uint32_t timer_Get_Counter(uint8_t Timer);
void timer_Reset(uint8_t Timer);
void timer_Run(uint8_t Timer);
void timer_Stop(uint8_t Timer);
/*MCU外设定时器-PWM通道功能*/
void timer_Ch_PWM_OutPut(TIMER_CHANNAL_t *PWMCh, int32_t Value);
void timer_Ch_PWM_OutPut_Disable(TIMER_CHANNAL_t *PWMCh);
/*软件定时器*/
void timer_Start_SW_Timer(uint8_t Nb);
void timer_Stop_SW_Timer(uint8_t Nb);
void timer_Reset_SW_Timer(uint8_t Nb);
void timer_Abort_SW_Timer(uint8_t Nb);
void timer_ResetStart_SW_Timer(uint8_t Nb);
void timer_Set_SW_Timer_AlarmTime(uint8_t Nb, uint16_t AlarmTime_ms);
uint8_t timer_Get_SW_Timer_Event(uint8_t Nb, uint16_t Event);
void timer_Clr_SW_Timer_Event(uint8_t Nb, uint16_t Event);
void timer_SW_Timer_Run(void);
/*****************************变量声明(公开)**********************************
*
*备注: 不建议用extern声明本文件的变量直接给外部使用(解耦).
*公开本文件变量建议方式: 开放返回变量值的接口.
*
*****************************************************************************/

/*****************************变量引用(全局)**********************************
*
*备注: 不建议用extern引用其他文件的变量(解耦).
*引用其他文件变量建议方式: 包含其他文件.h并调用相应接口or传参方式获取其他文件的变量
*
*****************************************************************************/

/*****************************函数引用(全局)**********************************
*
*备注: 不建议用extern引用其他文件的函数(解耦).
*引用其他文件函数的方式: 可包含其他文件.h并调用相应接口
*
*****************************************************************************/


#endif
