/********************************文件说明*************************************
*文件名: mc_spd.h

*作者: Yuchen Tan

*版本: V1.0.3

*功能简介:

*备注: 无

*修改履历:

*****************************************************************************/
#ifndef MC_SPD_H_
#define MC_SPD_H_

/*****************************文件包含(公开)**********************************
*
*备注: 可在此包含底层的库文件
*
*****************************************************************************/
#include "common.h"
#include "mc_common.h"
#include "pid.h"
/******************************宏定义(公开)***********************************
*
*备注: 需要被外部使用或可任意修改的宏在这里定义
*
*****************************************************************************/
/*速度环PID模式选择*/
#define SPD_LOOP_PID_MODE       (1)     /*0-速度环使用增量式PID  1-速度环使用位置式PID*/
/*速度环周期*/
#define SPD_LOOP_PERIOD         (5)         //单位：ms
/*速度环增量式PID增量分比*/
#define SPD_LOOP_PID_ZL_DIV     (8)//7//(10)        //增量的右移位数
/*速度环位置式PID输出分比*/
#define SPD_LOOP_PID_WZ_DIV     7//(6)//7//(10)     //输出的右移位数(DIV↓: 稳态波动↑,稳态误差↓)
/**************************数据类型及结构定义(公开)***************************
*
*备注: 需要被外部使用的数据结构及类型在这里定义
*
*****************************************************************************/
/*马达速度斜坡控制器步骤定义*/
typedef enum
{
    E_SPEED_CHANGE_DONE = 0,    /*速度控制完成*/
    E_CTRL_INIT,                /*控制初始化*/
    E_CHANGING_SPEED,           /*速度变化中*/
    E_CTRL_PARAM_CHANGE,        /*控制参数变化*/
}RAMP_CTRL_STATE_t;

/*马达速度斜坡控制器类型定义*/
typedef struct
{
    /*速度斜坡控制器相关控制*/
    BOOL                RampCtrlEn;         /*斜坡控制器使能*/
    RAMP_CTRL_STATE_t   RampCtrlState;      /*斜坡控制器状态*/
    uint16_t            RampCtrlTimer;      /*计数器*/
    uint8_t             RampCtrlTimerEn;    /*计数器开关*/
    /*速度斜坡控制器参数*/
    MOTOR_SPD_t         InitialSpeed;       /*斜坡起始速度(±)(单位:RPM)*/
    MOTOR_SPD_t         TargetSpeed;        /*斜坡最终目标速度(±)(单位:RPM)*/
    MOTOR_SPD_t         ProcessSpeed;       /*斜坡加减速过程速度(±)(单位:RPM)*/
    MOTOR_SPD_t         Acceleration;       /*斜坡加速度(±)(单位:RPM/s)*/
    uint16_t            AccelerationTime;   /*斜坡加速时间(给定InitialSpeed,TargetSpeed,Acceleration后,用于速度控制中间计算)(单位:ms)*/
}SPD_RAMP_Handle_t;

/*电机速度闭环控制结构定义*/
typedef struct
{
    BOOL                SpdCLCtrlEn;
    uint16_t            Period;
    uint16_t            PeriodTimer;

    MOTOR_SPD_t         FdbkSpeed;		/*速度环反馈速度(±)(单位:RPM)*/
    MOTOR_SPD_t         TargetSpeed;	/*速度环目标速度(±)(单位:RPM)*/

    int16_t             Output;         /*速度环输出(±)*/

    MOTOR_SPD_t         MinSpeed;       /*速度环最小输出速率(+)(单位:RPM)*/
    MOTOR_SPD_t         MaxSpeed;       /*速度环最大输出速率(+)(单位:RPM)*/

    PID_ZL_Handle_t     *pSpdLoopPIDZL; /*速度环PID控制器句柄*/
    PID_WZ_Handle_t     *pSpdLoopPIDWZ; /*速度环PID控制器句柄*/

    SPD_RAMP_Handle_t   *pSpdRamp;      /*速度斜坡控制器句柄*/
}SPD_CL_Handle_t;

/*电机占空比开环控制结构定义*/
typedef struct
{
    /*开环斜坡控制*/
    BOOL            SpdOLCtrlEn;
    BOOL            DCRampOverFlag;
    int16_t         Output;				/*开环输出(±)*/
    /*开环斜坡参数*/
    int16_t         DCModValue;         /*100%占空比对应的值*/
    int16_t         MaxDutyCycle;       /*开环最大输出占空比(+)(单位:百分比)*/
    int16_t         MinDutyCycle;       /*开环最小输出占空比(+)(单位:百分比)*/
    int16_t         InitDCPercent;      /*斜坡起始占空比(±)(单位:百分比)*/
    int16_t         ProcessDCPercent;	/*斜坡过程占空比(±)(单位:百分比)*/
    int16_t         TargetDCPercent;    /*斜坡目标占空比(±)(单位:百分比)*/
    int16_t         Acceleration;       /*斜坡占空比加速度(+)(单位:百分比/s)(范围:1-1000)*/
    uint16_t        AccelerationTimer;
}SPD_OL_Handle_t;
/*****************************函数声明(公开)**********************************
*
*备注: 需要被外部使用的接口函数在这里声明
*
*****************************************************************************/
/*模块功能测试*/
void mc_spd_Test(SPD_CL_Handle_t *pSpdCL, SPD_RAMP_Handle_t* pHandle);
/*电机速度斜坡控制器(用于速度闭环控制)*/
void mc_spd_Ramp_Controller_hInit(SPD_RAMP_Handle_t* pHandle);
void mc_spd_Ramp_Set_Acceleration(SPD_RAMP_Handle_t* pHandle, MOTOR_SPD_t Acceleration);
void mc_spd_Ramp_Set_TargetSpeed(SPD_RAMP_Handle_t* pHandle, MOTOR_SPD_t TargetSpeed);
void mc_spd_Ramp_Set_InitialSpeed(SPD_RAMP_Handle_t* pHandle, MOTOR_SPD_t InitialSpeed);
MOTOR_SPD_t mc_spd_Ramp_Get_ProcessSpeed(const SPD_RAMP_Handle_t* pHandle);
void mc_spd_Reset_Ramp_Controller(SPD_RAMP_Handle_t* pHandle);
void mc_spd_Ramp_Controller_Enable(SPD_RAMP_Handle_t* pHandle);
void mc_spd_Ramp_Controller_Disable(SPD_RAMP_Handle_t* pHandle);
BOOL mc_spd_Ramp_Is_Ramp_Over(SPD_RAMP_Handle_t* pHandle);
void mc_spd_Ramp_Controller(SPD_RAMP_Handle_t* pHandle);
void mc_spd_Ramp_Controller_Timer(SPD_RAMP_Handle_t* pHandle);
/*电机速度闭环控制器*/
void mc_spd_CloseLoop_hInit(SPD_CL_Handle_t *pSpdCL, PID_ZL_Handle_t *pPIDZL, PID_WZ_Handle_t *pPIDWZ, SPD_RAMP_Handle_t *pSpdRamp);
void mc_spd_Reset_CloseLoop_Controller(SPD_CL_Handle_t *pSpdCL);
void mc_spd_Set_CloseLoop_Period(SPD_CL_Handle_t *pSpdCL, uint16_t Period);
void mc_spd_CloseLoop_Controller_Enable(SPD_CL_Handle_t *pSpdCL);
void mc_spd_CloseLoop_Controller_Disable(SPD_CL_Handle_t *pSpdCL);
void mc_spd_CloseLoop_Controller_Timer(SPD_CL_Handle_t *pSpdCL);
void mc_spd_Set_CloseLoop_MaxSpeed(SPD_CL_Handle_t* pSpdCL, MOTOR_SPD_t MaxSpeed);
void mc_spd_Set_CloseLoop_MinSpeed(SPD_CL_Handle_t* pSpdCL, MOTOR_SPD_t MinSpeed);
void mc_spd_Set_CloseLoop_TargetSpeed(SPD_CL_Handle_t *pSpdCL, MOTOR_SPD_t TargetSpeed);
MOTOR_SPD_t mc_spd_Get_CloseLoop_TargetSpeed(const SPD_CL_Handle_t *pSpdCL);
MOTOR_SPD_t mc_spd_Get_CloseLoop_RealSpeed(const SPD_CL_Handle_t *pSpdCL);
void mc_spd_CloseLoop_Controller(SPD_CL_Handle_t *pSpdCL, MOTOR_SPD_t FdbkSpeed);
/*电机速度开环控制器*/
void mc_spd_OpenLoop_hInit(SPD_OL_Handle_t *pSpdOL);
void mc_spd_Reset_OpenLoop_Controller(SPD_OL_Handle_t *pSpdOL);
void mc_spd_OpenLoop_Controller_Enable(SPD_OL_Handle_t *pSpdOL);
void mc_spd_OpenLoop_Controller_Disable(SPD_OL_Handle_t *pSpdOL);
void mc_spd_OpenLoop_Controller_Timer(SPD_OL_Handle_t *pSpdOL);
void mc_spd_Set_OpenLoop_DutyCycleModValue(SPD_OL_Handle_t *pSpdOL, int16_t DCModValue);
void mc_spd_Set_OpenLoop_MaxDutyCycle(SPD_OL_Handle_t *pSpdOL, int16_t MaxDutyCycle);
void mc_spd_Set_OpenLoop_MinDutyCycle(SPD_OL_Handle_t *pSpdOL, int16_t MinDutyCycle);
void mc_spd_Set_OpenLoop_TargetDutyCycle(SPD_OL_Handle_t *pSpdOL, int16_t TargetDCPercent);
void mc_spd_Set_OpenLoop_InitDutyCycle(SPD_OL_Handle_t *pSpdOL, int16_t InitDCPercent);
void mc_spd_Set_OpenLoop_Acceleration(SPD_OL_Handle_t *pSpdOL, int16_t Acceleration);
BOOL mc_spd_Is_OpenLoop_Ramp_Over(SPD_OL_Handle_t* pSpdOL);
void mc_spd_OpenLoop_Controller(SPD_OL_Handle_t *pSpdOL);
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
