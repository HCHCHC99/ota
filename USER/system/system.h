/********************************文件说明*************************************
*文件名: system.h

*作者: Yuchen Tan

*版本: V1.0.0

*功能简介:

*备注: 无

*修改履历:

*****************************************************************************/
#ifndef SYSTEM_H_
#define SYSTEM_H_

/*****************************文件包含(公开)**********************************
*
*备注: 可在此包含底层的库文件
*
*****************************************************************************/
#include "common.h"
#include "sys_cfg.h"
/******************************宏定义(公开)***********************************
*
*备注: 需要被外部使用或可任意修改的宏在这里定义
*
*****************************************************************************/

/*系统电机数定义*/
#define SYS_MOTOR_NB        (MOTOR_NB)

/*CAN接口电机控制命令*/
typedef enum
{
	e_can_none = 0,
	e_can_up,
	e_can_dn,
	e_can_stop,
	e_can_goto,
	e_can_reset,
	e_can_clrfault,
}can_m_cmd_t;


/*推杆欠压过压恢复的迟滞电压(单位: V)*/
#define HYSTERESIS_VOLTAGE  (2.0f)      //保护恢复迟滞电压
#define OVV_COUNT_MAX		200
/*推杆缓停参数定义*/
#define SPEED_INDEX         (0.2f)     	//推速系数
#define MOTOR_SLOW_STOP     (25.0f)		//缓停系数

/*系统掉电阈值电压*/
#define	ODV_STOP_SYSTEM				8000

#define	POWER_DOWN_SAVE_FLASH		10000
void system_set_CanCmd(can_m_cmd_t CanCmd);

/*设置can控制命令*/
void system_set_CanCmd(can_m_cmd_t CanCmd);
/*读取已完成复位标志*/
uint8_t system_get_ZeroFound(void);
/*获取故障标志*/
uint16_t system_get_FaultFlag(void);
/*获取推杆当前位置*/
float system_get_columnPosMM(void);
/*获取限位标志*/
uint16_t system_get_LimitFlag(void);
//获取当前手刹是否拉紧
uint8_t system_get_LockedFlag(void);
/*获取刹车状态*/
uint8_t system_get_BrakeState(void);

/*系统错误定义*/
#define FAULT_M1_OVC        (1<<0)                      //M1过流
#define FAULT_M1_HAB        (1<<1)                      //M1霍尔异常
#define FAULT_M2_OVC        (1<<4)                      //M2过流
#define FAULT_M2_HAB        (1<<5)                      //M2霍尔异常
#define MOTOR_FAULT_NB      (4)                         //电机异常数

#define FAULT_UDV           (1<<8)                      //欠压
#define FAULT_OVV           (1<<9)                      //过压
#define FAULT_OVT           (1<<10)                     //过热
#define FAULT_POS           (1<<11)                     //位置错误
#define FAULT_M1_ALL        (0X000F)                    //M1故障集合
#define FAULT_M2_ALL        (0X00F0)                    //M2故障集合
#define FAULT_MOTOR_ALL     (FAULT_M1_ALL|FAULT_M2_ALL) //电机故障集合
#define FAULT_NOT_MOTOR_ALL (0xFF00)                    //非电机故障集合
#define FAULT_ALL           (0XFFFF)                    //所有故障集合
/**************************数据类型及结构定义(公开)***************************
*
*备注: 需要被外部使用的数据结构及类型在这里定义
*
*****************************************************************************/
/*系统工作配置参数*/
typedef struct
{
	/*推杆参数*/
    float                           	Sys_GearRatio;              //齿轮减速比  
    float                           	Sys_Lead;                   //丝杆导程
    float                           	Sys_Route;                  //行程
    float                           	Sys_SpeedMmps;              //推杆速度
	/*系统参数*/
    uint8_t                         	Sys_NodeSlaveAddr;          //通讯节点ID
    syscon_cfg_com_type_t        		Sys_CommunicationType;      //通讯类型
	
    uint16_t                       	 	Sys_OvcValue;               //过流保护阈值
    float                           	Sys_OverVoltage;            //过压保护阈值
    float                           	Sys_UnderVoltage;           //欠压保护阈值
    syscon_cfg_di_fun_t             	Sys_DIFunction;             //引脚输入功能
	syscon_cfg_do_fun_t             	Sys_DOFunction;             //引脚输出功能
	syscon_cfg_pin_polarity_t         	Sys_ActiveValue;            //引脚极性
	
	float                           	Sys_ResetRaise;             //复位抬高
    syscon_cfg_reset_run_mode_t         Sys_ResetRunMode;           //复位运行模式
	syscon_cfg_reset_direction_t        Sys_ResetDirection;         //复位方向
	syscon_cfg_reset_judgment_mode_t	Sys_ResetMode;				//复位判断方式
    syscon_cfg_motor_run_mode_t         Sys_MotorRunMode;           //电机运行模式
	
	syscon_cfg_top_detect_t				Sys_TopDetection;           //到顶检测
    syscon_cfg_btm_detect_t          	Sys_BtmDetection;           //到底检测
}SYSTEM_CONFIG_t;

/*****************************函数声明(公开)**********************************
*
*备注: 需要被外部使用的接口函数在这里声明
*
*****************************************************************************/
void system_Init(void);
void system_Loop_Task(void);
void system_Timer_Task(void);

int8_t system_msgHandler_MB_AppCB(uint8_t* RcvData, uint8_t Len);

/*中断ISR*/
#if (MCU_TYPE == MCU_TYPE_STM32)

#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
void TIM_Callback(void);
#elif(MCU_TYPE == MCU_TYPE_HC32_F4)
void Timer01B_CallBack(void);
void ExtInt_Callback(void);
void Uart_Callback_Rx(void);
void Uart_Callback_Err(void);
void Uart_Callback_Tx(void);
void Uart_Callback_TC(void);
void Uart_Callback_Idle(void);
void CAN_RxIrqCallBack(void);
void Uart_MCU_Callback_Rx(void);
void Uart_MCU_Callback_Err(void);
void Uart_MCU_Callback_Tx(void);
void Uart_MCU_Callback_TC(void);
void Uart_MCU_Callback_Idle(void);


void system_CJ_Task(void );
#endif
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
