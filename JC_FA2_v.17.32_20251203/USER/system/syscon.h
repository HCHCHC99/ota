/********************************文件说明*************************************
*文件名: syscon.h

*作者: Xiaodong Qu

*版本: V1.0.0

*功能简介:与上位机交互的头文件

*备注: 无

*修改履历:

*****************************************************************************/
#ifndef SYS_CON_H_
#define SYS_CON_H_

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

#define	UART_MAX_DATA_SIZE		(100)

/*上位机->控制器*/
typedef enum
{
	/*50:进入配置模式命令*/
	CLIENT_ENTER_CONFIG_CMD = 50,
	/*51:驱动参数配置命令*/
	CLIENT_DRIVE_CONFIG_SET_CMD,
	/*52:驱动参数读取命令*/
	CLIENT_SYSCONFIG_GET_CMD,
}syscon_client_cfg_cmd_t;

/*控制器->上位机*/
typedef enum
{
	/*200:控制器退出配置模式命令*/
	CONTROLLER_EXIT_CONFIG_MODE_CMD = 200,
}syscon_control_cfg_cmd_t;

/*通讯类型*/
typedef enum
{
	E_NOCOM = 0,				    //无通讯
    E_MODBUS = 0x05,                //modbus通讯
    E_CAN = 0x0A,                   //CAN通讯
}syscon_cfg_com_type_t;

/*复位运行模式*/
typedef enum
{
	E_RESET_INCHING = 0x05,			    //点动
    E_RESET_CONTINUOUS = 0x0A,          //续动
}syscon_cfg_reset_run_mode_t;

/*复位方向*/
typedef enum
{
	E_RESET_BTM = 0x05,				    //到底复位
    E_RESET_TOP = 0x0A,                 //到顶复位
}syscon_cfg_reset_direction_t;

/*复位判断方式*/
typedef enum
{
	E_DETECTION_SIGNAL = 0x05,			//检测信号开关（不需要复位）
	E_DETECTION_ANOMALY = 0x0A,			//检测推杆异常（需要复位）
}syscon_cfg_reset_judgment_mode_t;

/*推杆运行模式*/
typedef enum
{
	E_MOTOR_INCHING = 0x05,				//推杆点动运行
    E_MOTOR_CONTINUOUS = 0x0A,          //推杆续动运行
}syscon_cfg_motor_run_mode_t;

/*上限位类型*/
typedef enum
{
	E_TOP_SIGNAL_SWITCH = 1,            //信号开关
    E_TOP_SOFT_LIMIT,                 	//软限位
    E_TOP_HALL_ABNORMAL,                //HALL异常
	E_TOP_MAX,
}syscon_cfg_top_detect_t;

/*下限位类型*/
typedef enum
{
	E_BTM_SIGNAL_SWITCH = 1,            //信号开关
    E_BTM_SOFT_LIMIT,                 	//软限位
    E_BTM_HALL_ABNORMAL,                //HALL异常
	E_BTM_MAX,
}syscon_cfg_btm_detect_t;

/*引脚输入功能*/
typedef enum
{
    E_DINO_FUN = 0,                       //无功能
    E_UP_FUN,                           //上升功能
    E_DOWN_FUN,                         //下降功能
    E_RESET_FUN,                        //复位功能
    E_TOP_LIMIT_FUN,                    //上限位功能
    E_BTM_LIMIT_FUN,                    //下限位功能
    E_DI_MAX,
}syscon_cfg_di_fun_t;

/*引脚输出功能*/
typedef enum
{
    E_DONO_FUN = 0,                     //无功能
    E_REACH_TOP_LIMIT_FUN,              //到达上限位
    E_REACH_BTM_LIMIT_FUN,              //到达下限位
    E_FAILURE_FUN,            		 	//故障指示
    E_STATUS_FUN,              			//状态指示
    E_DO_MAX,
}syscon_cfg_do_fun_t;

/*引脚极性*/
typedef enum
{
    E_LOW_LEVEL = 0,
    E_HIGH_LEVEL,
}syscon_cfg_pin_polarity_t;


/* 配置项结构体定义 */

/*配置项参数*/
typedef struct
{
    /*推杆参数*/
    uint32_t                        Config_GearRatio;              	/* 减速比  	(0 ~ 2000000)  (电机端转速与推杆转速的比值，千分比表示) */
	uint16_t                        Config_Route;                  	/* 行程		(0 ~ 5000 mm) */
    uint16_t                        Config_SpeedMmps;              	/* 推杆速度	(0 ~ 2000) (单位：0.1mm/s) */
    uint16_t                        Config_OvcValue;               	/* 过流保护阈值	(100 ~ 30000 mA) */
    uint16_t                        Config_OverVoltage;            	/* 过压保护阈值	(100 ~ 1000) (单位：0.1V) */
    uint16_t                        Config_UnderVoltage;           	/* 欠压保护阈值	(100 ~ 1000) (单位：0.1V) */
	uint8_t                         Config_Lead;                   	/* 丝杆导程 	(0 ~ 200 mm/圈) */
    /*系统参数*/
    uint8_t                         Config_NodeSlaveAddr;          	/* 通讯节点ID */
    uint8_t        					Config_CommunicationType;      	/* 通讯类型(枚举类型定义见：CON_COM_TYPE) */

    uint8_t            				Config_ResetRunMode;           	/* 复位运行模式   (枚举类型定义见：CON_RESET_RUN_MODE) */
    uint8_t							Config_ResetDirection;         	/* 复位方向	(枚举类型定义见：CON_RESET_DIRECTION) */
	uint8_t							Config_ResetMode;			    /* 复位模式	(枚举类型定义见：CON_RESET_JUDGMENT_MODE) */
	uint8_t            				Config_MotorRunMode;            /* 电机运行模式	(枚举类型定义见：CON_MOTOR_RUN_MODE) */
	

    uint8_t            				Config_TopDetection;            /* 到顶检测	(枚举类型定义见：CON_TOP_DETECTION) */
    uint8_t            				Config_BtmDetection;            /* 到底检测	(枚举类型定义见：CON_BTM_DETECTION) */
	int8_t                         	Config_ResetRaise;              /* 复位抬高   (-100 ~ 100 mm) */
	uint8_t             			Config_DIFunction;              /* 引脚输入功能	(枚举类型定义见：CON_DI_FUN) */
	uint8_t             			Config_DOFunction;             	/* 引脚输出功能	(枚举类型定义见：CON_DO_FUN) */
	uint8_t	         				Config_ActiveValue;           	/* 引脚极性	(枚举类型定义见：CON_PIN_POLARITY) */
	/*底层参数配置*/
	uint8_t							Config_HallDirectionSel;		/* hall方向选择 */
	uint8_t							Config_PhaseDirectionSel;		/* 驱动器方向选择 */
	uint8_t                         Resv[7];
	
}syscon_cfg_handle_t;

/* 报文头 */
typedef struct
{
    uint16_t    Cmd;  /* 命令号，见 HOST_TO_CB_CMD 和 CLIENT_TO_CB_CMD */
    uint16_t    MsgLen;  /* 报文长度，单位字节 */
    int16_t     Ret;  /* 返回结果，0表示成功，其余值表示失败 */
	uint8_t		Resv[2]; /*预留*/
}syscon_msg_head_t;

/* 报文头 */
typedef struct
{
    uint16_t	CRC;	/*CRC校验*/
	uint8_t		Resv[2]; /*预留*/
}syscon_msg_tail_t;

/* 不带数据的通用报文 */
typedef struct
{
    syscon_msg_head_t  		Head;/* 报文头 */
    syscon_msg_tail_t  		Tail;/* 报文尾 */
}syscon_without_data_handle_t;

/* 带驱动参数数据的报文 */
typedef struct
{
    syscon_msg_head_t    	Head;
    syscon_cfg_handle_t 	Para;/* 驱动参数结构体 */
    syscon_msg_tail_t    	Tail;
}syscon_interface_data_handle_t;
/* 报文分析 */
typedef struct
{
	syscon_msg_head_t  		Head;
	uint8_t 				Data[UART_MAX_DATA_SIZE];
	syscon_msg_tail_t		Tail;
}syscon_uart_cfg_handle_t;

/**************************数据类型及结构定义(公开)***************************
*
*备注: 需要被外部使用的数据结构及类型在这里定义
*
*****************************************************************************/

/*****************************函数声明(公开)**********************************
*
*备注: 需要被外部使用的接口函数在这里声明
*
*****************************************************************************/
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
