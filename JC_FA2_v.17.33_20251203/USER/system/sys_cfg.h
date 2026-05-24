/********************************文件说明*************************************
*文件名: sys_cfg.h

*作者: Xiaodong Qu

*版本: V1.0.0

*功能简介:系统配置函数

*备注: 无

*修改履历:

*****************************************************************************/
#ifndef SYSTEM_CONFIG_H_
#define SYSTEM_CONFIG_H_

/*****************************文件包含(公开)**********************************
*
*备注: 可在此包含底层的库文件
*
*****************************************************************************/
#include "common.h"
#include "syscon.h"
#include "gpio_adapter.h"
/******************************宏定义(公开)***********************************
*
*备注: 需要被外部使用或可任意修改的宏在这里定义
*
*****************************************************************************/
/*注: FLAG,VALUE必须单独用括号括起来,防止VALUE是一个位与组合值,展开后出错*/
#define SYSTEM_SET_FLAG(FLAG, VALUE)    ((FLAG) |= (VALUE))
#define SYSTEM_CLR_FLAG(FLAG, VALUE)    ((FLAG) &= ~(VALUE))
#define SYSTEM_GET_FLAG(FLAG, VALUE)    ((FLAG) & (VALUE))
#define SYSTEM_MATCH_FLAG(FLAG, VALUE)  ((FLAG) == (VALUE))

/*命令号*/
#define CMD_NUMBER	2

/*帧尾校验长度*/
#define TAIL_CRC_L	4	
#define	TAIL_CRC_H	3
#define	TAIL_LEN	4

/*串口接收数据返回值*/
#define	SYS_CFG_UART_RET_OK			0
#define SYS_CFG_UART_RET_RIGHT		1
#define SYS_CFG_UART_RET_ERROR		-1

/*配置参数的返回值*/
#define CFG_RET_GEARRATIO        	(1<<0)                      
#define CFG_RET_ROUTE        		(1<<1)                      
#define CFG_RET_LEAD        		(1<<3)                      
#define CFG_RET_SPEEDMMPS        	(1<<4)  
#define CFG_RET_OVCVALUE        	(1<<5)                      
#define CFG_RET_OVERVOLTAGE        	(1<<6)                      
#define CFG_RET_UNDERVOLTAGE        (1<<7)   
#define CFG_RET_NODESLAVEADDR       (1<<8)
#define CFG_RET_RESETRAISE        	(1<<9)  
//#define CFG_RET_GEARRATIO        (1<<10)                      
//#define CFG_RET_GEARRATIO        (1<<11)                      
//#define CFG_RET_GEARRATIO        (1<<12)                      
//#define CFG_RET_GEARRATIO        (1<<13)  
//#define CFG_RET_GEARRATIO        (1<<14)                      
//#define CFG_RET_GEARRATIO        (1<<15)                      
//#define CFG_RET_GEARRATIO        (1<<16) 

/*配置参数阈值*/
#define NODE_SLAVE_ADDR_MAX			(255) //节点ID
#define	NODE_SLAVE_ADDR_MIN			(0)

#define GEAR_RATIO_MAX				(2000000) //减速比(千分比表示)
#define	GEAR_RATIO_MIN				(0)

#define ROUTE_MAX					(5000)	//行程(单位：mm)
#define	ROUTE_MIN					(0)

#define SPEED_MMPS_MAX				(2000)	//推杆速度(单位：0.1mm/s)
#define	SPEED_MMPS_MIN				(0)

#define OVC_VALUE_MAX				(30000)	//过流保护阈值(单位：mA)
#define	OVC_VALUE_MIN				(0)

#define OVER_VOLTAGE_MAX			(1000)	//过压保护阈值(单位：0.1V)
#define	OVER_VOLTAGE_MIN			(0)

#define UNDER_VOLTAGE_MAX			(1000)	//欠压保护阈值(单位：0.1V)
#define	UNDER_VOLTAGE_MIN			(0)

#define LEAD_MAX					(200)	//丝杆导程(单位：mm/圈)
#define	LEAD_MIN					(0)

#define RESET_RAISE_MAX				(100)	//复位抬高距离(单位：mm)
#define	RESET_RAISE_MIN				(-100)

#define	FSA_SYS_CONFIG				BLOCK_L4_ADDR	/* 存系统配置参数(常改) */
#define	FSA_MOTOR_CONFIG			BLOCK_L5_ADDR	/* 存电机配置参数(少改动) */
#define	FSA_BOOT_CONFIG				BLOCK_L6_ADDR	/* 底层配置参数 */

#define	SYS_CFG_ENTER_CFG_FLAG		(1<<0)
#define	SYS_CFG_SAVE_FLASH_FLAG		(1<<1)
#define	SYS_CFG_CONTROL_MODE		(1<<2)
#define	SYS_CFG_PERAMETER_SET_MODE	(1<<3)

/*****************************配置项的默认值************************/

/*固件信息*/
#define SOFTWARE_MAINVER_SYS   	1  /*软件版本*/
#define SOFTWARE_SUBVER1_SYS   	0
#define SOFTWARE_SUBVER2_SYS   	0  

#define HARDWARE_MAINVER_SYS   	1  /*硬件版本*/
#define HARDWARE_SUBVER1_SYS  	0
#define HARDWARE_SUBVER2_SYS	0

#define RELEASE_DATE			230822  /*软件下发日期*/


/*推杆参数*/
#define GEAR_RATIO_CFG              (13614) /*齿轮箱减速比(千分比)*/
#define LEAD_CFG                   	(10.0f) /*丝杆导程(mm/圈)*/
#define ROUTE_CFG                   (450.0f) /*行程(mm)*/
#define SPEED_MMPS_CFG              (400) /*推杆推速(0.1mm/s)*/
/*系统参数*/
#define NODE_SLAVE_ADDR_CFG         (0) /*通讯节点ID*/
#define COMMUNICATION_TYPE_CFG      (E_NOCOM) /*通讯类型*/
#define MOTOR_OVC_VALUE_CFG         (10000) /*过流保护阈值(mA)*/
#define OVER_VOLTAGE_CFG            (320) /*过压保护阈值(单位:0.1V)*/
#define UNDER_VOLTAGE_CFG           (180) /*欠压保护阈值(单位:0.1V)*/
#define RESET_RUN_MODE_CFG          (E_RESET_CONTINUOUS)/*复位运行模式  0x05--点动  0x0A--续动*/
#define RESET_DIRECTION_CFG         (E_RESET_BTM)/*复位方向  0x05--到底复位  0x0A--到顶复位*/
#define RESETMODE_CFG				(E_DETECTION_SIGNAL)/*复位方式  0x05--不需要复位  0x0A--需要复位*/
#define RESET_RAISE_CFG             (0)/*复位抬高 mm*/

#define MOTOR_RUN_MODE_CFG          (E_MOTOR_CONTINUOUS)/*推杆运行模式  0x05--点动  0x0A--续动*/
#define TOP_DETECTION_CFG			(E_TOP_SOFT_LIMIT)/*到顶检测*/
#define BTM_DETECTION_CFG 			(E_BTM_SOFT_LIMIT)/*到底检测*/

/*底层参数配置*/
#define HALL_REVERSE            	(0)             //0-HALL正向    1-HALL反向
#define DRIVER_REVERSE          	(1)             //0-DRIVER正向  1-DRIVER反向

/**************************数据类型及结构定义(公开)***************************
*
*备注: 需要被外部使用的数据结构及类型在这里定义
*
*****************************************************************************/

/*引脚IO配置*/
typedef struct 
{
   syscon_cfg_di_fun_t             	DIFunction;                     //引脚输入功能
   syscon_cfg_do_fun_t             	DOFunction;                     //引脚输出功能
   GPIO_PORT_t              		Port;                           //端口port
   GPIO_PIN_t               		Pin;                            //对应pin
   syscon_cfg_pin_polarity_t        ActiveValue;                    //引脚极性
}sys_cfg_dido_handle_t;

/*控制器当前处于的模式*/
typedef enum
{
	E_CONFIGMODE = 0,		/*配置模式*/
	E_IDLEMODE = -1,		/*空闲模式*/
	
}control_mode_t;

/*参数设置当前处于的模式*/
typedef enum
{
	E_CONFIGMODESET = 0,	/*配置模式下的设置*/
	E_IDLEMODESET = -1,		/*空闲模式下的设置*/
	
}perameter_set_t;


typedef struct
{
	uint16_t 			CfgRetFluat;			/*参数配置对应错误的返回值*/
	uint8_t				EnterConfigFlag;		/*系统进入配置模式的标志*/
	uint8_t				SaveCfgFlashFlag;		/*系统Flash保存数据标志*/
	uint8_t  			ControlMode;			/*系统当前模式*/
	perameter_set_t		PerameterSetMode;		/*驱动参数当前模式*/
}sys_cfg_flag_handle_t;

/*****************************函数声明(公开)**********************************
*
*备注: 需要被外部使用的接口函数在这里声明
*
*****************************************************************************/
syscon_cfg_handle_t sys_cfg_Hadnle(void);
sys_cfg_flag_handle_t* sys_cfg_FlagHandle(void);
void sys_cfg_NodeSlaveAddr(uint8_t SlaveAddr);
/*函数初始化*/
void sys_cfg_Init(void);
void sys_cfg_UartConfigInit(void);
/*串口相关函数*/
void sys_cfg_MsgUartConfigHandler(syscon_uart_cfg_handle_t* SysConfig);
int8_t sys_cfg_UartDataUnpack(syscon_uart_cfg_handle_t* SysConfig, uint8_t *PkData, uint8_t DataLen);
int8_t sys_cfg_UartDataProcess(syscon_uart_cfg_handle_t* SysConfig);
void sys_cfg_Controller(void);
void sys_cfg_DeliveryData(void);
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
