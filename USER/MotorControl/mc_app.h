/********************************文件说明*************************************
*文件名: mc_app.h

*作者: Yuchen Tan

*版本: V1.0.5

*功能简介:

*备注: 无

*修改履历:

*****************************************************************************/
#ifndef MC_APP_H_
#define MC_APP_H_

/*****************************文件包含(公开)**********************************
*
*备注: 可在此包含底层的库文件
*
*****************************************************************************/
#include "common.h"
#include "mc_common.h"
/******************************宏定义(公开)***********************************
*
*备注: 需要被外部使用或可任意修改的宏在这里定义
*
*****************************************************************************/
/*电机编号*/
#define MOTOR1          (uint16_t)(1<<0)
#define MOTOR2          (uint16_t)(1<<1)
#define MOTOR3          (uint16_t)(1<<2)
#define MOTOR4          (uint16_t)(1<<3)

/*多电机控制组编号*/
#define MOTOR_GROUP1    (uint16_t)(1<<0)
#define MOTOR_GROUP2    (uint16_t)(1<<1)
#define MOTOR_GROUP3    (uint16_t)(1<<2)
#define MOTOR_GROUP4    (uint16_t)(1<<3)

/*电机动作命令*/
/*注：所有的动作和停止都带斜坡控制,若想实现急启急停的效果,可通过调整斜坡参数实现(ACC调大)!*/
typedef enum
{
	e_mac_none = 0,
	//单电机
	e_mac_stop = 2,				//急停(参数：无)
	e_mac_start_closeloop,		//闭环启动(参数：速度斜坡参数(初速度,加速度,目标速度))
	e_mac_stop_closeloop,		//闭环停止(参数：速度斜坡参数(加速度,末速度))
	e_mac_start_openloop,		//开环启动(参数：占空比斜坡参数(初占空比,占空比斜率,目标占空比))
	e_mac_stop_openloop,		//开环停止(参数：占空比斜坡参数(占空比斜率,末占空比))
	e_mac_goto_targetpos,		//闭环运行到目标位置(参数：目标位置+速度斜坡参数(初速度,加速度,目标速度,末速度))
	//多电机(电机组)
	e_mac_sync_start,			//同步启动(基于速度闭环)(参数：速度斜坡参数(初速度,加速度,目标速度))
	e_mac_sync_stop,			//同步停止(基于速度闭环)(参数：速度斜坡参数(加速度,末速度))
	e_mac_sync_goto_targetpos,	//同步运行到目标位置(基于速度闭环)(参数：目标位置+速度斜坡参数(初速度,加速度,目标速度,末速度))
	//
	mac_ValidCheck,
}mc_app_cmd_t;

/*电机状态*/
typedef enum
{
	e_mas_idle = 0,				//无动作
	e_mas_pseudo_idle,			//无动作(切换方向的运行由此状态启动)
	//单电机
	e_mas_stopping = 2,			//停止中
	e_mas_start_closeloop,		//闭环启动(参数：速度斜坡参数(初速度,加速度,目标速度))
	e_mas_stop_closeloop,		//闭环停止(参数：速度斜坡参数(加速度,末速度))
	e_mas_start_openloop,		//开环启动(参数：占空比斜坡参数(初占空比,占空比斜率,目标占空比))
	e_mas_stop_openloop,		//开环停止(参数：占空比斜坡参数(占空比斜率,末占空比))
	e_mas_goto_targetpos,		//闭环运行到目标位置(参数：目标位置+速度斜坡参数(初速度,加速度,目标速度,末速度))
	//多电机(电机组)
	e_mas_sync_start,			//同步启动(基于速度闭环)(参数：速度斜坡参数(初速度,加速度,目标速度))
	e_mas_sync_stop,			//同步停止(基于速度闭环)(参数：速度斜坡参数(加速度,末速度))
	e_mas_sync_goto_targetpos,	//同步运行到目标位置(基于速度闭环)(参数：目标位置+速度斜坡参数(初速度,加速度,目标速度,末速度))
	//
	mas_ValidCheck,
	e_mas_unknown = 0xff
}mc_app_sta_t;

/*电机读写参数*/
typedef enum
{
	e_map_state = 0,			//运行状态(R)
	e_map_dir,					//运行方向(R)
	e_map_fault,				//故障(RW)
	e_map_fdbkpos,				//当前位置(RW)
	e_map_targetspd,			//目标速度(RW)
	e_map_fdbkspd,				//当前速度(R)
	e_map_acc,					//加速度(RW)
	e_map_startspd,				//启动速度(RW)
	e_map_stopspd,				//停止速度(RW)
	e_map_ol_targetdc,			//开环目标占空比(RW)
	e_map_drvoutput_max,		//驱动器最大占空比(RW)
	e_map_current,				//电流(R)
	e_map_ocp_thh,				//过流保护值(RW)(1=1mA)
	//
	map_ValidCheck
}mc_app_param_t;

/*电机故障标志位定义(Todo: 可细化到某个hall或某相过流)*/
#define MOTOR_NO_FAULT              (0)     //无故障
#define MOTOR_FAULT_OVC             (1<<0)  //过流
#define MOTOR_FAULT_HALL            (1<<1)  //HALL异常
#define MOTOR_FAULT_OVV             (1<<2)  //过压
#define MOTOR_FAULT_UDV             (1<<3)  //欠压
#define MOTOR_FAULT_LR              (1<<4)  //堵转

/*电机接口返回值定义*/
#define MC_RET_OK                   (1)     //ok
#define MC_RET_ERR_CMD              (-1)    //命令错误
#define MC_RET_CMD_NOT_EXEC         (-2)    //命令不可被相应
#define MC_RET_ERR_ARGV             (-3)    //错误指令参数
#define MC_RET_PARAM_NOT_ACCESS     (-4)    //参数不可访问
#define MC_RET_ERR_MOTOR            (-5)    //错误目标电机
/**************************数据类型及结构定义(公开)***************************
*
*备注: 需要被外部使用的数据结构及类型在这里定义
*
*****************************************************************************/
/*电机控制器-闭环参数结构定义*/
typedef struct
{
    MOTOR_SPD_t     StartSpd;           //启动初速度
    MOTOR_SPD_t     TargetSpd;          //目标速度
    MOTOR_SPD_t     StopSpd;            //停止末速度
    MOTOR_SPD_t     SpdUpAcc;           //加速过程加速度(绝对值)(单位: rpm/s)
    MOTOR_SPD_t     SpdDownAcc;         //减速过程加速度(绝对值)
}MC_CLP_Handle_t;

/*电机控制器-开环参数结构定义*/
typedef struct
{
    int16_t         InitDCPercent;      //启动初占空比(单位: 百分比)
    int16_t         TargetDCPercent;    //目标占空比
    int16_t         FinalDutyCycle;     //停止末占空比
    int16_t         SpdUpAcc;           //加速过程占空比斜率(绝对值)(单位: 占空比百分比/s)
    int16_t         SpdDownAcc;         //减速过程占空比斜率(绝对值)
}MC_OLP_Handle_t;
/*****************************函数声明(公开)**********************************
*
*备注: 需要被外部使用的接口函数在这里声明
*
*****************************************************************************/
/*模块入口函数*/
void mc_app_Init(void);
void mc_app_Timer_250us(void);
void mc_app_Timer_1ms(void);
void mc_app_Trigger_Task_HallEdge(void);
void mc_app_Loop_Task(void);
void mc_app_Test(uint16_t Motor1, uint16_t Motor2);
/*单电机应用功能相关接口*/
mc_app_sta_t mc_app_Get_State(uint16_t Motor);
int8_t mc_app_Write_Param(uint16_t Motor, mc_app_param_t WhichParam, int32_t argv);
int8_t mc_app_Read_Param(uint16_t Motor, mc_app_param_t WhichParam, int32_t* argv);
int8_t mc_app_Set_Single_Motor_Cmd(uint16_t Motor, mc_app_cmd_t Cmd, int32_t argv);
/*多电机组合应用功能相关接口*/
mc_app_sta_t mc_app_Get_MGC_State(uint16_t MotorGroup);
int8_t mc_app_Set_Multi_Motor_Cmd(uint16_t MotorGroup, mc_app_cmd_t Cmd, int32_t argv);
/*底层配置修改*/
int8_t mc_app_ModifyHallDir(uint8_t Motor, uint8_t Dir);
int8_t mc_app_ModifyDrvDir(uint8_t Motor, uint8_t Dir);
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
