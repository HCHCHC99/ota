/********************************文件说明*************************************
*文件名: mc_hall.h

*作者: Yuchen Tan

*版本: V2.0.0

*功能简介:

*备注: 无

*修改履历:

*****************************************************************************/
#ifndef MC_HALL_H_
#define MC_HALL_H_

/*****************************文件包含(公开)**********************************
*
*备注: 可在此包含底层的库文件
*
*****************************************************************************/
#include "common.h"
#include "mc_common.h"
#include "gpio_adapter.h"
/******************************宏定义(公开)***********************************
*
*备注: 需要被外部使用或可任意修改的宏在这里定义
*
*****************************************************************************/
/*HALL传感器相关定义*/
#define HALL_A                  (0)
#define HALL_B                  (1)
#define HALL_C                  (2)
#define HALL_NB_MAX             (3)     //传感器个数(目前最多就是无刷电机的3霍尔)

/*M法的HALL脉冲步数采样窗口时间*/
#define HALLSMP_WINDOW_WIDTH_M  (5)     //单位：ms

/*HALL采样值存储区大小定义*/
#define HALLSMP_BUF_SIZE_T      (HPR)   //存储电机一圈的HALL脉宽值(不易受磁环充磁不均匀,HALL传感器位置偏影响),求和后导出RPM(*4表示每步都采集并存储)
#define HALLSMP_BUF_SIZE_M      (20)    //存储连续N个采样窗口的脉冲数,求和后导出RPM(N越大,速度细分精度越高,但等效于低通滤波深度越大,速度滞后越明显)

/*HALL异常状态定义*/
#define HALL_ABN_NO_ABN         (0x00)
#define HALL_ABN_TOO_FEW        (0x01<<0)   //HALL信号太少
#define HALL_ABN_TOO_MANY       (0x01<<1)   //HALL信号太多
#define HALL_ABN_REVERSE        (0x01<<2)   //HALL信号反向
#define HALL_ABN_ILLEGAL_STATE  (0x01<<3)   //HALL信号电平状态错误
/**************************数据类型及结构定义(公开)***************************
*
*备注: 需要被外部使用的数据结构及类型在这里定义
*
*****************************************************************************/
/*HALL端口类型定义*/
typedef struct
{
    GPIO_PORT_t     HallPort;
    GPIO_PIN_t      HallPin;
}HALL_t;

/*HALL脉冲计数器方向定义*/
typedef enum
{
	HALL_DIR_INCREASE = 1,			//脉冲增加
	HALL_DIR_DECREASE = -1			//脉冲减少
}HALL_DIR_t;

/*电机HALL脉冲计步器结构定义*/
typedef struct
{
    uint8_t         State;
    uint8_t         PrevState;
    uint8_t         StateNb;                    /*HALL状态总数*/
    const uint8_t   *StateTbl;                  /*HALL状态跳转表(const修饰uint8_t*,指针指向数据不可改)*/
    HALL_DIR_t		OneHallSetDir;              /*单HALL传感器预设方向(单hall无法自行判断方向)*/
    MOTOR_POS_t     HallData;                   /*HALL记步数*/
}HALL_PEDOMETER_Handle_t;

/*电机HALL测速器(M法)结构定义*/
typedef struct
{
    BOOL        		MeasureSpdEn_M;                     /*TRUE：开启测速; FALSE：关闭测速 */
    uint16_t    		WindowCnt;                          /*窗口平移式HALL脉冲数采样器-窗口当前时间*/
    uint16_t    		WindowWidth;                        /*窗口平移式HALL脉冲数采样器-窗口宽度(单位:ms)*/
    uint8_t     		HallNbSmpCnt;                       /*窗口平移式HALL脉冲数采样器-存储索引计数*/
	uint8_t    			HallNbSmpDoneCnt;					/*窗口平移式HALL脉冲数采样器-存储区中的已采样个数*/
    MOTOR_POS_t			HallNbSmp[HALLSMP_BUF_SIZE_M];		/*窗口平移式HALL脉冲数采样器-存储区*/
    MOTOR_POS_t 		PrevHallData;                       /*窗口起始时刻(上次窗口的末尾)的Hall脉冲步数值*/
    MOTOR_SPD_t 		MeasureSpd_M;						/*M法测速结果*/
}HALL_SPDMEAS_M_Handle_t;

/*电机HALL测速器(T法)结构定义*/
typedef struct
{
    BOOL        	MeasureSpdEn_T;                         /*TRUE：开启测速; FALSE：关闭测速 */
    uint8_t     	HallWidthSmpCnt;                        /*窗口平移式HALL脉宽采样器-存储索引计数*/
	uint8_t			HallWidthSmpDoneCnt;					/*窗口平移式HALL脉宽采样器-存储区中的已采样个数*/
    uint32_t    	HallWidthSmp[HALLSMP_BUF_SIZE_T];       /*窗口平移式HALL脉宽采样器-存储区*/
    uint8_t     	PreShieldCnt;                           /*屏蔽前XXX个信号不用于速度计算*/
    uint16_t    	TP0;                                    /*上一次HALL有效边沿定时器溢出次数*/
    uint16_t    	TC0;                                    /*上一次HALL有效边沿定时器计数值*/
    uint16_t    	TP1;                                    /*当前HALL有效边沿定时器溢出次数*/
    uint16_t    	TC1;                                    /*当前HALL有效边沿定时器计数值*/
    MOTOR_SPD_t 	MeasureSpd_T;							/*T法测速结果*/
    MOTOR_SPD_t 	PrevMeasureSpd_T;
}HALL_SPDMEAS_T_Handle_t;

/*电机HALL异常检测器结构定义*/
typedef struct
{
    BOOL        CheckAbnEn;                 /*TRUE：开启HALL异常检测; FALSE：关闭HALL异常检测 */
    uint16_t    CheckAbnTimer;              /*HALL异常检测计数*/
    uint16_t    CheckAbnShieldTimer;        /*HALL异常检测屏蔽时间计数*/
    uint8_t     LegalStateMin;              /*HALL电平状态-合法最小状态值*/
    uint8_t     LegalStateMax;              /*HALL电平状态-合法最大状态值*/
    uint8_t     IllegalStateCnt;            /*HALL非法状态计数*/
    HALL_DIR_t	SetDir;                     /*HALL异常检测预期方向*/
    MOTOR_POS_t HallDataA;                  /*HALL采样点A*/
    MOTOR_POS_t HallDataB;                  /*HALL采样点B*/
    uint16_t    ReverseTHH;                 /*HALL反向判断阈值*/
    uint16_t    TooFewTHH;                  /*HALL过少判断阈值*/
    uint16_t    TooManyTHH;                 /*HALL过多判断阈值*/
    uint8_t     HallAbnType;                /*HALL异常类型*/
}HALL_ABN_CHECKER_Handle_t;

/*电机HALL反馈结构定义*/
typedef struct
{
    /*HALL信号端口抽象定义*/
    HALL_t                      Hall[HALL_NB_MAX];
    /*HALL传感器个数*/
    uint8_t                     HallNb;
    /*HALL序列选择*/
    uint8_t                     SequenceSel;
	/*HALL方向选择*/
	uint8_t						HallDirectionSel;
    /*HALL磁极对数*/
    uint8_t                     PoleNb;
    /*Hall脉冲计步*/
    HALL_PEDOMETER_Handle_t     *pHP;
    /*Hall测速-M法*/
    HALL_SPDMEAS_M_Handle_t     *pHSMM;
    /*Hall测速-T法*/
    HALL_SPDMEAS_T_Handle_t     *pHSMT;
    /*Hall信号异常检测*/
    HALL_ABN_CHECKER_Handle_t   *pHAC;
}HALL_Handle_t;
/*****************************函数声明(公开)**********************************
*
*备注: 需要被外部使用的接口函数在这里声明
*
*****************************************************************************/
/*句柄初始化*/
void mc_hall_hInit(HALL_Handle_t *pHall, HALL_t *Hall, uint8_t HallNb, uint8_t PoleNb, uint8_t SequenceSel, HALL_PEDOMETER_Handle_t *pHP, HALL_SPDMEAS_M_Handle_t *pHSMM, HALL_SPDMEAS_T_Handle_t *pHSMT, HALL_ABN_CHECKER_Handle_t *pHAC);
/*模块功能测试*/
void mc_hall_Test(HALL_Handle_t *pHall);
/*HALL信号脉冲计数*/
MOTOR_POS_t mc_hall_Get_HallData(HALL_PEDOMETER_Handle_t *pHP);
void mc_hall_Set_HallData(HALL_PEDOMETER_Handle_t *pHP, MOTOR_POS_t HallData);
void mc_hall_Set_OneHallDir(HALL_PEDOMETER_Handle_t *pHP, HALL_DIR_t Dir);
uint8_t mc_hall_Get_HallState(HALL_Handle_t *pHall);
void mc_hall_Update_HallData(HALL_Handle_t *pHall);
/*HALL信号测速-M法*/
void mc_hall_SpeedMeasure_M_Cal_WindowWidth(HALL_Handle_t *pHall, MOTOR_SPD_t TargetRPM, uint16_t SpdDivision);
uint16_t mc_hall_Get_M_WindowWidth(HALL_SPDMEAS_M_Handle_t *pHSMM);
void mc_hall_Reset_SpeedMeasure_M(HALL_Handle_t *pHall);
void mc_hall_SpeedMeasure_M_Enable(HALL_SPDMEAS_M_Handle_t *pHSMM);
void mc_hall_SpeedMeasure_M_Disable(HALL_SPDMEAS_M_Handle_t *pHSMM);
void mc_hall_SpeedMeasure_M(HALL_Handle_t *pHall);
MOTOR_SPD_t mc_hall_Get_MeasureSpeed_M(HALL_SPDMEAS_M_Handle_t *pHSMM);
/*HALL信号测速-T法*/
void mc_hall_SpeedMeasure_T_Timer(void);
void mc_hall_Reset_SpeedMeasure_T(HALL_SPDMEAS_T_Handle_t *pHSMT);
void mc_hall_SpeedMeasure_T_Enable(HALL_SPDMEAS_T_Handle_t *pHSMT);
void mc_hall_SpeedMeasure_T_Disable(HALL_SPDMEAS_T_Handle_t *pHSMT);
void mc_hall_SpeedMeasure_T(HALL_Handle_t *pHall);
MOTOR_SPD_t mc_hall_Get_MeasureSpeed_T(HALL_SPDMEAS_T_Handle_t *pHSMT);
/*HALL信号异常检测*/
void mc_hall_Check_Hall_Abnormal_Enable(HALL_ABN_CHECKER_Handle_t *pHAC);
void mc_hall_Check_Hall_Abnormal_Disable(HALL_ABN_CHECKER_Handle_t *pHAC);
void mc_hall_Check_Hall_Abnormal_Init(HALL_ABN_CHECKER_Handle_t *pHAC, HALL_DIR_t Dir, MOTOR_SPD_t MinSpeed, MOTOR_SPD_t MaxSpeed, int32_t Hpr);
void mc_hall_Reset_Check_Hall_Abnormal(HALL_ABN_CHECKER_Handle_t *pHAC);
void mc_hall_Check_Hall_Abnormal(HALL_Handle_t *pHall);
void mc_hall_Clear_Hall_Abnormal(HALL_ABN_CHECKER_Handle_t *pHAC);
uint8_t mc_hall_Get_Hall_Abnormal(HALL_ABN_CHECKER_Handle_t *pHAC);
/********************************函数定义************************************
*函数名:

*函数功能描述: 马达控制-电机HALL-HALL信号异常检测

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*句柄初始化*/

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
