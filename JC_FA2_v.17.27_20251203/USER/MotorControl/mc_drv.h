/********************************文件说明*************************************
*文件名: mc_drv.c

*作者: Yuchen Tan

*版本: V2.0.0

*功能简介:

*备注: 无

*修改履历:

*****************************************************************************/
#ifndef MC_DRV_HB_H_
#define MC_DRV_HB_H_

/*****************************文件包含(公开)**********************************
*
*备注: 可在此包含底层的库文件
*
*****************************************************************************/
#include "common.h"
#include "mc_common.h"
#include "gpio_adapter.h"
#include "timer_adapter.h"
/******************************宏定义(公开)***********************************
*
*备注: 需要被外部使用或可任意修改的宏在这里定义
*
*****************************************************************************/
/*驱动器参数定义-相*/
#define DRIVER_PHASE_A          (0)
#define DRIVER_PHASE_B          (1)
#define DRIVER_PHASE_C          (2)
#define DRIVER_PHASE_NB_MAX     (3)

/*电机驱动器状态定义*/
#define DRIVER_STATE_IDLE                   (0x01<<0)   //马达空闲(马达下桥短接提供静态自锁)
#define DRIVER_STATE_IDLE_START             (0x01<<1)   //马达准备启动
#define DRIVER_STATE_START_CHARGE_BOOT      (0x01<<2)   //启动-自举电容充电
#define DRIVER_STATE_START_WAIT_CHARGE_BOOT (0x01<<3)   //启动-等待自举电容充电完成
#define DRIVER_STATE_START_CLOSE_3L         (0x01<<4)   //启动-关闭所有下桥
#define DRIVER_STATE_START_WAIT_CLOSE_3L    (0x01<<5)   //启动-等待所有下桥关闭
#define DRIVER_STATE_START_OPEN_MOS         (0x01<<6)   //启动-根据转子所在扇区,开启对应相的MOS
#define DRIVER_STATE_RUN                    (0x01<<7)   //马达运行(此状态下可响应MOS-PWM占空比调节)
#define DRIVER_STATE_STOP_CLOSE_3HL         (0x01<<8)   //停止-关闭所有MOS
#define DRIVER_STATE_STOP_WAIT_CLOSE_3HL    (0x01<<9)   //停止-等待所有MOS关闭
#define DRIVER_STATE_STOP_OPEN_3L           (0x01<<10)  //停止-开启所有下桥(刹车)
#define DRIVER_STATE_STOP_BRAKE             (0x01<<11)  //停止-刹车持续时间
#define DRIVER_STATE_NB                     (12)        //马达驱动器状态总数(增加新状态该值要同步加)
/**************************数据类型及结构定义(公开)***************************
*
*备注: 需要被外部使用的数据结构及类型在这里定义
*
*****************************************************************************/
/*Relay类型定义*/
typedef struct
{
    GPIO_PORT_t         RelayPort;
    GPIO_PIN_t          RelayPin;
}RELAY_t;

/*Mos类型定义*/
typedef struct
{
    GPIO_PORT_t         MosPort;
    GPIO_PIN_t          MosPin;
    TIMER_INSTANCE_t    MosPWMTimer;
    TIMER_CHANNEL_t     MosPWMCh;
    GPIO_FUNC_t         GpioMode;
    GPIO_FUNC_t         PwmMode;
}MOS_t;

/*驱动器换相器类型定义*/
typedef struct
{
    const uint8_t*  CommutationTableFO;  	/*驱动器换相表-正向输出(假定)*/
    const uint8_t*  CommutationTableRO; 	/*驱动器换相表-反向输出(假定)*/
    uint8_t         RotorSector;            /*马达转子位置(即HallState)*/
    uint8_t         OpenH;                  /*当前位置需开启的上桥*/
    uint8_t         OpenL;                  /*当前位置需开启的下桥*/
    uint8_t         CloseHL;                /*当前位置需关闭的上下桥*/
}COMMUTATOR_t;

/*驱动器命令定义*/
typedef enum
{
	e_mdc_none = 0,
	e_mdc_output_forward,					//驱动器正向输出
	e_mdc_output_reverse,					//驱动器反向输出
	e_mdc_stop,								//驱动器停止(刹车)
}mc_drv_cmd_t;

/*驱动器输出方向定义*/
typedef enum
{
    e_nooutput = 0,	//不输出
    e_forward,		//正向输出
    e_reverse,		//反向输出
}mc_drv_od_t;

/*电机驱动器类型定义*/
typedef struct
{
    /*驱动器控制引脚抽象定义*/
    MOS_t           MosH[DRIVER_PHASE_NB_MAX];
    MOS_t           MosL[DRIVER_PHASE_NB_MAX];
    uint8_t         PhaseNb;                /*驱动器相数*/
    uint8_t         PhaseSequenceSel;       /*驱动器相序列选择*/
    /*驱动器换向控制器*/
    COMMUTATOR_t    *Commutator;
    /*驱动器标签*/
    uint16_t        Tag;
    /*驱动器状态*/
    uint16_t        State;
    /*驱动器动作命令*/
    mc_drv_cmd_t	Cmd;
	/*驱动器预设输出方向*/
    mc_drv_od_t		SetDir;
    /*驱动器延时控制*/
    uint8_t         DelayTimerEn;
    uint16_t        DelayTimerCnt;
    /*驱动器PWM占空比*/
    int16_t         PWMDuty;				/*驱动器输出(±)*/
    int16_t         MaxPWMDuty;				/*驱动器最大输出(+)*/
    int16_t         MinPWMDuty;				/*驱动器最小输出(+)*/
}MC_DRV_Handle_t;
/*****************************函数声明(公开)**********************************
*
*备注: 需要被外部使用的接口函数在这里声明
*
*****************************************************************************/
/*句柄操作*/
void mc_drv_hInit(MC_DRV_Handle_t *pDrv, uint16_t Tag, MOS_t *MosH, MOS_t *MosL, uint8_t PhaseNb, uint8_t PhaseSequence, COMMUTATOR_t *Commutator, uint8_t PhaseDirectionSel);
void mc_drv_Commutator_hInit(COMMUTATOR_t *Commutator, uint8_t PhaseNb, uint8_t PhaseDirectionSel);
/*驱动器命令及设置接口*/
void mc_drv_SetCmd(MC_DRV_Handle_t *pDrv, mc_drv_cmd_t Cmd);
void mc_drv_SetPWMDuty(MC_DRV_Handle_t *pDrv, int16_t PWMValue);
int16_t mc_drv_GetPWMDuty(MC_DRV_Handle_t *pDrv);
void mc_drv_SetPWMDutyMax(MC_DRV_Handle_t *pDrv, int16_t MaxPWMValue);
void mc_drv_SetPWMDutyMin(MC_DRV_Handle_t *pDrv, int16_t MinPWMValue);
/*驱动器换向控制*/
void mc_drv_Set_Motor_Rotor_Sector(MC_DRV_Handle_t *pDrv, uint8_t RotorSector);
void mc_drv_Commutation(MC_DRV_Handle_t *pDrv);
/*驱动器状态机*/
uint16_t mc_drv_Get_State(MC_DRV_Handle_t *pDrv);
void mc_drv_StateMachine(MC_DRV_Handle_t *pDrv);
/*MOS开关延时计数器控制*/
void mc_drv_Delay_Timer_Run(MC_DRV_Handle_t *pDrv);
/*模块测试*/
void mc_drv_Test(MC_DRV_Handle_t *pDrv1, MC_DRV_Handle_t *pDrv2);
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
