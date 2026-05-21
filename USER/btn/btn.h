/********************************文件说明*************************************
*文件名: btn.h

*作者: Yuchen Tan

*版本: V1.2.3

*功能简介:

*备注: 无

*修改履历:

*****************************************************************************/
#ifndef BTN_H_
#define BTN_H_

/*****************************文件包含(公开)**********************************
*
*备注: 可在此包含底层的库文件
*
*****************************************************************************/
#include "common.h"
#include "btn_drv.h"
/******************************宏定义(公开)***********************************
*
*备注: 需要被外部使用或可任意修改的宏在这里定义
*
*****************************************************************************/
//按键个数定义
#define BTN_NB			(2)		//注: BTN_NB会限制索引的范围(eg: BTN_NB == 3,则实例索引只能使用BTN_1,BTN_2,BTN_3,不能任意选3个)
#if (BTN_NB > 16)			
//目前最多支持16个按键,扩展到32键需同步修改BTN_MASK_t类型重定义为uint32_t
#error "btn number is limited within 16!"
#endif

/*按钮标签定义(根据需要按顺序增加,必须从0开始!)*/
typedef enum
{
	BTN_1 = 0,
	BTN_2,
	BTN_3,
	BTN_4,
	BTN_5,
	BTN_6,
	BTN_7,
	BTN_8,
	BTN_9,
	BTN_10,
	BTN_11,	
	BTN_12,
	BTN_13,	
	BTN_14,
	BTN_15,
	BTN_16,
	//end
}BTN_t;

/*按钮掩码定义(用于选择多按键事件状态)(和标签对应)*/
#define BTN_1_MASK      (BTN_MASK_t)(1<<BTN_1)
#define BTN_2_MASK      (BTN_MASK_t)(1<<BTN_2)
#define BTN_3_MASK      (BTN_MASK_t)(1<<BTN_3)
#define BTN_4_MASK      (BTN_MASK_t)(1<<BTN_4)
#define BTN_5_MASK      (BTN_MASK_t)(1<<BTN_5)
#define BTN_6_MASK      (BTN_MASK_t)(1<<BTN_6)
#define BTN_7_MASK      (BTN_MASK_t)(1<<BTN_7)
#define BTN_8_MASK      (BTN_MASK_t)(1<<BTN_8)
#define BTN_9_MASK      (BTN_MASK_t)(1<<BTN_9)
#define BTN_10_MASK		(BTN_MASK_t)(1<<BTN_10)
#define BTN_11_MASK		(BTN_MASK_t)(1<<BTN_11)
#define BTN_12_MASK		(BTN_MASK_t)(1<<BTN_12)
#define BTN_13_MASK		(BTN_MASK_t)(1<<BTN_13)
#define BTN_14_MASK		(BTN_MASK_t)(1<<BTN_14)
#define BTN_15_MASK		(BTN_MASK_t)(1<<BTN_15)
#define BTN_16_MASK		(BTN_MASK_t)(1<<BTN_16)
#define BTN_ALL_MASK    (BTN_MASK_t)(0xFFFFFFFF)
/**************************数据类型及结构定义(公开)***************************
*
*备注: 需要被外部使用的数据结构及类型在这里定义
*
*****************************************************************************/
/*按钮选择掩码类型定义*/
typedef uint16_t BTN_MASK_t;    //uint16_t最多支持16个按键

/*按钮状态定义*/
typedef enum
{
    E_BTN_STA_RELEASED = 0,     //释放
    E_BTN_STA_FIRSTPRESSHOLD,   //首次按下及保持
    E_BTN_STA_LONGPRESS,        //长按
    E_BTN_STA_FIRSTUP,          //首次弹起
    E_BTN_STA_MULTIPRESS_LOOP,  //连按
    E_BTN_STA_UNDEFINED = 0XFF, //无效值
}BTN_STATE_t;

/*按钮事件定义*/
typedef enum
{
    E_BTN_EVT_NONE = 0,         //无事件
    E_BTN_EVT_PRESS = 1,        //按下
    E_BTN_EVT_LONG_PRESS,       //长按
    E_BTN_EVT_LONG_UP,          //长按弹起
    E_BTN_EVT_UP,               //弹起
    E_BTN_EVT_MULTI_PRESS,      //连续按下
    E_BTN_EVT_MULTI_UP,         //连续弹起
    E_BTN_EVT_UNDEFINED = 0XFF, //无效值
}BTN_EVEVT_t;

/*按钮控制器结构定义*/
typedef struct
{
    uint8_t         Index;              /*按钮控制器索引*/
    BTN_MASK_t      Mask;               /*按钮掩码(和索引值对应,用于群组访问)*/
    BTN_DRIVER_t    Driver;             /*按钮驱动器句柄*/
    /*按钮控制*/
    BTN_VALUE_t     ActiveValue;        /*按钮有效值*/
    BTN_VALUE_t     BtnValueMask;       /*按钮键值掩码(按钮按下: (驱动器电平&掩码)==有效值)*/
    BTN_VALUE_t     BtnValue;           /*当前按钮驱动器有效值(已消抖)*/
    BTN_VALUE_t     BtnValuePrev;       /*上次按钮驱动器有效值(已消抖)*/
    BTN_STATE_t     BtnState;           /*状态机状态*/
    BTN_EVEVT_t     BtnEvent;           /*按钮事件*/
    uint8_t         MuitiPress;         /*按钮连按次数*/
    uint16_t        LongPressEvtTime;   /*上次长按事件产生时间*/
    uint16_t        LongPressTime;      /*长按总时间*/
    uint16_t        ActiveTime;         /*按钮有效(按下)时间*/
    uint16_t        InactiveTime;       /*按钮无效(弹起)时间*/
    BOOL            Toggle;             /*按钮值有效性翻转标志(按下->释放 || 释放->按下)*/
}BTN_HANDLE_t;
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
/*入口函数*/
int8_t btn_Init(void);
void btn_Loop_Task(void);
void btn_Timer_Task_1ms(void);
void btn_Test(void);
/*单个按钮接口*/
BTN_STATE_t btn_Get_State(uint8_t Index);
int8_t btn_Clr_Event(uint8_t Index);
int8_t btn_Get_Event(uint8_t Index, BTN_EVEVT_t *Event, uint16_t *argv);
BOOL btn_Match_Event(uint8_t Index, BTN_EVEVT_t Event, uint16_t argv);
/*群组按钮接口*/
BTN_MASK_t btn_Match_GroupState(BTN_MASK_t Mask, BTN_STATE_t State);
BTN_MASK_t btn_Clr_GroupEvent(BTN_MASK_t Mask);
BTN_MASK_t btn_Match_GroupEvent(BTN_MASK_t Mask, BTN_EVEVT_t Event, uint16_t argv);
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
