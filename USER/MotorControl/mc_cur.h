/********************************文件说明*************************************
*文件名: mc_cur.h

*作者: Yuchen Tan

*版本: V1.0.2

*功能简介:

*备注: 无

*修改履历:

*****************************************************************************/
#ifndef MC_CUR_H_
#define MC_CUR_H_

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
/*电流采样器参数*/
#define CUR_SMP_WINDOW      (20)    /*电流采样串口宽度(单位:个)*/
#define CUR_SMP_PERIOD      (5)     /*电流单次采样周期(单位:ms)*/

/*过流保护参数*/
#define OVC_SHIELD_TIME     (200)   /*启动延时(单位：ms)*/
/**************************数据类型及结构定义(公开)***************************
*
*备注: 需要被外部使用的数据结构及类型在这里定义
*
*****************************************************************************/
/*电机电流数据处理结构定义*/
typedef struct
{
    /*对应AD采样通道*/
#if 1
    uint8_t     Channel;        /*电流采样对应的ADC通道索引*/
#else
    ADCH_t      ADChannel;      /*ADC通道(与MCU有关的抽象定义)*/
#endif
    /*电流采样*/
    BOOL        SampleEn;       //电流AD采样开启标志
    uint16_t    CurrentADData;  //电流AD采样值
    uint16_t    CurrentMA;      //电流AD采样值->电流mA值
    /*过流保护判断*/
    BOOL        OVCEn;          //过流保护开启标志
    uint16_t    OVCShieldCnt;   //过流保护开启延时计数
    uint16_t    OVCShieldTime;  //过流保护开启延时时间(单位:ms)
    uint16_t    OVCTHHmA;       //过流保护阈值(1=1mA)
    BOOL        OVCFlag;        //过流保护标志
}CUR_Handle_t;
/*****************************函数声明(公开)**********************************
*
*备注: 需要被外部使用的接口函数在这里声明
*
*****************************************************************************/
/*句柄初始化*/
void mc_cur_hInit(CUR_Handle_t *pCur, uint8_t Channel);
/*电流采样*/
void mc_cur_Current_Sample_Enable(CUR_Handle_t *pCur);
void mc_cur_Current_Sample_Disable(CUR_Handle_t *pCur);
void mc_cur_Reset_Current_Sample(CUR_Handle_t *pCur);
void mc_cur_Sample(CUR_Handle_t *pCur);
uint16_t mc_cur_Get_Current_Value(CUR_Handle_t *pCur);
/*过流保护*/
void mc_cur_Set_OVCTHH(CUR_Handle_t *pCur, uint16_t OVCTHHmA);
uint16_t mc_cur_Get_OVCTHH(CUR_Handle_t *pCur);
BOOL mc_cur_Get_OVC_Flag(CUR_Handle_t *pCur);
void mc_cur_Clear_OVC_Flag(CUR_Handle_t *pCur);
void mc_cur_OVC_Enable(CUR_Handle_t *pCur);
void mc_cur_OVC_Disable(CUR_Handle_t *pCur);
void mc_cur_Reset_OVC(CUR_Handle_t *pCur);
void mc_cur_Set_OVC_Shield_Time(CUR_Handle_t *pCur, uint16_t TimeMs);
void mc_cur_OVC_Shield_Timer(CUR_Handle_t *pCur);
void mc_cur_OVC_Protect(CUR_Handle_t *pCur);
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
