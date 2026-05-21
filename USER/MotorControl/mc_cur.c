/********************************文件说明*************************************
*文件名: mc_cur.c

*作者: Yuchen Tan

*版本: V1.0.2

*功能简介:
*1.电机电流采样及数据处理;
*2.电机过流保护功能;

*备注:

*修改履历:
------------------------------------V1.0.1------------------------------------
20220824: 过流保护阈值设置精度提高至1mA,增加读取过流阈值的接口.
20220922: 修改过流保护判断函数,读取电流接口.直接采用电流值(单位: mA)而非AD值.
------------------------------------V1.0.2------------------------------------
20230220: 配合mc_config.h文件的V1.0.2修改,详见mc_config.h修改履历;
*****************************************************************************/

/*****************************文件包含(私有)**********************************
*
*备注: 无
*
*****************************************************************************/
#include "mc_cur.h"
#include "adc_adapter.h"
/*****************************宏定义(私有)************************************
*
*备注: 本文件中,不希望被外部使用或随意修改的宏在这里定义
*
*****************************************************************************/

/**************************数据类型及结构定义(私有)***************************
*
*备注: 本文件中,不希望被外部使用的数据结构及类型在这里定义
*
*****************************************************************************/

/*****************************函数声明(私有)**********************************
*
*备注: 本文件中,不希望被外部调用的函数统一在这里声明
*
*****************************************************************************/

/********************************变量定义*************************************
*
*备注: 不要在.h中定义变量,防止多个.c包含这个.h,导致编译后报错变量重复定义
*
*****************************************************************************/

/********************************函数定义************************************
*函数名:

*函数功能描述: 电机电流-电机电流-句柄初始化

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
void mc_cur_hInit(CUR_Handle_t *pCur, uint8_t Channel)
{
    pCur->Channel = Channel;

    pCur->SampleEn = FALSE;
    pCur->CurrentADData = 0;
    pCur->CurrentMA = 0;

    pCur->OVCEn = FALSE;
    pCur->OVCShieldCnt = 0;
    pCur->OVCShieldTime = 0;
    pCur->OVCTHHmA = 0;
    pCur->OVCFlag = FALSE;
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 马达控制-电机电流-电流采样

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*使能电流采样*/
void mc_cur_Current_Sample_Enable(CUR_Handle_t *pCur)
{
    pCur->SampleEn = TRUE;
    adc_adapter_Channel_Enable(pCur->Channel);
}
/*禁止电流采样*/
void mc_cur_Current_Sample_Disable(CUR_Handle_t *pCur)
{
    pCur->SampleEn = FALSE;
    adc_adapter_Channel_Disable(pCur->Channel);
}
/*重置电流采样*/
void mc_cur_Reset_Current_Sample(CUR_Handle_t *pCur)
{
    pCur->CurrentADData = 0;
    pCur->CurrentMA = 0;
}
/*电流采样控制*/
void mc_cur_Sample(CUR_Handle_t *pCur)
{
    float   Voltage_mV = 0.0;   //AD值对应电压(mV)

    if(1 == adc_adapter_SCM_1Ch_Convert(pCur->Channel))
    {
        pCur->CurrentADData = adc_adapter_Get_Channel_Result(pCur->Channel);
        Voltage_mV = (float)pCur->CurrentADData * ADC_REF_VOLTAGE * 1000 / ADC_FULL_SCALE;
        pCur->CurrentMA = (float)(Voltage_mV / (SMP_R * OA_MULT));
    }
}
/*获取平均电流(注：第1轮采集完成前，返回值为0)*/
uint16_t mc_cur_Get_Current_Value(CUR_Handle_t *pCur)
{
    return pCur->CurrentMA;
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 马达控制-电机电流-过流保护

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*设置过流保护阈值(mA)*/
void mc_cur_Set_OVCTHH(CUR_Handle_t *pCur, uint16_t OVCTHHmA)
{
    pCur->OVCTHHmA = OVCTHHmA;
}
/*获取过流保护阈值(mA)*/
uint16_t mc_cur_Get_OVCTHH(CUR_Handle_t *pCur)
{
    return pCur->OVCTHHmA;
}
/*获取过流保护标志位*/
BOOL mc_cur_Get_OVC_Flag(CUR_Handle_t *pCur)
{
    return pCur->OVCFlag;
}
/*清除过流保护标志位*/
void mc_cur_Clear_OVC_Flag(CUR_Handle_t *pCur)
{
    pCur->OVCFlag = FALSE;
}
/*使能过流保护功能*/
void mc_cur_OVC_Enable(CUR_Handle_t *pCur)
{
    pCur->OVCEn = TRUE;
}
/*关闭过流保护功能*/
void mc_cur_OVC_Disable(CUR_Handle_t *pCur)
{
    pCur->OVCEn = FALSE;
}
/*重置过流保护监测*/
void mc_cur_Reset_OVC(CUR_Handle_t *pCur)
{
    pCur->OVCShieldCnt = 0;
    pCur->OVCFlag = FALSE;
}
/*设置过流保护功能使能后的延时时间*/
void mc_cur_Set_OVC_Shield_Time(CUR_Handle_t *pCur, uint16_t TimeMs)
{
    if(TimeMs < 65000)
    {
        pCur->OVCShieldTime = TimeMs;
    }
}
/*过流保护功能使能后的延时计数(1ms调用一次)*/
void mc_cur_OVC_Shield_Timer(CUR_Handle_t *pCur)
{
    if(pCur->OVCEn == TRUE)
    {
        if(pCur->OVCShieldCnt < 60000)
            pCur->OVCShieldCnt++;
    }else
    {
        pCur->OVCShieldCnt = 0;
    }
}
/*过流判断*/
void mc_cur_OVC_Protect(CUR_Handle_t *pCur)
{
    if(pCur->OVCShieldCnt >= pCur->OVCShieldTime)
    {
        if(pCur->CurrentMA >= pCur->OVCTHHmA)
        {
            pCur->OVCFlag = TRUE;
        }
    }
}
