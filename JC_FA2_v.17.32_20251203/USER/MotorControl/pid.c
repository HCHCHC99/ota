/********************************文件说明*************************************
*文件名: pid.c

*作者: Yuchen Tan

*版本: V1.0.0

*功能简介:
*1.增量式PID控制器;
*2.位置式PID控制器;

*备注:

*修改履历:

*****************************************************************************/

/*****************************文件包含(私有)**********************************
*
*备注: 无
*
*****************************************************************************/
#include "pid.h"
/*****************************宏定义(私有)************************************
*
*备注: 本文件中,不希望被外部使用或随意修改的宏在这里定义
*
*****************************************************************************/
/*默认输出限幅定义*/
#define DEFAULT_OUTPUT_MIN          (0)
#define DEFAULT_OUTPUT_MAX          (500000)

/*默认积分项限幅定义*/
#define DEFAULT_INTERGERAL_MIN      (0)
#define DEFAULT_INTERGERAL_MAX      (100000000)
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

*函数功能描述: 马达控制-电机速度控制-增量式PID控制器

*函数参数: 无

*函数返回值: 无

*备注: 无
*****************************************************************************/
/*增量式PID控制器-句柄初始化*/
void pid_PID_ZL_hInit(PID_ZL_Handle_t *pPID)
{
    pPID->KpGain = 0;
    pPID->KiGain = 0;
    pPID->KdGain = 0;

    pPID->Err[0] = 0;
    pPID->Err[1] = 0;
    pPID->Err[2] = 0;

    pPID->Increment = 0;
    pPID->IncrementDiv = 0;
    pPID->PrevOutput = 0;
    pPID->Output = 0;

    pPID->OutputMax = DEFAULT_OUTPUT_MAX;
    pPID->OutputMin = DEFAULT_OUTPUT_MIN;
}
/*增量式PID控制器-重置控制器*/
void pid_Reset_PID_ZL_Controller(PID_ZL_Handle_t *pPID)
{
    pPID->Err[0] = 0;
    pPID->Err[1] = 0;
    pPID->Err[2] = 0;
    pPID->Increment = 0;
    pPID->PrevOutput = 0;
    pPID->Output = 0;
}
/*增量式PID控制器-设置增量分频比*/
void pid_PID_ZL_Set_IncrementDiv(PID_ZL_Handle_t *pPID, uint16_t IncrementDiv)
{
    pPID->IncrementDiv = IncrementDiv;
}
/*增量式PID控制器-设置Kp*/
void pid_PID_ZL_Set_Kp(PID_ZL_Handle_t *pPID, int32_t KpGain)
{
    pPID->KpGain = KpGain;
}
/*增量式PID控制器-设置Ki*/
void pid_PID_ZL_Set_Ki(PID_ZL_Handle_t *pPID, int32_t KiGain)
{
    pPID->KiGain = KiGain;
}
/*增量式PID控制器-设置Kd*/
void pid_PID_ZL_Set_Kd(PID_ZL_Handle_t *pPID, int32_t KdGain)
{
    pPID->KdGain = KdGain;
}
/*增量式PID控制器-设置KpKiKd*/
void pid_PID_ZL_Set_KpKiKd(PID_ZL_Handle_t *pPID, int32_t KpGain, int32_t KiGain, int32_t KdGain)
{
    pPID->KpGain = KpGain;
    pPID->KiGain = KiGain;
    pPID->KdGain = KdGain;
}
/*增量式PID控制器-设置控制器最大输出*/
void pid_PID_ZL_Set_Output_Max(PID_ZL_Handle_t *pPID, int32_t MaxOutput)
{
    pPID->OutputMax = MaxOutput;
}
/*增量式PID控制器-设置控制器最小输出*/
void pid_PID_ZL_Set_Output_Min(PID_ZL_Handle_t *pPID, int32_t MinOutput)
{
    pPID->OutputMin = MinOutput;
}
/*增量式PID控制器-控制器*/
int32_t pid_PID_ZL_Controller(PID_ZL_Handle_t *pPID, int32_t Error)
{
    int32_t Term_P, Term_I, Term_D = 0; //使用32位数据类型,防止计算溢出
    int32_t Increment = 0;
    int32_t IncrementAbs = 0;
    int32_t Output = 0;
    int32_t Sign = 0;
    /*计算PID输出(增量)*/
    pPID->Err[2] = pPID->Err[1];
    pPID->Err[1] = pPID->Err[0];
    pPID->Err[0] = Error;
    Term_P = (pPID->Err[0] - pPID->Err[1]) * pPID->KpGain;
    Term_I = pPID->Err[0] * pPID->KiGain;
    Term_D = (pPID->Err[2] - 2 * pPID->Err[1] + pPID->Err[0]) * pPID->KdGain;
    Increment = Term_P + Term_I + Term_D;
    /*增量值Div截断处理,并消除截断误差*/
    IncrementAbs = abs(Increment);
#if 0
    Increment >>= pPID->IncrementDiv;
#else
    if( (pPID->Err[0] != 0) && \
        ((IncrementAbs < ((uint32_t)0x01<<pPID->IncrementDiv)) && (IncrementAbs >= ((uint32_t)0x01<<pPID->IncrementDiv) / 2)) )
    {
        Increment = Increment / (int32_t)IncrementAbs;  //输出+1或-1   //Note: 当被除数为负数时,除数不能是uint类型,否则计算结果不是-1!!!!!
    }else
    {
        Increment >>= pPID->IncrementDiv;
    }
#endif
    pPID->Increment = Increment;
    /*计算PID输出(绝对值)*/
    pPID->PrevOutput = pPID->Output;
    Output = pPID->PrevOutput + pPID->Increment;
    /*PID输出限幅*/
    Sign = abs(Output)/Output;
    if(abs(Output) > pPID->OutputMax)
    {
        Output = pPID->OutputMax * Sign;
    }
    if(abs(Output) < pPID->OutputMin)
    {
        Output = pPID->OutputMin * Sign;
    }
    pPID->Output = Output;
    return pPID->Output;
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 马达控制-电机速度控制-位置式PID控制器

*函数参数: 无

*函数返回值: 无

*备注: 无
*****************************************************************************/
/*位置式PID控制器-句柄初始化*/
void pid_PID_WZ_hInit(PID_WZ_Handle_t *pPID)
{
    pPID->KpGain = 0;
    pPID->KiGain = 0;
    pPID->KdGain = 0;

    pPID->Err = 0;
    pPID->PrevErr = 0;

    pPID->IntegralTerm = 0;
    pPID->IntegralTermMax = DEFAULT_INTERGERAL_MAX;
    pPID->IntegralTermMin = DEFAULT_INTERGERAL_MIN;
    pPID->Output = 0;
    pPID->OutputDiv = 0;

    pPID->OutputMax = DEFAULT_OUTPUT_MAX;
    pPID->OutputMin = DEFAULT_OUTPUT_MIN;
}
/*位置式PID控制器-重置控制器*/
void pid_Reset_PID_WZ_Controller(PID_WZ_Handle_t *pPID)
{
    pPID->Err = 0;
    pPID->PrevErr = 0;

    pPID->IntegralTerm = 0;
    pPID->Output = 0;
}
/*位置式PID控制器-设置输出分频比*/
void pid_PID_WZ_Set_OutputDiv(PID_WZ_Handle_t *pPID, uint16_t Div)
{
    pPID->OutputDiv = Div;
}
/*位置式PID控制器-设置Kp*/
void pid_PID_WZ_Set_Kp(PID_WZ_Handle_t *pPID, int32_t KpGain)
{
    pPID->KpGain = KpGain;
}
/*位置式PID控制器-设置Ki*/
void pid_PID_WZ_Set_Ki(PID_WZ_Handle_t *pPID, int32_t KiGain)
{
    pPID->KiGain = KiGain;
}
/*位置式PID控制器-设置Kd*/
void pid_PID_WZ_Set_Kd(PID_WZ_Handle_t *pPID, int32_t KdGain)
{
    pPID->KdGain = KdGain;
}
/*位置式PID控制器-设置KpKiKd*/
void pid_PID_WZ_Set_KpKiKd(PID_WZ_Handle_t *pPID, int32_t KpGain, int32_t KiGain, int32_t KdGain)
{
    pPID->KpGain = KpGain;
    pPID->KiGain = KiGain;
    pPID->KdGain = KdGain;
}
/*位置式PID控制器-设置控制器最大输出*/
void pid_PID_WZ_Set_Output_Max(PID_WZ_Handle_t *pPID, int32_t MaxOutput)
{
    pPID->OutputMax = MaxOutput;
}
/*位置式PID控制器-设置控制器最小输出*/
void pid_PID_WZ_Set_Output_Min(PID_WZ_Handle_t *pPID, int32_t MinOutput)
{
    pPID->OutputMin = MinOutput;
}
/*位置式PID控制器-设置控制器积分项最大值*/
void pid_PID_WZ_Set_IntegralTerm_Max(PID_WZ_Handle_t *pPID, int32_t MaxIntegralTerm)
{
    pPID->IntegralTermMax = MaxIntegralTerm;
}
/*位置式PID控制器-设置控制器积分项最小值*/
void pid_PID_WZ_Set_IntegralTerm_Min(PID_WZ_Handle_t *pPID, int32_t MinIntegralTerm)
{
    pPID->IntegralTermMin = MinIntegralTerm;
}
/*位置式PID控制器-控制器*/
int32_t pid_PID_WZ_Controller(PID_WZ_Handle_t *pPID, int32_t Error)
{
    int32_t Term_P, Term_I, Term_D = 0; //使用32位数据类型,防止计算溢出
    int32_t Output = 0;
    int32_t Sign = 0;

    /*计算PID输出*/
    pPID->PrevErr = pPID->Err;
    pPID->Err = Error;
    Term_P = Error * pPID->KpGain;
    if(pPID->KiGain == 0)
    {
        Term_I = 0;
    }else
    {
        Term_I = Error * pPID->KiGain + pPID->IntegralTerm;
        /*积分项限幅*/
        Sign = Term_I / abs(Term_I);
        if(abs(Term_I) > pPID->IntegralTermMax)
        {
            Term_I = pPID->IntegralTermMax * Sign;
        }
        if(abs(Term_I) < pPID->IntegralTermMin)
        {
            Term_I = pPID->IntegralTermMin * Sign;
        }
    }
    pPID->IntegralTerm = Term_I;
    Term_D = (pPID->Err - pPID->PrevErr) * pPID->KdGain;
    /*计算PID输出(绝对值)*/
    Output = Term_P + Term_I + Term_D;
    Output >>= pPID->OutputDiv;
    /*PID输出限幅*/
    Sign = Output / abs(Output);
    if(abs(Output) > pPID->OutputMax)
    {
        Output = pPID->OutputMax * Sign;
    }
    if(abs(Output) < pPID->OutputMin)
    {
        Output = pPID->OutputMin * Sign;
    }
    pPID->Output = Output;
    return pPID->Output;
}
