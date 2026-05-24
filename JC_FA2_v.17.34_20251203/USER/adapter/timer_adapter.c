/********************************文件说明*************************************
*文件名: timer_adapter.c

*作者: Yuchen Tan

*版本: V1.0.2

*功能简介:

*备注: Todo: 继续补充定时器通道功能(PWM, IC, OC)相关抽象代码

*修改履历:
------------------------------------V1.0.1------------------------------------
20230217:
1.增加接口timer_ResetStart_SW_Timer().
------------------------------------V1.0.2------------------------------------
20230301:
1.增加定时器通道的PWM功能代码.
------------------------------------V1.0.3------------------------------------
20231109:
1.增加hc32f460的timer4互补PWM通道代码.
*****************************************************************************/

/*****************************文件包含(私有)**********************************
*
*备注: 无
*
*****************************************************************************/
#include "timer_adapter.h"
/*****************************宏定义(私有)************************************
*
*备注: 本文件中,不希望被外部使用或随意修改的宏在这里定义
*
*****************************************************************************/
/*MCU-ADC外设相关底层定义*/
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
	#if (MCU_DRIVER_LIB == MCU_DRIVER_LIB_hc32f46x)
	#define IS_VALID_TIMER4(x)				((M4_TMR41 == (x)) || (M4_TMR42 == (x)) || (M4_TMR43 == (x)))
	#define IS_VALID_NORMAL_TIMERA_UNIT(x)	((M4_TMRA1 == (x)) || (M4_TMRA2 == (x)) || (M4_TMRA3 == (x)) || (M4_TMRA4 == (x)) || (M4_TMRA5 == (x)) || (M4_TMRA6 == (x)))
	#endif
#endif
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
SW_TIMER_t  SWTimer[SW_TIMER_NB];
/********************************函数定义************************************
*函数名:

*函数功能描述: TIMER适配文件-外设定时器-基本功能

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*获取外设定时器句柄*/
static TIMER_t* timer_Get_Timer(uint8_t Timer)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    if(Timer == TIMER_0)
    {

    }
#endif
    return 0;
}
/*使能定时器中断*/
void timer_IT_Enable(uint8_t Timer, uint8_t ITEnable)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
    TIM_HandleTypeDef* htim;
    if(Timer == 16)
        htim = &htim16;
    if(ITEnable == TIMER_IT_UPDATE)
    {
        __HAL_TIM_ENABLE_IT(htim,TIM_IT_UPDATE);
        HAL_TIM_Base_Start_IT(htim);
    }
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
#endif
}
/*获取计数器*/
uint32_t timer_Get_Counter(uint8_t Timer)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
    TIM_HandleTypeDef* htim = 0;
    if(Timer == 16)
        htim = &htim16;
    if(Timer)
        return __HAL_TIM_GET_COUNTER(htim);
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
#endif
    return 0;
}
/*计数器清零*/
void timer_Reset(uint8_t Timer)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
    TIM_HandleTypeDef* htim;
    if(Timer == 16)
        htim = &htim16;
    __HAL_TIM_SET_COUNTER(htim, 0);
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
#endif
}
/*计数器开启*/
void timer_Run(uint8_t Timer)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
    TIM_HandleTypeDef* htim;
    if(Timer == 16)
        htim = &htim16;
    HAL_TIM_Base_Start(htim);
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
#endif
}
/*计数器停止*/
void timer_Stop(uint8_t Timer)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
    TIM_HandleTypeDef* htim;
    if(Timer == 16)
        htim = &htim16;
    HAL_TIM_Base_Stop(htim);
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
#endif
}
/********************************函数定义************************************
*函数名:

*函数功能描述: TIMER适配文件-外设定时器-PWM通道功能

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*PWM占空比输出*/
void timer_Ch_PWM_OutPut(TIMER_CHANNAL_t *PWMCh, int32_t Value)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
    __HAL_TIM_SET_COMPARE(PWMCh->TimerIns, PWMCh->Channal, Value);
#if 1   /*HAL库补丁: 同时在中断和mainLoop中调用HAL_TIM_PWM_Start(),有时ChannelState无法从HAL_TIM_CHANNEL_STATE_BUSY变为HAL_TIM_CHANNEL_STATE_READY,导致PWM无法开启!*/
    if (TIM_CHANNEL_STATE_GET(PWMCh->TimerIns, PWMCh->Channal) != HAL_TIM_CHANNEL_STATE_READY)
    {
        TIM_CHANNEL_STATE_SET(PWMCh->TimerIns, PWMCh->Channal, HAL_TIM_CHANNEL_STATE_READY);
    }
#endif
    HAL_TIM_PWM_Start(PWMCh->TimerIns, PWMCh->Channal);
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
    Tim3_M23_CCR_Set(PWMCh->Channal, Value);
    Gpio_SetAfMode(Mosfet->NMosPort, Mosfet->NMosPin, Mosfet->PwmMode); //设置为PWM通道
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
	if(IS_VALID_NORMAL_TIMERA_UNIT((M4_TMRA_TypeDef*)PWMCh->TimerIns))
	{
		TIMERA_SpecifyOutputSta((M4_TMRA_TypeDef*)PWMCh->TimerIns, (en_timera_channel_t)PWMCh->Channal, TimeraSpecifyOutputInvalid);
		TIMERA_SetCompareValue((M4_TMRA_TypeDef*)PWMCh->TimerIns, (en_timera_channel_t)PWMCh->Channal, Value);
		TIMERA_CompareCmd((M4_TMRA_TypeDef*)PWMCh->TimerIns, (en_timera_channel_t)PWMCh->Channal, Enable);
		//PORT_SetFunc(Mosfet->MosPort, Mosfet->MosPin, Mosfet->PwmMode, Disable);	//设置为PWM通道(注:第3个参数说明:Disable-配置为PWM功能, Enable-配置为双周边(副)功能!)
	}else if(IS_VALID_TIMER4((M4_TMR4_TypeDef*)PWMCh->TimerIns))
	{
		//注1: timer4的互补输出H通道(UH,VH,WH)通过设置OCCRxl(x=u,v,w)给占空比,L通道(UL,VL,WL)硬件取反
		//注2: 下面代码也写入了OCCRxh(x=u,v,w)的原因是让OCCRxh和OCCRxl的值不一样,否则无法正确输出互补PWM(肯定是OCMRxl(x=u,v,w)设置不合适导致的,但它太复杂了,我搞不清正确的配置)
		if(PWMCh->Channal % 2)	//Timer4OcoOul || Timer4OcoOvl || Timer4OcoOwl
		{
			TIMER4_OCO_WriteOccr((M4_TMR4_TypeDef*)PWMCh->TimerIns, (en_timer4_oco_ch_t)(PWMCh->Channal - 1), Value + 1); /* Set OCO high channel compare value */
			TIMER4_OCO_WriteOccr((M4_TMR4_TypeDef*)PWMCh->TimerIns, (en_timer4_oco_ch_t)PWMCh->Channal, Value); /* Set OCO low channel compare value */	
//			TIMER4_OCO_OutputCompareCmd((M4_TMR4_TypeDef*)PWMCh->TimerIns, (en_timer4_oco_ch_t)(PWMCh->Channal - 1), Enable);
//			TIMER4_OCO_OutputCompareCmd((M4_TMR4_TypeDef*)PWMCh->TimerIns, (en_timer4_oco_ch_t)PWMCh->Channal, Enable);
		}else		//Timer4OcoOuh || Timer4OcoOvh || Timer4OcoOwh
		{
			TIMER4_OCO_WriteOccr((M4_TMR4_TypeDef*)PWMCh->TimerIns, (en_timer4_oco_ch_t)PWMCh->Channal, Value + 1); /* Set OCO high channel compare value */
			TIMER4_OCO_WriteOccr((M4_TMR4_TypeDef*)PWMCh->TimerIns, (en_timer4_oco_ch_t)(PWMCh->Channal + 1), Value); /* Set OCO low channel compare value */	
//			TIMER4_OCO_OutputCompareCmd((M4_TMR4_TypeDef*)PWMCh->TimerIns, (en_timer4_oco_ch_t)PWMCh->Channal, Enable);
//			TIMER4_OCO_OutputCompareCmd((M4_TMR4_TypeDef*)PWMCh->TimerIns, (en_timer4_oco_ch_t)(PWMCh->Channal + 1), Enable);		
		}
	}
#endif
}
/*PWM占空比输出关闭*/
void timer_Ch_PWM_OutPut_Disable(TIMER_CHANNAL_t *PWMCh)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
#if 0   //调用HAL_TIM_PWM_Stop()后PWM引脚仍然是高电平,改成下面调用HAL_TIM_PWM_Start()
    HAL_TIM_PWM_Stop(PWMCh->TimerIns, PWMCh->Channal);
#else
    __HAL_TIM_SET_COMPARE(PWMCh->TimerIns, PWMCh->Channal, 0);
    #if 1   /*HAL库补丁: 同时在中断和mainLoop中调用HAL_TIM_PWM_Start(),有时ChannelState无法从HAL_TIM_CHANNEL_STATE_BUSY变为HAL_TIM_CHANNEL_STATE_READY,导致PWM无法开启!*/
    if (TIM_CHANNEL_STATE_GET(PWMCh->TimerIns, PWMCh->Channal) != HAL_TIM_CHANNEL_STATE_READY)
    {
        TIM_CHANNEL_STATE_SET(PWMCh->TimerIns, PWMCh->Channal, HAL_TIM_CHANNEL_STATE_READY);
    }
    #endif
    HAL_TIM_PWM_Start(PWMCh->TimerIns, PWMCh->Channal);
#endif
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
    Tim3_M23_CCR_Set(PWMCh->Channal, 0);
    Gpio_ClrIO(Mosfet->NMosPort, Mosfet->NMosPin);
    Gpio_SetAfMode(Mosfet->NMosPort, Mosfet->NMosPin, Mosfet->GpioMode);    //设置为GPIO
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
	if(IS_VALID_NORMAL_TIMERA_UNIT((M4_TMRA_TypeDef*)PWMCh->TimerIns))
	{
		TIMERA_SpecifyOutputSta((M4_TMRA_TypeDef*)PWMCh->TimerIns, (en_timera_channel_t)PWMCh->Channal, TimeraSpecifyOutputLow);
		TIMERA_SetCompareValue((M4_TMRA_TypeDef*)PWMCh->TimerIns, (en_timera_channel_t)PWMCh->Channal, 0);
		//TIMERA_CompareCmd(PWMCh->TimerIns, PWMCh->Channal, Disable);
		//PORT_ResetBits(Mosfet->MosPort, Mosfet->MosPin);
		//PORT_SetFunc(Mosfet->MosPort, Mosfet->MosPin, Mosfet->GpioMode, Enable);    //设置为GPIO
	}else if(IS_VALID_TIMER4((M4_TMR4_TypeDef*)PWMCh->TimerIns))
	{
		//注1: timer4的互补输出H通道(UH,VH,WH)通过设置OCCRxl(x=u,v,w)给占空比,L通道(UL,VL,WL)硬件取反
		//注2: 下面代码也写入了OCCRxh(x=u,v,w)的原因是让OCCRxh和OCCRxl的值不一样,否则无法正确输出互补PWM(肯定是OCMRxl(x=u,v,w)设置不合适导致的,但它太复杂了,我搞不清正确的配置)	
		if(PWMCh->Channal % 2)	//Timer4OcoOul || Timer4OcoOvl || Timer4OcoOwl
		{
			TIMER4_OCO_WriteOccr((M4_TMR4_TypeDef*)PWMCh->TimerIns, (en_timer4_oco_ch_t)(PWMCh->Channal - 1), 1); /* Set OCO high channel compare value */
			TIMER4_OCO_WriteOccr((M4_TMR4_TypeDef*)PWMCh->TimerIns, (en_timer4_oco_ch_t)PWMCh->Channal, 0); /* Set OCO low channel compare value */	
			//TIMER4_OCO_OutputCompareCmd((M4_TMR4_TypeDef*)PWMCh->TimerIns, (en_timer4_oco_ch_t)(PWMCh->Channal - 1), Disable);
			//TIMER4_OCO_OutputCompareCmd((M4_TMR4_TypeDef*)PWMCh->TimerIns, (en_timer4_oco_ch_t)PWMCh->Channal, Disable);
		}else		//Timer4OcoOuh || Timer4OcoOvh || Timer4OcoOwh
		{
			TIMER4_OCO_WriteOccr((M4_TMR4_TypeDef*)PWMCh->TimerIns, (en_timer4_oco_ch_t)PWMCh->Channal, 1); /* Set OCO high channel compare value */
			TIMER4_OCO_WriteOccr((M4_TMR4_TypeDef*)PWMCh->TimerIns, (en_timer4_oco_ch_t)(PWMCh->Channal + 1), 0); /* Set OCO low channel compare value */	
			//TIMER4_OCO_OutputCompareCmd((M4_TMR4_TypeDef*)PWMCh->TimerIns, (en_timer4_oco_ch_t)PWMCh->Channal, Disable);
			//TIMER4_OCO_OutputCompareCmd((M4_TMR4_TypeDef*)PWMCh->TimerIns, (en_timer4_oco_ch_t)(PWMCh->Channal + 1), Disable);		
		}
	}
#endif
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 软件定时器

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*获取定时器句柄*/
static SW_TIMER_t* timer_Get_SW_Timer(uint8_t Nb)
{
    //assert(Nb < SW_TIMER_NB);
    return &SWTimer[Nb];
}
/*使能定时器*/
void timer_Start_SW_Timer(uint8_t Nb)
{
    SW_TIMER_t* Timer = timer_Get_SW_Timer(Nb);
    Timer->TimerEn = 1;
}
/*失能定时器*/
void timer_Stop_SW_Timer(uint8_t Nb)
{
    SW_TIMER_t* Timer = timer_Get_SW_Timer(Nb);
    Timer->TimerEn = 0;
}
/*重置定时器*/
void timer_Reset_SW_Timer(uint8_t Nb)
{
    SW_TIMER_t* Timer = timer_Get_SW_Timer(Nb);
    Timer->TimerCnt = 0;
}
/*终止定时器*/
void timer_Abort_SW_Timer(uint8_t Nb)
{
    SW_TIMER_t* Timer = timer_Get_SW_Timer(Nb);
    Timer->TimerEn = 0;
    Timer->TimerCnt = 0;
    Timer->TimerEvent = 0;
}
/*复位并重新启动定时器*/
void timer_ResetStart_SW_Timer(uint8_t Nb)
{
    SW_TIMER_t* Timer = timer_Get_SW_Timer(Nb);
    Timer->TimerEvent = 0;
    Timer->TimerCnt = 0;
    Timer->TimerEn = 1;
}
/*设置闹钟时间*/
void timer_Set_SW_Timer_AlarmTime(uint8_t Nb, uint16_t AlarmTime_ms)
{
    SW_TIMER_t* Timer = timer_Get_SW_Timer(Nb);
    Timer->AlarmTime = AlarmTime_ms;
}
/*获取事件*/
uint8_t timer_Get_SW_Timer_Event(uint8_t Nb, uint16_t Event)
{
    SW_TIMER_t* Timer = timer_Get_SW_Timer(Nb);
    return (Timer->TimerEvent & Event);
}
/*清除事件*/
void timer_Clr_SW_Timer_Event(uint8_t Nb, uint16_t Event)
{
    SW_TIMER_t* Timer = timer_Get_SW_Timer(Nb);
    Timer->TimerEvent &= (~Event);
}
/*定时器运行,1ms调用一次*/
void timer_SW_Timer_Run(void)
{
    SW_TIMER_t* Timer = 0;

    for(uint8_t Nb = 0; Nb < SW_TIMER_NB; Nb++)
    {
        Timer = timer_Get_SW_Timer(Nb);
        if(Timer->TimerEn)
        {
            /*闹钟功能*/
            if(Timer->TimerCnt < Timer->AlarmTime)
                Timer->TimerCnt++;
            else
                Timer->TimerEvent |= SW_TIMER_EVT_ALARM;
        }
    }
}
