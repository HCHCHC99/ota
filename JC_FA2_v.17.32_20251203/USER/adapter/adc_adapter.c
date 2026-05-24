/********************************文件说明*************************************
*文件名: adc_adapter.c

*作者: Yuchen Tan

*版本: V1.0.5

*功能简介: ADC适配器(介于应用功能和MCU的ADC底层驱动之间的中间层)

*备注:
Todo: 1.通道的每组采样值窗口改为步进平移式(可提高过流保护等异常的检测速率)!

*修改履历:
------------------------------------V1.0.1------------------------------------
*20220328: 将不同MCU的ADC外设操作进行抽象封装.
------------------------------------V1.0.2------------------------------------
*20220530: 补充适配HC32L130.
------------------------------------V1.0.3------------------------------------
*20220623: adc_adapter_hInit()接口的结构体参数ADCH_t改为指针ADCH_t*.
------------------------------------V1.0.4------------------------------------
*20220705: adc操作的一系列抽象函数增加STM32的驱动.
*20220712: 删除代码重复错误,修改适配不同MCU的预编译命令书写错误.
*20220713: 修改STM32的AD通道数MCU_AD_CHANNEL_NB定义.
16(外部通道总数)改为19(总通道数,3个内部通道),防止使用ADC_IN16,ADC_IN17,ADC_IN18
这3个外部通道导致数组hADCChannel[MCU_AD_CHANNEL_NB]越界访问导致硬错误中断!
(因为hADCChannel[Channel]->ADChannel.Port是空指针)
*20220921: 修改adc_adapter_SampleInterval_Timer(),去掉参数uint8_t Channel,改为
内部轮训hADCChannel[MCU_AD_CHANNEL_NB],对每个非空成员进行相应操作.
目的:应用层增加AD通道不需要在1ms中断中调用adc_adapter_SampleInterval_Timer(Channel).
------------------------------------V1.0.5------------------------------------
*20230525: 增加宏定义适配hc32f46x驱动库的代码(之前的驱动库是hc32f460,有一点点区别).
*****************************************************************************/

/*****************************文件包含(私有)**********************************
*
*备注: 无
*
*****************************************************************************/
#include "adc_adapter.h"
#include "main.h"
#include "hz_timer.h"
#include "ring_buffer.h"



/*****************************宏定义(私有)************************************
*
*备注: 本文件中,不希望被外部使用或随意修改的宏在这里定义
*
*****************************************************************************/
/*MCU-ADC外设相关底层定义*/
#if (MCU_TYPE == MCU_TYPE_STM32)
//注1: STM32的ADC-频率/采样时间/转换时间    \
<<STM32G070CB-datasheet>>-5.3.17: fADC最大值=35MHz.需要根据实际的主频和时钟源选择,合理设置ADC的时钟分频.\
<<STM32G0x0-Reference Manual>>-14.3.9: 采样周期: 1.5 / 3.5 / 7.5 / 12.5 / 19.5 / 39.5 / 79.5 / 160.5Cycles \
<<STM32G0x0-Reference Manual>>-14.3.9: 转换时间: (12.5Cycles + 采样周期) * f_ADC_Clock. \
采样周期的选择需实际调试,太小会导致采样值偏小(采样保持电路充电时间不够)!
//注2: STM32的ADC-单次转换/连续转换   \
<<STM32G0x0-Reference Manual>>-14.3.10: 单次转换：启动->单个序列所有通道采样1次(每个通道转换完成置位EOC flag,所有通道转换完成置位EOS flag)->结束\
<<STM32G0x0-Reference Manual>>-14.3.11: 连续转换：启动->单个序列所有通道采样1次(每个通道转换完成置位EOC flag,所有通道转换完成置位EOS flag)->重启.
//注3: STM32的ADC-通道选择(Sequencer not fully configurable / Sequencer fully configurable)   \
<<STM32G0x0-Reference Manual>>-14.3.8: not fully: 通道的扫描顺序: 由ADC_CHSELR(14.12.9)寄存器选中的通道按由低到高/由高到低的顺序依次转换    \
<<STM32G0x0-Reference Manual>>-14.3.8: fully: 通道的扫描顺序: 由ADC_CHSELR(14.12.10)寄存器配置任意个通道(最多8个)按任意顺序转换.
//注4: STM32的ADC-校准  \
<<STM32G0x0-Reference Manual>>-14.3.3: \
校准目的: 消除芯片制程差异导致的Offset Error \
校准: 转换开始前校准 / 校准过程中禁止使用ADC / 通过ADC_CALFACT[6:0]获取或修改校准因子(14.12.15) / VDDA(主要)或温度变化建议重新校准    \
校准因子丢失: ADC外设复位 / ADC掉电(进入STANDBY/VBAT模式/MCU断电)

#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)

#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
#define ADC1_SA_CHANNEL_SAMPLE_TIME (0x50)

#endif

/*ADC通道切换控制参数*/
#define SWITCH_PIN_MODE_TO_GPIO     (1)     //1:失能AD通道变为数字端口  0:失能AD通道仍为模拟端口
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
/*ADC通道控制句柄数组*/
/*定义结构体实例的指针(句柄)数组和直接在本模块内部定义结构体实例数组相比,有以下好处：
1.因为不知道总共需要用多少个通道,直接在本模块定义MCU_AD_CHANNEL_NB个结构体实例太浪费RAM;若是在其他文件
    分别按需定义结构体实例又不方便统一索引;
2.定义句柄数组后,可在其他模块按需进行实例定义并将其与本模块的数组成员关联(统一使用Channal编号进行句柄索引)*/
ADC_CH_CTRL_t*  hADCChannel[MCU_AD_CHANNEL_NB];

uint8_t     g_TestADCChannalCmd = 0;

uint16_t    g_STM32ADCCalibrationFactor = 0;


/********************************函数定义************************************
*函数名:

*函数功能描述: ADC适配模块-句柄初始化

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
void adc_adapter_hInit(ADC_CH_CTRL_t* pCHCtrl, ADCH_t* ADChannel)
{
    if(ADChannel->Channel >= MCU_AD_CHANNEL_NB)
    {
        return;
    }
    /*将Channel与数组元素(对应1个句柄)绑定,实现通过Channel访问对应的结构体实例*/
    hADCChannel[ADChannel->Channel] = pCHCtrl;
    /*句柄初始化*/
    pCHCtrl->ADChannel = *ADChannel;
    pCHCtrl->ConvtEn = FALSE;
    pCHCtrl->State = E_CONVERT_IDLE;
    pCHCtrl->SampleTime = 0;
    pCHCtrl->SampledTime = 0;
    pCHCtrl->SampleIntervalTimer = 0;
    pCHCtrl->SampleIntervalTime = 0;
    pCHCtrl->ResultOnce = 0;
    pCHCtrl->ResultTotal = 0;
    pCHCtrl->ResultAverage = 0;
}
/********************************函数定义************************************
*函数名:

*函数功能描述: ADC适配模块-封装部分MCU驱动库的函数,供本模块抽象函数使用

*函数参数:
@ Mode: 0-设置为GPIO  1-设置为模拟引脚

*函数返回值: 无

*备注:
*****************************************************************************/
#if (MCU_TYPE == MCU_TYPE_STM32)
//不需要
#elif (MCU_TYPE == MCU_TYPE_HC32_F0)
//不需要
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
/**
 *******************************************************************************
 ** \brief  Config the pin which is mapping the channel to analog or digit mode.
 **
 ******************************************************************************/
static void AdcSetPinMode(uint8_t u8AdcPin, en_pin_mode_t enMode)
{
    en_port_t enPort = PortA;
    en_pin_t enPin   = Pin00;
    bool bFlag       = true;
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enPinMode = enMode;
    stcPortInit.enPullUp  = Disable;

    switch (u8AdcPin)
    {
        case ADC1_IN0:
            enPort = PortA;
            enPin  = Pin00;
            break;

        case ADC1_IN1:
            enPort = PortA;
            enPin  = Pin01;
            break;

        case ADC1_IN2:
            enPort = PortA;
            enPin  = Pin02;
            break;

        case ADC1_IN3:
            enPort = PortA;
            enPin  = Pin03;
            break;

        case ADC12_IN4:
            enPort = PortA;
            enPin  = Pin04;
            break;

        case ADC12_IN5:
            enPort = PortA;
            enPin  = Pin05;
            break;

        case ADC12_IN6:
            enPort = PortA;
            enPin  = Pin06;
            break;

        case ADC12_IN7:
            enPort = PortA;
            enPin  = Pin07;
            break;

        case ADC12_IN8:
            enPort = PortB;
            enPin  = Pin00;
            break;

        case ADC12_IN9:
            enPort = PortB;
            enPin  = Pin01;
            break;

        case ADC12_IN10:
            enPort = PortC;
            enPin  = Pin00;
            break;

        case ADC12_IN11:
            enPort = PortC;
            enPin  = Pin01;
            break;

        case ADC1_IN12:
            enPort = PortC;
            enPin  = Pin02;
            break;

        case ADC1_IN13:
            enPort = PortC;
            enPin  = Pin03;
            break;

        case ADC1_IN14:
            enPort = PortC;
            enPin  = Pin04;
            break;

        case ADC1_IN15:
            enPort = PortC;
            enPin  = Pin05;
            break;

        default:
            bFlag = false;
            break;
    }
    if (true == bFlag)
    {
        PORT_Init(enPort, enPin, &stcPortInit);
    }
}
/**
 *******************************************************************************
 ** \brief  Set an ADC pin as analog input mode or digit mode.
 **
 ******************************************************************************/
static void AdcSetChannelPinMode(const M4_ADC_TypeDef *ADCx, uint32_t u32Channel, en_pin_mode_t enMode)
{
    uint8_t u8ChIndex;
#if (ADC_CH_REMAP)
    uint8_t u8AdcPin;
#else
    uint8_t u8ChOffset = 0u;
#endif
    if (M4_ADC1 == ADCx)
    {
        u32Channel &= ADC1_PIN_MASK_ALL;
    }else
    {
        u32Channel &= ADC2_PIN_MASK_ALL;
    #if (!ADC_CH_REMAP)
        u8ChOffset = 4u;
    #endif
    }
    u8ChIndex = 0u;
    while (0u != u32Channel)
    {
        if (u32Channel & 0x1ul)
        {
    #if (ADC_CH_REMAP)
            u8AdcPin = ADC_GetChannelPinNum(ADCx, u8ChIndex);
            AdcSetPinMode(u8AdcPin, enMode);
    #else
            AdcSetPinMode((u8ChIndex + u8ChOffset), enMode);
    #endif
        }
        u32Channel >>= 1u;
        u8ChIndex++;
    }
}
#endif
/********************************函数定义************************************
*函数名:

*函数功能描述: ADC适配模块-MCU的ADC外设操作(抽象封装)

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*ADC校准*/
void adc_adapter_Calibration(void)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
    HAL_ADCEx_Calibration_Start(&hadc1);
    g_STM32ADCCalibrationFactor = HAL_ADCEx_Calibration_GetValue(&hadc1);
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
//不需要
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
//不需要
#endif
}
/*设置引脚为模拟or数字模式*/
static void adc_adapter_Set_ADCH_Mode(ADCH_t ADCH, uint8_t Mode)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
    GPIO_InitTypeDef    GPIO_InitStruct = {0};
    if(Mode)
    {
        GPIO_InitStruct.Pin = ADCH.Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(ADCH.Port, &GPIO_InitStruct);
    }else
    {
        GPIO_InitStruct.Pin = ADCH.Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(ADCH.Port, &GPIO_InitStruct);
    }
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
    if(Mode)
    {
        Gpio_SetAnalogMode(ADCH.Port, ADCH.Pin);
    }else
    {
        Gpio_SetDigitalMode(ADCH.Port, ADCH.Pin);
    }
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    if(Mode)
    {
        AdcSetChannelPinMode(ADCH.ADCx, (uint32_t)(0x1<<ADCH.Channel), Pin_Mode_Ana);
    }else
    {
        AdcSetChannelPinMode(ADCH.ADCx, (uint32_t)(0x1<<ADCH.Channel), Pin_Mode_In);
    }
#endif
}
/*启动单通道转换*/
static void adc_adapter_Start_1Ch_Conversion(ADCH_t ADCH)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
    LL_ADC_REG_SetSequencerChannels(ADCH.hadc->Instance, (0x01<<ADCH.Channel) );    //< ADC 采样通道配置
    HAL_ADC_Start(ADCH.hadc);           // 启动AD采样转换
#elif (MCU_TYPE == MCU_TYPE_HC32_F0)
    Adc_ConfigSglChannel(ADCH.Channel); ///< ADC 采样通道配置
    Adc_ClrSglIrqState();
    Adc_SGL_Start();                    ///< 启动单次转换采样
#elif (MCU_TYPE == MCU_TYPE_HC32_L1)
    Adc_CfgSglChannel(ADCH.Channel);    ///< ADC 采样通道配置
    Adc_ClrIrqStatus(AdcMskIrqSgl);
    Adc_SGL_Start();                    ///< 启动单次转换采样
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    stc_adc_ch_cfg_t stcChCfg;
    uint8_t au8Adc1SaSampTime = ADC1_SA_CHANNEL_SAMPLE_TIME;

    MEM_ZERO_STRUCT(stcChCfg);

    stcChCfg.u32Channel  = (uint32_t)(0x1<<ADCH.Channel);
    stcChCfg.u8Sequence  = ADC_SEQ_A;
    stcChCfg.pu8SampTime = &au8Adc1SaSampTime;
    ADC_AddAdcChannel(ADCH.ADCx, &stcChCfg);
#endif
}
/*等待单通道转换完成*/
static void adc_adapter_Wait_1Ch_Conversion(ADCH_t ADCH)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
    HAL_StatusTypeDef Res = HAL_ADC_PollForConversion(ADCH.hadc, 2);
    //HAL_ADC_Stop(ADCH.hadc);      //HAL_ADC_Stop()会失能ADC(ADDIS=0)导致校准因子丢失,因此不调用!
#elif (MCU_TYPE == MCU_TYPE_HC32_F0)
    stc_adc_irq_t   ADCIrq;
    ADCIrq.bAdcIrq = FALSE;
    while(ADCIrq.bAdcIrq == FALSE)
    {
        Adc_GetIrqState(&ADCIrq);
    }
    Adc_ClrSglIrqState();
    //Adc_SGL_Stop();
#elif (MCU_TYPE == MCU_TYPE_HC32_L1)
    while(Adc_GetIrqStatus(AdcMskIrqSgl) == FALSE)
    {

    }
    Adc_ClrIrqStatus(AdcMskIrqSgl);
    //Adc_SGL_Stop();
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    //根据驱动库提供的接口,转换和结果提取都放到adc_adapter_Get_1Ch_Conversion_Result()中.
#endif
}
/*获取单通道转换完成结果*/
static void adc_adapter_Get_1Ch_Conversion_Result(ADCH_t ADCH, uint16_t *Result)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
    *Result = HAL_ADC_GetValue(ADCH.hadc);
#elif (MCU_TYPE == MCU_TYPE_HC32_F0)
    Adc_GetSglResult(Result);
#elif (MCU_TYPE == MCU_TYPE_HC32_L1)
    *Result = Adc_GetSglResult();
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    ADC_PollingSa(ADCH.ADCx, Result, 1, 10);
	#if (MCU_DRIVER_LIB == MCU_DRIVER_LIB_hc32f460)	//驱动库是hc32f460_adc
	ADC_DelAdcChannel(ADCH.ADCx, (uint32_t)(0x1<<ADCH.Channel));    //每次只进行单通道转换,转换前添加通道,转换完成后删除通道.
	#else 		//驱动库是hc32f46x_adc
	stc_adc_ch_cfg_t stcChCfg = {.u32Channel = (uint32_t)(0x1<<ADCH.Channel), .u8Sequence = ADC_SEQ_A};
	ADC_DelAdcChannel(ADCH.ADCx, &stcChCfg);    //每次只进行单通道转换,转换前添加通道,转换完成后删除通道.
	#endif
#endif
}
/********************************函数定义************************************
*函数名:

*函数功能描述: ADC适配模块-ADC通道单次转换控制

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*根据AD通道值,获取通道控制句柄*/
static ADC_CH_CTRL_t* adc_adapter_Get_Handle(uint8_t Channel)
{
    uint8_t index;

    for(index=0; index<MCU_AD_CHANNEL_NB; index++)
    {
        if(hADCChannel[index]->ADChannel.Channel == Channel)
        {
            return (hADCChannel[index]);
        }
    }
    return NULL;
}
/*使能通道转换*/
void adc_adapter_Channel_Enable(uint8_t Channel)
{
    ADC_CH_CTRL_t*  pCHCtrl = adc_adapter_Get_Handle(Channel);

    pCHCtrl->ConvtEn = TRUE;
#if (SWITCH_PIN_MODE_TO_GPIO == 1)  //Todo: 每个端口使能/失能AD转换时,端口模式是否转换可选
    adc_adapter_Set_ADCH_Mode(pCHCtrl->ADChannel, 1);
#endif
}
/*关闭通道转换*/
void adc_adapter_Channel_Disable(uint8_t Channel)
{
    ADC_CH_CTRL_t*  pCHCtrl = adc_adapter_Get_Handle(Channel);

    pCHCtrl->ConvtEn = FALSE;
#if (SWITCH_PIN_MODE_TO_GPIO == 1)  //Todo: 每个端口使能/失能AD转换时,端口模式是否转换可选
    adc_adapter_Set_ADCH_Mode(pCHCtrl->ADChannel, 0);
#endif
}
/*重置通道采样*/
static void adc_adapter_Reset_Channal_Convert(ADC_CH_CTRL_t* pCHCtrl)
{
    pCHCtrl->State = E_CONVERT_IDLE;

    pCHCtrl->SampledTime = 0;
    pCHCtrl->SampleIntervalTimer = 0;   /*AD通道采样间隔控制计数器(1ms自加)*/

    pCHCtrl->ResultOnce = 0;
    pCHCtrl->ResultTotal = 0;
    //pCHCtrl->ResultAverage = 0;
}
/*通道采样间隔控制计数器*/
void adc_adapter_SampleInterval_Timer(void)
{
    ADC_CH_CTRL_t*  pCHCtrl;

    for(uint8_t i = 0; i < MCU_AD_CHANNEL_NB; i++)
    {
        pCHCtrl = hADCChannel[i];
        if(pCHCtrl != 0 && pCHCtrl->ConvtEn == TRUE)
        {
            pCHCtrl->SampleIntervalTimer++;
            pCHCtrl->SampledTime++;
        }
    }
}
/*设置通道采样次数*/
void adc_adapter_Set_Channal_SmpTime(uint8_t Channel, uint16_t SampleTime)
{
    ADC_CH_CTRL_t*  pCHCtrl = adc_adapter_Get_Handle(Channel);

    pCHCtrl->SampleTime = SampleTime;
}
/*设置通道采样次数间隔时间*/
void adc_adapter_Set_Channal_SmpIntervalTime(uint8_t Channel, uint16_t SampleIntervalTime)
{
    ADC_CH_CTRL_t*  pCHCtrl = adc_adapter_Get_Handle(Channel);

    pCHCtrl->SampleIntervalTime = SampleIntervalTime;
}
/*获取通道转换值(多次之和的平均值)*/
uint16_t adc_adapter_Get_Channel_Result(uint8_t Channel)
{
    ADC_CH_CTRL_t*  pCHCtrl = adc_adapter_Get_Handle(Channel);

    return (pCHCtrl->ResultAverage);
}
/*单通道单次转换模式状态机(Todo: TimeOut功能)*/
/*函数返回值: 0-转换中 1-转换完成*/
static int adc_adapter_SCM_1CH_Convert_StateMachine(ADC_CH_CTRL_t *pCHCtrl, uint32_t TimeOut)
{
    int8_t  Res = 0;

    switch(pCHCtrl->State)
    {
        case E_CONVERT_IDLE:
            pCHCtrl->State = E_CONVERT_START;
            break;

        case E_CONVERT_START:
            adc_adapter_Start_1Ch_Conversion(pCHCtrl->ADChannel);
            pCHCtrl->State = E_CONVERTING;
            break;

        case E_CONVERTING:
            adc_adapter_Wait_1Ch_Conversion(pCHCtrl->ADChannel);
            pCHCtrl->State = E_CONVERT_CPLT;
            break;

        case E_CONVERT_CPLT:
            adc_adapter_Get_1Ch_Conversion_Result(pCHCtrl->ADChannel, (uint16_t*)&pCHCtrl->ResultOnce);
            Res = 1;
            pCHCtrl->State = E_CONVERT_IDLE;
            break;

        default:
            break;
    }
    return Res;
}
/*单通道转换处理APP*/
int8_t adc_adapter_SCM_1Ch_Convert(uint8_t Channel)
{

    uint64_t currentTime = tickTimer_GetCount();
    ADC_CH_CTRL_t*  pCHCtrl = adc_adapter_Get_Handle(Channel);
    int8_t  Res = 0;

    if(pCHCtrl->ConvtEn == FALSE)
    {
        adc_adapter_Reset_Channal_Convert(pCHCtrl);
        return -1;
    }

    if(pCHCtrl->SampleIntervalTimer >= pCHCtrl->SampleIntervalTime)
    {
        pCHCtrl->SampleIntervalTimer = 0;
        /*单通道采样*/
        do {Res = adc_adapter_SCM_1CH_Convert_StateMachine(pCHCtrl, 0);}
        while (Res == 0);
        if(Res == 1)
        {
            pCHCtrl->ResultTotal += pCHCtrl->ResultOnce;
            pCHCtrl->SampledTime++;
        }
        if(pCHCtrl->SampledTime >= pCHCtrl->SampleTime)
        {
            pCHCtrl->ResultAverage = pCHCtrl->ResultTotal / pCHCtrl->SampleTime;
            adc_adapter_Reset_Channal_Convert(pCHCtrl);
            return 1;
        }
    }
    return 0;
}
/********************************函数定义************************************
*函数名:

*函数功能描述: ADC适配模块-模块功能测试

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
void adc_adapter_Test(uint8_t Channel)
{
//    static uint16_t ADResult = 0;

    if(g_TestADCChannalCmd == 1)
    {
        adc_adapter_Set_Channal_SmpTime(Channel, 10);
        g_TestADCChannalCmd = 0;
    }
    if(g_TestADCChannalCmd == 2)
    {
        adc_adapter_Set_Channal_SmpIntervalTime(Channel, 2);
        g_TestADCChannalCmd = 0;
    }
    if(g_TestADCChannalCmd == 3)
    {
        if(adc_adapter_SCM_1Ch_Convert(Channel) == 1)
        {
//            ADResult = adc_adapter_Get_Channel_Result(Channel);
            g_TestADCChannalCmd = 0;
        }
    }
    /*可用于正在工作的ADC通道,测试ADC通道被使能or失能时,采样是否正常*/
    if(g_TestADCChannalCmd == 4)
    {
        adc_adapter_Channel_Enable(Channel);
        g_TestADCChannalCmd = 0;
    }
    if(g_TestADCChannalCmd == 5)
    {
        adc_adapter_Channel_Disable(Channel);
        g_TestADCChannalCmd = 0;
    }
}

/* 单通道ADC转换处理模块 */

/**
 * @brief 单通道ADC转换处理（电压测量）
 * @param Channel ADC通道号
 * @return 转换状态：1-完成，0-未完成，-1-禁用
 */
int8_t hz_adc_adapter_SCM_1Ch_Convert_VM(uint8_t Channel)
{
    ADC_CH_CTRL_t* pCHCtrl = adc_adapter_Get_Handle(Channel);
    int8_t Res = 0;
    
    if (pCHCtrl->ConvtEn == FALSE) {
        adc_adapter_Reset_Channal_Convert(pCHCtrl);
        return -1;
    }

    if (pCHCtrl->SampleIntervalTimer >= pCHCtrl->SampleIntervalTime) {
        pCHCtrl->SampleIntervalTimer = 0;
        
        // 等待单次采样完成
        do {
            Res = adc_adapter_SCM_1CH_Convert_StateMachine(pCHCtrl, 0);
        } while (Res == 0);
        
        // 采样成功时，累加采样值
        if (Res == 1) {
            adc_adapter_Add_Sample(Channel, pCHCtrl->ResultOnce);
            
            if(pCHCtrl->SampledTime>=pCHCtrl->SampleTime)
            {
                pCHCtrl->SampledTime = 0;
                
                return true;
            }
        }
        
        return false;
    }
    
    return false;
}
/**
 * @brief 添加采样值到统计缓冲区
 * @param Channel ADC通道号
 * @param sample_value 采样值
 */
void adc_adapter_Add_Sample(uint8_t Channel, uint16_t sample_value)
{
    // 获取当前通道的控制结构体
    ADC_CH_CTRL_t* pCHCtrl = adc_adapter_Get_Handle(Channel);
    if (pCHCtrl == NULL) return;  // 无效通道则返回
    
    // 将采样值存入当前通道的专属缓冲区（循环存储）
    if (pCHCtrl->buffer_index < SAMPLE_BUFFER_SIZE) {
        pCHCtrl->sample_buffer[pCHCtrl->buffer_index++] = sample_value;
    } else {
        // 缓冲区满了，从头开始覆盖
        pCHCtrl->buffer_index = 0;
        pCHCtrl->sample_buffer[pCHCtrl->buffer_index++] = sample_value;
    }

    // 为每个通道添加独立的统计变量（需要在ADC_CH_CTRL_t中定义）
    if (pCHCtrl->first_sample) {
        // 第一次采样，初始化最大值和最小值
        pCHCtrl->sample_max = sample_value;
        pCHCtrl->sample_min = sample_value;
        pCHCtrl->sample_total = sample_value;
        pCHCtrl->sample_count = 1;
        pCHCtrl->first_sample = false;
    } else {
        // 更新最大/最小值
        if (sample_value > pCHCtrl->sample_max) {
            pCHCtrl->sample_max = sample_value;
        }
        if (sample_value < pCHCtrl->sample_min) {
            pCHCtrl->sample_min = sample_value;
        }
        
        // 累加总和
        pCHCtrl->sample_total += sample_value;
        pCHCtrl->sample_count++;
    }
}

/**
 * @brief 获取采样缓冲区数据
 * @param Channel ADC通道号
 * @param buffer 输出缓冲区
 * @param count 实际数据个数
 * @param max_size 缓冲区最大容量
 */
void adc_adapter_Get_Sample_Buffer(uint8_t Channel, uint16_t* buffer, uint32_t* count, uint32_t max_size)
{
    if (buffer == NULL || count == NULL) return;
    
    // 获取当前通道的控制结构体（已有的获取句柄函数）
    ADC_CH_CTRL_t* pCHCtrl = adc_adapter_Get_Handle(Channel);
    if (pCHCtrl == NULL) {  // 增加空指针保护
        *count = 0;
        return;
    }
    
    // 使用当前通道的buffer_index，避免全局冲突
    *count = (pCHCtrl->buffer_index < max_size) ? pCHCtrl->buffer_index : max_size;
    for (uint32_t i = 0; i < *count; i++) {
        buffer[i] = pCHCtrl->sample_buffer[i];  // 复制通道专属缓冲区数据
    }
}
/**
 * @brief 重置采样缓冲区
 * @param Channel ADC通道号
 */
void adc_adapter_Reset_Sample_Buffer(uint8_t Channel)
{
    ADC_CH_CTRL_t* pCHCtrl = adc_adapter_Get_Handle(Channel);
    if (pCHCtrl == NULL) return;
    
    pCHCtrl->buffer_index = 0;  // 重置通道专属索引
    memset(pCHCtrl->sample_buffer, 0, sizeof(pCHCtrl->sample_buffer));  // 清空通道专属缓冲区

}
/**
 * @brief 重置采样统计信息
 * @param Channel ADC通道号
 */
void adc_adapter_Reset_Sampling_Stats(uint8_t Channel)
{
    ADC_CH_CTRL_t* pCHCtrl = adc_adapter_Get_Handle(Channel);
    if (pCHCtrl == NULL) return;
    
    pCHCtrl->sample_count = 0;
    pCHCtrl->sample_total = 0;
    pCHCtrl->sample_max = 0;
    pCHCtrl->sample_min = 0xFFFF;
    pCHCtrl->first_sample = true;
    // 同时重置该通道的缓冲区
    adc_adapter_Reset_Sample_Buffer(Channel);
}

/**
 * @brief 获取去极值后的采样平均值
 * @param Channel ADC通道号
 * @return 平均值
 */
uint16_t adc_adapter_Get_Sampling_Average(uint8_t Channel)
{
    ADC_CH_CTRL_t* pCHCtrl = adc_adapter_Get_Handle(Channel);
    if (pCHCtrl == NULL) return 0;
    
    if (pCHCtrl->sample_count == 0) {
        return 0;
    }
    
    if (pCHCtrl->sample_count <= 2) {
        // 采样点数不足，直接计算平均值
        return pCHCtrl->sample_total / pCHCtrl->sample_count;
    }
    
    // 减去最大最小值后计算平均值
    uint32_t filtered_total = pCHCtrl->sample_total - pCHCtrl->sample_max - pCHCtrl->sample_min;
    uint32_t filtered_count = pCHCtrl->sample_count - 2;
    
    return filtered_total / filtered_count;
}

/**
 * @brief 单通道ADC转换处理（电流测量）
 * @param Channel ADC通道号
 * @return 转换状态：1-完成，0-未达到采样个数，-1-未到采样间隔， -2-禁用
 */
int8_t hz_adc_adapter_SCM_1Ch_Convert_CUR(uint8_t Channel)
{
    ADC_CH_CTRL_t* pCHCtrl = adc_adapter_Get_Handle(Channel);
    int8_t Res = 0;
    
    if (pCHCtrl->ConvtEn == FALSE) {
        adc_adapter_Reset_Channal_Convert(pCHCtrl);
        return -2;
    }

    if (pCHCtrl->SampleIntervalTimer >= pCHCtrl->SampleIntervalTime) {
        pCHCtrl->SampleIntervalTimer = 0;
        
        // 等待单次采样完成
        do {
            Res = adc_adapter_SCM_1CH_Convert_StateMachine(pCHCtrl, 0);
        } while (Res == 0);
        
        // 采样成功时，累加采样值
        if (Res == 1) {
            adc_adapter_Add_Sample(Channel, pCHCtrl->ResultOnce);
            
            if(pCHCtrl->SampledTime>=pCHCtrl->SampleTime)
            {
                pCHCtrl->SampledTime = 0;
                
                return 1;
            }
        }
        
        return 0;//未达到采样个数时返回0
    }
    
    return -1;//未到采样间隔时返回-1
}



