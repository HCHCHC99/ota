 /**
 *******************************************************************************
 * @file  main.c
 * @brief Main program template.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2020-06-30        CDT         First version
 @endverbatim
 *******************************************************************************
 * Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by HDSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 */
/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ddl.h"
#include "main.h"
#include "system.h"
#include "mc_config.h"
#include "delay_adapter.h"
#include "gpio_adapter.h"
// 在文件顶部添加这些声明
#include <stdbool.h>

#include "uart_adapter.h"
#include "hz_timer.h"

#include "mc_cur.h"
#include "mc_app.h"
#include "adc_adapter.h"
#include "sys_main_state.h"
#include "hardware.h"
#include "hz_timer.h"
#include "adapter_pwm.h"
#include "usart_usb.h"
#include "hz_nopwm.h"
#include "hc32_common.h"
#include "ring_buffer.h"
#include "dev_pwm.h"
#include "device_manager.h"
#include "hc32f46x_flash.h"
#include "dev_flash.h"
#include "dev_timer0.h"
#include "CanJ1939.h"
#include "rtt_log.h"
#include "flash_download.h"
#include "isotp_transport.h"
/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*定时器相关计算*/
/*! Macro to convert a microsecond period to raw count value */
#define USEC_TO_COUNT(us, clockFreqInHz) (uint16_t)(((uint64_t)(us) * (clockFreqInHz)) / 1000000U)
/*! Macro to convert a raw count value to microsecond */
#define COUNT_TO_USEC(count, clockFreqInHz) (uint64_t)((uint64_t)(count) * 1000000U / (clockFreqInHz))
/*! Macro to convert a millisecond period to raw count value */
#define MSEC_TO_COUNT(ms, clockFreqInHz) (uint64_t)((uint64_t)(ms) * (clockFreqInHz) / 1000U)
/*! Macro to convert a raw count value to millisecond */
#define COUNT_TO_MSEC(count, clockFreqInHz) (uint64_t)((uint64_t)(count) * 1000U / (clockFreqInHz))

//hz
volatile uint8_t g_pwm_level = 0;

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 *******************************************************************************
 ** \brief System clock init function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
/*__DEBUG,HC32F460,USE_DEVICE_DRIVER_LIB*/
static void SystemClk_Init(void)
{
    stc_clk_sysclk_cfg_t    stcSysClkCfg;
    stc_clk_xtal_cfg_t      stcXtalCfg;
    stc_clk_mpll_cfg_t      stcMpllCfg;

    MEM_ZERO_STRUCT(stcSysClkCfg);
    MEM_ZERO_STRUCT(stcXtalCfg);
    MEM_ZERO_STRUCT(stcMpllCfg);

    /* Set bus clk div. */
    stcSysClkCfg.enHclkDiv = ClkSysclkDiv1;   // 128MHz
    stcSysClkCfg.enExclkDiv = ClkSysclkDiv2;  // 64MHz
    stcSysClkCfg.enPclk0Div = ClkSysclkDiv1;  // 128MHz
    stcSysClkCfg.enPclk1Div = ClkSysclkDiv2;  // 64MHz
    stcSysClkCfg.enPclk2Div = ClkSysclkDiv4;  // 32MHz
    stcSysClkCfg.enPclk3Div = ClkSysclkDiv4;  // 32MHz
    stcSysClkCfg.enPclk4Div = ClkSysclkDiv2;  // 64MHz
    CLK_SysClkConfig(&stcSysClkCfg);

    /* Switch system clock source to MPLL. */
    /* Use Xtal as MPLL source. */
    stcXtalCfg.enMode = ClkXtalModeOsc;
    stcXtalCfg.enDrv = ClkXtalLowDrv;
    stcXtalCfg.enFastStartup = Enable;
    CLK_XtalConfig(&stcXtalCfg);
    CLK_XtalCmd(Enable);
    CLK_HrcCmd(Enable);     //hz 开启内部时钟源

    /* MPLL config. */
    stcMpllCfg.pllmDiv = 1u;
    stcMpllCfg.plln = 32u;
    stcMpllCfg.PllpDiv = 2u;
    stcMpllCfg.PllqDiv = 2u;
    stcMpllCfg.PllrDiv = 2u;
    CLK_SetPllSource(ClkPllSrcHRC);
    CLK_MpllConfig(&stcMpllCfg);

    /* flash read wait cycle setting */
    EFM_Unlock();
    EFM_SetLatency(EFM_LATENCY_4);
    EFM_Lock();

    /* Enable MPLL. */
    CLK_MpllCmd(Enable);

    /* Wait MPLL ready. */
    while (Set != CLK_GetFlagStatus(ClkFlagMPLLRdy))
    {
    }

    /* Switch system clock source to MPLL. */
    CLK_SetSysClkSource(ClkSysSrcHRC);
}
/**
 *******************************************************************************
 ** \brief  Configure pins
 **
 ** \param  None
 **
 ** \retval
 **
 ******************************************************************************/
void gpio_Init(void)
{
    stc_port_init_t     stcPortInit;
    stc_port_pub_set_t  stcPortPubSet;
    stc_exint_config_t  stcExintCfg;
    stc_irq_regi_conf_t stcPortIrqCfg;

    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcPortPubSet);
    MEM_ZERO_STRUCT(stcExintCfg);
    MEM_ZERO_STRUCT(stcPortIrqCfg);

    // /*特殊引脚初始化*/
    // PORT_DebugPortSetting(TDI|TDO_SWO|TRST,Disable);    //PB3,PB4,PA13,PA14,PA15默认是烧录接口,需要禁止烧录功能才能用作GPIO

    /*GPIO-输入引脚初始化-不触发外部中断*/
    stcPortInit.enPinMode = Pin_Mode_In;
    stcPortInit.enPullUp = Disable;
    stcPortInit.enExInt = Disable;
    PORT_Init(GPIO_IN1_KUP_PORT, GPIO_IN1_KUP_PIN, &stcPortInit);
    PORT_Init(GPIO_IN2_KDOWN_PORT, GPIO_IN2_KDOWN_PIN, &stcPortInit);
	PORT_Init(GPIO_IN3_PORT, GPIO_IN3_PIN, &stcPortInit);
//    PORT_Init(GPIO_K3_PORT, GPIO_K3_PIN, &stcPortInit);
//    PORT_Init(GPIO_K4_PORT, GPIO_K4_PIN, &stcPortInit);
    PORT_Init(GPIO_M1_LIMIT_BTM_PORT, GPIO_M1_LIMIT_BTM_PIN, &stcPortInit);
    PORT_Init(GPIO_M1_LIMIT_TOP_PORT, GPIO_M1_LIMIT_TOP_PIN, &stcPortInit);

    /*GPIO-输入引脚初始化-触发外部中断*/
    stcPortInit.enPinMode = Pin_Mode_In;
    stcPortInit.enPullUp = Disable;
    stcPortInit.enExInt = Enable;
    PORT_Init(EXINT_M1_HALLA_PORT, EXINT_M1_HALLA_PIN, &stcPortInit);
    PORT_Init(EXINT_M1_HALLB_PORT, EXINT_M1_HALLB_PIN, &stcPortInit);
//    PORT_Init(EXINT_M1_HALLC_PORT, EXINT_M1_HALLC_PIN, &stcPortInit);

    /*GPIO-输出引脚初始化*/
    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enPinDrv = Pin_Drv_H;
    stcPortInit.enPullUp = Disable;
    stcPortInit.enExInt = Disable;
    stcPortInit.enPinOType = Pin_OType_Cmos;
    /*初始电平设置*/
	PORT_ResetBits(GPIO_LED_PORT, GPIO_LED_PIN);
    PORT_ResetBits(GPIO_SLEEP_PORT, GPIO_SLEEP_PIN);
    PORT_ResetBits(GPIO_OUT3_PORT, GPIO_OUT3_PIN);
	PORT_ResetBits(GPIO_OUT4_PORT, GPIO_OUT4_PIN);
//#if (PROTOCOL_TYPE == MODBUS_PROTOCOL)
    PORT_ResetBits(UART_485_DIR_PORT, UART_485_DIR_PIN);
//#elif (PROTOCOL_TYPE == CAN_PROTOCOL) 
//#endif
#if (DRV_OUTPUT_TYPE == DO_H_PWM_L_NPWM)	//初始化打开下桥
	PORT_SetBits(GPIO_PLU_PORT, GPIO_PLU_PIN);
	PORT_SetBits(GPIO_PLV_PORT, GPIO_PLV_PIN);
	PORT_SetBits(GPIO_PLW_PORT, GPIO_PLW_PIN);
#elif (DRV_OUTPUT_TYPE == DO_H_PWM_L_IO)
	PORT_ResetBits(GPIO_PLU_PORT, GPIO_PLU_PIN);
	PORT_ResetBits(GPIO_PLV_PORT, GPIO_PLV_PIN);
	PORT_ResetBits(GPIO_PLW_PORT, GPIO_PLW_PIN);
#endif
	PORT_ResetBits(AT_TOP_GPIO_Port, AT_TOP_Pin);
    PORT_ResetBits(AT_BTM_GPIO_Port, AT_BTM_Pin);
    /*使能输出*/
	PORT_OE(GPIO_LED_PORT, GPIO_LED_PIN, Enable);
    PORT_OE(GPIO_SLEEP_PORT, GPIO_SLEEP_PIN, Enable);
    PORT_OE(GPIO_OUT3_PORT, GPIO_OUT3_PIN, Enable);
	PORT_OE(GPIO_OUT4_PORT, GPIO_OUT4_PIN, Enable);
//#if (PROTOCOL_TYPE == MODBUS_PROTOCOL)
    PORT_OE(UART_485_DIR_PORT, UART_485_DIR_PIN, Enable);
//#elif (PROTOCOL_TYPE == CAN_PROTOCOL)  
//#endif
	PORT_OE(GPIO_PLU_PORT, GPIO_PLU_PIN, Enable);
	PORT_OE(GPIO_PLV_PORT, GPIO_PLV_PIN, Enable);
	PORT_OE(GPIO_PLW_PORT, GPIO_PLW_PIN, Enable);
	PORT_OE(AT_TOP_GPIO_Port, AT_TOP_Pin, Enable);
    PORT_OE(AT_BTM_GPIO_Port, AT_BTM_Pin, Enable);
    /*端口设置*/
	PORT_Init(GPIO_LED_PORT, GPIO_LED_PIN, &stcPortInit);
    PORT_Init(GPIO_SLEEP_PORT, GPIO_SLEEP_PIN, &stcPortInit);
    PORT_Init(GPIO_OUT3_PORT, GPIO_OUT3_PIN, &stcPortInit);
	PORT_Init(GPIO_OUT4_PORT, GPIO_OUT4_PIN, &stcPortInit);
//#if (PROTOCOL_TYPE == MODBUS_PROTOCOL)
    PORT_Init(UART_485_DIR_PORT, UART_485_DIR_PIN, &stcPortInit);
//#elif (PROTOCOL_TYPE == CAN_PROTOCOL)
//#endif
	PORT_Init(GPIO_PLU_PORT, GPIO_PLU_PIN, &stcPortInit);
	PORT_Init(GPIO_PLV_PORT, GPIO_PLV_PIN, &stcPortInit);
	PORT_Init(GPIO_PLW_PORT, GPIO_PLW_PIN, &stcPortInit);
	PORT_Init(AT_TOP_GPIO_Port, AT_TOP_Pin, &stcPortInit);
    PORT_Init(AT_BTM_GPIO_Port, AT_BTM_Pin, &stcPortInit);
}
/**
 *******************************************************************************
 ** \brief SW2 init function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void intc_exint_Init(void)
{
    stc_exint_config_t stcExtiConfig;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_port_init_t stcPortInit;

    /* 1 */
    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcPortInit);

    stcExtiConfig.enExitCh = M1_HALLA_EIRQ_CH;          /* Set External Int Ch */
    stcExtiConfig.enFilterEn = Enable;                  /* Filter setting */
    stcExtiConfig.enFltClk = Pclk3Div8;
    stcExtiConfig.enExtiLvl = ExIntBothEdge;            /* Triger edge setting */
    EXINT_Init(&stcExtiConfig);

    /* Set GPIO as External Int Ch input */
    stcPortInit.enExInt = Enable;
    PORT_Init(EXINT_M1_HALLA_PORT, EXINT_M1_HALLA_PIN, &stcPortInit);

    stcIrqRegiConf.enIntSrc = M1_HALLA_EIRQ_NUM;        /*  */
    stcIrqRegiConf.enIRQn = M1_HALLA_EIRQ_IRQN;         /* Register External Int to Vect.No.XXX */
    stcIrqRegiConf.pfnCallback = &ExtInt_Callback;      /* register Callback function */
    enIrqRegistration(&stcIrqRegiConf);

    EXINT_IrqFlgClr(M1_HALLA_EIRQ_CH);
//    NVIC_ClearPendingIRQ(M1_HALLA_EIRQ_IRQN);                    /* Clear pending */
//    NVIC_SetPriority(M1_HALLA_EIRQ_IRQN, M1_HALL_EIRQ_PRIORITY);   /* Set priority */
//    NVIC_EnableIRQ(M1_HALLA_EIRQ_IRQN);                          /* Enable NVIC */

    /* 2 */
    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcPortInit);

    stcExtiConfig.enExitCh = M1_HALLB_EIRQ_CH;          /* Set External Int Ch */
    stcExtiConfig.enFilterEn = Enable;                  /* Filter setting */
    stcExtiConfig.enFltClk = Pclk3Div8;
    stcExtiConfig.enExtiLvl = ExIntBothEdge;            /* Triger edge setting */
    EXINT_Init(&stcExtiConfig);

    /* Set GPIO as External Int Ch input */
    stcPortInit.enExInt = Enable;
    PORT_Init(EXINT_M1_HALLB_PORT, EXINT_M1_HALLB_PIN, &stcPortInit);

    stcIrqRegiConf.enIntSrc = M1_HALLB_EIRQ_NUM;        /*  */
    stcIrqRegiConf.enIRQn = M1_HALLB_EIRQ_IRQN;         /* Register External Int to Vect.No.XXX */
    stcIrqRegiConf.pfnCallback = &ExtInt_Callback;      /* register Callback function */
    enIrqRegistration(&stcIrqRegiConf);

    EXINT_IrqFlgClr(M1_HALLB_EIRQ_CH);
//    NVIC_ClearPendingIRQ(M1_HALLB_EIRQ_IRQN);                    /* Clear pending */
//    NVIC_SetPriority(M1_HALLB_EIRQ_IRQN, M1_HALL_EIRQ_PRIORITY);   /* Set priority */
//    NVIC_EnableIRQ(M1_HALLB_EIRQ_IRQN);                          /* Enable NVIC */


  /* 3 */
    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcPortInit);

    stcExtiConfig.enExitCh = M1_HALLC_EIRQ_CH;            /* Set External Int Ch */
    stcExtiConfig.enFilterEn = Enable;                    /* Filter setting */
    stcExtiConfig.enFltClk = Pclk3Div8;
    stcExtiConfig.enExtiLvl = ExIntBothEdge;          /* Triger edge setting */
    EXINT_Init(&stcExtiConfig);

    /* Set GPIO as External Int Ch input */
    stcPortInit.enExInt = Enable;
//    PORT_Init(EXINT_M1_HALLC_PORT, EXINT_M1_HALLC_PIN, &stcPortInit);

    stcIrqRegiConf.enIntSrc = M1_HALLC_EIRQ_NUM;      /*  */
    stcIrqRegiConf.enIRQn = M1_HALLC_EIRQ_IRQN;           /* Register External Int to Vect.No.XXX */
    stcIrqRegiConf.pfnCallback = &ExtInt_Callback;        /* register Callback function */
    enIrqRegistration(&stcIrqRegiConf);

  EXINT_IrqFlgClr(M1_HALLC_EIRQ_CH);
//    NVIC_ClearPendingIRQ(M1_HALLC_EIRQ_IRQN);                  /* Clear pending */
//    NVIC_SetPriority(M1_HALLC_EIRQ_IRQN, M1_HALL_EIRQ_PRIORITY); /* Set priority */
//    NVIC_EnableIRQ(M1_HALLC_EIRQ_IRQN);                            /* Enable NVIC */
}

/**
 *******************************************************************************
 ** \brief Configure Timera peripheral function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void Timera_Pwm_Config(void)  //hz PWM TIMERA
{
#if (DRV_OUTPUT_TYPE != DO_H_PWM_L_NPWM)	//无刷: 用timera上桥pwm,下桥io; 有刷: 用timer4互补PWM输出
    stc_timera_base_init_t stcTimeraInit;
    stc_timera_compare_init_t stcTimerCompareInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcTimeraInit);
    MEM_ZERO_STRUCT(stcTimerCompareInit);

    /* Configuration peripheral clock */
    PWC_Fcg2PeriphClockCmd(PWMCH_M1_UH_TIMERCLOCK , Enable);
    PWC_Fcg2PeriphClockCmd(PWMCH_M1_VH_TIMERCLOCK , Enable);
//    PWC_Fcg2PeriphClockCmd(PWMCH_M1_WH_TIMERCLOCK , Enable);

    /* Configuration TIMERA compare pin */
    PORT_SetFunc(PWMCH_M1_UH_PORT, PWMCH_M1_UH_PIN, PWMCH_M1_UH_FUNC_PWM, Disable);
    PORT_SetFunc(PWMCH_M1_VH_PORT, PWMCH_M1_VH_PIN, PWMCH_M1_VH_FUNC_PWM, Disable);
//    PORT_SetFunc(PWMCH_M1_WH_PORT, PWMCH_M1_WH_PIN, PWMCH_M1_WH_FUNC_PWM, Disable);

    /* Configuration timera unit 1 base structure */
    stcTimeraInit.enClkDiv = TimeraPclkDiv2;    //fCLK = 64MHz / Div
    stcTimeraInit.enCntMode = TimeraCountModeSawtoothWave;
    stcTimeraInit.enCntDir = TimeraCountDirUp;
    stcTimeraInit.enSyncStartupEn = Disable;
    stcTimeraInit.u16PeriodVal = 2000;        //fPWM = fCLK / Period = 16kHz
    TIMERA_BaseInit(PWMCH_M1_UH_TIMER, &stcTimeraInit);
    TIMERA_BaseInit(PWMCH_M1_VH_TIMER, &stcTimeraInit);
//    TIMERA_BaseInit(PWMCH_M1_WH_TIMER, &stcTimeraInit);

    /* Configuration timera unit 1 compare structure */
    stcTimerCompareInit.u16CompareVal = 0;
    stcTimerCompareInit.enStartCountOutput = TimeraCountStartOutputLow;
    stcTimerCompareInit.enStopCountOutput = TimeraCountStopOutputLow;
    stcTimerCompareInit.enCompareMatchOutput = TimeraCompareMatchOutputLow;
    stcTimerCompareInit.enPeriodMatchOutput = TimeraPeriodMatchOutputHigh;
    stcTimerCompareInit.enSpecifyOutput = TimeraSpecifyOutputInvalid;
    stcTimerCompareInit.enCacheEn = Disable;
    stcTimerCompareInit.enTriangularTroughTransEn = Disable;
    stcTimerCompareInit.enTriangularCrestTransEn = Disable;
    stcTimerCompareInit.u16CompareCacheVal = stcTimerCompareInit.u16CompareVal;
    /* Configure Channel 1 */
    TIMERA_CompareInit(PWMCH_M1_UH_TIMER, PWMCH_M1_UH_CH, &stcTimerCompareInit);
    TIMERA_CompareInit(PWMCH_M1_VH_TIMER, PWMCH_M1_VH_CH, &stcTimerCompareInit);
//    TIMERA_CompareInit(PWMCH_M1_WH_TIMER, PWMCH_M1_WH_CH, &stcTimerCompareInit);
    /* Configure channel 3 */
    TIMERA_Cmd(PWMCH_M1_UH_TIMER, Enable);
    TIMERA_Cmd(PWMCH_M1_VH_TIMER, Enable);
//    TIMERA_Cmd(PWMCH_M1_WH_TIMER, Enable);
#endif
}

/**
 *******************************************************************************
 ** \brief Configure Timer4 peripheral function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
#define TEST1	1
#define TEST2	2
#define TEST	TEST2
uint16_t g_PWMDuty = 0;
/* Timer4 CNT */
#define TIMER4_UNIT                     (M4_TMR43)
#define TIMER4_CNT_CYCLE_VAL            (2000)        /* Timer4 counter cycle value */

/* Timer4 OCO */
#define TIMER4_OCO_HIGH_CH              (Timer4OcoOuh)  /* only Timer4OcoOuh  Timer4OcoOvh  Timer4OcoOwh */
#define TIMER4_OCO_LOW_CH				(Timer4OcoOul)  /* only Timer4OcoOuh  Timer4OcoOvh  Timer4OcoOwh */

/* Timer4 PWM */
#define TIMER4_PWM_CH                   (Timer4PwmU)    /* only Timer4PwmU  Timer4PwmV  Timer4PwmW */

/* Define port and pin for Timer4Pwm */
#define TIMER4_PWM_H_PORT               (PortB)         /* TIM4_1_OUH_B:PE9   TIM4_1_OVH_B:PE11   TIM4_1_OWH_B:PE13 */
#define TIMER4_PWM_H_PIN                (Pin09)
#define TIMER4_PWM_L_PORT               (PortB)         /* TIM4_1_OUL_B:PE8   TIM4_1_OVL_B:PE10   TIM4_1_OWL_B:PE12 */
#define TIMER4_PWM_L_PIN                (Pin08)
void hc32f460_timer4_complementary_pwm_cfg(void)
{
#if (DRV_OUTPUT_TYPE == DO_H_PWM_L_NPWM)	//无刷: 用timera上桥pwm,下桥io; 有刷: 用timer4互补PWM输出
#if (TEST == TEST1)		//H,L独立输出
    stc_timer4_cnt_init_t stcCntInit;
    stc_timer4_oco_init_t stcOcoInit;
    stc_timer4_pwm_init_t stcPwmInit;
    stc_oco_low_ch_compare_mode_t stcLowChCmpMode;
    stc_oco_high_ch_compare_mode_t stcHighChCmpMode;

    /* Clear structures */
    MEM_ZERO_STRUCT(stcCntInit);
    MEM_ZERO_STRUCT(stcOcoInit);
    MEM_ZERO_STRUCT(stcPwmInit);
    MEM_ZERO_STRUCT(stcLowChCmpMode);
    MEM_ZERO_STRUCT(stcHighChCmpMode);

    /* Enable peripheral clock */
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM43, Enable);

    /* Timer4 CNT : Initialize CNT configuration structure */
    stcCntInit.enBufferCmd = Disable;
    stcCntInit.enClk = Timer4CntPclk;
    stcCntInit.enClkDiv = Timer4CntPclkDiv2;  /* CNT clock divide */
    stcCntInit.u16Cycle = TIMER4_CNT_CYCLE_VAL;
    stcCntInit.enCntMode = Timer4CntSawtoothWave;
    stcCntInit.enZeroIntCmd = Disable;
    stcCntInit.enPeakIntCmd = Disable;
    stcCntInit.enZeroIntMsk = Timer4CntIntMask0;
    stcCntInit.enPeakIntMsk = Timer4CntIntMask0;
    TIMER4_CNT_Init(TIMER4_UNIT, &stcCntInit); /* Initialize CNT */

    /* Timer4 OCO : Initialize OCO configuration structure */
    stcOcoInit.enOccrBufMode = OccrBufDisable;
    stcOcoInit.enOcmrBufMode = OcmrBufDisable;
    stcOcoInit.enPortLevel = OcPortLevelLow;
    stcOcoInit.enOcoIntCmd = Disable;
    TIMER4_OCO_Init(TIMER4_UNIT, TIMER4_OCO_HIGH_CH, &stcOcoInit);	/* Initialize OCO high channel */
    TIMER4_OCO_Init(TIMER4_UNIT, TIMER4_OCO_LOW_CH, &stcOcoInit);	/* Initialize OCO low channel */

    /* OCMR[31:0] Ox 0FF0 0FFF    0000 1111 1111 0000   0000 1111 1111 1111 */
    stcHighChCmpMode.enCntZeroMatchOpState = OcoOpOutputLow;			//bit[11:10]<-->(0==CNT==OCCR) 		通道值==0的特例
    stcHighChCmpMode.enCntZeroNotMatchOpState = OcoOpOutputHigh;		//bit[15:14]<-->(0==CNT!=OCCR)		CNT自加的过程
    stcHighChCmpMode.enCntUpCntMatchOpState = OcoOpOutputLow;			//bit[9:8]<-->(CNT↑==OCCR!=0)		CNT自加到通道值的时刻
    stcHighChCmpMode.enCntPeakMatchOpState = OcoOpOutputHigh;			//bit[7:6]<-->(CNT==OCCR==CPSR)		通道值==模值的特例
    stcHighChCmpMode.enCntPeakNotMatchOpState = OcoOpOutputHold;		//bit[13:12]<-->(CNT==CPSR!=OCCR)	通道值>模值的特例
	TIMER4_OCO_SetHighChCompareMode(TIMER4_UNIT, TIMER4_OCO_HIGH_CH, &stcHighChCmpMode);  /* Set OCO high channel compare mode */

    /*************Timer4 OCO ocmr1[31:0] = 0x0FF0 0FFF*****************************/
	/* OCMR[31:0] Ox 0FF0 0FFF    0000 1111 1111 0000   0000 1111 1111 1111 */
	stcLowChCmpMode.enCntZeroLowMatchHighNotMatchLowChOpState = OcoOpOutputLow;		//bit[11:10]<-->(0==CNT==OCCR) 		通道值==0的特例
	stcLowChCmpMode.enCntZeroLowNotMatchHighNotMatchLowChOpState = OcoOpOutputHigh;	//bit[15:14]<-->(0==CNT!=OCCR)		CNT自加的过程
	stcLowChCmpMode.enCntUpCntLowMatchHighNotMatchLowChOpState = OcoOpOutputLow; 	//bit[9:8]<-->(CNT↑==OCCR!=0)		CNT自加到通道值的时刻
	stcLowChCmpMode.enCntPeakLowMatchHighNotMatchLowChOpState = OcoOpOutputHigh;	//bit[7:6]<-->(CNT==OCCR==CPSR)		通道值==模值的特例
	stcLowChCmpMode.enCntPeakLowNotMatchHighNotMatchLowChOpState = OcoOpOutputHold;	//bit[13:12]<-->(CNT==CPSR!=OCCR)	通道值>模值的特例
	//stcLowChCmpMode.enMatchConditionExtendCmd = Enable;
	TIMER4_OCO_SetLowChCompareMode(TIMER4_UNIT, TIMER4_OCO_LOW_CH, &stcLowChCmpMode);  /* Set OCO low channel compare mode */

    /* Enable OCO */
    TIMER4_OCO_OutputCompareCmd(TIMER4_UNIT, TIMER4_OCO_HIGH_CH, Enable);
    TIMER4_OCO_OutputCompareCmd(TIMER4_UNIT, TIMER4_OCO_LOW_CH, Enable);

    /* Initialize PWM I/O */
    PORT_SetFunc(TIMER4_PWM_H_PORT, TIMER4_PWM_H_PIN, Func_Tim4, Disable);
    PORT_SetFunc(TIMER4_PWM_L_PORT, TIMER4_PWM_L_PIN, Func_Tim4, Disable);



    /* Timer4 PWM: Initialize PWM configuration structure */
    stcPwmInit.enRtIntMaskCmd = Enable;
    stcPwmInit.enClkDiv = PwmPlckDiv2;
    stcPwmInit.enOutputState = PwmHPwmLHold;//PwmHHoldPwmLReverse;
    stcPwmInit.enMode = PwmThroughMode;
    TIMER4_PWM_Init(TIMER4_UNIT, TIMER4_PWM_CH, &stcPwmInit); /* Initialize timer4 pwm */

    /* Clear && Start CNT */
    TIMER4_CNT_ClearCountVal(TIMER4_UNIT);
    TIMER4_CNT_Start(TIMER4_UNIT);
	
    while (1)
    {
		TIMER4_OCO_WriteOccr(M4_TMR43, TIMER4_OCO_HIGH_CH, g_PWMDuty); /* Set OCO low channel compare value */
		TIMER4_OCO_WriteOccr(M4_TMR43, TIMER4_OCO_LOW_CH, g_PWMDuty+1); /* Set OCO low channel compare value */
		if(g_PWMDuty < TIMER4_CNT_CYCLE_VAL)
			g_PWMDuty += TIMER4_CNT_CYCLE_VAL / 5;
		else
			g_PWMDuty = 0;
		Ddl_Delay1ms(200);
		SWDT_RefreshCounter();		/*看门狗喂狗函数*/
    }
#elif (TEST == TEST2)	//H,L互补输出
    stc_timer4_cnt_init_t stcCntInit;
    stc_timer4_oco_init_t stcOcoInit;
    stc_timer4_pwm_init_t stcPwmInit;
    stc_oco_low_ch_compare_mode_t stcLowChCmpMode;
	
    /* Clear structures */
    MEM_ZERO_STRUCT(stcCntInit);
    MEM_ZERO_STRUCT(stcOcoInit);
    MEM_ZERO_STRUCT(stcPwmInit);
    MEM_ZERO_STRUCT(stcLowChCmpMode);
	
    /* Enable peripheral clock */
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM43, Enable);

    /* Timer4 CNT : Initialize CNT configuration structure */
    stcCntInit.enBufferCmd = Disable;
    stcCntInit.enClk = Timer4CntPclk;
    stcCntInit.enClkDiv = Timer4CntPclkDiv2;  //fDR_CNT = fPCLK1/2 = 64MHz/2 = 32MHz
    stcCntInit.u16Cycle = TIMER4_CNT_CYCLE_VAL;
    stcCntInit.enCntMode = Timer4CntSawtoothWave;
    stcCntInit.enZeroIntCmd = Disable;
    stcCntInit.enPeakIntCmd = Disable;
    stcCntInit.enZeroIntMsk = Timer4CntIntMask0;
    stcCntInit.enPeakIntMsk = Timer4CntIntMask0;
    TIMER4_CNT_Init(M4_TMR43, &stcCntInit); /* Initialize CNT */

    /* Timer4 OCO : Initialize OCO configuration structure */
    stcOcoInit.enOcoIntCmd = Disable;
    stcOcoInit.enPortLevel = OcPortLevelLow;
    stcOcoInit.enOccrBufMode = OccrBufDisable;
    stcOcoInit.enOcmrBufMode = OcmrBufDisable;
    TIMER4_OCO_Init(M4_TMR43, Timer4OcoOul, &stcOcoInit);  /* Initialize OCO low channel */
    TIMER4_OCO_Init(M4_TMR43, Timer4OcoOvl, &stcOcoInit);  /* Initialize OCO low channel */
    TIMER4_OCO_Init(M4_TMR43, Timer4OcoOwl, &stcOcoInit);  /* Initialize OCO low channel */
	
    /*************Timer4 OCO ocmr1[31:0] = 0x0FF0 0FFF*****************************/
	/* OCMR[31:0] Ox 0FF0 0FFF    0000 1111 1111 0000   0000 1111 1111 1111 */
	stcLowChCmpMode.enCntZeroLowMatchHighNotMatchLowChOpState = OcoOpOutputLow;		//bit[11:10]<-->(0==CNT==OCCR) 		通道值==0的特例
	stcLowChCmpMode.enCntZeroLowNotMatchHighNotMatchLowChOpState = OcoOpOutputHigh;	//bit[15:14]<-->(0==CNT!=OCCR)		CNT自加的过程
	stcLowChCmpMode.enCntUpCntLowMatchHighNotMatchLowChOpState = OcoOpOutputLow; 	//bit[9:8]<-->(CNT↑==OCCR!=0)		CNT自加到通道值的时刻
	stcLowChCmpMode.enCntPeakLowMatchHighNotMatchLowChOpState = OcoOpOutputHigh;	//bit[7:6]<-->(CNT==OCCR==CPSR)		通道值==模值的特例
	stcLowChCmpMode.enCntPeakLowNotMatchHighNotMatchLowChOpState = OcoOpOutputHold;	//bit[13:12]<-->(CNT==CPSR!=OCCR)	通道值>模值的特例
	TIMER4_OCO_SetLowChCompareMode(M4_TMR43, Timer4OcoOul, &stcLowChCmpMode);  /* Set OCO low channel compare mode */
	TIMER4_OCO_SetLowChCompareMode(M4_TMR43, Timer4OcoOvl, &stcLowChCmpMode);  /* Set OCO low channel compare mode */
	TIMER4_OCO_SetLowChCompareMode(M4_TMR43, Timer4OcoOwl, &stcLowChCmpMode);  /* Set OCO low channel compare mode */
	
	/* Enable OCO */
    TIMER4_OCO_OutputCompareCmd(M4_TMR43, Timer4OcoOul, Enable);
    TIMER4_OCO_OutputCompareCmd(M4_TMR43, Timer4OcoOvl, Enable);
    TIMER4_OCO_OutputCompareCmd(M4_TMR43, Timer4OcoOwl, Enable);
	
    /* Initialize PWM I/O */    
	PORT_SetFunc(PortB, Pin09, Func_Tim4, Disable);	//为什么是Disable的解释：Disable的是引脚的双周边(副)功能,Enable则引脚就同时配置为2个功能!
    PORT_SetFunc(PortB, Pin08, Func_Tim4, Disable);
	PORT_SetFunc(PortB, Pin07, Func_Tim4, Disable);	//为什么是Disable的解释：Disable的是引脚的双周边(副)功能,Enable则引脚就同时配置为2个功能!
    PORT_SetFunc(PortB, Pin06, Func_Tim4, Disable);
	PORT_SetFunc(PortB, Pin05, Func_Tim4, Disable);	//为什么是Disable的解释：Disable的是引脚的双周边(副)功能,Enable则引脚就同时配置为2个功能!
    PORT_SetFunc(PortB, Pin04, Func_Tim4, Disable);


	
    /* Timer4 PWM: Initialize PWM configuration structure */
    stcPwmInit.enRtIntMaskCmd = Enable;
    stcPwmInit.enClkDiv = PwmPlckDiv2;	//fDR_CNT = fPCLK1/2 = 64MHz/2 = 32MHz
    stcPwmInit.enOutputState = PwmHPwmLHold;	//OXH、OXL输出不反转
    stcPwmInit.enMode = PwmDeadTimerMode;
    TIMER4_PWM_Init(M4_TMR43, Timer4PwmU, &stcPwmInit); /* Initialize timer4 pwm */
    TIMER4_PWM_Init(M4_TMR43, Timer4PwmV, &stcPwmInit); /* Initialize timer4 pwm */
    TIMER4_PWM_Init(M4_TMR43, Timer4PwmW, &stcPwmInit); /* Initialize timer4 pwm */
    TIMER4_PWM_WriteDeadRegionValue(M4_TMR43, Timer4PwmV, 10u, 10);	//左右死区时间 = 1/32MHz * 10 = 0.3125us
    TIMER4_PWM_WriteDeadRegionValue(M4_TMR43, Timer4PwmU, 10u, 10);	//左右死区时间 = 1/32MHz * 10 = 0.3125us
    TIMER4_PWM_WriteDeadRegionValue(M4_TMR43, Timer4PwmW, 10u, 10);	//左右死区时间 = 1/32MHz * 10 = 0.3125us
	
    /* Clear && Start CNT */
    TIMER4_CNT_ClearCountVal(M4_TMR43);
    TIMER4_CNT_Start(M4_TMR43);
//	//test
//    while (1)
//    {
//		TIMER4_OCO_WriteOccr(M4_TMR43, Timer4OcoOuh, g_PWMDuty+1); /* Set OCO low channel compare value */
//		TIMER4_OCO_WriteOccr(M4_TMR43, Timer4OcoOul, g_PWMDuty); /* Set OCO low channel compare value */
//		TIMER4_OCO_WriteOccr(M4_TMR43, Timer4OcoOvh, g_PWMDuty+1); /* Set OCO low channel compare value */
//		TIMER4_OCO_WriteOccr(M4_TMR43, Timer4OcoOvl, g_PWMDuty); /* Set OCO low channel compare value */
//		TIMER4_OCO_WriteOccr(M4_TMR43, Timer4OcoOwh, g_PWMDuty+1); /* Set OCO low channel compare value */
//		TIMER4_OCO_WriteOccr(M4_TMR43, Timer4OcoOwl, g_PWMDuty); /* Set OCO low channel compare value */		
//		if(g_PWMDuty < TIMER4_CNT_CYCLE_VAL)
//			g_PWMDuty += TIMER4_CNT_CYCLE_VAL / 5;
//		else
//			g_PWMDuty = 0;
//		Ddl_Delay1ms(200);
//		SWDT_RefreshCounter();		/*看门狗喂狗函数*/
//    }
#endif
#endif
}
/**
 *******************************************************************************
 ** \brief  ADC configuration, including clock configuration, initial configuration,
 **         channel configuration and trigger source configuration.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
void AdcConfig(void)
{
    stc_adc_init_t stcAdcInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcAdcInit);

    CLK_SetPeriClkSource(ClkPeriSrcPclk);

    /* Configuration peripheral clock */
    PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_ADC1, Enable);

    stcAdcInit.enResolution = AdcResolution_12Bit;
    stcAdcInit.enDataAlign  = AdcDataAlign_Right;
    stcAdcInit.enAutoClear  = AdcClren_Disable;
    stcAdcInit.enScanMode   = AdcMode_SAOnce;

    
    ADC_Init(M4_ADC1, &stcAdcInit);
    // ADC_Init(M4_ADC2, &stcAdcInit);

    // AdcChannelConfig();
}
/**
 *******************************************************************************
 ** \brief  Main function of example project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
void timer0_uart_Init(void) //HC32F460的空闲中断基于timer0实现
{
    stc_clk_freq_t stcClkTmp;
    stc_tim0_base_init_t stcTimerCfg;
    stc_tim0_trigger_init_t StcTimer0TrigInit;

    MEM_ZERO_STRUCT(stcClkTmp);
    MEM_ZERO_STRUCT(stcTimerCfg);
    MEM_ZERO_STRUCT(StcTimer0TrigInit);

    /* Timer0 peripheral enable */
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM02, Enable);

    /* Clear CNTAR register for channel A */
    TIMER0_WriteCntReg(M4_TMR02, Tim0_ChannelA, 0u);
    TIMER0_WriteCntReg(M4_TMR02, Tim0_ChannelB, 0u);

    /* Config register for channel A */
    stcTimerCfg.Tim0_CounterMode = Tim0_Async;
    stcTimerCfg.Tim0_AsyncClockSource = Tim0_XTAL32;
    stcTimerCfg.Tim0_ClockDivision = Tim0_ClkDiv8;
    stcTimerCfg.Tim0_CmpValue = 16;
    TIMER0_BaseInit(M4_TMR02, Tim0_ChannelA, &stcTimerCfg);

    /* Clear compare flag */
    TIMER0_ClearFlag(M4_TMR02, Tim0_ChannelA);

    /* Config timer0 hardware trigger */
    StcTimer0TrigInit.Tim0_InTrigEnable = false;
    StcTimer0TrigInit.Tim0_InTrigClear = true;
    StcTimer0TrigInit.Tim0_InTrigStart = true;
    StcTimer0TrigInit.Tim0_InTrigStop = false;
    TIMER0_HardTriggerInit(M4_TMR02, Tim0_ChannelA, &StcTimer0TrigInit);
}
/**
 *******************************************************************************
 ** \brief Configure uart peripheral function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void Uart1_Init(uint32_t bdr)
{
    stc_irq_regi_conf_t stcIrqRegiCfg;
    stc_usart_uart_init_t stcInitCfg;

    MEM_ZERO_STRUCT(stcIrqRegiCfg);
    MEM_ZERO_STRUCT(stcInitCfg);

    /* Enable peripheral clock */
    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_USART1, Enable);
    /* Initialize USART IO */
    PORT_SetFunc(UART_MCU_RX_PORT, UART_MCU_RX_PIN, UART_MCU_RX_FUNC, Disable);
    PORT_SetFunc(UART_MCU_TX_PORT, UART_MCU_TX_PIN, UART_MCU_TX_FUNC, Disable);

    /* Initialize UART */
    stcInitCfg.enClkMode = UsartIntClkCkNoOutput;
    if(bdr <= 2400)
        stcInitCfg.enClkDiv = UsartClkDiv_64;       //1200bps
    else if(bdr <= 19200)
        stcInitCfg.enClkDiv = UsartClkDiv_16;       //9600bps
    else
        stcInitCfg.enClkDiv = UsartClkDiv_4;        //115200bps
    stcInitCfg.enDataLength = UsartDataBits8;
    stcInitCfg.enDirection = UsartDataLsbFirst;
    stcInitCfg.enStopBit = UsartOneStopBit;
    stcInitCfg.enParity = UsartParityNone;
    stcInitCfg.enSampleMode = UsartSamleBit8;
    stcInitCfg.enDetectMode = UsartStartBitFallEdge;
    stcInitCfg.enHwFlow = UsartRtsEnable;
    if(Ok != USART_UART_Init(UART_MCU, &stcInitCfg))
    {
        while (1);
    }

    /* Set baudrate */
    if(Ok != USART_SetBaudrate(UART_MCU, bdr))
    {
        while (1);
    }

    /* Set USART RX IRQ */
    stcIrqRegiCfg.enIRQn = UART_MCU_RI_IRQN;
    stcIrqRegiCfg.pfnCallback = &Uart_MCU_Callback_Rx;
    stcIrqRegiCfg.enIntSrc = UART_MCU_RI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
//    NVIC_SetPriority(UART_485_RI_IRQN, UART_IRQ_PRIORITY);
//    NVIC_ClearPendingIRQ(UART_485_RI_IRQN);
//    NVIC_EnableIRQ(UART_485_RI_IRQN);
    /* Set USART RX error IRQ */
    stcIrqRegiCfg.enIRQn = UART_MCU_EI_IRQN;
    stcIrqRegiCfg.pfnCallback = &Uart_MCU_Callback_Err;
    stcIrqRegiCfg.enIntSrc = UART_MCU_EI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
//    NVIC_SetPriority(UART_485_EI_IRQN, UART_IRQ_PRIORITY);
//    NVIC_ClearPendingIRQ(UART_485_EI_IRQN);
//    NVIC_EnableIRQ(UART_485_EI_IRQN);
#if 0	//HC32的rxtimeout中断不好用
    /* Set USART RX timeout error IRQ */
    stcIrqRegiCfg.enIRQn = UART_MCU_RTOI_IRQN;
    stcIrqRegiCfg.pfnCallback = &Uart_MCU_Callback_Idle;
    stcIrqRegiCfg.enIntSrc = UART_485_RTOI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(UART_485_RTOI_IRQN, UART_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(UART_485_RTOI_IRQN);
    NVIC_EnableIRQ(UART_485_RTOI_IRQN);
#endif
    /* Set USART TX IRQ */
    stcIrqRegiCfg.enIRQn = UART_MCU_TI_IRQN;
    stcIrqRegiCfg.pfnCallback = &Uart_MCU_Callback_Tx;
    stcIrqRegiCfg.enIntSrc = UART_MCU_TI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
//    NVIC_SetPriority(UART_485_TI_IRQN, UART_IRQ_PRIORITY);
//    NVIC_ClearPendingIRQ(UART_485_TI_IRQN);
//    NVIC_EnableIRQ(UART_485_TI_IRQN);
    /* Set USART TX complete IRQ */
    stcIrqRegiCfg.enIRQn = UART_MCU_TCI_IRQN;
    stcIrqRegiCfg.pfnCallback = &Uart_MCU_Callback_TC;
    stcIrqRegiCfg.enIntSrc = UART_MCU_TCI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
//    NVIC_SetPriority(UART_485_TCI_IRQN, UART_IRQ_PRIORITY);
//    NVIC_ClearPendingIRQ(UART_485_TCI_IRQN);
//    NVIC_EnableIRQ(UART_485_TCI_IRQN);

    /*Enable RX && TX function*/
    USART_FuncCmd(UART_MCU, UsartRx, Enable);
    USART_FuncCmd(UART_MCU, UsartTx, Enable);
    USART_FuncCmd(UART_MCU, UsartRxInt, Enable);
    USART_FuncCmd(UART_MCU, UsartTimeOut, Enable);
    USART_FuncCmd(UART_MCU, UsartTimeOutInt, Enable);
}
/**
 *******************************************************************************
 ** \brief Configure can peripheral function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void can_bdr_Init(uint32_t bdr, stc_can_bt_t *pInit)
{
	if(!pInit)
		return;
	//HC32F460的can_clk == 外部晶振时钟频率(这块板子是8MHz)
	//公式1: fTQ = can_clk / (PRESC + 1) --> TQ = (PRESC + 1) * (1 / can_clk)
	//公式2: BT = tSEG1 + tSEG2 = ((SEG_1 + 2) + (SEG_2 + 1)) * TQ
	//规则: SEG_1 >= SEG_2 + 1 && SEG_2 >= SJW
	//范围：SEG_1 ∈[0,63]  SEG_2 ∈[0,7]  SJW ∈[0,7]
	if(bdr == 1000000)			//1M
	{
		pInit->PRESC = 1u-1u;	//1分频
		pInit->SEG_1 = 5u-2u;
		pInit->SEG_2 = 3u-1u;
		pInit->SJW   = 3u-1u;		
	}else if(bdr == 500000)		//500k
	{
		pInit->PRESC = 2u-1u;	//2分频
		pInit->SEG_1 = 5u-2u;
		pInit->SEG_2 = 3u-1u;
		pInit->SJW   = 3u-1u;	
	}else						//default == 250k
	{
		pInit->PRESC = 4u-1u;	//4分频
		pInit->SEG_1 = 5u-2u;
		pInit->SEG_2 = 3u-1u;
		pInit->SJW   = 3u-1u;	
	}
}

/**
 *******************************************************************************
 ** \brief Configure uart peripheral function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
//#if (PROTOCOL_TYPE == MODBUS_PROTOCOL)
void Uart3_Init(uint32_t bdr)
{
    stc_irq_regi_conf_t stcIrqRegiCfg;
    stc_usart_uart_init_t stcInitCfg;

    MEM_ZERO_STRUCT(stcIrqRegiCfg);
    MEM_ZERO_STRUCT(stcInitCfg);

    /* Enable peripheral clock */
    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_USART3, Enable);
    /* Initialize USART IO */
    PORT_SetFunc(UART_485_RX_PORT, UART_485_RX_PIN, UART_485_RX_FUNC, Disable);
    PORT_SetFunc(UART_485_TX_PORT, UART_485_TX_PIN, UART_485_TX_FUNC, Disable);

    /* Initialize UART */
    //stcInitCfg.enClkMode = UsartIntClkCkNoOutput;
    stcInitCfg.enClkMode = UsartIntClkCkOutput;     //想用IDLE(接收超时)功能需要选UsartIntClkCkOutput
    if(bdr <= 2400)
        stcInitCfg.enClkDiv = UsartClkDiv_64;       //1200bps
    else if(bdr <= 19200)
        stcInitCfg.enClkDiv = UsartClkDiv_16;       //9600bps
    else
        stcInitCfg.enClkDiv = UsartClkDiv_4;        //115200bps
    stcInitCfg.enDataLength = UsartDataBits8;
    stcInitCfg.enDirection = UsartDataLsbFirst;
    stcInitCfg.enStopBit = UsartOneStopBit;
//#if (PROTOCOL_TYPE == MODBUS_PROTOCOL)
    stcInitCfg.enParity = UsartParityEven;
//#elif (PROTOCOL_TYPE == CAN_PROTOCOL)
//    stcInitCfg.enParity = UsartParityNone;
//#endif
    stcInitCfg.enSampleMode = UsartSamleBit8;//UsartSampleBit8;
    stcInitCfg.enDetectMode = UsartStartBitFallEdge;
    stcInitCfg.enHwFlow = UsartRtsEnable;
    if(Ok != USART_UART_Init(UART_485, &stcInitCfg))
    {
        while (1);
    }

    /* Set baudrate */
    if(Ok != USART_SetBaudrate(UART_485, bdr))
    {
        while (1);
    }

    /* Set USART RX IRQ */
    stcIrqRegiCfg.enIRQn = UART_485_RI_IRQN;
    stcIrqRegiCfg.pfnCallback = &Uart_Callback_Rx;
    stcIrqRegiCfg.enIntSrc = UART_485_RI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
//    NVIC_SetPriority(UART_485_RI_IRQN, UART_IRQ_PRIORITY);
//    NVIC_ClearPendingIRQ(UART_485_RI_IRQN);
//    NVIC_EnableIRQ(UART_485_RI_IRQN);
    /* Set USART RX error IRQ */
    stcIrqRegiCfg.enIRQn = UART_485_EI_IRQN;
    stcIrqRegiCfg.pfnCallback = &Uart_Callback_Err;
    stcIrqRegiCfg.enIntSrc = UART_485_EI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
//    NVIC_SetPriority(UART_485_EI_IRQN, UART_IRQ_PRIORITY);
//    NVIC_ClearPendingIRQ(UART_485_EI_IRQN);
//    NVIC_EnableIRQ(UART_485_EI_IRQN);
#if 0	//HC32的rxtimeout中断不好用
    /* Set USART RX timeout error IRQ */
    stcIrqRegiCfg.enIRQn = UART_485_RTOI_IRQN;
    stcIrqRegiCfg.pfnCallback = &Uart_Callback_Idle;
    stcIrqRegiCfg.enIntSrc = UART_485_RTOI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(UART_485_RTOI_IRQN, UART_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(UART_485_RTOI_IRQN);
    NVIC_EnableIRQ(UART_485_RTOI_IRQN);
#endif
    /* Set USART TX IRQ */
    stcIrqRegiCfg.enIRQn = UART_485_TI_IRQN;
    stcIrqRegiCfg.pfnCallback = &Uart_Callback_Tx;
    stcIrqRegiCfg.enIntSrc = UART_485_TI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
//    NVIC_SetPriority(UART_485_TI_IRQN, UART_IRQ_PRIORITY);
//    NVIC_ClearPendingIRQ(UART_485_TI_IRQN);
//    NVIC_EnableIRQ(UART_485_TI_IRQN);
    /* Set USART TX complete IRQ */
    stcIrqRegiCfg.enIRQn = UART_485_TCI_IRQN;
    stcIrqRegiCfg.pfnCallback = &Uart_Callback_TC;
    stcIrqRegiCfg.enIntSrc = UART_485_TCI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
//    NVIC_SetPriority(UART_485_TCI_IRQN, UART_IRQ_PRIORITY);
//    NVIC_ClearPendingIRQ(UART_485_TCI_IRQN);
//    NVIC_EnableIRQ(UART_485_TCI_IRQN);

    /*Enable RX && TX function*/
    USART_FuncCmd(UART_485, UsartRx, Enable);
    USART_FuncCmd(UART_485, UsartTx, Enable);
    USART_FuncCmd(UART_485, UsartRxInt, Enable);
    USART_FuncCmd(UART_485, UsartTimeOut, Enable);
    USART_FuncCmd(UART_485, UsartTimeOutInt, Enable);
}    
//#elif (PROTOCOL_TYPE == CAN_PROTOCOL)
void can_Init(uint32_t bdr)
{
	stc_pwc_ram_cfg_t       stcRamCfg;
    stc_can_init_config_t   stcCanInitCfg;
    stc_can_filter_t        stcFilter;
    stc_can_txframe_t       stcTxFrame;

    stc_irq_regi_conf_t     stcIrqRegiConf;

    MEM_ZERO_STRUCT(stcRamCfg);
    MEM_ZERO_STRUCT(stcCanInitCfg);
    MEM_ZERO_STRUCT(stcFilter);
    MEM_ZERO_STRUCT(stcTxFrame);
	
	//<<Enable can peripheral clock and buffer(ram)
#if 0	//RAM配置? 干嘛用的?
    stcRamCfg.enRamOpMd = HighSpeedMd;
    stcRamCfg.enCan = DynamicCtl;
    PWC_RamCfg(&stcRamCfg);
#endif
    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_CAN, Enable);
	
	//<<Can filter config
 //   stcFilter.enFilterSel = CanFilterSel1;	//禁止所有ID的接收(f460上电默认开启Filter1关闭Filter2-Filter8,只要禁止Filter1就行)
 //   CAN_FilterConfig(&stcFilter, Disable);
	
    //<<CAN GPIO config
    PORT_SetFunc(CAN_RX_PORT, CAN_RX_PIN, CAN_RX_FUNC, Disable);
    PORT_SetFunc(CAN_TX_PORT, CAN_TX_PIN, CAN_TX_FUNC, Disable);
	
    //<<Can bit time config
	can_bdr_Init(bdr, &stcCanInitCfg.stcCanBt);
	
	//err config
	stcCanInitCfg.stcWarningLimit.CanErrorWarningLimitVal = 10u;	//
    stcCanInitCfg.stcWarningLimit.CanWarningLimitVal = 16-1u;	//(N+1)*8
	
	//rx config
    stcCanInitCfg.enCanRxBufAll  = CanRxNormal;			//CanRxNormal: 不存储错误数据; CanRxAll: 错误数据也存储
    stcCanInitCfg.enCanRxBufMode = CanRxBufOverwritten;	//CanRxBufNotStored: 接收buf溢出时,不存储新的数据! CanRxBufOverwritten: 接收buf溢出时,新数据覆盖最早收到的数据!
    stcCanInitCfg.enCanSAck      = CanSelfAckEnable;	//CanSelfAckEnable: LBME=1时,使能自应答功能
    stcCanInitCfg.enCanSTBMode   = CanSTBFifoMode;		//CanSTBPrimaryMode: 优先级模式根据ID自动判断,ID越小优先级越高; CanSTBFifoMode: FIFO模式根据数据帧写入的先后顺序发送

	CAN_Init(&stcCanInitCfg);
	
	//<<Can Irq Enable
    CAN_IrqCmd(CanRxIrqEn, Enable);
    CAN_IrqCmd(CanRxOverIrqEn, Enable);
    //CAN_IrqCmd(CanRxBufFullIrqEn, Enable);
	//CAN_IrqCmd(CanRxBufAlmostFullIrqEn, Enable);
	//CAN_IrqCmd(CanTxPrimaryIrqEn, Enable);
	//CAN_IrqCmd(CanTxSecondaryIrqEn, Enable);
    CAN_IrqCmd(CanErrorIrqEn, Enable);
	CAN_IrqCmd(CanBusErrorIrqEn, Enable);
	//CAN_IrqCmd(CanErrorPassiveIrqEn, Enable);
	CAN_IrqCmd(CanArbiLostIrqEn, Enable);
	
	CAN_IrqFlgClr(CanRxIrqFlg);
	CAN_IrqFlgClr(CanRxBufAlmostFullIrqFlg);
	CAN_IrqFlgClr(CanErrorIrqFlg);
	CAN_IrqFlgClr(CanBusErrorIrqFlg);
	CAN_IrqFlgClr(CanErrorWarningIrqFlg);
	CAN_IrqFlgClr(CanErrorPassivenodeIrqFlg);
	CAN_IrqFlgClr(CanErrorPassiveIrqFlg);
	CAN_IrqFlgClr(CanArbiLostIrqFlg);
	CAN_IrqFlgClr(CanRxBufFullIrqFlg);
	CAN_IrqFlgClr(CanRxOverIrqFlg);

	stcIrqRegiConf.enIRQn = CAN_RX_IRQN;			//中断号
    stcIrqRegiConf.enIntSrc = INT_CAN_INT;			//源
    stcIrqRegiConf.pfnCallback = &CAN_RxIrqCallBack;//回调函数
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_SetPriority(CAN_RX_IRQN, CAN_IRQ_PRIORITY);//优先级
    NVIC_ClearPendingIRQ(CAN_RX_IRQN);
    NVIC_EnableIRQ(CAN_RX_IRQN);
}    


/**
 *******************************************************************************
 ** \brief  Main function of example project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
void flash_Init(void)
{
    /* Unlock EFM. */
    EFM_Unlock();

    /* Enable flash. */
    EFM_FlashCmd(Enable);
    /* Wait flash ready. */
    while(Set != EFM_GetFlagStatus(EFM_FLAG_RDY))
    {
        ;
    }
    /* Lock EFM. */
    EFM_Lock();
}

void Interrupt_Enable(void)
{
	/*所有外设中断放在进入mainLoop前最后开启(尽量避免外部通讯口中断挂起)*/
#if (MOTOR_TYPE == MOTOR_TYPE_BLDC)
	/*边沿中断 or 输入捕捉中断*/
    NVIC_ClearPendingIRQ(M1_HALLA_EIRQ_IRQN);                    /* Clear pending */
    NVIC_SetPriority(M1_HALLA_EIRQ_IRQN, M1_HALL_EIRQ_PRIORITY);   /* Set priority */
    NVIC_EnableIRQ(M1_HALLA_EIRQ_IRQN);                          /* Enable NVIC */

    NVIC_ClearPendingIRQ(M1_HALLB_EIRQ_IRQN);                    /* Clear pending */
    NVIC_SetPriority(M1_HALLB_EIRQ_IRQN, M1_HALL_EIRQ_PRIORITY);   /* Set priority */
    NVIC_EnableIRQ(M1_HALLB_EIRQ_IRQN);                          /* Enable NVIC */

    NVIC_ClearPendingIRQ(M1_HALLC_EIRQ_IRQN);                  /* Clear pending */
    NVIC_SetPriority(M1_HALLC_EIRQ_IRQN, M1_HALL_EIRQ_PRIORITY); /* Set priority */
    NVIC_EnableIRQ(M1_HALLC_EIRQ_IRQN);                            /* Enable NVIC */
#endif

	/*定时器中断*/
    NVIC_SetPriority(TIME0_IRQN, TIME0_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(TIME0_IRQN);
    NVIC_EnableIRQ(TIME0_IRQN);
    
	 /*UART中断*/
    NVIC_SetPriority(UART_MCU_RI_IRQN, UART_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(UART_MCU_RI_IRQN);
    NVIC_EnableIRQ(UART_MCU_RI_IRQN);

    NVIC_SetPriority(UART_MCU_EI_IRQN, UART_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(UART_MCU_EI_IRQN);
    NVIC_EnableIRQ(UART_MCU_EI_IRQN);

    NVIC_SetPriority(UART_MCU_TI_IRQN, UART_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(UART_MCU_TI_IRQN);
    NVIC_EnableIRQ(UART_MCU_TI_IRQN);

    NVIC_SetPriority(UART_MCU_TCI_IRQN, UART_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(UART_MCU_TCI_IRQN);
    NVIC_EnableIRQ(UART_MCU_TCI_IRQN);
	
//#if (PROTOCOL_TYPE == MODBUS_PROTOCOL)
    /*UART中断*/
    NVIC_SetPriority(UART_485_RI_IRQN, UART_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(UART_485_RI_IRQN);
    NVIC_EnableIRQ(UART_485_RI_IRQN);

    NVIC_SetPriority(UART_485_EI_IRQN, UART_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(UART_485_EI_IRQN);
    NVIC_EnableIRQ(UART_485_EI_IRQN);

    NVIC_SetPriority(UART_485_TI_IRQN, UART_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(UART_485_TI_IRQN);
    NVIC_EnableIRQ(UART_485_TI_IRQN);

    NVIC_SetPriority(UART_485_TCI_IRQN, UART_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(UART_485_TCI_IRQN);
    NVIC_EnableIRQ(UART_485_TCI_IRQN);
//#elif (PROTOCOL_TYPE == CAN_PROTOCOL)
	/*CAN中断*/
    NVIC_SetPriority(CAN_RX_IRQN, CAN_IRQ_PRIORITY);//优先级
    NVIC_ClearPendingIRQ(CAN_RX_IRQN);
    NVIC_EnableIRQ(CAN_RX_IRQN);    
//#endif
}

/**
 * @brief  Main function of template project
 * @param  None
 * @retval int32_t return value, if needed
 */

void bor_swdt_Init(void)
{
	/*f460的BOR配置见hc32f46x_icg.h中的宏ICG1_VDU0_REG_CONFIG*/
	
	/*f460的SWDT配置见hc32f46x_icg.h中的宏ICG0_SWDT_REG_CONFIG*/
	/*
	SWDT实用内部专用的RC时钟源 10KHZ
	复位时间=ICG0_SWDT_PERI*ICG0_SWDT_CKS/10k(s)
	目前使用看门狗复位定时800ms
	*/
}

//hz  pwm init
void hz_init(void)
{
    // 使能外设时钟
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIMA2, Enable);
    PWC_Fcg1PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);  // 确认GPIO时钟使能方式

    // 配置PA03为TIMA2_CH4功能（修正复用功能号）
    PORT_SetFunc(PortA, Pin03, Func_Tima0, Disable);     // 确认Func_Tima2_Ch4是否为正确功能号

    // TIMA2时基参数配置
    stc_timera_base_init_t stcTimeraInit;
    MEM_ZERO_STRUCT(stcTimeraInit);
    stcTimeraInit.enClkDiv = TimeraPclkDiv1;
    stcTimeraInit.enCntMode = TimeraCountModeSawtoothWave;
    stcTimeraInit.enCntDir = TimeraCountDirUp;
    stcTimeraInit.enSyncStartupEn = Disable;
    stcTimeraInit.u16PeriodVal = 4000;
    TIMERA_BaseInit(M4_TMRA2, &stcTimeraInit);

    // 通道4比较输出配置（修正极性和占空比计算）
    stc_timera_compare_init_t stcTimerCompareInit;
    MEM_ZERO_STRUCT(stcTimerCompareInit);
    stcTimerCompareInit.u16CompareVal = 1500;      // 50%占空比（根据新极性配置）
    stcTimerCompareInit.enStartCountOutput = TimeraCountStartOutputHigh;
    stcTimerCompareInit.enStopCountOutput = TimeraCountStopOutputLow;
    stcTimerCompareInit.enCompareMatchOutput = TimeraCompareMatchOutputHigh; // 比较匹配时变高
    stcTimerCompareInit.enPeriodMatchOutput = TimeraPeriodMatchOutputLow;    // 周期匹配时变低
    stcTimerCompareInit.enCacheEn = Enable;
    stcTimerCompareInit.enTriangularTroughTransEn = Enable; // 三角波谷点更新缓存
    
    // 初始化通道4比较输出
    TIMERA_CompareInit(M4_TMRA2, TimeraCh4, &stcTimerCompareInit);
    TIMERA_CompareCmd(M4_TMRA2, TimeraCh4, Enable);  // 使能通道4输出
    
    // 启动TIMA2
    TIMERA_Cmd(M4_TMRA2, Enable);
}

// 全局环形缓冲区实例，供ADC采样函数访问
RingBuffer current_buffer;



//flashtest
#include "main.h"
#include "dev_flash.h"
#include "device_manager.h"

// 测试结果变量
volatile uint32_t flash_test_result_1 = 0;
volatile uint32_t flash_operation_status = 0;

/**
 * @brief Flash设备基础功能测试
 */
void Flash_BasicFunctionTest(void)
{
    en_result_t result = Error;
    DeviceCommandData_t cmd;
    FlashWriteParams_t write_params;
    FlashReadParams_t read_params;
    FlashEraseParams_t erase_params;
    
    // 测试地址（使用配置区设备）
    uint32_t test_address = 0x00020000;
    
    // 步骤1：初始化设备
    result = Device_Init(10);  // 初始化配置区Flash设备
    if (result != Ok) {
        flash_test_result_1 = 0xAAAAAAAA; // 初始化失败标记
        return;
    }
    
    // 步骤2：读取原始数据
    memset(&cmd, 0, sizeof(DeviceCommandData_t));
    memset(&read_params, 0, sizeof(FlashReadParams_t));
    
    read_params.address = test_address;
    cmd.cmd = CMD_FLASH_READ_WORD;
    cmd.device_id = 10;
    cmd.params = &read_params;
    cmd.param_size = sizeof(FlashReadParams_t);
    
    result = Device_Control(10, &cmd);


    
    // 步骤3：擦除扇区
    memset(&cmd, 0, sizeof(DeviceCommandData_t));
    memset(&erase_params, 0, sizeof(FlashEraseParams_t));
    
    erase_params.address = test_address;
    cmd.cmd = CMD_FLASH_ERASE_SECTOR;
    cmd.device_id = 10;
    cmd.params = &erase_params;
    cmd.param_size = sizeof(FlashEraseParams_t);
    
    result = Device_Control(10, &cmd);
    flash_operation_status = (result == Ok) ? 0x55555555 : 0xAAAAAAAA;
    
    // 步骤4：验证擦除后数据为0xFFFFFFFF
    memset(&cmd, 0, sizeof(DeviceCommandData_t));
    memset(&read_params, 0, sizeof(FlashReadParams_t));
    
    read_params.address = test_address;
    cmd.cmd = CMD_FLASH_READ_WORD;
    cmd.device_id = 10;
    cmd.params = &read_params;
    cmd.param_size = sizeof(FlashReadParams_t);
    
    result = Device_Control(10, &cmd);
    uint32_t erased_data = read_params.data;
    
    // 步骤5：写入测试数据（无校验）
    memset(&cmd, 0, sizeof(DeviceCommandData_t));
    memset(&write_params, 0, sizeof(FlashWriteParams_t));
    
    write_params.address = test_address;
    write_params.data = 0x12345678;
    cmd.cmd = CMD_FLASH_WRITE_WORD_NOCHECK;
    cmd.device_id = 10;
    cmd.params = &write_params;
    cmd.param_size = sizeof(FlashWriteParams_t);
    
    result = Device_Control(10, &cmd);
    
    // 步骤6：写入测试数据（带校验）
    memset(&cmd, 0, sizeof(DeviceCommandData_t));
    memset(&write_params, 0, sizeof(FlashWriteParams_t));
    
    write_params.address = test_address + 4; // 下一个字地址
    write_params.data = 0x87654321;
    cmd.cmd = CMD_FLASH_WRITE_WORD_CHECK;
    cmd.device_id = 10;
    cmd.params = &write_params;
    cmd.param_size = sizeof(FlashWriteParams_t);
    
    result = Device_Control(10, &cmd);
    
    // 步骤7：验证写入的数据
    memset(&cmd, 0, sizeof(DeviceCommandData_t));
    memset(&read_params, 0, sizeof(FlashReadParams_t));
    
    read_params.address = test_address;
    cmd.cmd = CMD_FLASH_READ_WORD;
    cmd.device_id = 10;
    cmd.params = &read_params;
    cmd.param_size = sizeof(FlashReadParams_t);
    
    result = Device_Control(10, &cmd);
    uint32_t final_data1 = read_params.data;
    
    read_params.address = test_address + 4;
    result = Device_Control(10, &cmd);
    uint32_t final_data2 = read_params.data;
    
    // 存储测试结果
    if ((final_data1 == 0x12345678) && (final_data2 == 0x87654321) && (erased_data == 0xFFFFFFFF)) {
        flash_test_result_1 = 0x55555555; // 测试成功
    } else {
        flash_test_result_1 = 0xAAAAAAAA; // 测试失败
    }
}

/**
 * @brief 多设备操作测试
 */
void Flash_MultiDeviceTest(void)
{
    en_result_t result = Error;
    DeviceCommandData_t cmd;
    FlashWriteParams_t write_params;
    FlashReadParams_t read_params;
    
    // 初始化两个设备
    result = Device_Init(10); // 配置区设备
    if (result != Ok) return;
    
    result = Device_Init(11); // 数据区设备
    if (result != Ok) return;
    
    // 在配置区设备写入数据
    memset(&cmd, 0, sizeof(DeviceCommandData_t));
    memset(&write_params, 0, sizeof(FlashWriteParams_t));
    
    write_params.address = 0x00020000;
    write_params.data = 0x11223344;
    cmd.cmd = CMD_FLASH_WRITE_WORD_CHECK;
    cmd.device_id = 10;
    cmd.params = &write_params;
    cmd.param_size = sizeof(FlashWriteParams_t);
    
    result = Device_Control(10, &cmd);
    
    // 在数据区设备写入数据
    memset(&cmd, 0, sizeof(DeviceCommandData_t));
    memset(&write_params, 0, sizeof(FlashWriteParams_t));
    
    write_params.address = 0x00024000;
    write_params.data = 0x44332211;
    cmd.cmd = CMD_FLASH_WRITE_WORD_CHECK;
    cmd.device_id = 11;
    cmd.params = &write_params;
    cmd.param_size = sizeof(FlashWriteParams_t);
    
    result = Device_Control(11, &cmd);
    
    // 从两个设备读取数据验证
    memset(&cmd, 0, sizeof(DeviceCommandData_t));
    memset(&read_params, 0, sizeof(FlashReadParams_t));
    
    read_params.address = 0x00020000;
    cmd.cmd = CMD_FLASH_READ_WORD;
    cmd.device_id = 10;
    cmd.params = &read_params;
    cmd.param_size = sizeof(FlashReadParams_t);
    
    result = Device_Control(10, &cmd);
    uint32_t config_data = read_params.data;
    
    read_params.address = 0x00024000;
    cmd.device_id = 11;
    result = Device_Control(11, &cmd);
    uint32_t data_area_data = read_params.data;
    
    // 验证多设备操作结果
    if ((config_data == 0x11223344) && (data_area_data == 0x44332211)) {
        flash_test_result_1 = 0x55555555;
    } else {
        flash_test_result_1 = 0xAAAAAAAA;
    }
}

/**
 * @brief 统计信息获取测试
 */
void Flash_StatisticsTest(void)
{
    en_result_t result = Error;
    DeviceCommandData_t cmd;
    FlashStatistics_t stats;
    FlashDeviceData_t device_data;
    
    // 初始化设备
    result = Device_Init(10);
    if (result != Ok) return;
    
    // 方法1：通过Device_Read获取设备数据（包含统计信息）
    result = Device_Read(10, (void*)&device_data, sizeof(FlashDeviceData_t));
    if (result == Ok) {
        // 检查设备数据中的统计信息
        if (device_data.statistics.total_operations >= 0) {
            flash_operation_status = 0x11111111;
        }
    }
    
    // 方法2：通过控制命令获取详细统计信息
    memset(&cmd, 0, sizeof(DeviceCommandData_t));
    cmd.cmd = CMD_FLASH_GET_STATISTICS;
    cmd.device_id = 10;
    cmd.params = &stats;
    cmd.param_size = sizeof(FlashStatistics_t);
    
    result = Device_Control(10, &cmd);
    if (result == Ok) {
        // 验证统计信息结构有效
        if (stats.total_operations >= 0) {
            flash_test_result_1 = 0x11111111;
        }
    }
}

/**
 * @brief 操作历史测试
 */
void Flash_OperationHistoryTest(void)
{
    en_result_t result = Error;
    DeviceCommandData_t cmd;
    FlashOperationRecord_t history[FLASH_OPERATION_HISTORY_SIZE];
    uint8_t history_count = 0;
    
    // 初始化设备
    result = Device_Init(10);
    if (result != Ok) return;
    
    // 执行一些操作来生成历史记录
    FlashWriteParams_t write_params;
    FlashReadParams_t read_params;
    
    // 写入操作
    memset(&cmd, 0, sizeof(DeviceCommandData_t));
    memset(&write_params, 0, sizeof(FlashWriteParams_t));
    
    write_params.address = 0x00020010;
    write_params.data = 0xABCD1234;
    cmd.cmd = CMD_FLASH_WRITE_WORD_CHECK;
    cmd.device_id = 10;
    cmd.params = &write_params;
    cmd.param_size = sizeof(FlashWriteParams_t);
    
    result = Device_Control(10, &cmd);
    
    // 读取操作
    memset(&cmd, 0, sizeof(DeviceCommandData_t));
    memset(&read_params, 0, sizeof(FlashReadParams_t));
    
    read_params.address = 0x00020010;
    cmd.cmd = CMD_FLASH_READ_WORD;
    cmd.device_id = 10;
    cmd.params = &read_params;
    cmd.param_size = sizeof(FlashReadParams_t);
    
    result = Device_Control(10, &cmd);
    
    // 获取操作历史
    memset(&cmd, 0, sizeof(DeviceCommandData_t));
    cmd.cmd = CMD_FLASH_GET_OPERATION_HISTORY;
    cmd.device_id = 10;
    cmd.params = history;
    cmd.param_size = sizeof(FlashOperationRecord_t) * FLASH_OPERATION_HISTORY_SIZE;
    
    result = Device_Control(10, &cmd);
    if (result == Ok) {
        // 检查历史记录是否有效
        for (int i = 0; i < FLASH_OPERATION_HISTORY_SIZE; i++) {
            if (history[i].sequence_number > 0) {
                history_count++;
            }
        }
        
        if (history_count >= 2) { // 至少应该有我们刚才的两次操作
            flash_test_result_1 = 0x22222222;
        }
    }
}

/**
 * @brief 寿命信息测试
 */
void Flash_LifetimeInfoTest(void)
{
    en_result_t result = Error;
    DeviceCommandData_t cmd;
    FlashLifetimeInfo_t lifetime_info;
    
    // 初始化设备
    result = Device_Init(10);
    if (result != Ok) return;
    
    // 获取寿命信息
    memset(&cmd, 0, sizeof(DeviceCommandData_t));
    cmd.cmd = CMD_FLASH_GET_LIFETIME_INFO;
    cmd.device_id = 10;
    cmd.params = &lifetime_info;
    cmd.param_size = sizeof(FlashLifetimeInfo_t);
    
    result = Device_Control(10, &cmd);
    if (result == Ok) {
        // 验证寿命信息结构有效
        if (lifetime_info.max_erase_cycles > 0 && 
            lifetime_info.health_status <= 100) {
            flash_test_result_1 = 0x33333333;
            flash_operation_status = lifetime_info.health_status;
        }
    }
}

/**
 * @brief 重置统计信息测试
 */
void Flash_ResetStatisticsTest(void)
{
    en_result_t result = Error;
    DeviceCommandData_t cmd;
    FlashStatistics_t stats_before, stats_after;
    
    // 初始化设备
    result = Device_Init(10);
    if (result != Ok) return;
    
    // 获取重置前的统计信息
    memset(&cmd, 0, sizeof(DeviceCommandData_t));
    cmd.cmd = CMD_FLASH_GET_STATISTICS;
    cmd.device_id = 10;
    cmd.params = &stats_before;
    cmd.param_size = sizeof(FlashStatistics_t);
    
    result = Device_Control(10, &cmd);
    
    // 重置统计信息
    memset(&cmd, 0, sizeof(DeviceCommandData_t));
    cmd.cmd = CMD_FLASH_RESET_STATISTICS;
    cmd.device_id = 10;
    cmd.params = NULL;
    cmd.param_size = 0;
    
    result = Device_Control(10, &cmd);
    
    // 获取重置后的统计信息
    memset(&cmd, 0, sizeof(DeviceCommandData_t));
    cmd.cmd = CMD_FLASH_GET_STATISTICS;
    cmd.device_id = 10;
    cmd.params = &stats_after;
    cmd.param_size = sizeof(FlashStatistics_t);
    
    result = Device_Control(10, &cmd);
    
    // 验证统计信息已重置
    if (result == Ok && stats_after.total_operations == 0 && 
        stats_after.erase_count == 0 && stats_after.write_count == 0) {
        flash_test_result_1 = 0x44444444;
    }
}

/**
 * @brief 错误处理测试
 */
void Flash_ErrorHandlingTest(void)
{
    en_result_t result = Error;
    DeviceCommandData_t cmd;
    FlashWriteParams_t write_params;
    FlashStatistics_t stats;
    
    // 初始化设备
    result = Device_Init(10);
    if (result != Ok) return;
    
    // 获取初始错误计数
    memset(&cmd, 0, sizeof(DeviceCommandData_t));
    cmd.cmd = CMD_FLASH_GET_STATISTICS;
    cmd.device_id = 10;
    cmd.params = &stats;
    cmd.param_size = sizeof(FlashStatistics_t);
    
    result = Device_Control(10, &cmd);
    uint32_t initial_errors = stats.error_count;
    
    // 尝试非法操作：不对齐的地址写入
    memset(&cmd, 0, sizeof(DeviceCommandData_t));
    memset(&write_params, 0, sizeof(FlashWriteParams_t));
    
    write_params.address = 0x00020001; // 不对齐的地址
    write_params.data = 0x12345678;
    cmd.cmd = CMD_FLASH_WRITE_WORD_CHECK;
    cmd.device_id = 10;
    cmd.params = &write_params;
    cmd.param_size = sizeof(FlashWriteParams_t);
    
    result = Device_Control(10, &cmd);
    
    // 获取操作后的错误计数
    memset(&cmd, 0, sizeof(DeviceCommandData_t));
    cmd.cmd = CMD_FLASH_GET_STATISTICS;
    cmd.device_id = 10;
    cmd.params = &stats;
    cmd.param_size = sizeof(FlashStatistics_t);
    
    result = Device_Control(10, &cmd);
    
    // 验证错误计数增加
    if (stats.error_count > initial_errors) {
        flash_test_result_1 = 0x55555555;
        flash_operation_status = stats.consecutive_errors;
    }
}

/**
 * @brief 综合测试函数
 */
void Flash_ComprehensiveTest(void)
{
    // 先注册设备
    Flash_Device_Registration();
    
    // // 执行各种测试
    Flash_BasicFunctionTest();    
    Flash_BasicFunctionTest();
    if (flash_test_result_1 != 0x55555555) return;
    
    // Flash_MultiDeviceTest();
    // if (flash_test_result_1 != 0x55555555) return;
    
    Flash_StatisticsTest();
    if (flash_test_result_1 != 0x11111111) return;
    
    Flash_OperationHistoryTest();
    if (flash_test_result_1 != 0x22222222) return;
    
    Flash_LifetimeInfoTest();
    if (flash_test_result_1 != 0x33333333) return;
    
    Flash_ResetStatisticsTest();
    if (flash_test_result_1 != 0x44444444) return;
    
    Flash_ErrorHandlingTest();
    if (flash_test_result_1 != 0x55555555) return;
    
    // 所有测试通过
    flash_test_result_1 = 0xFFFFFFFF; // 所有测试成功标记
}

/**
 * @brief 简单应用示例：数据存储和读取
 */
void Flash_DataStorageExample(void)
{
    en_result_t result = Error;
    DeviceCommandData_t cmd;
    FlashWriteParams_t write_params;
    FlashReadParams_t read_params;
    FlashEraseParams_t erase_params;
    
    // 配置参数结构
    typedef struct {
        uint32_t version;
        uint32_t config_value;
        uint32_t checksum;
    } AppConfig_t;
    
    AppConfig_t config_to_save = {
        .version = 0x00010001,
        .config_value = 0x12345678,
        .checksum = 0
    };
    
    // 计算校验和
    config_to_save.checksum = config_to_save.version ^ config_to_save.config_value;
    
    // 初始化设备
    result = Device_Init(10);
    if (result != Ok) return;
    
    // 擦除配置扇区
    memset(&cmd, 0, sizeof(DeviceCommandData_t));
    memset(&erase_params, 0, sizeof(FlashEraseParams_t));
    
    erase_params.address = 0x00020000;
    cmd.cmd = CMD_FLASH_ERASE_SECTOR;
    cmd.device_id = 10;
    cmd.params = &erase_params;
    cmd.param_size = sizeof(FlashEraseParams_t);
    
    result = Device_Control(10, &cmd);
    if (result != Ok) return;
    
    // 保存配置数据
    uint32_t* config_data = (uint32_t*)&config_to_save;
    for (int i = 0; i < sizeof(AppConfig_t) / 4; i++) {
        memset(&cmd, 0, sizeof(DeviceCommandData_t));
        memset(&write_params, 0, sizeof(FlashWriteParams_t));
        
        write_params.address = 0x00020000 + (i * 4);
        write_params.data = config_data[i];
        cmd.cmd = CMD_FLASH_WRITE_WORD_CHECK;
        cmd.device_id = 10;
        cmd.params = &write_params;
        cmd.param_size = sizeof(FlashWriteParams_t);
        
        result = Device_Control(10, &cmd);
        if (result != Ok) return;
    }
    
    // 读取并验证配置数据
    AppConfig_t read_back_config;
    uint32_t* read_data = (uint32_t*)&read_back_config;
    
    for (int i = 0; i < sizeof(AppConfig_t) / 4; i++) {
        memset(&cmd, 0, sizeof(DeviceCommandData_t));
        memset(&read_params, 0, sizeof(FlashReadParams_t));
        
        read_params.address = 0x00020000 + (i * 4);
        cmd.cmd = CMD_FLASH_READ_WORD;
        cmd.device_id = 10;
        cmd.params = &read_params;
        cmd.param_size = sizeof(FlashReadParams_t);
        
        result = Device_Control(10, &cmd);
        if (result != Ok) return;
        
        read_data[i] = read_params.data;
    }
    
    // 验证数据完整性
    uint32_t calculated_checksum = read_back_config.version ^ read_back_config.config_value;
    
    if (read_back_config.version == config_to_save.version &&
        read_back_config.config_value == config_to_save.config_value &&
        read_back_config.checksum == calculated_checksum) {
        flash_test_result_1 = 0x88888888; // 数据存储成功
    } else {
        flash_test_result_1 = 0x99999999; // 数据验证失败
    }
}

/**
 * @brief 主测试入口函数
 */
void Run_All_Flash_Tests(void)
{
    // 执行综合测试
    Flash_ComprehensiveTest();
    
    // 如果综合测试失败，执行基础功能测试
    if (flash_test_result_1 != 0xFFFFFFFF) {
        Flash_BasicFunctionTest();
    }
    
    // 如果基础功能正常，执行数据存储示例
    if (flash_test_result_1 == 0x55555555) {
        Flash_DataStorageExample();
    }
}
#include "uds_diagnostic.h"

void uds_dl_init_fw(void);
#include "can_adapter.h"
static CAN_CTRL_t g_can_ctrl;           /* CAN 控制器实例 */
static volatile uint32_t g_sys_tick_ms = 0;  /* 系统 tick 计数 */
NonBlockingDelay_t Test;
volatile int tt = 0;
int32_t main(void)
{
    SystemClk_Init();
    SysMainState_Init();
    Hardware_Init();
    ring_buffer_init(&current_buffer);


    can_hInit(&g_can_ctrl, M4_CAN, CAN1);
    can_Init(250000);

    /* 初始化 UDS 诊断协议 */
    uds_init();
	/*can控制器初始化*/
    system_Commsg_Init();   
    timer0_Init(1000, s_tickCount_Reset);  //100000 即1000ms
    // nbDelay_Init(&Test,4000);
    // nbDelay_Start(&Test);
    // PORT_Toggle(GPIO_LED_PORT1, GPIO_LED_PIN1);


    // DeviceManager_Init();
    
    // // 注册Flash设备
    // Flash_Device_Registration();

    // // 执行Flash测试
    // Application_Flash_TestSequence();

    // Application_Flash_DataAreaTest();


    // Run_All_Flash_Tests();
    // 初始化固件升级模块
    FlashDownloadConfig_t fw_config = {
        .max_firmware_size = 256 * 1024,
        .flash_sector_size = 0x2000,
        .verify_enabled = 1,
        .auto_reset_on_complete = 0
    };
    FlashDownload_Init(&fw_config);
    /* 初始化 ISO-TP 层 */
    isotp_init(CAN1);  // CAN1 是你的 CAN 通道号
    uds_dl_init_fw();  // 注册固件下载接口
    while(1)
    {
        /* UDS异步接收: CAN ISR存数据到全局缓冲区并置标志, 主循环处理 */
        if (g_uds_rx_pending)
        {
            uds_receive_handler(CAN1, g_uds_rx_can_id, g_uds_rx_buffer, g_uds_rx_len);
            g_uds_rx_pending = 0;
        }
        
        uds_process();
        FlashDownload_Task();
        isotp_tx_process();

    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
