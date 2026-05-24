#include "dev_timer0.h"
#include "hc32f46x_interrupts.h"
#include "hc32f46x_clk.h"
#include "hc32f46x_pwc.h"
#include "main.h"
#include "system.h"
#include "hz_timer.h"


/*! Macro to convert a microsecond period to raw count value */
#define USEC_TO_COUNT(us, clockFreqInHz) (uint16_t)(((uint64_t)(us) * (clockFreqInHz)) / 1000000U)
/*! Macro to convert a raw count value to microsecond */
#define COUNT_TO_USEC(count, clockFreqInHz) (uint64_t)((uint64_t)(count) * 1000000U / (clockFreqInHz))
/*! Macro to convert a millisecond period to raw count value */
#define MSEC_TO_COUNT(ms, clockFreqInHz) (uint64_t)((uint64_t)(ms) * (clockFreqInHz) / 1000U)
/*! Macro to convert a raw count value to millisecond */
#define COUNT_TO_MSEC(count, clockFreqInHz) (uint64_t)((uint64_t)(count) * 1000U / (clockFreqInHz))


/**
 *******************************************************************************
 ** \brief  Main function of example project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
void timer0_Init(uint32_t us, ReCount cmd)
{
    stc_tim0_base_init_t stcTimerCfg;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_tim0_trigger_init_t StcTimer0TrigInit;

    MEM_ZERO_STRUCT(stcTimerCfg);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(StcTimer0TrigInit);

    uint32_t u32Pclk1;
    stc_clk_freq_t stcClkTmp;

    /* Get pclk1 */
    CLK_GetClockFreq(&stcClkTmp);
    u32Pclk1 = stcClkTmp.pclk1Freq;
    
    /* Timer0 peripheral enable */
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM01, Enable);

    /* Clear CNTAR register for channel A */
    TIMER0_WriteCntReg(M4_TMR01, Tim0_ChannelB, 0u);

    /*config register for channel B */
    stcTimerCfg.Tim0_CounterMode = Tim0_Sync;//Sync 同步时钟源  ； Async异步时钟源
    stcTimerCfg.Tim0_SyncClockSource = Tim0_Pclk1;
    //  stcTimerCfg.Tim0_AsyncClockSource = Tim0_XTAL32;
    stcTimerCfg.Tim0_ClockDivision = Tim0_ClkDiv64;
    stcTimerCfg.Tim0_CmpValue = USEC_TO_COUNT(us,u32Pclk1/64) - 1;//(us*pclk1/Timdiv)-1 <= 65535
    TIMER0_BaseInit(M4_TMR01,Tim0_ChannelB,&stcTimerCfg);


    /* Clear compare flag */
    TIMER0_ClearFlag(M4_TMR01, Tim0_ChannelB);
    TIMER0_IntCmd(M4_TMR01,Tim0_ChannelB,Enable);

    /* Register TMR_INI_GCMB Int to Vect.No.009 */
    stcIrqRegiConf.enIRQn = TIME0_IRQN;
    /* Select I2C Error or Event interrupt function */
    stcIrqRegiConf.enIntSrc = TIME_GCM_NUM;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = &Timer01B_CallBack;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
//    /* Clear Pending */
   NVIC_ClearPendingIRQ(TIME0_IRQN);
//    /* Set priority */
   NVIC_SetPriority(TIME0_IRQN, TIME0_IRQ_PRIORITY);
//    /* Enable NVIC */
   NVIC_EnableIRQ(TIME0_IRQN);

    /*start timer0*/
    TIMER0_Cmd(M4_TMR01,Tim0_ChannelB,Enable);

    if(cmd==s_tickCount_Reset)
    {
        tickTimer_Init();  // 初始化滴答计数  
    }
    
}

void Timer0_Disable(M4_TMR0_TypeDef* timer, en_tim0_channel_t channel)
{
    // 停止计数器
    TIMER0_Cmd(timer, channel, Disable);
    
    // 关闭中断
    TIMER0_IntCmd(timer, channel, Disable);
    
    // 清除可能挂起的标志位
    TIMER0_ClearFlag(timer, channel);
}
