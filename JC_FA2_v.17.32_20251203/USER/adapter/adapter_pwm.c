#include "adapter_pwm.h"
#include "hz_timer.h"
#include <math.h>


#define PWM_START_FREQ                  (15000)                                     // 初始PWM波的频率
#define PCLK_FREQ_HZ                    (8000000u)                                  // PCLK为8MHz
#define CALC_PERIOD(PWM_START_FREQ)     ((PCLK_FREQ_HZ / (PWM_START_FREQ)) - 1)     // 频率对应的溢出值
#define FREQ_MIN                        (10000)                                     // 最小频率（PWM_setFrequency）
#define FREQ_MAX                        (20000)                                     // 最大频率（PWM_setFrequency）
#define DUTY_MIN                        (0.f)
#define DUTY_MAX                        (100.f)

pwm_ops_t hz_pwm_ops = {
    .init = PWM_Init,
    .set_frequency = PWM_setFrequency,
    .get_frequency = PWM_getFrequency,
    .set_duty = PWM_SetDuty,
    .get_duty = PWM_GetCurrentDuty,
    .compare_enable = PWMCompare_Enable,
    .compare_disable = PWMCompare_Disable,
    .timer_enable = PWMTimerA_Enable,
    .timer_disable = PWMTimerA_Disable
};
// 每个定时器模块的周期值存储
typedef struct {
    M4_TMRA_TypeDef* Timer;
    uint16_t period;
} TimerPeriod_t;

static TimerPeriod_t s_timerPeriods[4] = {0}; // 假设最多4个定时器模块
static uint8_t s_numTimers = 0;

static uint16_t* GetPeriodValue(M4_TMRA_TypeDef* Timer)
{
    for(int i = 0; i < s_numTimers; i++) {
        if(s_timerPeriods[i].Timer == Timer) {
            return &s_timerPeriods[i].period;
        }
    }
    
    if(s_numTimers < 4) {
        s_timerPeriods[s_numTimers].Timer = Timer;
        s_timerPeriods[s_numTimers].period = CALC_PERIOD(PWM_START_FREQ);  
        return &s_timerPeriods[s_numTimers++].period;
    }
    
    return NULL;
}

void PWM_Init(en_port_t port, en_pin_t pin, 
              M4_TMRA_TypeDef* Timer, 
              en_timera_channel_t timerChannel,
              float pwmInitDuty,
              bool polarity)
{
    // 启用 Timer 时钟和引脚复用
    // 根据Timer选择对应的时钟使能位
    uint32_t timeraPeriph;
    
    if(Timer == M4_TMRA1) {
        timeraPeriph = PWC_FCG2_PERIPH_TIMA1;
    }
    else if(Timer == M4_TMRA2) {
        timeraPeriph = PWC_FCG2_PERIPH_TIMA2;
    }
    else if(Timer == M4_TMRA3) {
        timeraPeriph = PWC_FCG2_PERIPH_TIMA3;
    }
    else if(Timer == M4_TMRA4) {
        timeraPeriph = PWC_FCG2_PERIPH_TIMA4;
    }
    else {
        // 无效的定时器模块
        return;
    }
    // 启用 Timer 时钟
    PWC_Fcg2PeriphClockCmd(timeraPeriph, Enable);

    // 引脚复用配置（需要根据实际Timer调整Func_Timax）
    PORT_SetFunc(port, pin, Func_Tima0, Disable); 

    // 获取周期值
    uint16_t* period = GetPeriodValue(Timer);
    if(!period) return;

    // 配置 Timer 基础参数
    stc_timera_base_init_t stcTimeraInit;
    MEM_ZERO_STRUCT(stcTimeraInit);
    stcTimeraInit.enClkDiv = TimeraPclkDiv1; // PCLK / 1 = 8MHz
    stcTimeraInit.enCntMode = TimeraCountModeSawtoothWave;
    stcTimeraInit.enCntDir = TimeraCountDirUp;
    stcTimeraInit.enSyncStartupEn = Disable;
    stcTimeraInit.u16PeriodVal = *period; // 周期值直接写入寄存器
    TIMERA_BaseInit(Timer, &stcTimeraInit);

    stc_timera_compare_init_t stcCompare;
    MEM_ZERO_STRUCT(stcCompare);
    // 计算初始比较值：增加四舍五入并减1
    stcCompare.u16CompareVal = (uint16_t)round((*period + 1) * pwmInitDuty * 0.01f) - 1; 
    
    // 根据极性参数配置输出模式
    if(!polarity) {
        stcCompare.enCompareMatchOutput = TimeraCompareMatchOutputHigh;
        stcCompare.enPeriodMatchOutput = TimeraPeriodMatchOutputLow;
    } else {
        stcCompare.enCompareMatchOutput = TimeraCompareMatchOutputLow;
        stcCompare.enPeriodMatchOutput = TimeraPeriodMatchOutputHigh;
    }
    
    TIMERA_CompareInit(Timer, timerChannel, &stcCompare);
    // PWMCompare_Enable(timerChannel);
    //开启定时器
    PWMTimerA_Enable(Timer);
}

en_result_t PWM_setFrequency(M4_TMRA_TypeDef* Timer, uint16_t freqHz) {
    /* 参数合法性检查 */
    if (freqHz < FREQ_MIN || freqHz > FREQ_MAX) {
        return ErrorInvalidParameter;
    }

    /* 获取周期值 */
    uint16_t* period = GetPeriodValue(Timer);
    if(!period) return ErrorInvalidParameter;

    /* 计算新的周期值 */
    uint32_t newPeriod = (PCLK_FREQ_HZ / freqHz) - 1;

    /* 获取当前占空比：修正为(CMP + 1)计算实际占空比 */
    stc_tmra_cmpar_field_t *pCompareReg = 
        (stc_tmra_cmpar_field_t *)TIMERA_CALC_REG_ADDR(Timer->CMPAR1, TimeraCh1);//可以默认为通道1，因为调整频率是定时器模块级别的，同一定时器模块下所有通道频率一致
    float currentDuty = ((pCompareReg->CMP + 1) * 100.0f) / (*period + 1);
    
    /* 暂停PWM输出 */
    TIMERA_Cmd(Timer, Disable);
    
    /* 更新周期值和全局变量 */
    *period = (uint16_t)newPeriod;
    TIMERA_SetPeriodValue(Timer, *period);
    
    /* 根据新周期和原占空比计算新的比较值：增加四舍五入并减1 */
    uint16_t newCompareVal = (uint16_t)round((*period + 1) * currentDuty / 100.0f) - 1;
    TIMERA_SetCompareValue(Timer, TimeraCh1, newCompareVal);
    
    /* 重新启用PWM输出 */
    TIMERA_Cmd(Timer, Enable);
    
    return Ok;
}

uint16_t PWM_getFrequency(M4_TMRA_TypeDef* Timer)
{
    /* 参数合法性检查 */
    if (Timer == NULL) {
        return 0;
    }

    /* 获取周期值 */
    uint16_t* period = GetPeriodValue(Timer);
    if(!period) return 0;

    /* 计算频率：频率 = PCLK_FREQ_HZ / (周期值 + 1) */
    return (uint16_t)(PCLK_FREQ_HZ / (*period + 1));
}

// en_result_t PWM_SetDuty(M4_TMRA_TypeDef* Timer, en_timera_channel_t timerChannel, float duty) {
//     /* 参数合法性检查 */
//     if (duty < DUTY_MIN || duty > DUTY_MAX) {
//         return ErrorInvalidParameter;
//     }
    
//     /* 获取周期值 */
//     uint16_t* period = GetPeriodValue(Timer);
//     if(!period) return ErrorInvalidParameter;
    
//     /* 计算目标比较值：增加四舍五入并减1 */
//     uint16_t cmpVal = (uint16_t)round((*period + 1) * duty / 100.0f) - 1;
    
//     /* 设置比较值 */
//     return TIMERA_SetCompareValue(Timer, timerChannel, cmpVal);
// }
en_result_t PWM_SetDuty(M4_TMRA_TypeDef* Timer, en_timera_channel_t timerChannel, float duty) {
    /* 参数合法性检查 */
    if (duty < 0.0f || duty > 100.0f) {
        return ErrorInvalidParameter;
    }
    
    /* 获取周期值 */
    uint16_t* period = GetPeriodValue(Timer);
    if(!period) return ErrorInvalidParameter;
    
    uint16_t cmpVal;
    
    /* 假设PWM配置为高电平有效 */
    if (duty <= 0.0f) {
        // 0%占空比：输出持续低电平
        cmpVal = 0;
    } else if (duty >= 100.0f) {
        // 100%占空比：输出持续高电平
        cmpVal = *period;
    } else {
        // 正常占空比计算
        cmpVal = (uint16_t)round((*period) * duty / 100.0f);
    }
    
    return TIMERA_SetCompareValue(Timer, timerChannel, cmpVal);
}

float PWM_GetCurrentDuty(M4_TMRA_TypeDef* Timer, en_timera_channel_t timerChannel) {
    /* 获取周期值 */
    uint16_t* period = GetPeriodValue(Timer);
    if(!period) return 0.0f;
    
    /* 获取当前比较值：修正为(CMP + 1)计算实际占空比 */
    stc_tmra_cmpar_field_t *pCompareReg = 
        (stc_tmra_cmpar_field_t *)TIMERA_CALC_REG_ADDR(Timer->CMPAR1, timerChannel);
    return ((pCompareReg->CMP + 1) * 100.0f) / (*period + 1);
}

void PWMCompare_Enable(M4_TMRA_TypeDef* Timer, en_timera_channel_t timerChannel) {
    TIMERA_CompareCmd(Timer, timerChannel, Enable);
}

void PWMCompare_Disable(M4_TMRA_TypeDef* Timer, en_timera_channel_t timerChannel) {
    TIMERA_CompareCmd(Timer, timerChannel, Disable);
}

void PWMTimerA_Enable(M4_TMRA_TypeDef* Timer) {
    TIMERA_Cmd(Timer, Enable);
}

void PWMTimerA_Disable(M4_TMRA_TypeDef* Timer) {
    TIMERA_Cmd(Timer, Disable);
}
