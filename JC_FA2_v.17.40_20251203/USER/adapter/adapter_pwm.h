#ifndef ADAPTER_PWM_H_
#define ADAPTER_PWM_H_

#include "main.h"
#include "hc32f46x_timera.h"
#include "gpio_adapter.h"

#define TIMERA_CALC_REG_ADDR(reg, chl)          ((uint32_t)(&(reg)) + (chl)*0x4u)


// 定义函数指针类型
typedef en_result_t (*pwm_set_frequency_t)(M4_TMRA_TypeDef*, uint16_t);
typedef uint16_t (*pwm_get_frequency_t)(M4_TMRA_TypeDef*);
typedef en_result_t (*pwm_set_duty_t)(M4_TMRA_TypeDef*, en_timera_channel_t, float);
typedef float (*pwm_get_duty_t)(M4_TMRA_TypeDef*, en_timera_channel_t);
typedef void (*pwm_compare_enable_t)(M4_TMRA_TypeDef*, en_timera_channel_t);
typedef void (*pwm_compare_disable_t)(M4_TMRA_TypeDef*, en_timera_channel_t);
typedef void (*pwm_timer_enable_t)(M4_TMRA_TypeDef*);
typedef void (*pwm_timer_disable_t)(M4_TMRA_TypeDef*);
typedef void (*pwm_init_t)(en_port_t, en_pin_t, M4_TMRA_TypeDef*, en_timera_channel_t, float, bool);

// 定义操作集结构体
typedef struct {
    pwm_init_t init;
    pwm_set_frequency_t set_frequency;
    pwm_get_frequency_t get_frequency;
    pwm_set_duty_t set_duty;
    pwm_get_duty_t get_duty;
    pwm_compare_enable_t compare_enable;
    pwm_compare_disable_t compare_disable;
    pwm_timer_enable_t timer_enable;
    pwm_timer_disable_t timer_disable;
} pwm_ops_t;

// 声明驱动层的操作集实例
extern pwm_ops_t hz_pwm_ops;

// 原有的函数声明（这些函数将被操作集中的函数指针指向）
void PWM_Init(en_port_t port, en_pin_t pin, 
              M4_TMRA_TypeDef* Timer, 
              en_timera_channel_t timerChannel,
              float pwmInitDuty,
              bool polarity);
en_result_t PWM_setFrequency(M4_TMRA_TypeDef* Timer, uint16_t freqHz);
uint16_t PWM_getFrequency(M4_TMRA_TypeDef* Timer);
en_result_t PWM_SetDuty(M4_TMRA_TypeDef* Timer, en_timera_channel_t timerChannel, float duty);
float PWM_GetCurrentDuty(M4_TMRA_TypeDef* Timer, en_timera_channel_t timerChannel);
void PWMCompare_Enable(M4_TMRA_TypeDef* Timer, en_timera_channel_t timerChannel);
void PWMCompare_Disable(M4_TMRA_TypeDef* Timer, en_timera_channel_t timerChannel);
void PWMTimerA_Enable(M4_TMRA_TypeDef* Timer);
void PWMTimerA_Disable(M4_TMRA_TypeDef* Timer);

#endif
