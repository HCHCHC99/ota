/********************************文件说明*************************************
*文件名: gpio_adapter.h

*作者: Yuchen Tan

*版本: V1.0.0

*功能简介:

*备注: 无

*修改履历:

*****************************************************************************/
#ifndef GPIO_ADAPTER_H_
#define GPIO_ADAPTER_H_

/*****************************文件包含(公开)**********************************
*
*备注: 可在此包含底层的库文件
*
*****************************************************************************/
#include "common.h"
/******************************宏定义(公开)***********************************
*
*备注: 需要被外部使用或可任意修改的宏在这里定义
*
*****************************************************************************/

/**************************数据类型及结构定义(公开)***************************
*
*备注: 需要被外部使用的数据结构及类型在这里定义
*
*****************************************************************************/
/*引脚电平类型定义*/
typedef enum
{
    E_GPIO_PIN_RESET = 0,
    E_GPIO_PIN_SET,
}GPIO_PIN_LEVEL_t;

/*引脚功能类型定义*/
typedef enum
{
    E_GPIO_FUNC_INPUT = 0,
    E_GPIO_FUNC_OUTPUT_PP,
    E_GPIO_FUNC_OUTPUT_OD,
}GPIO_PIN_FUNC_t;

/*与MCU平台相关的抽象类型定义*/
#if (MCU_TYPE == MCU_TYPE_STM32)
typedef GPIO_TypeDef*   GPIO_PORT_t;
typedef uint32_t        GPIO_PIN_t;
typedef uint32_t        GPIO_FUNC_t;

#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
typedef en_gpio_port_t  GPIO_PORT_t;
typedef en_gpio_pin_t   GPIO_PIN_t;
typedef en_gpio_af_t    GPIO_FUNC_t;

#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
typedef en_port_t       GPIO_PORT_t;
typedef en_pin_t        GPIO_PIN_t;
typedef en_port_func_t  GPIO_FUNC_t;

#endif
/*****************************函数声明(公开)**********************************
*
*备注: 需要被外部使用的接口函数在这里声明
*
*****************************************************************************/
/*GPIO引脚模式选择*/
void gpio_adapter_Mode_Sel(GPIO_PORT_t Port, GPIO_PIN_t Pin, uint8_t Mode);
/*GPIO输出引脚输出高电平*/
void gpio_adapter_Set_Pin(GPIO_PORT_t Port, GPIO_PIN_t Pin);
/*GPIO输出引脚输出低电平*/
void gpio_adapter_Reset_Pin(GPIO_PORT_t Port, GPIO_PIN_t Pin);
/*GPIO输出引脚电平翻转*/
void gpio_adapter_Toggle_Pin(GPIO_PORT_t Port, GPIO_PIN_t Pin);
/*GPIO输入引脚电平读取*/
GPIO_PIN_LEVEL_t gpio_adapter_Read_Pin(GPIO_PORT_t Port, GPIO_PIN_t Pin);
/*GPIO引脚复用功能选择*/
void gpio_adapter_Fuc_Sel(GPIO_PORT_t Port, GPIO_PIN_t Pin, GPIO_FUNC_t Func, uint8_t FuncEn);
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
