/********************************文件说明*************************************
*文件名: common.h

*作者: Yuchen Tan

*版本: V1.0.3

*功能简介:

*备注: 无

*修改履历:
------------------------------------V1.0.2------------------------------------
20220623: 把MCU_TYPE的定义从mc_common.h移动到common.h
------------------------------------V1.0.3------------------------------------
20220705: 头文件包含根据MCU_TYPE自行切换(MCU_TYPE的宏定义需放到文件顶部)
20231109: 新增宏MCU_DRIVER_LIB,用于适配同一芯片有不同驱动库版本(eg: hc32f460),
导致同一功能的官方底层接口原型不同,需要在adapter层区分调用的情况!
*****************************************************************************/
#ifndef COMMON_H_
#define COMMON_H_

/******************************宏定义(公开)***********************************
*
*备注: 需要被外部使用或可任意修改的宏在这里定义
*
*****************************************************************************/
/*程序中标注NEED_NOTE的地方建议后续写文档进行整理,一般是重要bug修复*/
#define NEED_NOTE   (1)

/*程序运行MCU平台选择*/
#define MCU_TYPE_STM32      (1)
#define MCU_TYPE_HC32_F0    (2)
#define MCU_TYPE_HC32_F4    (3)
#define MCU_TYPE_HC32_L1    (4)

#define MCU_TYPE            MCU_TYPE_HC32_F4

//F460有2个版本的驱动库
#if (MCU_TYPE == MCU_TYPE_HC32_F4)
#define	MCU_DRIVER_LIB_hc32f460		(0)
#define MCU_DRIVER_LIB_hc32f46x		(1)
#define MCU_DRIVER_LIB				MCU_DRIVER_LIB_hc32f46x
#endif
/*****************************文件包含(公开)**********************************
*
*备注: 可在此包含底层的库文件
*
*****************************************************************************/
/*公共头文件*/
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h> //stm8s.h中已有数据类型的同名定义,故屏蔽，否则报错
#include <stdio.h>
#include <stddef.h>
#include <stdbool.h>
#include "math.h"
#include "assert.h"

/*MCU的驱动库头文件*/
#if (MCU_TYPE == MCU_TYPE_HC32_F0)
/*HC32F030*/
#include "base_types.h"
#include "gpio.h"
#include "timer3.h"
#include "bt.h"
#include "adc.h"
#include "reset.h"
#include "sysctrl.h"
#include "ddl.h"
#include "flash.h"
#include "bgr.h"
#include "i2c.h"
#include "uart.h"
#include "wdt.h"
#include "lvd.h"
#include "lpm.h"

#elif (MCU_TYPE == MCU_TYPE_HC32_L1)
/*HC32L130*/
#include "base_types.h"

#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
/*HC32F460(F460通过hc32_ddl.h/ddl_config.h包含开启的外设驱动头文件)*/
#include "hc32_ddl.h"


#elif (MCU_TYPE == MCU_TYPE_STM32)
/*STM32G070CB-HAL lib*/
#include "stm32g0xx_hal.h"
#include "tim.h"
#include "adc.h"
#include "gpio.h"
//#include "usart.h"

#else
/*STM8S003*/
#include "stm8s.h"
#include "stm8s_clk.h"
#include "stm8s_gpio.h"
#include "stm8s_tim1.h"
#include "stm8s_tim2.h"
#include "stm8s_adc1.h"
#include "stm8s_uart1.h"
#include "stm8s_it.h"
#include "stm8s_iwdg.h"

#endif
/**************************数据类型及结构定义(公开)***************************
*
*备注: 需要被外部使用的数据结构及类型在这里定义
*
*****************************************************************************/
/*数据类型定义*/
//字符型(本质上是整形,是U8或S8中的一种)
typedef char            CH;     //至少是0-127

//整形
typedef unsigned char   U8;
typedef unsigned short  U16;
typedef unsigned int    U32;
typedef signed char     S8;
typedef signed short    S16;
typedef signed int      S32;

//浮点型
typedef float           F32;
typedef double          F64;

//布尔型
//typedef boolean_t     BOOL;
typedef enum
{
    FALSE = 0,
    TRUE,
}BOOL;

#define TEST_TOGGLE(X)  do{ X ^= 1; }while(0);
/*****************************函数声明(公开)**********************************
*
*备注: 需要被外部使用的接口函数在这里声明
*
*****************************************************************************/

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
