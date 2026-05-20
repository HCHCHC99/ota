/********************************文件说明*************************************
*文件名: mc_config.h

*作者: Yuchen Tan

*版本: V1.0.3

*功能简介:

*备注: 无

*修改履历:
------------------------------------V1.0.1------------------------------------
20220506: 修改HPR定义
------------------------------------V1.0.2------------------------------------
20230220:
1.增加电机类型(有刷/无刷)选择宏定义MOTOR_TYPE及相关条件编译;
2.增加预驱类型选择宏定义PRE_DRIVE_TYPE及相关条件编译;
3.将mc_drv.h, mc_hall.h, mc_cur.h相关宏定义转到此文件中,使用MOTOR_NB, HALL_NB,
  DRIVER_PHASE_NB等条件编译进行自动适配!
  实现效果: 后续修改电机相关的参数配置及端口定义,只需要修改mc_config文件即可.
20230224: 增加电机驱动器类型(H桥+预驱/NPMOS)选择宏定义DRV_TYPE及相关条件编译.
注: 不同驱动器的mc_drv.c/.h不同,目前需要在创建工程时由开发者选择驱动器对应
    的mc_drv.c/h文件放入MotorControl文件夹
    (Todo: 后续改为通过DRV_TYPE自动选择适配!)
------------------------------------V1.0.3------------------------------------
20230309: 使用#error检测宏定义MOTOR_NB的值,实现在编译阶段的参数检查并报错超限!
20231109：定义宏DRV_OUTPUT_TYPE替代PRE_DRIVE_TYPE,补充互补输出模式!
*****************************************************************************/
#ifndef MC_CONFIG_H_
#define MC_CONFIG_H_

/*****************************文件包含(公开)**********************************
*
*备注: 可在此包含底层的库文件
*
*****************************************************************************/
#include "common.h"
#include "main.h"
/******************************宏定义(公开)***********************************
*
*备注: 需要被外部使用或可任意修改的宏在这里定义
*
*****************************************************************************/
/*使用电机总数*/
#define MOTOR_NB                (1)
#if (MOTOR_NB > 2)
#error "motor number is limited within 2 for now, and it won`t exceed 4 before I change the code to created motor instance dynamically instead of static!"
#endif
/*电机端口定义是否反向*/


/*反向输出使能*/
#define REVERSE_OUTPUT_EN		(1)				//0-无反向输出	1-支持反向输出(正转可输出负占空比,反转可输出正占空比)

/*电机驱动硬件定义-电机及驱动类型*/
/*电机类型定义*/
#define MOTOR_TYPE_DC           (1)             //直流有刷
#define MOTOR_TYPE_BLDC         (2)             //直流无刷
#define MOTOR_TYPE              MOTOR_TYPE_DC
/*驱动器类型定义*/
#define DRV_TYPE_HB_WITH_PD     (1)             //H桥+预驱
#define DRV_TYPE_PNMOS          (2)             //NPMOS直驱
#define DRV_TYPE                DRV_TYPE_HB_WITH_PD
#if (DRV_TYPE == DRV_TYPE_HB_WITH_PD)
/*预驱(Pre-Drive)类型及电路定义*/
#define DO_H_PWM_L_NULL      	(1)             //上下PWM,下桥不操作(适用于: SDH21263预驱2(HIN)3(LIN~)引脚短接))
#define DO_H_PWM_L_IO      		(2)             //上桥PWM,下桥IO(适用于:SDH21263预驱2(HIN)接PWM,3(LIN~)接IO,无刷方波驱动)
#define DO_H_PWM_L_NPWM      	(3)             //上下桥互补PWM(适用于:直驱)
#define DRV_OUTPUT_TYPE			DO_H_PWM_L_IO
#endif

/*电机软件参数配置*/
/*drv模块-电机PWM载波参数*/
#define PWM_OUTPUT_FULLSCALE    (2000)          //PWM输出模值
#if (DRV_TYPE == DRV_TYPE_HB_WITH_PD)
#define MIN_PWM_PERCENT         (0)				//PWM输出最小百分比(绝对值)
#define MAX_PWM_PERCENT         (98)            //PWM输出最大百分比(绝对值)(H桥不能100%输出)
#elif (DRV_TYPE == DRV_TYPE_PNMOS)
#define MIN_PWM_PERCENT         (0)             //PWM输出最小百分比(绝对值)
#define MAX_PWM_PERCENT         (100)           //PWM输出最大百分比(绝对值)
#endif
/*drv模块-驱动器相数定义*/
#if (MOTOR_TYPE == MOTOR_TYPE_DC)
#define DRIVER_PHASE_NB         (2)
#elif (MOTOR_TYPE == MOTOR_TYPE_BLDC)
#define DRIVER_PHASE_NB         (3)
#endif
/*hall模块-电机HALL参数定义*/
#if (MOTOR_TYPE == MOTOR_TYPE_DC)
#define HALL_NB                 (2)             //传感器个数(有刷电机1/2,无刷电机3)
#define HPP                     (4)             //每个HALL周期(对极)的有效HALL信号数(单位: 步)(双HALL==4,三HALL==6)
#elif (MOTOR_TYPE == MOTOR_TYPE_BLDC)
#define HALL_NB                 (3)             //传感器个数(有刷电机1/2,无刷电机3)
#define HPP                     (6)             //每个HALL周期(对极)的有效HALL信号数(单位: 步)(双HALL==4,三HALL==6)
#endif
#define POLE_NB                 (3)             //磁环对极数(单位: 个N+S)
#define HPR                     (HPP * POLE_NB) //电机每转一圈的有效HALL信号数
/*cur模块*/
#define SMP_R                   (0.01f)         /*采样电阻值(单位:Ω)*/
#define OA_MULT                 (20.0f)         /*运放倍数(单位:无量纲)*/
#define MOTOR_OVC_VALUE         (12000)			/*过流阈值(单位: mA), 马达真实电流 = 采样电流 / 电机PWM驱动占空比*/
/*spd模块-速度闭环控制参数(单位:rpm)*/
#define M_START_RPM_UP          (500)               //上升启动初速度
#define M_START_RPM_DOWN        (500)               //下降启动初速度
#define M_SLOWSTOP_RPM          (500)               //慢停止末速度
#define M_TARGRT_RPM            (3000)              //目标速度
#define M_START_ACCELERATION    (2000)              //速度变化加速度(单位:rpm/s)
#define M_STOP_ACCELERATION     (2000)              //速度变化加速度(单位:rpm/s)
#define M_MAX_RPM               (6000)				//速度环最大速度
#define M_MIN_RPM               0//(200)				//速度环最小速度
/*spd模块-速度开环控制参数(单位:占空比百分比)*/
#define M_START_DC_UP           (10)                //上升启动初占空比
#define M_START_DC_DOWN         (10)                //下降启动初占空比
#define M_SLOWSTOP_DC           (10)                //慢停止末占空比
#define M_TARGRT_DC             (40)                //目标占空比
#define M_START_DC_ACCELERATION (60)                //占空比变化加速度(单位:占空比百分比/s)(1-1000)
#define M_STOP_DC_ACCELERATION  (60)                //占空比变化加速度(单位:占空比百分比/s)(1-1000)
#define M_MAX_DC                (MAX_PWM_PERCENT)   //最大占空比
#define M_MIN_DC                (MIN_PWM_PERCENT)	//最小占空比

/*电机驱动硬件定义-使用的MCU引脚定义*/
/*注: 0表示未使用*/
/*AD引脚(M1-M4)*/
#define ADCH_M1_IM_ADC          M4_ADC1			//&hadc1
#define ADCH_M1_IM_PORT         PortA  	
#define ADCH_M1_IM_PIN          Pin05
#define ADCH_M1_IM_ADCH         (ADC12_IN5) 	//ADC1_IN8
#define ADCH_M2_IM_ADC          (0)
#define ADCH_M2_IM_PORT         (0) //PA05  (ADC12_IN5)
#define ADCH_M2_IM_PIN          (0)
#define ADCH_M2_IM_ADCH         (0)
#define ADCH_M3_IM_ADC          (0)
#define ADCH_M3_IM_PORT         (0) //PA05  (ADC12_IN5)
#define ADCH_M3_IM_PIN          (0)
#define ADCH_M3_IM_ADCH         (0)
#define ADCH_M4_IM_ADC          (0)
#define ADCH_M4_IM_PORT         (0) //PA05  (ADC12_IN5)
#define ADCH_M4_IM_PIN          (0)
#define ADCH_M4_IM_ADCH         (0)

/*hall传感器引脚-M1*/
#if (MOTOR_NB >= 1)
    #if (HALL_NB >= 1)
    #define EXINT_M1_HALLA_PORT     PortA
    #define EXINT_M1_HALLA_PIN      Pin10
    #endif
    #if (HALL_NB >= 2)
    #define EXINT_M1_HALLB_PORT     PortA
    #define EXINT_M1_HALLB_PIN      Pin09
    #endif
    #if (HALL_NB >= 3)
    #define EXINT_M1_HALLC_PORT     PortA
    #define EXINT_M1_HALLC_PIN      Pin08
    #endif
#endif  //(MOTOR_NB >= 1)
/*hall传感器引脚-M2*/
#if (MOTOR_NB >= 2)
    #if (HALL_NB >= 1)
    #define EXINT_M2_HALLA_PORT     0
    #define EXINT_M2_HALLA_PIN      0
    #endif
    #if (HALL_NB >= 2)
    #define EXINT_M2_HALLB_PORT     0
    #define EXINT_M2_HALLB_PIN      0
    #endif
    #if (HALL_NB == 3)
    #define EXINT_M2_HALLC_PORT     0
    #define EXINT_M2_HALLC_PIN      0
    #endif
#endif  //(MOTOR_NB >= 2)

/*驱动器引脚-M1*/
#if (MOTOR_NB >= 1)
	#if (DRV_OUTPUT_TYPE == DO_H_PWM_L_IO)             //上下PWM,下桥不操作(适用于: SDH21263预驱2(HIN)3(LIN~)引脚短接))
		#if (DRIVER_PHASE_NB >= 2)
		#define PWMCH_M1_UH_TIMERCLOCK	PWC_FCG2_PERIPH_TIMA4     //PWC_FCG2_PERIPH_TIMA3
		#define PWMCH_M1_VH_TIMERCLOCK	PWC_FCG2_PERIPH_TIMA4     //PWC_FCG2_PERIPH_TIMA3
		//UH
		#define PWMCH_M1_UH_PORT        (PortB)
		#define PWMCH_M1_UH_PIN         (Pin09)
		#define PWMCH_M1_UH_TIMER		M4_TMRA4     //M4_TMRA3
		#define PWMCH_M1_UH_CH          (TimeraCh4)
		#define PWMCH_M1_UH_FUNC_GPIO   (Func_Gpio)
		#define PWMCH_M1_UH_FUNC_PWM    (Func_Tima0)
		//UL
		#define PWMCH_M1_UL_PORT        (PortB)
		#define PWMCH_M1_UL_PIN         (Pin08)
		#define PWMCH_M1_UL_TIMER       (0)
		#define PWMCH_M1_UL_CH          (0)
		#define PWMCH_M1_UL_FUNC_GPIO   ((en_port_func_t)0)
		#define PWMCH_M1_UL_FUNC_PWM    ((en_port_func_t)0)
		//VH
		#define PWMCH_M1_VH_PORT        (PortB)
		#define PWMCH_M1_VH_PIN         (Pin07)
		#define PWMCH_M1_VH_TIMER       M4_TMRA4
		#define PWMCH_M1_VH_CH          (TimeraCh2)
		#define PWMCH_M1_VH_FUNC_GPIO   (Func_Gpio)
		#define PWMCH_M1_VH_FUNC_PWM    (Func_Tima0)
		//VL
		#define PWMCH_M1_VL_PORT        (PortB)
		#define PWMCH_M1_VL_PIN         (Pin06)
		#define PWMCH_M1_VL_TIMER		(0)
		#define PWMCH_M1_VL_CH          (0)
		#define PWMCH_M1_VL_FUNC_GPIO   ((en_port_func_t)0)
		#define PWMCH_M1_VL_FUNC_PWM    ((en_port_func_t)0)
		#endif
		#if (DRIVER_PHASE_NB >= 3)
		#define PWMCH_M1_WH_TIMERCLOCK  (PWC_FCG2_PERIPH_TIMA3) //PWC_FCG2_PERIPH_TIMA4
		//WH
		#define PWMCH_M1_WH_PORT        PortB
		#define PWMCH_M1_WH_PIN         Pin05
		#define PWMCH_M1_WH_TIMER       M4_TMRA3    //M4_TMRA4
		#define PWMCH_M1_WH_CH          TimeraCh2
		#define PWMCH_M1_WH_FUNC_GPIO   Func_Gpio
		#define PWMCH_M1_WH_FUNC_PWM    Func_Tima0
		//WL
		#define PWMCH_M1_WL_PORT        (GPIO_PLW_PORT)
		#define PWMCH_M1_WL_PIN         (GPIO_PLW_PIN)
		#define PWMCH_M1_WL_TIMER       (0)
		#define PWMCH_M1_WL_CH          ((en_timera_channel_t)0)
		#define PWMCH_M1_WL_FUNC_GPIO   ((en_port_func_t)0)
		#define PWMCH_M1_WL_FUNC_PWM    ((en_port_func_t)0)
		#endif
	#elif (DRV_OUTPUT_TYPE == DO_H_PWM_L_NPWM)
		#if (DRIVER_PHASE_NB >= 2)
		#define PWMCH_M1_UH_TIMERCLOCK	PWC_FCG2_PERIPH_TIMA4     //PWC_FCG2_PERIPH_TIMA3
		#define PWMCH_M1_VH_TIMERCLOCK	PWC_FCG2_PERIPH_TIMA4     //PWC_FCG2_PERIPH_TIMA3
		//UH
		#define PWMCH_M1_UH_PORT        (PortB)
		#define PWMCH_M1_UH_PIN         (Pin09)
		#define PWMCH_M1_UH_TIMER       M4_TMR43		//M4_TMRA4     //M4_TMRA3
		#define PWMCH_M1_UH_CH          (Timer4OcoOuh)	//(TimeraCh4)
		#define PWMCH_M1_UH_FUNC_GPIO   (Func_Gpio)
		#define PWMCH_M1_UH_FUNC_PWM    (Func_Tim4)		//(Func_Tima0)
		//UL
		#define PWMCH_M1_UL_PORT        (PortB)
		#define PWMCH_M1_UL_PIN         (Pin08)
		#define PWMCH_M1_UL_TIMER       M4_TMR43		//(0)
		#define PWMCH_M1_UL_CH          (Timer4OcoOul)	//((en_timera_channel_t)0)
		#define PWMCH_M1_UL_FUNC_GPIO   (Func_Gpio)		//((en_port_func_t)0)
		#define PWMCH_M1_UL_FUNC_PWM    (Func_Tim4)		//((en_port_func_t)0)
		//VH
		#define PWMCH_M1_VH_PORT        (PortB)
		#define PWMCH_M1_VH_PIN         (Pin07)
		#define PWMCH_M1_VH_TIMER       M4_TMR43		//M4_TMRA4
		#define PWMCH_M1_VH_CH          (Timer4OcoOvh)	//(TimeraCh2)
		#define PWMCH_M1_VH_FUNC_GPIO   (Func_Gpio)
		#define PWMCH_M1_VH_FUNC_PWM    (Func_Tim4)		//(Func_Tima0)
		//VL
		#define PWMCH_M1_VL_PORT        (PortB)
		#define PWMCH_M1_VL_PIN         (Pin06)
		#define PWMCH_M1_VL_TIMER		M4_TMR43		//(0)
		#define PWMCH_M1_VL_CH          (Timer4OcoOvl)	//((en_timera_channel_t)0)
		#define PWMCH_M1_VL_FUNC_GPIO   (Func_Gpio)		//((en_port_func_t)0)
		#define PWMCH_M1_VL_FUNC_PWM    (Func_Tim4)		//((en_port_func_t)0)
		#endif
		#if (DRIVER_PHASE_NB >= 3)
		#define PWMCH_M1_WH_TIMERCLOCK  (PWC_FCG2_PERIPH_TIMA3) //PWC_FCG2_PERIPH_TIMA4
		//WH
		#define PWMCH_M1_WH_PORT        PortB
		#define PWMCH_M1_WH_PIN         Pin05
		#define PWMCH_M1_WH_TIMER       M4_TMRA3    //M4_TMRA4
		#define PWMCH_M1_WH_CH          TimeraCh2
		#define PWMCH_M1_WH_FUNC_GPIO   Func_Gpio
		#define PWMCH_M1_WH_FUNC_PWM    Func_Tima0
		//WL
		#define PWMCH_M1_WL_PORT        (GPIO_PLW_PORT)
		#define PWMCH_M1_WL_PIN         (GPIO_PLW_PIN)
		#define PWMCH_M1_WL_TIMER       (0)
		#define PWMCH_M1_WL_CH          ((en_timera_channel_t)0)
		#define PWMCH_M1_WL_FUNC_GPIO   ((en_port_func_t)0)
		#define PWMCH_M1_WL_FUNC_PWM    ((en_port_func_t)0)
		#endif
	#endif  //(DRV_OUTPUT_TYPE == DO_H_PWM_L_NPWM)
#endif  //(MOTOR_NB >= 1)

/*驱动器引脚-M2*/
#if (MOTOR_NB >= 2)
    #if (DRIVER_PHASE_NB >= 2)
    #define PWMCH_M2_UH_TIMERCLOCK  (PWC_FCG2_PERIPH_TIMA4) //PWC_FCG2_PERIPH_TIMA3
    #define PWMCH_M2_VH_TIMERCLOCK  (PWC_FCG2_PERIPH_TIMA4) //PWC_FCG2_PERIPH_TIMA3
    //UH
    #define PWMCH_M2_UH_PORT        M2_UH_GPIO_Port
    #define PWMCH_M2_UH_PIN         M2_UH_Pin
    #define PWMCH_M2_UH_TIMER       M4_TMRA4        //M4_TMRA3
    #define PWMCH_M2_UH_CH          TimeraCh4
    #define PWMCH_M2_UH_FUNC_GPIO   Func_Gpio
    #define PWMCH_M2_UH_FUNC_PWM    Func_Tima0
    //UL
    #define PWMCH_M2_UL_PORT        NULL
    #define PWMCH_M2_UL_PIN         NULL
    #define PWMCH_M2_UL_TIMER       NULL
    #define PWMCH_M2_UL_CH          ((en_timera_channel_t)0)
    #define PWMCH_M2_UL_FUNC_GPIO   ((en_port_func_t)0)
    #define PWMCH_M2_UL_FUNC_PWM    ((en_port_func_t)0)
    //VH
    #define PWMCH_M2_VH_PORT        M2_VH_GPIO_Port
    #define PWMCH_M2_VH_PIN         M2_VH_Pin
    #define PWMCH_M2_VH_TIMER       M4_TMRA4
    #define PWMCH_M2_VH_CH          TimeraCh3
    #define PWMCH_M2_VH_FUNC_GPIO   Func_Gpio
    #define PWMCH_M2_VH_FUNC_PWM    Func_Tima0
    //VL
    #define PWMCH_M2_VL_PORT        NULL
    #define PWMCH_M2_VL_PIN         NULL
    #define PWMCH_M2_VL_TIMER       NULL
    #define PWMCH_M2_VL_CH          ((en_timera_channel_t)0)
    #define PWMCH_M2_VL_FUNC_GPIO   ((en_port_func_t)0)
    #define PWMCH_M2_VL_FUNC_PWM    ((en_port_func_t)0)
    #endif
    #if (DRIVER_PHASE_NB >= 3)
    #define PWMCH_M2_WH_TIMERCLOCK  (PWC_FCG2_PERIPH_TIMA4) //PWC_FCG2_PERIPH_TIMA3
    //WH
    #define PWMCH_M2_WH_PORT        M2_WH_GPIO_Port
    #define PWMCH_M2_WH_PIN         M2_WH_Pin
    #define PWMCH_M2_WH_TIMER       M4_TMRA4    //M4_TMRA3
    #define PWMCH_M2_WH_CH          TimeraCh2
    #define PWMCH_M2_WH_FUNC_GPIO   Func_Gpio
    #define PWMCH_M2_WH_FUNC_PWM    Func_Tima0
    //WL
    #define PWMCH_M2_WL_PORT        NULL
    #define PWMCH_M2_WL_PIN         NULL
    #define PWMCH_M2_WL_TIMER       NULL
    #define PWMCH_M2_WL_CH          ((en_timera_channel_t)0)
    #define PWMCH_M2_WL_FUNC_GPIO   ((en_port_func_t)0)
    #define PWMCH_M2_WL_FUNC_PWM    ((en_port_func_t)0)
    #endif
#endif  //(MOTOR_NB >= 2)
/**************************数据类型及结构定义(公开)***************************
*
*备注: 需要被外部使用的数据结构及类型在这里定义
*
*****************************************************************************/

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
