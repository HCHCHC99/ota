/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "common.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */
#define PWM_OUTPUT      1 //1:PWM波输出     2：noPWM波输出



/*限位标志*/
#define M1_TOP                  (0x01)
#define M1_BTM                  (0x01<<8)
#define M2_TOP                  (0x02)
#define M2_BTM                  (0x02<<8)

#define M_LOCKED_FLAG			      (0x01)
/* USER CODE BEGIN Private defines */

/*GPIO-输入引脚定义-LED*/
#define GPIO_LED_PORT			(PortH)	//PH02
#define GPIO_LED_PIN			(Pin02)

#define GPIO_LED_PORT1		(PortB)
#define GPIO_LED_PIN1			(Pin13)

#define GPIO_LED_PORT2		(PortB)
#define GPIO_LED_PIN2 		(Pin14)
/*GPIO-输入引脚定义-按键*/
#define GPIO_IN1_KUP_PORT		(PortC)	//PC13
#define GPIO_IN1_KUP_PIN		(Pin13)
#define GPIO_IN2_KDOWN_PORT		(PortC)	//PC14
#define GPIO_IN2_KDOWN_PIN		(Pin14)
/*GPIO-输入引脚定义-限位*/
#define GPIO_M1_LIMIT_BTM_PORT	(PortB)	//PB10
#define GPIO_M1_LIMIT_BTM_PIN	(Pin10)
#define GPIO_M1_LIMIT_TOP_PORT	(PortB)	//PB02
#define GPIO_M1_LIMIT_TOP_PIN	(Pin02)
/*GPIO-输入引脚定义-预留*/
#define GPIO_IN3_PORT			(PortC)	//PC15
#define GPIO_IN3_PIN			(Pin15)

/*GPIO-输出引脚定义-到达限位*/
#define AT_TOP_GPIO_Port		(PortA)	//PA12
#define AT_TOP_Pin				(Pin12)
#define AT_BTM_GPIO_Port		(PortA)	//PA11
#define AT_BTM_Pin				(Pin11)
/*GPIO-输出引脚定义-SLEEP*/
#define	GPIO_SLEEP_PORT			(PortB) //PB03
#define GPIO_SLEEP_PIN			(Pin03)
/*GPIO-输出引脚定义-PWMIO*/
#define GPIO_PLU_PORT			(PortB)	//PB08
#define GPIO_PLU_PIN			(Pin08)
#define GPIO_PLV_PORT			(PortB) //PB06
#define GPIO_PLV_PIN			(Pin06)
#define GPIO_PLW_PORT			(PortB) //PB04
#define GPIO_PLW_PIN			(Pin04)
/*GPIO-输出引脚定义-预留*/
#define GPIO_OUT3_PORT			(PortA)	//PA02
#define GPIO_OUT3_PIN			(Pin02)
#define GPIO_OUT4_PORT			(PortA)	//PA01
#define GPIO_OUT4_PIN			(Pin01)


/*AD通道定义-电流采样(见mc_config.h)*/
/*AD通道定义-母线电压*/
#define ADCH_VM_ADC				(M4_ADC1)
#define ADCH_VM_PORT			(PortA)
#define ADCH_VM_PIN				(Pin04)
#define	ADCH_VM_ADCH			(ADC12_IN4)

//电流检测
#define ADCH_IVM_ADC      (M4_ADC1)
#define ADCH_IVM_PORT			(PortA)
#define ADCH_IVM_PIN			(Pin05)
#define	ADCH_IVM_ADCH			(ADC12_IN5)

/*AD通道定义-温度采样*/
#define ADCH_TEMP_ADC			(M4_ADC1)
#define ADCH_TEMP_PORT			(PortA)	//PA07	(ADC12_IN7)
#define ADCH_TEMP_PIN			(Pin07)
#define	ADCH_TEMP_ADCH			(ADC12_IN7)
/*AD通道定义-电位计旋钮*/
#define ADCH_DWJ_ADC			(M4_ADC1)
#define ADCH_DWJ_PORT			(PortA)	//PA04	(ADC1_IN4)
#define ADCH_DWJ_PIN			(Pin04)	
#define	ADCH_DWJ_ADCH			(ADC1_IN4)

//#if (PROTOCOL_TYPE == MODBUS_PROTOCOL)
/*485方向线引脚功能定义*/
#define UART_485_DIR_PORT       (PortA)
#define UART_485_DIR_PIN        (Pin03)      

/*UART引脚及功能定义-UART3(RS485)*/
#define UART_485				(M4_USART3)
#define UART_485_TX_PORT		(PortB)	//PB13
#define UART_485_TX_PIN			(Pin13)
#define UART_485_TX_FUNC		(Func_Usart3_Tx)
#define UART_485_RX_PORT		(PortB)	//PB12
#define UART_485_RX_PIN			(Pin12)
#define UART_485_RX_FUNC		(Func_Usart3_Rx)
/*UART中断配置*/
#define UART_IRQ_PRIORITY		(DDL_IRQ_PRIORITY_DEFAULT)	//Possible values are 0(high priority) to 15(low priority)
#define UART_485_RI_NUM			(INT_USART3_RI)
#define UART_485_EI_NUM			(INT_USART3_EI)
#define UART_485_TI_NUM			(INT_USART3_TI)
#define UART_485_TCI_NUM		(INT_USART3_TCI)
#define UART_485_RTOI_NUM		(INT_USART3_RTO)	//空闲(接收超时)
#define UART_485_RI_IRQN		(Int005_IRQn)
#define UART_485_EI_IRQN		(Int010_IRQn)
#define UART_485_TI_IRQN		(Int006_IRQn)
#define UART_485_TCI_IRQN		(Int011_IRQn)
#define UART_485_RTOI_IRQN		(Int012_IRQn)
//#elif (PROTOCOL_TYPE == CAN_PROTOCOL)
/*CAN引脚及功能定义*/
#define CAN_INS					(M4_CAN)
#define CAN_TX_PORT				(PortB)		
#define CAN_TX_PIN				(Pin15)
#define CAN_TX_FUNC				(Func_Can1_Tx)
#define CAN_RX_PORT				(PortB)		//PB14
#define CAN_RX_PIN				(Pin14)
#define CAN_RX_FUNC				(Func_Can1_Rx)
/* CAN中断配置*/
#define CAN_IRQ_PRIORITY		(DDL_IRQ_PRIORITY_DEFAULT)	//Possible values are 0(high priority) to 15(low priority)
#define CAN_NUM					(INT_CAN_INT)
#define CAN_RX_IRQN				(Int007_IRQn)
   
//#endif

/*UART引脚及功能定义-UART1(预留)*/
#define UART_MCU				(M4_USART1)
#define UART_MCU_TX_PORT		(PortA)	//PA00
#define UART_MCU_TX_PIN			(Pin00)
#define UART_MCU_TX_FUNC		(Func_Usart1_Tx)
#define UART_MCU_RX_PORT		(PortA)	//PA01
#define UART_MCU_RX_PIN			(Pin01)
#define UART_MCU_RX_FUNC		(Func_Usart1_Rx)
/*UART中断配置*/
#define UART_MCU_RI_NUM			(INT_USART1_RI)
#define UART_MCU_EI_NUM			(INT_USART1_EI)
#define UART_MCU_TI_NUM			(INT_USART1_TI)
#define UART_MCU_TCI_NUM		(INT_USART1_TCI)
#define UART_MCU_RTOI_NUM		(INT_USART1_RTO)	//空闲(接收超时)
#define UART_MCU_RI_IRQN		(Int008_IRQn)
#define UART_MCU_EI_IRQN		(Int013_IRQn)
#define UART_MCU_TI_IRQN		(Int009_IRQn)
#define UART_MCU_TCI_IRQN		(Int014_IRQn)
#define UART_MCU_RTOI_IRQN		(Int015_IRQn)


/*外部中断通道定义-HALL*/
/*优先级*/
#define M1_HALL_EIRQ_PRIORITY	(DDL_IRQ_PRIORITY_00)	//Possible values are 0(high priority) to 15(low priority)
/*通道*/
#define M1_HALLA_EIRQ_CH		(ExtiCh10)
#define M1_HALLB_EIRQ_CH		(ExtiCh09)
#define M1_HALLC_EIRQ_CH		(ExtiCh08)
/*ISR映射*/
#define M1_HALLA_EIRQ_IRQN		(Int000_IRQn)
#define M1_HALLB_EIRQ_IRQN		(Int001_IRQn)
#define M1_HALLC_EIRQ_IRQN		(Int002_IRQn)
/*中断号*/
#define M1_HALLA_EIRQ_NUM		(INT_PORT_EIRQ10)
#define M1_HALLB_EIRQ_NUM		(INT_PORT_EIRQ9)
#define M1_HALLC_EIRQ_NUM		(INT_PORT_EIRQ8)

/*定时器-timer0(用于计时器)*/
#define TIME0_IRQ_PRIORITY		(DDL_IRQ_PRIORITY_01)	//Possible values are 0(high priority) to 15(low priority)
#define TIME0_IRQN          	(Int004_IRQn)
#define TIME_GCM_NUM			(INT_TMR01_GCMB)
/* USER CODE END Private defines */



#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif


#endif /* __MAIN_H__ */







/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

