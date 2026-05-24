/********************************ÎÄ¼şËµÃ÷*************************************

*ÎÄ¼şÃû: system.h



*×÷Õß: Yuchen Tan



*°·Ú: V1.0.0



*¹¦ÄÜ¼ò½é:



*±¸×¢: ÎŞ



*ĞŞ¸ÄÂÄÀú:



*****************************************************************************/

#ifndef SYSTEM_H_

#define SYSTEM_H_



/*****************************ÎÄ¼ş°üº¬(¹«¿ª)**********************************

*

*±¸×¢: ¿ÉÔÚ´Ë°üº¬µ×²ãµÄ¿âÎÄ¼ş

*

*****************************************************************************/

#include "common.h"

#include "sys_cfg.h"

/******************************ºê¶¨Òå(¹«¿ª)***********************************

*

*±¸×¢: ĞèÒª±»Íâ²¿Ê¹ÓÃ»ò¿ÉÈÎÒâĞŞ¸ÄµÄºêÔÚÕâÀï¶¨Òå

*

*****************************************************************************/



/*ÏµÍ³µç»úÊı¶¨Òå*/

#define SYS_MOTOR_NB        (MOTOR_NB)



/*CAN½Ó¿Úµç»ú¿ØÖÆÃüÁî*/

typedef enum

{

	e_can_none = 0,

	e_can_up,

	e_can_dn,

	e_can_stop,

	e_can_goto,

	e_can_reset,

	e_can_clrfault,

}can_m_cmd_t;





/*ÍÆ¸ËÇ·Ñ¹¹ıÑ¹»Ö¸´µÄ³ÙÖÍµçÑ¹(µ¥Î»: V)*/

#define HYSTERESIS_VOLTAGE  (2.0f)      //±£»¤»Ö¸´³ÙÖÍµçÑ¹

#define OVV_COUNT_MAX		200

/*ÍÆ¸Ë»ºÍ£²ÎÊı¶¨Òå*/

#define SPEED_INDEX         (0.2f)     	//ÍÆËÙÏµÊı

#define MOTOR_SLOW_STOP     (25.0f)		//»ºÍ£ÏµÊı



/*ÏµÍ³µôµçãĞÖµµçÑ¹*/

#define	ODV_STOP_SYSTEM				8000



#define	POWER_DOWN_SAVE_FLASH		10000

void system_set_CanCmd(can_m_cmd_t CanCmd);



/*ÉèÖÃcan¿ØÖÆÃüÁî*/

void system_set_CanCmd(can_m_cmd_t CanCmd);

/*¶ÁÈ¡ÒÑÍê³É¸´Î»±êÖ¾*/

uint8_t system_get_ZeroFound(void);

/*»ñÈ¡¹ÊÕÏ±êÖ¾*/

uint16_t system_get_FaultFlag(void);

/*»ñÈ¡ÍÆ¸Ëµ±Ç°Î»ÖÃ*/

float system_get_columnPosMM(void);

/*»ñÈ¡ÏŞÎ»±êÖ¾*/

uint16_t system_get_LimitFlag(void);

//»ñÈ¡µ±Ç°ÊÖÉ²ÊÇ·ñÀ­½ô

uint8_t system_get_LockedFlag(void);

/*»ñÈ¡É²³µ×´Ì¬*/

uint8_t system_get_BrakeState(void);



/*ÏµÍ³´íÎó¶¨Òå*/

#define FAULT_M1_OVC        (1<<0)                      //M1¹ıÁ÷

#define FAULT_M1_HAB        (1<<1)                      //M1»ô¶ûÒì³£

#define FAULT_M2_OVC        (1<<4)                      //M2¹ıÁ÷

#define FAULT_M2_HAB        (1<<5)                      //M2»ô¶ûÒì³£

#define MOTOR_FAULT_NB      (4)                         //µç»úÒì³£Êı



#define FAULT_UDV           (1<<8)                      //Ç·Ñ¹

#define FAULT_OVV           (1<<9)                      //¹ıÑ¹

#define FAULT_OVT           (1<<10)                     //¹ıÈÈ

#define FAULT_POS           (1<<11)                     //Î»ÖÃ´íÎó

#define FAULT_M1_ALL        (0X000F)                    //M1¹ÊÕÏ¼¯ºÏ

#define FAULT_M2_ALL        (0X00F0)                    //M2¹ÊÕÏ¼¯ºÏ

#define FAULT_MOTOR_ALL     (FAULT_M1_ALL|FAULT_M2_ALL) //µç»ú¹ÊÕÏ¼¯ºÏ

#define FAULT_NOT_MOTOR_ALL (0xFF00)                    //·Çµç»ú¹ÊÕÏ¼¯ºÏ

#define FAULT_ALL           (0XFFFF)                    //ËùÓĞ¹ÊÕÏ¼¯ºÏ

/**************************Êı¾İÀàĞÍ¼°½á¹¹¶¨Òå(¹«¿ª)***************************

*

*±¸×¢: ĞèÒª±»Íâ²¿Ê¹ÓÃµÄÊı¾İ½á¹¹¼°ÀàĞÍÔÚÕâÀï¶¨Òå

*

*****************************************************************************/

/*ÏµÍ³¹¤×÷ÅäÖÃ²ÎÊı*/

typedef struct

{

	/*ÍÆ¸Ë²ÎÊı*/

    float                           	Sys_GearRatio;              //³İÂÖ¼õËÙ±È  

    float                           	Sys_Lead;                   //Ë¿¸Ëµ¼³Ì

    float                           	Sys_Route;                  //ĞĞ³Ì

    float                           	Sys_SpeedMmps;              //ÍÆ¸ËËÙ¶È

	/*ÏµÍ³²ÎÊı*/

    uint8_t                         	Sys_NodeSlaveAddr;          //Í¨Ñ¶½ÚµãID

    syscon_cfg_com_type_t        		Sys_CommunicationType;      //Í¨Ñ¶ÀàĞÍ

	

    uint16_t                       	 	Sys_OvcValue;               //¹ıÁ÷±£»¤ãĞÖµ

    float                           	Sys_OverVoltage;            //¹ıÑ¹±£»¤ãĞÖµ

    float                           	Sys_UnderVoltage;           //Ç·Ñ¹±£»¤ãĞÖµ

    syscon_cfg_di_fun_t             	Sys_DIFunction;             //Òı½ÅÊäÈë¹¦ÄÜ

	syscon_cfg_do_fun_t             	Sys_DOFunction;             //Òı½ÅÊä³ö¹¦ÄÜ

	syscon_cfg_pin_polarity_t         	Sys_ActiveValue;            //Òı½Å¼«ĞÔ

	

	float                           	Sys_ResetRaise;             //¸´Î»Ì§¸ß

    syscon_cfg_reset_run_mode_t         Sys_ResetRunMode;           //¸´Î»ÔËĞĞÄ£Ê½

	syscon_cfg_reset_direction_t        Sys_ResetDirection;         //¸´Î»·½Ïò

	syscon_cfg_reset_judgment_mode_t	Sys_ResetMode;				//¸´Î»ÅĞ¶Ï·½Ê½

    syscon_cfg_motor_run_mode_t         Sys_MotorRunMode;           //µç»úÔËĞĞÄ£Ê½

	

	syscon_cfg_top_detect_t				Sys_TopDetection;           //µ½¶¥¼ì²â

    syscon_cfg_btm_detect_t          	Sys_BtmDetection;           //µ½µ×¼ì²â

}SYSTEM_CONFIG_t;



/*****************************º¯ÊıÉùÃ÷(¹«¿ª)**********************************

*

*±¸×¢: ĞèÒª±»Íâ²¿Ê¹ÓÃµÄ½Ó¿Úº¯ÊıÔÚÕâÀïÉùÃ÷

*

*****************************************************************************/

void system_Init(void);

void system_Loop_Task(void);

void system_Timer_Task(void);



int8_t system_msgHandler_MB_AppCB(uint8_t* RcvData, uint8_t Len);



/*ÖĞ¶ÏISR*/

#if (MCU_TYPE == MCU_TYPE_STM32)



#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)

void TIM_Callback(void);

#elif(MCU_TYPE == MCU_TYPE_HC32_F4)

void Timer01B_CallBack(void);

void ExtInt_Callback(void);

void Uart_Callback_Rx(void);

void Uart_Callback_Err(void);

void Uart_Callback_Tx(void);

void Uart_Callback_TC(void);

void Uart_Callback_Idle(void);

void CAN_RxIrqCallBack(void);

/* UDSÒì²½½ÓÊÕ½Ó¿Ú (CAN ISR ¡ú Ö÷Ñ­»·) */
extern volatile uint8_t g_uds_rx_pending;
extern uint8_t g_uds_rx_buffer[];
extern uint16_t g_uds_rx_len;
extern uint32_t g_uds_rx_can_id;

void Uart_MCU_Callback_Rx(void);

void Uart_MCU_Callback_Err(void);

void Uart_MCU_Callback_Tx(void);

void Uart_MCU_Callback_TC(void);

void Uart_MCU_Callback_Idle(void);





void system_CJ_Task(void );

#endif

/*****************************±äÁ¿ÉùÃ÷(¹«¿ª)**********************************

*

*±¸×¢: ²»½¨ÒéÓÃexternÉùÃ÷±¾ÎÄ¼şµÄ±äÁ¿Ö±½Ó¸øÍâ²¿Ê¹ÓÃ(½âñî).

*¹«¿ª±¾ÎÄ¼ş±äÁ¿½¨Òú›Ê½: ¿ª·Å·µ»Ø±äÁ¿ÖµµÄ½Ó¿Ú.

*

*****************************************************************************/



/*****************************±äÁ¿ÒıÓÃ(È«¾Ö)**********************************

*

*±¸×¢: ²»½¨ÒéÓÃexternÒıÓÃÆäËûÎÄ¼şµÄ±äÁ¿(½âñî).

*ÒıÓÃÆäËûÎÄ¼ş±äÁ¿½¨Òú›Ê½: °üº¬ÆäËûÎÄ¼ş.h²¢µ÷ÓÃÏàÓ¦½Ó¿Úor´«²Î·½Ê½»ñÈ¡ÆäËûÎÄ¼şµÄ±äÁ¿

*

*****************************************************************************/



/*****************************º¯ÊıÒıÓÃ(È«¾Ö)**********************************

*

*±¸×¢: ²»½¨ÒéÓÃexternÒıÓÃÆäËûÎÄ¼şµÄº¯Êı(½âñî).

*ÒıÓÃÆäËûÎÄ¼şº¯ÊıµÄ·½Ê½: ¿É°üº¬ÆäËûÎÄ¼ş.h²¢µ÷ÓÃÏàÓ¦½Ó¿Ú

*

*****************************************************************************/







#endif

