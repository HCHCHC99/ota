/********************************文件说明*************************************
*文件名: sys_cfg.c

*作者: Xiaodong Qu

*版本: V1.0.0

*功能简介:系统配置函数

*备注: 无

*修改履历:

*****************************************************************************/

/*****************************文件包含(私有)**********************************
*
*备注: 无
*
*****************************************************************************/
#include "main.h"
#include "sys_cfg.h"
#include "uart_adapter.h"
#include "flash_adapter.h"
#include "crc.h"
//#include "debug.h"
/*****************************宏定义(私有)************************************
*
*备注: 本文件中,不希望被外部使用或随意修改的宏在这里定义
*
*****************************************************************************/

/*版本信息和下发日期*/
uint8_t Soft_Ver[3] ={SOFTWARE_MAINVER_SYS,SOFTWARE_SUBVER1_SYS,SOFTWARE_SUBVER2_SYS};
uint8_t Hard_Ver[3] ={HARDWARE_MAINVER_SYS,HARDWARE_SUBVER1_SYS,HARDWARE_SUBVER2_SYS};
uint8_t Release_Date[4] = {0};


/*IO配置信息*/
sys_cfg_dido_handle_t Sys_CONDI[E_DI_MAX];
sys_cfg_dido_handle_t Sys_CONDO[E_DO_MAX];

syscon_cfg_handle_t			hSysConfig;
UART_CTRL_t     			hUartConfig;
sys_cfg_flag_handle_t		hSysCfg;
/*****************************函数声明(私有)**********************************
*
*备注: 本文件中,不希望被外部调用的函数统一在这里声明
*
*****************************************************************************/
void sys_cfg_SysCfgInit(void);
void sys_cfg_MotorCfgInit(void);
void sys_cfg_BootCfgInit(void);

/********************************函数定义************************************
*函数名:

*函数功能描述: 配置初始化

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/

void sys_cfg_Init(void)
{
	uint32_t SYSData[10] = {0};
    uint32_t SYSCheck = 0;
	uint32_t MOTORData[20] = {0};
    uint32_t MOTORCheck = 0;
	uint32_t BOOTData[10] = {0};
	/*读取系统配置的参数*/
	flash_adapter_Read(WORD, FSA_SYS_CONFIG, WORD, SYSData, sizeof SYSData / sizeof SYSData[0]);
	if(SYSData[0] == 0x5A5A)
	{
		SYSCheck = SYSData[0]+SYSData[1]+SYSData[2]+SYSData[3]+SYSData[4]+SYSData[5]+SYSData[6]+SYSData[7];
		if(SYSCheck == SYSData[8])
		{
			/*推杆参数*/
			hSysConfig.Config_GearRatio = SYSData[1];
			hSysConfig.Config_Lead = SYSData[2];
			hSysConfig.Config_Route = SYSData[3];
			hSysConfig.Config_SpeedMmps = SYSData[4];
			
			hSysConfig.Config_OvcValue = SYSData[5];
			hSysConfig.Config_OverVoltage = SYSData[6];
			hSysConfig.Config_UnderVoltage = SYSData[7];
		}
		else
			sys_cfg_SysCfgInit();
	}else
		sys_cfg_SysCfgInit();
	/*读取电机配置的参数*/
	flash_adapter_Read(WORD, FSA_MOTOR_CONFIG, WORD, MOTORData, sizeof MOTORData / sizeof MOTORData[0]);
	if(MOTORData[0] == 0x5AA5)
	{
		MOTORCheck = MOTORData[0]+MOTORData[1]+MOTORData[2]+MOTORData[3]+MOTORData[4]+MOTORData[5]+MOTORData[6]+MOTORData[7]+MOTORData[8]+MOTORData[9];
		if(MOTORCheck == MOTORData[10])
		{
			/*系统参数初始化*/	
			hSysConfig.Config_NodeSlaveAddr = MOTORData[1];
			hSysConfig.Config_CommunicationType = MOTORData[2];
			
			hSysConfig.Config_ResetRunMode = MOTORData[3];
			hSysConfig.Config_ResetDirection = MOTORData[4];
			hSysConfig.Config_ResetMode = MOTORData[5];
			hSysConfig.Config_ResetRaise = MOTORData[6];
			hSysConfig.Config_MotorRunMode = MOTORData[7];
			
			hSysConfig.Config_TopDetection = MOTORData[8];
			hSysConfig.Config_BtmDetection = MOTORData[9];
		}else
			sys_cfg_MotorCfgInit();
	}else
		sys_cfg_MotorCfgInit();
	
	/*读取底层配置的参数*/
	flash_adapter_Read(WORD, FSA_BOOT_CONFIG, WORD, BOOTData, sizeof BOOTData / sizeof BOOTData[0]);
	if(BOOTData[0] == 0xAA55)
	{
		MOTORCheck = BOOTData[0]+BOOTData[1]+BOOTData[2];
		if(MOTORCheck == BOOTData[3])
		{
			/*系统参数初始化*/	
			hSysConfig.Config_HallDirectionSel = BOOTData[1];
			hSysConfig.Config_PhaseDirectionSel = BOOTData[2];
		}else
			sys_cfg_BootCfgInit();
	}else
		sys_cfg_BootCfgInit();
	hSysCfg.ControlMode = SYS_CFG_CONTROL_MODE;
//	Config.Config_DIFunction;
//	Config.Config_DOFunction;
//	Config.Config_ActiveValue;
}

/********************************函数定义************************************
*函数名:

*函数功能描述: 下发日期32-8

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
void sys_cfg_DeliveryData(void)
{
	Release_Date[0] = (RELEASE_DATE >>24)&0xff;
	Release_Date[1] = (RELEASE_DATE >>16)&0xff;
	Release_Date[2] = (RELEASE_DATE >>8)&0xff;
	Release_Date[3] = (RELEASE_DATE &0xff);
//	uart_adapter_Transmit_Polling(UART1, DeliveryData, 4);
}

void sys_cfg_SysCfgInit(void)
{
	/*推杆参数*/
	hSysConfig.Config_GearRatio = (uint16_t)(GEAR_RATIO_CFG);
	hSysConfig.Config_Lead = LEAD_CFG;
	hSysConfig.Config_Route = ROUTE_CFG;
	hSysConfig.Config_SpeedMmps = SPEED_MMPS_CFG;
	
	hSysConfig.Config_OvcValue = MOTOR_OVC_VALUE_CFG;
	hSysConfig.Config_OverVoltage = OVER_VOLTAGE_CFG;
	hSysConfig.Config_UnderVoltage = UNDER_VOLTAGE_CFG;
}
	
void sys_cfg_MotorCfgInit(void)
{
	/*系统参数初始化*/	
	hSysConfig.Config_NodeSlaveAddr = NODE_SLAVE_ADDR_CFG;
	hSysConfig.Config_CommunicationType = COMMUNICATION_TYPE_CFG;
	
	hSysConfig.Config_ResetRunMode = RESET_RUN_MODE_CFG;
	hSysConfig.Config_ResetDirection = RESET_DIRECTION_CFG;
	hSysConfig.Config_ResetMode = RESETMODE_CFG;
	hSysConfig.Config_ResetRaise = RESET_RAISE_CFG;
	hSysConfig.Config_MotorRunMode = MOTOR_RUN_MODE_CFG;
	
	hSysConfig.Config_TopDetection = TOP_DETECTION_CFG;
	hSysConfig.Config_BtmDetection = BTM_DETECTION_CFG;
}

void sys_cfg_BootCfgInit(void)
{
	/*底层参数配置初始化*/
	hSysConfig.Config_HallDirectionSel = HALL_REVERSE;
	hSysConfig.Config_PhaseDirectionSel = DRIVER_REVERSE;
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 推杆参数的设置和读取

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*配置参数结构体返回值*/
syscon_cfg_handle_t sys_cfg_Hadnle(void)
{
	return hSysConfig;
}
/*配置标志结构体返回值*/
sys_cfg_flag_handle_t* sys_cfg_FlagHandle(void)
{
	return &hSysCfg;
}

void sys_cfg_NodeSlaveAddr(uint8_t SlaveAddr)
{
	if(SlaveAddr >= 0 || SlaveAddr < 255)
	{
		hSysConfig.Config_NodeSlaveAddr = SlaveAddr;
	}
}

/********************************函数定义************************************
*函数名:

*函数功能描述: 系统参数的设置和读取

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/	

/*IO配置*/
void sys_cfg_SetDI(sys_cfg_dido_handle_t *pSysIO)
{
	if(pSysIO->DIFunction < E_DI_MAX)
	{
		Sys_CONDI[pSysIO->DIFunction].DIFunction = pSysIO->DIFunction;
		Sys_CONDI[pSysIO->DIFunction].Port = pSysIO->Port;
		Sys_CONDI[pSysIO->DIFunction].Pin = pSysIO->Pin;
		Sys_CONDI[pSysIO->DIFunction].ActiveValue = pSysIO->ActiveValue;
	}
}

void sys_cfg_SetDO(sys_cfg_dido_handle_t *pSysIO)
{
	if(pSysIO->DOFunction < E_DO_MAX)
	{
		Sys_CONDO[pSysIO->DOFunction].DOFunction = pSysIO->DOFunction;
		Sys_CONDO[pSysIO->DOFunction].Port = pSysIO->Port;
		Sys_CONDO[pSysIO->DOFunction].Pin = pSysIO->Pin;
		Sys_CONDO[pSysIO->DOFunction].ActiveValue = pSysIO->ActiveValue;
	}
}


/********************************函数定义************************************
*函数名:

*函数功能描述: 串口配置处理

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/	
/*串口控制初始化*/
void sys_cfg_UartConfigInit(void)
{
	UART_INS_t UartMcuConfig = {UART_MCU, UART1};
    uart_hInit(&hUartConfig, &UartMcuConfig, 0);
}

/*判断接口是否接收完成*/
bool sys_cfg_MsgConfigRcv(uint8_t *RcvData, uint16_t *RcvLen)
{
	bool Ret = 0;
	if(uart_Get_Receive(hUartConfig.Ins.Channel, RcvData, RcvLen) == UART_RET_OK)
    {
        Ret = TRUE;
    }
    return Ret;
}

void sys_cfg_MsgUartConfigHandler(syscon_uart_cfg_handle_t* UartConfig)
{
	uint8_t ReData[100] = {0};
	uint16_t RevLen = 0;

	if(sys_cfg_MsgConfigRcv(ReData, &RevLen))
	{
		if(sys_cfg_UartDataUnpack(UartConfig,ReData,RevLen))
			sys_cfg_UartDataProcess(UartConfig);
	}
}

/*串口配置系统参数*/
void sys_cfg_UartSystemCfgWrite(syscon_uart_cfg_handle_t* UartConfig)
{
	syscon_cfg_handle_t pCfg;
	
	memcpy(&pCfg, &UartConfig->Data[8], sizeof(pCfg));
	
	/*串口数据给到结构体*/
	hSysConfig.Config_GearRatio = pCfg.Config_GearRatio;
	hSysConfig.Config_Route = pCfg.Config_Route;
	hSysConfig.Config_Lead = pCfg.Config_Lead;
	hSysConfig.Config_SpeedMmps = pCfg.Config_SpeedMmps;
	hSysConfig.Config_OvcValue = pCfg.Config_OvcValue;
	hSysConfig.Config_OverVoltage = pCfg.Config_OverVoltage;
	hSysConfig.Config_UnderVoltage = pCfg.Config_UnderVoltage;
	hSysConfig.Config_NodeSlaveAddr = pCfg.Config_NodeSlaveAddr;
	hSysConfig.Config_CommunicationType = pCfg.Config_CommunicationType;
	hSysConfig.Config_ResetRunMode = pCfg.Config_ResetRunMode;
	hSysConfig.Config_ResetDirection = pCfg.Config_ResetDirection;
	hSysConfig.Config_ResetMode = pCfg.Config_ResetMode;
	hSysConfig.Config_MotorRunMode = pCfg.Config_MotorRunMode;
	hSysConfig.Config_TopDetection = pCfg.Config_TopDetection;
	hSysConfig.Config_BtmDetection = pCfg.Config_BtmDetection;
	hSysConfig.Config_ResetRaise = pCfg.Config_ResetRaise;
	hSysConfig.Config_DIFunction = pCfg.Config_DIFunction;
	hSysConfig.Config_DOFunction = pCfg.Config_DOFunction;
	hSysConfig.Config_ActiveValue = pCfg.Config_ActiveValue;
	hSysConfig.Config_HallDirectionSel = pCfg.Config_HallDirectionSel;
	hSysConfig.Config_PhaseDirectionSel = pCfg.Config_PhaseDirectionSel;
	
	/*数据存入FALSH*/
	SYSTEM_SET_FLAG(hSysCfg.SaveCfgFlashFlag, SYS_CFG_SAVE_FLASH_FLAG);
}
/*参数检查*/
int16_t sys_cfg_ParameterCheckout(syscon_cfg_handle_t* UartConfig)
{
	hSysCfg.CfgRetFluat = 0;
	if(UartConfig->Config_GearRatio <= GEAR_RATIO_MIN|| UartConfig->Config_GearRatio > GEAR_RATIO_MAX)
	{
		hSysCfg.CfgRetFluat |= CFG_RET_GEARRATIO;
//		 return -1;
	}
	if(UartConfig->Config_Route <= ROUTE_MIN || UartConfig->Config_Route > ROUTE_MAX)
	{
		hSysCfg.CfgRetFluat |= CFG_RET_ROUTE;
//		return -1;
	}
	if(UartConfig->Config_Lead <= LEAD_MIN || UartConfig->Config_Lead > LEAD_MAX)
	{
		hSysCfg.CfgRetFluat |= CFG_RET_LEAD;
//		return -1;
	}
	if(UartConfig->Config_SpeedMmps<= SPEED_MMPS_MIN || UartConfig->Config_SpeedMmps > SPEED_MMPS_MAX)
	{
		hSysCfg.CfgRetFluat |= CFG_RET_SPEEDMMPS;
//		return -1;
	}
	if(UartConfig->Config_OvcValue <= OVC_VALUE_MIN || UartConfig->Config_OvcValue > OVC_VALUE_MAX)
	{
		hSysCfg.CfgRetFluat |= CFG_RET_OVCVALUE;
//		return -1;
	}
	if(UartConfig->Config_OverVoltage <= OVER_VOLTAGE_MIN || UartConfig->Config_OverVoltage > OVER_VOLTAGE_MAX)
	{
		hSysCfg.CfgRetFluat |= CFG_RET_OVERVOLTAGE;
//		return -1;
	}
	if(UartConfig->Config_UnderVoltage <= UNDER_VOLTAGE_MIN || UartConfig->Config_UnderVoltage > UNDER_VOLTAGE_MAX)
	{
		hSysCfg.CfgRetFluat |= CFG_RET_UNDERVOLTAGE;
//		return -1;
	}
//	if(UartConfig->Config_NodeSlaveAddr <= NODE_SLAVE_ADDR_MIN || UartConfig->Config_NodeSlaveAddr > NODE_SLAVE_ADDR_MAX)
//	{
//		hSysCfg.CfgRetFluat |= CFG_RET_NODESLAVEADDR;
////		return -1;
//	}
	if(UartConfig->Config_CommunicationType!= E_NOCOM && UartConfig->Config_CommunicationType != E_MODBUS && UartConfig->Config_CommunicationType != E_CAN)
	{
		return -1;
	}
	if(UartConfig->Config_ResetRunMode != E_RESET_INCHING && UartConfig->Config_ResetRunMode != E_RESET_CONTINUOUS)
	{
		return -1;
	}
	if(UartConfig->Config_ResetDirection != E_RESET_BTM && UartConfig->Config_ResetDirection != E_RESET_TOP)
	{
		return -1;
	}
	if(UartConfig->Config_ResetMode != E_DETECTION_SIGNAL && UartConfig->Config_ResetMode != E_DETECTION_ANOMALY)
	{
		return -1;
	}
	if(UartConfig->Config_MotorRunMode != E_MOTOR_INCHING && UartConfig->Config_MotorRunMode != E_MOTOR_CONTINUOUS)
	{
		return -1;
	}
	if(UartConfig->Config_TopDetection != E_TOP_SIGNAL_SWITCH && UartConfig->Config_TopDetection != E_TOP_SOFT_LIMIT && UartConfig->Config_TopDetection != E_TOP_HALL_ABNORMAL)
	{
		return -1;
	}
	if(UartConfig->Config_BtmDetection != E_BTM_SIGNAL_SWITCH && UartConfig->Config_BtmDetection != E_BTM_SOFT_LIMIT && UartConfig->Config_BtmDetection != E_BTM_HALL_ABNORMAL)
	{
		return -1;
	}
	if(UartConfig->Config_ResetRaise< RESET_RAISE_MIN || UartConfig->Config_ResetRaise > RESET_RAISE_MAX)
	{
		hSysCfg.CfgRetFluat |= CFG_RET_RESETRAISE;
//		return -1;
	}
	if(UartConfig->Config_DIFunction != E_DINO_FUN && UartConfig->Config_DIFunction != E_UP_FUN && \
		UartConfig->Config_DIFunction != E_DOWN_FUN && UartConfig->Config_DIFunction != E_RESET_FUN &&\
		UartConfig->Config_DIFunction != E_TOP_LIMIT_FUN && UartConfig->Config_DIFunction != E_BTM_LIMIT_FUN)
	{
		return -1;
	}
	if(UartConfig->Config_DOFunction != E_DONO_FUN && UartConfig->Config_DOFunction != E_REACH_TOP_LIMIT_FUN && \
		UartConfig->Config_DOFunction != E_REACH_BTM_LIMIT_FUN && UartConfig->Config_DOFunction != E_FAILURE_FUN &&\
		UartConfig->Config_DOFunction != E_STATUS_FUN)
	{
		return -1;
	}
	if(UartConfig->Config_ActiveValue != E_LOW_LEVEL && UartConfig->Config_ActiveValue != E_HIGH_LEVEL)
	{
		return -1;
	}
	if(hSysCfg.CfgRetFluat != 0)
	{
		return -1;
	}
	return 0;
}

/*串口配置电机参数*/
void sys_cfg_Controller(void)
{
	syscon_without_data_handle_t pMsgSend;
	pMsgSend.Head.Cmd = CONTROLLER_EXIT_CONFIG_MODE_CMD;
	pMsgSend.Head.MsgLen = sizeof(pMsgSend);
	pMsgSend.Head.Ret = 0;
	pMsgSend.Tail.CRC = crc_Cal16((uint8_t*)&pMsgSend, (uint8_t)(sizeof(pMsgSend)-sizeof(pMsgSend.Tail)),CRC_POLYNOMIAL8005);
	uart_adapter_Transmit_Polling(hUartConfig.Ins.Channel,(uint8_t*)&pMsgSend,sizeof(pMsgSend));
}

/*串口功能函数*/
//数据解包
int8_t sys_cfg_UartDataUnpack(syscon_uart_cfg_handle_t* UartConfig, uint8_t *PkData, uint8_t DataLen)
{
	uint16_t Crc16 = 0;
	uint8_t *Data = UartConfig->Data;
	if (DataLen < (TAIL_LEN+CMD_NUMBER) )
	{
		return SYS_CFG_UART_RET_ERROR;
	}
	Crc16 = ((PkData[DataLen -TAIL_CRC_L]) | (PkData[DataLen - TAIL_CRC_H]<<8));
	if (Crc16 != crc_Cal16(PkData, DataLen - TAIL_LEN, CRC_POLYNOMIAL8005))
	{
		return SYS_CFG_UART_RET_ERROR;
	}
	memset(Data, 0, UART_MAX_DATA_SIZE);
	UartConfig->Head.MsgLen = DataLen;
	memcpy(Data,PkData,DataLen);
	return SYS_CFG_UART_RET_ERROR;
}

int8_t sys_cfg_UartDataProcess(syscon_uart_cfg_handle_t* UartConfig)
{
	syscon_without_data_handle_t pMsg;
	syscon_without_data_handle_t pMsgSend;
	syscon_interface_data_handle_t pParaMsg;
	syscon_interface_data_handle_t pParaMsgSend;
	syscon_msg_head_t temp_head;
	
	memcpy(&temp_head,UartConfig->Data,sizeof(temp_head));
	/*50:进入配置模式*/
	if (temp_head.Cmd == CLIENT_ENTER_CONFIG_CMD)
	{
		memcpy(&pMsg,UartConfig->Data,sizeof(pMsg));
		memcpy(&pMsgSend,UartConfig->Data,sizeof(pMsgSend));
		if(pMsg.Tail.CRC != crc_Cal16((uint8_t*)&pMsg, (uint8_t)(sizeof(pMsg)-sizeof(pMsg.Tail)),CRC_POLYNOMIAL8005))
		{
			return SYS_CFG_UART_RET_ERROR;
		}
		SYSTEM_SET_FLAG(hSysCfg.EnterConfigFlag, SYS_CFG_ENTER_CFG_FLAG);
		if(SYSTEM_GET_FLAG(hSysCfg.ControlMode,SYS_CFG_CONTROL_MODE))
		{
			SYSTEM_CLR_FLAG(hSysCfg.ControlMode,SYS_CFG_CONTROL_MODE);
			pMsgSend.Head.Ret = E_CONFIGMODE;
		}else
		{
			pMsgSend.Head.Ret = E_IDLEMODE;
		}
		pMsgSend.Tail.CRC = crc_Cal16((uint8_t*)&pMsgSend, (uint8_t)(sizeof(pMsgSend)-sizeof(pMsgSend.Tail)),CRC_POLYNOMIAL8005);
		uart_adapter_Transmit_Polling(hUartConfig.Ins.Channel,(uint8_t*)&pMsgSend,sizeof(pMsgSend));
	}
	/*51:驱动参数配置*/
	else if (temp_head.Cmd == CLIENT_DRIVE_CONFIG_SET_CMD)
	{
		memcpy(&pParaMsg,UartConfig->Data,sizeof(pParaMsg));
		if (pParaMsg.Tail.CRC != crc_Cal16((uint8_t*)&pParaMsg, (uint8_t)(sizeof(pParaMsg)-sizeof(pParaMsg.Tail)),CRC_POLYNOMIAL8005))
		{
			return SYS_CFG_UART_RET_ERROR;
		}
		/*参数检查*/
		if (sys_cfg_ParameterCheckout(&pParaMsg.Para) != 0)
		{
			memset(&pMsgSend, 0, sizeof(pMsgSend));
			pMsgSend.Head.Cmd = CLIENT_DRIVE_CONFIG_SET_CMD;
			pMsgSend.Head.MsgLen = sizeof(pMsgSend);
//			msg_send.head.ret = hSysCfg.CfgRetFluat;
			pMsgSend.Head.Ret = -1;
			pMsgSend.Tail.CRC = crc_Cal16((uint8_t*)&pMsgSend, (uint8_t)(sizeof(pMsgSend)-sizeof(pMsgSend.Tail)),CRC_POLYNOMIAL8005);
			uart_adapter_Transmit_Polling(hUartConfig.Ins.Channel,(uint8_t*)&pMsgSend,sizeof(pMsgSend));
//			SysCfg.CfgRetFluat = 0;
			return SYS_CFG_UART_RET_ERROR;
		}
		memset(&pMsgSend, 0, sizeof(pMsgSend));
		pMsgSend.Head.Cmd = CLIENT_DRIVE_CONFIG_SET_CMD;
		pMsgSend.Head.MsgLen = sizeof(pMsgSend);
		pMsgSend.Head.Ret = hSysCfg.PerameterSetMode;
		pMsgSend.Tail.CRC = crc_Cal16((uint8_t*)&pMsgSend, (uint8_t)(sizeof(pMsgSend)-sizeof(pMsgSend.Tail)),CRC_POLYNOMIAL8005);
		sys_cfg_UartSystemCfgWrite(UartConfig);
		uart_adapter_Transmit_Polling(hUartConfig.Ins.Channel,(uint8_t*)&pMsgSend,sizeof(pMsgSend));
	}
	/*52:驱动参数读取*/
	else if (temp_head.Cmd == CLIENT_SYSCONFIG_GET_CMD)
	{
		memcpy(&pMsg,UartConfig->Data,sizeof(pMsg));
		memcpy(&pParaMsgSend.Para,&hSysConfig,sizeof(pParaMsgSend.Para));
		if(pMsg.Tail.CRC != crc_Cal16((uint8_t*)&pMsg, (uint8_t)(sizeof(pMsg)-sizeof(pMsg.Tail)),CRC_POLYNOMIAL8005))
		{
			return SYS_CFG_UART_RET_ERROR;
		}
		pParaMsgSend.Head.Cmd = CLIENT_SYSCONFIG_GET_CMD;
		pParaMsgSend.Head.MsgLen = sizeof(pParaMsgSend);
		pParaMsgSend.Head.Ret = 0;
		pParaMsgSend.Tail.CRC = crc_Cal16((uint8_t*)&pParaMsgSend, (uint8_t)(sizeof(pParaMsgSend)-sizeof(pParaMsgSend.Tail)),CRC_POLYNOMIAL8005);
		uart_adapter_Transmit_Polling(hUartConfig.Ins.Channel,(uint8_t*)&pParaMsgSend,sizeof(pParaMsgSend));
	}
	return 0;
}

/*数据传输过程中的标志位参数*/



