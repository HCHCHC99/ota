/********************************文件说明*************************************
*文件名: co_sdo.c

*作者: Yuchen Tan

*版本: V1.0.0

*功能简介: 
 
*备注: 无

*修改履历: 

*****************************************************************************/

/*****************************文件包含(私有)**********************************
*
*备注: 无
*
*****************************************************************************/
#include "co_sdo.h"
#include "co.h"
#include "can_adapter.h"
/*****************************宏定义(私有)************************************
*
*备注: 本文件中,不希望被外部使用或随意修改的宏在这里定义
*
*****************************************************************************/

/**************************数据类型及结构定义(私有)***************************
*
*备注: 本文件中,不希望被外部使用的数据结构及类型在这里定义
*
*****************************************************************************/

/*****************************函数声明(私有)**********************************
*
*备注: 本文件中,不希望被外部调用的函数统一在这里声明
*
*****************************************************************************/

/********************************变量定义*************************************
*
*备注: 不要在.h中定义变量,防止多个.c包含这个.h,导致编译后报错变量重复定义
*
*****************************************************************************/
extern uint32_t CODict_Index2000[20];
/********************************函数定义************************************
*函数名: 

*函数功能描述: SDO报文处理

*函数参数: 无

*函数返回值: 无

*备注: 
*****************************************************************************/
/*SDO报文处理-SDO_TX写对象字典回调函数注册*/
void co_sdo_tx_RegWriteODCallback(co_node_t* pCONode, co_cb_sdo_tx_t Callback)
{
	if(Callback)
	{
		pCONode->WriteODCallback = Callback;
	}
}
/*SDO报文处理-SDO_TX读对象字典回调函数注册*/
void co_sdo_tx_RegReadODCallback(co_node_t* pCONode, co_cb_sdo_tx_t Callback)
{
	if(Callback)
	{
		pCONode->ReadODCallback = Callback;
	}
}
/*SDO报文处理-SDO_TX报文解析*/
int8_t co_sdo_tx_parse(co_node_t* pCONode, CAN_RCV_FRAME_t* pRcvMsg)
{
	/*参数检查*/
	if(pRcvMsg->Cst.Control_f.DLC != 8)		//SDO_TX报文的数据长度必须为8(空余data补0)
		return CO_RET_ERR_DATALEN;
	/*parse msg*/
	uint8_t CS = 0;		//Command specifier
	uint16_t Index = 0;
	uint8_t SubIndex = 0;
	uint32_t Value = 0;
	int8_t Ret = CO_RET_BUSY;
	
	CS = pRcvMsg->Data[SDO_OFFSET_CS] & 0xF0;	//取高4位
	Index = (pRcvMsg->Data[SDO_OFFSET_INDEX + 1] << 8 ) | pRcvMsg->Data[SDO_OFFSET_INDEX];
	SubIndex = pRcvMsg->Data[SDO_OFFSET_SUBINDEX];
	
	/*访问对象字典*/
	if(CS == SDO_CS_WRITE)	//写对象字典
	{
		Ret = CO_RET_SDOTX_WRITE_OK;
		Value = pRcvMsg->Data[7];
		Value = (Value << 8) + pRcvMsg->Data[6];
		Value = (Value << 8) + pRcvMsg->Data[5];
		Value = (Value << 8) + pRcvMsg->Data[4];
		if(Index == 0x2000)
		{
			switch(SubIndex)
			{
				case OD_SI_TRG_SPD:
					CODict_Index2000[OD_SI_TRG_SPD] = Value;
					break;

				case OD_SI_CMD_M_UP:
					CODict_Index2000[OD_SI_CMD_M_UP] = 1;
					CODict_Index2000[OD_SI_CMD_M_DN] = 0;
					CODict_Index2000[OD_SI_CMD_M_STOP] = 0;
					break;

				case OD_SI_CMD_M_DN:
					CODict_Index2000[OD_SI_CMD_M_UP] = 0;
					CODict_Index2000[OD_SI_CMD_M_DN] = 1;
					CODict_Index2000[OD_SI_CMD_M_STOP] = 0;
					break;
									
				case OD_SI_CMD_M_GOTO:
					CODict_Index2000[OD_SI_CMD_M_GOTO] = Value;
					break;
									
				case OD_SI_CLR_FAULT:
					CODict_Index2000[OD_SI_CLR_FAULT] = 1;
					break;
														
				case OD_SI_HEARTBEAT:
					if(Value <= 100)
						Value = 100;
					else if(Value >= 10000)
						Value = 10000;
					pCONode->HeartBeatMs = Value;
					CODict_Index2000[OD_SI_HEARTBEAT] = Value;
					break;
																			
				case OD_SI_CMD_M_STOP:
					CODict_Index2000[OD_SI_CMD_M_UP] = 0;
					CODict_Index2000[OD_SI_CMD_M_DN] = 0;
					CODict_Index2000[OD_SI_CMD_M_STOP] = 1;
					break;
				
				case OD_SI_NODEID:
					pCONode->NodeID = Value;
					CODict_Index2000[OD_SI_NODEID] = Value;
					break;	
				
//				case OD_SI_BAUDRATE:
//					CODict_Index2000[OD_SI_BAUDRATE] = Value;
//					break;
				case OD_SI_CMD_M_RESET:
					CODict_Index2000[OD_SI_CMD_M_RESET] = 1;
					break;
				default:
					Ret = CO_RET_ERR_ACCESSOD;
					break;
			}
		}else
			Ret = CO_RET_ERR_ACCESSOD;
	}
	else if(CS == SDO_CS_READ)	//读对象字典
	{
		Ret = CO_RET_SDOTX_READ_OK;
		if(Index == 0x2000)
		{
			switch(SubIndex)
			{
				case OD_SI_VERSION:
					Value = CODict_Index2000[OD_SI_VERSION];
					break;
									
				case OD_SI_MINIVERSION:
					Value = CODict_Index2000[OD_SI_MINIVERSION];
					break;
														
				case OD_SI_TRG_SPD:
					Value = CODict_Index2000[OD_SI_TRG_SPD];
					break;
																			
				case OD_SI_M_STATE:
					Value = CODict_Index2000[OD_SI_M_STATE];
					break;
																								
				case OD_SI_SYS_STATE:
					Value = CODict_Index2000[OD_SI_SYS_STATE];
					break;
									
				case OD_SI_FDBK_POS:
					Value = CODict_Index2000[OD_SI_FDBK_POS];
					break;
																			
				case OD_SI_HEARTBEAT:
					Value = CODict_Index2000[OD_SI_HEARTBEAT];
					break;
																								
				case OD_SI_BAUDRATE:
					Value = CODict_Index2000[OD_SI_BAUDRATE];
					break;
																													
				default:
					Ret = CO_RET_ERR_ACCESSOD;
					break;
			}
			pRcvMsg->Data[7] = (uint8_t)(Value >> 24);
			pRcvMsg->Data[6] = (uint8_t)(Value >> 16);
			pRcvMsg->Data[5] = (uint8_t)(Value >> 8);
			pRcvMsg->Data[4] = (uint8_t)(Value);
		}else
			Ret = CO_RET_ERR_ACCESSOD;	
	}else
	{
		Ret = CO_RET_ERR_CS;
	}
	return Ret;
}
