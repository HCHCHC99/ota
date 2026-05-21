/********************************文件说明*************************************
*文件名: co.c

*作者: Yuchen Tan

*版本: V1.0.0

*功能简介: CanOpen协议
 
*备注: 无

*修改履历: 

*****************************************************************************/

/*****************************文件包含(私有)**********************************
*
*备注: 无
*
*****************************************************************************/
#include "co.h"
#include "co_sdo.h"
#include "can_adapter.h"
/*****************************宏定义(私有)************************************
*
*备注: 本文件中,不希望被外部使用或随意修改的宏在这里定义
*
*****************************************************************************/
//根据COB_ID计算节点ID
#define CAL_NODE_ID(CAN_ID)	(CAN_ID & 0x7F)	

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
//CANOpen对象字典
uint32_t CODict_Index2000[20];
/********************************函数定义************************************
*函数名: 

*函数功能描述: CanOpen节点操作

*函数参数: 无

*函数返回值: 无

*备注: 
*****************************************************************************/
/*CanOpen节点-初始化*/
void co_hInit(co_node_t* pCONode, uint8_t NodeID)
{
	pCONode->NodeID = NodeID;
	pCONode->NodeState = e_ns_init;
	pCONode->HeartBeatMs = DEFAULT_HEART_BEAT;		//默认周期1s一次
}
/*CanOpen节点-节点状态跳转*/
int8_t co_SetNodeState(co_node_t* pCONode, co_node_state_t NewState)
{
	int8_t Ret = CO_RET_BUSY;
	if(NewState != pCONode->NodeState)
	{
		switch(NewState)
		{
			case e_ns_init:		//Power-on || Reset Node
				pCONode->NodeState = e_ns_init;	
				break;

			case e_ns_stop:
				if(pCONode->NodeState == e_ns_operational || pCONode->NodeState == e_ns_pre_operational)
					pCONode->NodeState = e_ns_stop;
				else
					Ret = CO_RET_ERR_NODESTATE;
				break;
			
			case e_ns_operational:
				if(pCONode->NodeState == e_ns_stop || pCONode->NodeState == e_ns_pre_operational)
					pCONode->NodeState = e_ns_operational;
				else
					Ret = CO_RET_ERR_NODESTATE;
				break;
			
			case e_ns_pre_operational:
				pCONode->NodeState = e_ns_pre_operational;
				break;
			
			default:
				Ret = CO_RET_ERR_NODESTATE;
				break;
		}
	}
	return Ret;
}
/*CanOpen节点-发送心跳计时(1ms)*/
void co_HeartBeatTimer(co_node_t* pCONode)
{
	pCONode->HeartBeatCnt++;
}
/*CanOpen节点-发送心跳*/
int8_t co_HeartBeat(co_node_t* pCONode)
{
	CAN_TSMT_FRAME_t 	SendMsg = {0};
	if(0)	//Todo: 和NodeState状态下使能的ReponsiveService对应
	{
		return CO_RET_BUSY;
	}
	if(pCONode->HeartBeatCnt >= pCONode->HeartBeatMs)
	{
		pCONode->HeartBeatCnt = 0;
		SendMsg.Data[0] = pCONode->NodeState;
		can_adapter_LoadStdFrame(&SendMsg, (0x700 + pCONode->NodeID), SendMsg.Data, 1);
		can_adapter_Transmit_Polling(CAN1, &SendMsg, 1);
	}
	return CO_RET_OK;
}
/*CanOpen节点-接收消息处理*/
int8_t co_RcvHandle(co_node_t* pCONode)
{
	CAN_RCV_FRAME_t		RcvMsg = {0};
	CAN_TSMT_FRAME_t 	SendMsg = {0};
	uint16_t COB_ID, FuncCode;
	uint8_t	Node_ID = 0;
	int8_t Ret = CO_RET_BUSY;
	uint8_t SubIndex = 0;
	
	if(can_Get_Receive(CAN1, &RcvMsg, 1) == CAN_RET_OK)	//接收到CAN报文
	{
		COB_ID = RcvMsg.StdID & STD_ID_MASK;
		Node_ID = COB_ID & 0x7F;		//COB_ID[6:0]
		FuncCode = COB_ID - Node_ID;	//COB_ID[10:7]
		switch(FuncCode)
		{
			case CO_FUNC_SDO_TX:
				if(pCONode->NodeState != e_ns_operational && pCONode->NodeState != e_ns_pre_operational)
				{
					Ret = CO_RET_ERR_NODESTATE;
				}else if(CAL_NODE_ID(RcvMsg.StdID) != pCONode->NodeID)	//ID不一致不响应
				{
					Ret = CO_RET_ERR_NODEID;
				}else
				{
					Ret = co_sdo_tx_parse(pCONode, &RcvMsg);
					if(Ret == CO_RET_SDOTX_WRITE_OK || Ret == CO_RET_SDOTX_READ_OK)
					{
						SubIndex = RcvMsg.Data[SDO_OFFSET_SUBINDEX];
						RcvMsg.Data[SDO_OFFSET_CS] = SDO_CS_WRITE_SUCCESS;
					}else
					{
						RcvMsg.Data[SDO_OFFSET_CS] = SDO_CS_FAIL_RESPONSE;
					}
					//注: 若接收频率太高,由于发送没有软件缓存,因此使用hc32f460的PTB发送时,可能会只回1包数据(PTB写入1包数据,发完前新的包不会装入PTB,故第2包的回复数据无法被发送)
					can_adapter_LoadStdFrame(&SendMsg, CO_FUNC_SDO_RX + Node_ID, RcvMsg.Data, 8);
					can_adapter_Transmit_Polling(CAN1, &SendMsg, 1);	
					/*执行回调函数不放到co_sdo_tx_parse()内的原因: 防止回调函数里面有延时导致回复Response延迟*/
					if(Ret == CO_RET_SDOTX_WRITE_OK && pCONode->WriteODCallback)
						pCONode->WriteODCallback(SubIndex);
					if(Ret == CO_RET_SDOTX_READ_OK && pCONode->ReadODCallback)
						pCONode->ReadODCallback(SubIndex);
				}
				break;
			
			case CO_FUNC_SDO_RX:
				if(0)	//Todo: 和NodeState状态下使能的ReponsiveService对应
				{
					Ret = CO_RET_ERR_NODESTATE;
				}
				break;
			
			default:
				Ret = CO_RET_ERR_FUNC;
				break;
		}
	}
	return Ret;
}
