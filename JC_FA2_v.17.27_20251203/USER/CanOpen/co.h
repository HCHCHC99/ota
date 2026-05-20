/********************************文件说明*************************************
*文件名: co.h

*作者: Yuchen Tan

*版本: V1.0.0

*功能简介: 
 
*备注: 无

*修改履历: 

*****************************************************************************/
#ifndef CO_H_
#define CO_H_

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
/*CanOpen对象字典访问*/
#define CO_SET_OD(SUBINDEX, VALUE)	CODict_Index2000[SUBINDEX] = VALUE
#define CO_CLR_OD(SUBINDEX)			CODict_Index2000[SUBINDEX] = 0
#define CO_GET_OD(SUBINDEX)			CODict_Index2000[SUBINDEX]

/*CanOpen接口功能返回值*/
#define CO_RET_OK         		(1)
#define CO_RET_SDOTX_WRITE_OK	(2)
#define CO_RET_SDOTX_READ_OK	(3)
#define CO_RET_BUSY       		(0)
#define CO_RET_ERR_INIT   		(-1)
#define CO_RET_ERR_NODEID  		(-2)	//节点ID错误
#define CO_RET_ERR_FUNC  		(-3)	//功能码错误
#define CO_RET_ERR_DATALEN  	(-4)	//数据长度错误
#define CO_RET_ERR_CS  			(-5)	//命令符错误
#define CO_RET_ERR_ACCESSOD  	(-6)	//对象字典访问错误(索引/子索引未定义,索引访问属性与目标操作不匹配)
#define CO_RET_ERR_NODESTATE	(-7)	//节点状态错误(当前状态不响应XX类型报文)

//CanOpen的ID(COB_ID)定义
//注: CanOpen使用标准11位CAN_ID,其中[10:7]为Function Code, [6:0]为Node-ID)
//ID(只需定义Function Code)
#define CO_FUNC_SDO_RX			(0x580)		//SDO-Rx(服务器→客户端)(从→主), COB_ID范围: 0x0581 - 0x05FF
#define CO_FUNC_SDO_TX			(0x600)		//SDO-Tx(客户端→服务器)(主→从), COB_ID范围: 0x0601 - 0x067F
//MASK
#define CO_ID_MASK_FUNC			(0x780)
#define CO_ID_MASK_NODEID		(0x07F)

/*CanOpen对象字典(Object dictionary)*/
//访问属性定义
#define OD_AA_RO				(1)		//只读
#define OD_AA_RW				(2)		//读写
#define OD_AA_WO				(3)		//只写
//子索引定义
#define OD_SI_VERSION			(0x00)	//软件版本(R)
#define OD_SI_MINIVERSION		(0x01)	//软件子版本(R)
#define OD_SI_TRG_SPD			(0x02)	//推杆速度mm/s(RW)
#define OD_SI_M_STATE			(0x03)	//推杆运行状态(R)
#define OD_SI_SYS_STATE			(0x04)	//控制器状态(R)
#define OD_SI_CMD_M_UP			(0x05)	//推杆上升(W)
#define OD_SI_CMD_M_DN			(0x06)	//推杆下降(W)
#define OD_SI_FDBK_POS			(0x07)	//推杆当前位置mm(R)
#define OD_SI_CMD_M_GOTO		(0x08)	//推杆运行到目标位置mm(W)
#define OD_SI_CLR_FAULT			(0x09)	//清除错误(W)
#define OD_SI_HEARTBEAT			(0x0A)	//心跳时间ms(RW)
#define OD_SI_NODEID			(0x0B)	//节点ID(W)
#define OD_SI_CMD_M_STOP		(0x0C)	//推杆停止(W)
#define OD_SI_BAUDRATE			(0x0D)	//CAN波特率(RW)
#define	OD_SI_CMD_M_RESET		(0x0E)	//推杆复位(W)

/*CanOpen节点默认参数*/
#define DEFAULT_CO_NODE_ID		(0)		//节点ID
#define DEFAULT_HEART_BEAT		(1000)	//心跳时间(ms)
/**************************数据类型及结构定义(公开)***************************
*
*备注: 需要被外部使用的数据结构及类型在这里定义
*
*****************************************************************************/
/*CanOpen节点状态定义*/
typedef enum
{
	e_ns_init = 0,					//初始化
	//e_ns_disconnected = 1,			//掉线
	//e_ns_connecting = 2,			//在线
	e_ns_preparing = 3,				//准备
	e_ns_stop = 4,					//停止
	e_ns_operational = 5,			//运行
	e_ns_pre_operational = 0x7F,	//预运行
}co_node_state_t;

/*CanOpen节点回调函数定义*/
typedef void (*co_cb_sdo_tx_t)(uint8_t);

/*CanOpen节点定义*/
typedef struct
{
	uint8_t				NodeID;
	
	co_node_state_t		NodeState;
	
	uint8_t				ReponsiveService;	//不同状态下的可响应服务
	
	uint16_t			HeartBeatMs;		//心跳包发送周期(ms)
		
	uint16_t			HeartBeatCnt;		//
	
	co_cb_sdo_tx_t		WriteODCallback;	//SDO-TX功能码写对象字典回调
	
	co_cb_sdo_tx_t		ReadODCallback;		//SDO-TX功能码读对象字典回调
}co_node_t;
/*****************************函数声明(公开)**********************************
*
*备注: 需要被外部使用的接口函数在这里声明
*
*****************************************************************************/
/*CanOpen节点*/
void co_hInit(co_node_t* pNode, uint8_t NodeID);
int8_t co_SetNodeState(co_node_t* pCONode, co_node_state_t NewState);
void co_HeartBeatTimer(co_node_t* pCONode);
int8_t co_HeartBeat(co_node_t* pCONode);
int8_t co_RcvHandle(co_node_t* pCONode);
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
