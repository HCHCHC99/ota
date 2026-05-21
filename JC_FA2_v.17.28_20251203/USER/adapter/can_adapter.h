/********************************文件说明*************************************
*文件名: can_adapter.h

*作者: Yuchen Tan

*版本: V1.0.0

*功能简介:

*备注: 无

*修改履历:

*****************************************************************************/
#ifndef CAN_ADAPTER_H_
#define CAN_ADAPTER_H_

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
/*mcu的CAN控制器索引号(必须从0开始)*/
#define CAN1           		(0)
#define CAN2           		(1)
#define CAN3          		(2)
#define CAN4           		(3)
#define MCU_CAN_NB     		(1)

/*CAN控制器接收缓冲大小*/
#define CAN_RX_BUF_SIZE		(5)

/*CAN控制器功能*/
#define CAN_FUNC_ENABLE    	(1)
#define CAN_FUNC_DISABLE   	(0)
#define CAN_FUNC_TX        	(1<<0)      //发送器使能
#define CAN_FUNC_RX        	(1<<1)      //接收器使能
#define CAN_FUNC_TXE_IE    	(1<<2)      //发送寄存器空中断使能
#define CAN_FUNC_RXNE_IE   	(1<<3)      //接收寄存器非空中断使能
#define CAN_FUNC_TC_IE     	(1<<4)      //发送完成中断使能
#define CAN_FUNC_IDLE_IE   	(1<<5)      //接收空闲中断使能

/*CAN控制器硬件状态标志*/
#define CAN_IDLE           	(1<<0)      //空闲(接收完成/接收超时)
#define CAN_TC             	(1<<1)      //发送完成
#define CAN_TXE            	(1<<2)      //发送寄存器空
#define CAN_RXNE           	(1<<3)      //接收寄存器非空
#define CAN_FE             	(1<<4)      //帧错误
#define CAN_NE             	(1<<5)      //噪声错误
#define CAN_PE             	(1<<6)      //奇偶校验错误
#define CAN_OVR            	(1<<7)      //接收过载
#define CAN_BE            	(1<<8)      //总线错误

/*CAN控制器接口功能返回值*/
#define CAN_RET_OK         	(1)
#define CAN_RET_BUSY       	(0)
#define CAN_RET_ERR_INIT   	(-1)
#define CAN_RET_ERR_PARAM  	(-2)
#define CAN_RET_ERR_NODATA 	(-3)

/*CAN控制器相关*/
#define CAN_RX_TIMEOUT		(5000)		//默认接收超时时间(ms)

/*CAN协议*/
#define STD_ID_MASK			(0x7FF)			//11位
#define EXT_ID_MASK			(0x1FFFFFFF)	//29位
/**************************数据类型及结构定义(公开)***************************
*
*备注: 需要被外部使用的数据结构及类型在这里定义
*
*****************************************************************************/
/*与MCU平台相关的抽象类型定义*/
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
/*数据类型定义-CAN实例*/
typedef M4_CAN_TypeDef*		CAN_INS_t;			/*CAN控制器实例*/
typedef	stc_can_txframe_t	CAN_TSMT_FRAME_t;	/*CAN发送帧*/
typedef	stc_can_rxframe_t	CAN_RCV_FRAME_t;	/*CAN接收帧*/
typedef en_can_filter_sel_t CAN_FILTERSEL_t;	/*CAN筛选器选择*/
#endif

/*数据类型定义-CAN控制器工作记录*/
typedef struct
{
    uint16_t    TxFrameCnt;			//发包计数
    uint16_t    RxFrameCnt;			//收包计数
    uint16_t    ErrCnt;             //错误计数
	uint16_t    OverRunCnt;			//接收过载错误
}CAN_RECORDER_t;

/*数据类型定义-CAN控制器*/
typedef struct
{
	uint8_t			Channel;    	/*CAN外设(索引号)*/
    CAN_INS_t		Ins;    		/*CAN控制器实例句柄*/
    uint32_t        BDR;			/*CAN波特率*/
	/*控制相关*/
	uint16_t		RxIdleTime;		/*接收空闲计时*/
	uint16_t		RxTimeOut;		/*接收超时时间*/
	/*CAN控制器接收缓存(环形)*/
	uint8_t			Head;
	uint8_t			Rear;
	CAN_RCV_FRAME_t	ReBuffer[CAN_RX_BUF_SIZE];
	/*CAN控制器诊断记录*/
    CAN_RECORDER_t	Recorder;
}CAN_CTRL_t;
/*****************************函数声明(公开)**********************************
*
*备注: 需要被外部使用的接口函数在这里声明
*
*****************************************************************************/
/*CAN控制器-控制器相关操作*/
int8_t can_hInit(CAN_CTRL_t *Handle, CAN_INS_t Ins, uint8_t Channel);
int8_t can_Get_Receive(uint8_t Channel, CAN_RCV_FRAME_t* pFrame, uint8_t Len);
int8_t can_Timer_1ms(uint8_t Channel);
/*CAN控制器-外设实例访问(接口)*/
int8_t can_adapter_Clr_RB(uint8_t Channel);
int8_t can_adapter_SetFilter_ID(uint8_t Channel, CAN_FILTERSEL_t FilterSel, uint32_t Value);
int8_t can_adapter_SetFilter_IDMask(uint8_t Channel, CAN_FILTERSEL_t FilterSel, uint32_t ID, uint32_t Mask);
int8_t can_adapter_EnableFilter(uint8_t Channel, CAN_FILTERSEL_t FilterSel);
int8_t can_adapter_DisableFilter(uint8_t Channel, CAN_FILTERSEL_t FilterSel);
int8_t can_adapter_Set_BDR(uint8_t Channel, uint32_t BDR);
int8_t can_adapter_Func_Sel(uint8_t Channel, uint16_t Func, uint8_t Enable);
int8_t can_adapter_LoadStdFrame(CAN_TSMT_FRAME_t* pFrame, uint32_t CAN_ID, uint8_t* pData, uint8_t Len);
/*扩展帧装载*/
int8_t can_adapter_LoadExtFrame(CAN_TSMT_FRAME_t* pFrame, uint32_t CAN_ID, uint8_t* pData, uint8_t Len);
int8_t can_adapter_Transmit_Polling(uint8_t Channel, const CAN_TSMT_FRAME_t* pFrame, uint8_t Len);
int8_t can_adapter_Transmit_IT(uint8_t Channel, const CAN_TSMT_FRAME_t* pFrame, uint8_t Len);
int8_t can_adapter_Receive_Polling(uint8_t Channel, CAN_RCV_FRAME_t *pFrame, uint8_t Len);
int8_t can_adapter_Receive_IT(uint8_t Channel, CAN_RCV_FRAME_t *pFrame, uint8_t Len);
int8_t can_adapter_BusRecover(uint8_t Channel);
/*CAN控制器-中断回调函数*/
int8_t can_Rx_Callback(uint8_t Channel);
int8_t can_Err_Callback(uint8_t Channel);
/*CAN控制器-模块测试*/
void can_Test(void);
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
