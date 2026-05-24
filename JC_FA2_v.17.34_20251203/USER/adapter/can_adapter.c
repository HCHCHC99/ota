/********************************文件说明*************************************
*文件名: can_adapter.c

*作者: Yuchen Tan

*版本: V1.0.0

*功能简介: CAN控制器

*备注: 
Todo: 数据收发接口参数Len(收发帧数)的功能实现,现在默认是收发1帧!

*修改履历:
*****************************************************************************/
/*****************************文件包含(私有)**********************************
*
*备注: 无
*
*****************************************************************************/
#include "can_adapter.h"
/*****************************宏定义(私有)************************************
*
*备注: 本文件中,不希望被外部使用或随意修改的宏在这里定义
*
*****************************************************************************/
/*获取CAN控制器句柄*/
#define __CAN_GET_HANDLE(__CHANNEL__)	(CanHandle[__CHANNEL__])

/*适配芯片的私有操作*/
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
#define CAN_RESET_ENABLE()                  (M4_CAN->CFG_STAT_f.RESET = 1u)
#define CAN_RESET_DISABLE()                                                     \
do{                                                                             \
    do{                                                                         \
        M4_CAN->CFG_STAT_f.RESET = 0u;                                          \
}while(M4_CAN->CFG_STAT_f.RESET);                                               \
}while(0)

#define RX_FIFO_MODE		(1)	//F460的CAN接收寄存器固有采用FIFO (RB)

#endif
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
CAN_CTRL_t* CanHandle[MCU_CAN_NB];		//注: CAN实例的指针(实例由上层创建,即“指针->内存块”模型)
/********************************函数定义************************************
*函数名:

*函数功能描述: CAN控制器-外设实例访问(非接口)

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*读硬件状态标志*/
static uint32_t can_adapter_Get_SR(CAN_CTRL_t* Handle, uint8_t Status)
{
	uint32_t State = 0;
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    if(Status & CAN_RXNE)
	{
		if(true == CAN_IrqFlgGet(CanRxIrqFlg))	//CanRxIrqFlg仅在收到数据时置位,与RB中是否有遗留数据无关
			State |= CanRxIrqFlg;
		if(true == CAN_IrqFlgGet(CanRxBufAlmostFullIrqFlg))
			State |= CanRxBufAlmostFullIrqFlg;
	}
	return State;
#endif
}
/*清硬件状态标志*/
static void can_adapter_Clr_SR(CAN_CTRL_t* Handle, uint8_t Status)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    if(Status & CAN_RXNE)
	{
		if(true == CAN_IrqFlgGet(CanRxIrqFlg))	//CanRxIrqFlg仅在收到数据时置位,与RB中是否有遗留数据无关
			CAN_IrqFlgClr(CanRxIrqFlg);
		if(true == CAN_IrqFlgGet(CanRxBufAlmostFullIrqFlg))//
			CAN_IrqFlgClr(CanRxBufAlmostFullIrqFlg);
		if(true == CAN_IrqFlgGet(CanRxBufFullIrqFlg))
			CAN_IrqFlgClr(CanRxBufFullIrqFlg);
	}
#endif
}
/*清硬件错误寄存器*/
static uint16_t can_adapter_ClrErrOccured(CAN_CTRL_t* Handle)
{
    uint16_t ErrOccur = 0;
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
	if(true == CAN_IrqFlgGet(CanErrorIrqFlg))
	{
		CAN_IrqFlgClr(CanErrorIrqFlg);
		ErrOccur |= CanErrorIrqFlg;
	}
	if(true == CAN_IrqFlgGet(CanBusErrorIrqFlg))
	{
		CAN_IrqFlgClr(CanBusErrorIrqFlg);
	#if 0	//BUSOFF故障恢复功能统一放到can_adapter_BusRecover()中,因为实测此处处理有一种情况无法覆盖,还是会导致BUSOFF故障无法恢复: 
			//长时间短路CANH和CANL(短路期间会一直重复"故障->触发中断->中断清故障->故障"这个过程),TECNT会逐渐累加,当TECNT>255会导致CFG_STAT->RESET置位,然后就无法进入此中断了!
		CAN_RESET_ENABLE();		
		CAN_RESET_DISABLE();
	#endif
		ErrOccur |= CanBusErrorIrqFlg;
	}
	if(true == CAN_IrqFlgGet(CanErrorWarningIrqFlg))
	{
		CAN_IrqFlgClr(CanErrorWarningIrqFlg);
		ErrOccur |= CanErrorWarningIrqFlg;
	}
	if(true == CAN_IrqFlgGet(CanErrorPassivenodeIrqFlg))
	{
		CAN_IrqFlgClr(CanErrorPassivenodeIrqFlg);
		ErrOccur |= CanErrorPassivenodeIrqFlg;
	}
	if(true == CAN_IrqFlgGet(CanErrorPassiveIrqFlg))
	{
		CAN_IrqFlgClr(CanErrorPassiveIrqFlg);
		ErrOccur |= CanErrorPassiveIrqFlg;
	}	
	if(true == CAN_IrqFlgGet(CanArbiLostIrqFlg))
	{
		CAN_IrqFlgClr(CanArbiLostIrqFlg);
		ErrOccur |= CanArbiLostIrqFlg;
	}
	if(true == CAN_IrqFlgGet(CanRxOverIrqFlg))
	{
		CAN_IrqFlgClr(CanRxOverIrqFlg);
		ErrOccur |= CanRxOverIrqFlg;
	}
	if(ErrOccur != 0)
	{
		CAN_IrqFlgClr(CanRxIrqFlg);
		CAN_IrqCmd(CanRxIrqEn, Enable);
	}
#endif
    return ErrOccur;
}
/*硬件接收缓存空*/
static BOOL can_adapter_RB_Not_Empty(CAN_CTRL_t* Handle)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
	//RSSTAT: 0-RB空, 1-RB中剩余帧数<AFWL, 2-RB中剩余帧数>=AFWL, 3-RB满或溢出
    return (BOOL)(M4_CAN->RCTRL_f.RSSTAT != CanRxBufEmpty);	
#endif
}
/*读硬件接收寄存器*/
static void can_adapter_Read_RDR(CAN_CTRL_t* Handle, CAN_RCV_FRAME_t *pFrame)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
	//芯片只有一个CAN,用不到Handle成员
    CAN_Receive(pFrame);
#endif
}
/*写硬件发送寄存器*/
static void can_adapter_Write_TDR(CAN_CTRL_t* Handle, CAN_TSMT_FRAME_t *pFrame)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
#endif
}
/*中断发送处理*/
static void can_adapter_IT_Send(CAN_CTRL_t *Handle)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
#endif
}
/********************************函数定义************************************
*函数名:

*函数功能描述: CAN控制器-外设实例访问(接口)

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*清除硬件接收缓存中的所有数据*/
int8_t can_adapter_Clr_RB(uint8_t Channel)
{
	CAN_CTRL_t* Handle = __CAN_GET_HANDLE(Channel);
	
	/*接口参数检查*/
	if(!Handle)
		return CAN_RET_ERR_PARAM;
	CAN_RCV_FRAME_t Frame = {0};
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    while(M4_CAN->RCTRL_f.RSSTAT != CanRxBufEmpty)
	{
		CAN_Receive(&Frame);
	}
	return CAN_RET_OK;
#endif
}
/*配置ID筛选器-只匹配ID*/
int8_t can_adapter_SetFilter_ID(uint8_t Channel, CAN_FILTERSEL_t FilterSel, uint32_t ID)
{
	CAN_CTRL_t* Handle = __CAN_GET_HANDLE(Channel);
	
	/*接口参数检查*/
	if(!Handle)
		return CAN_RET_ERR_PARAM;		
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)	//ID筛选功能在HC官方驱动库写的不好用,自己重写!
	CAN_RESET_ENABLE();
	//滤波器选择
	M4_CAN->ACFCTRL_f.ACFADR = FilterSel;
	//向所选滤波器写入配置值
	ID &= 0x1FFFFFFFu;
	M4_CAN->ACFCTRL_f.SELMASK = 0;		//0-ACF指向筛选器ID寄存器组
	M4_CAN->ACF = ID;
	M4_CAN->ACFCTRL_f.SELMASK = 1;		//1-ACF指向筛选器MASK寄存器组
	M4_CAN->ACF = 0x00000000u;			//29位MASK置0,表示ID所有位都比较
	M4_CAN->ACF_f.AIDEE = ((CanAllFrames >> 1ul) & 0x01u);
	//使能所选滤波器
	M4_CAN->ACFEN |= (uint8_t)(1ul << FilterSel);
	CAN_RESET_DISABLE();	
#endif
    return CAN_RET_OK;
}
/*配置ID筛选器-用MASK匹配ID*/
int8_t can_adapter_SetFilter_IDMask(uint8_t Channel, CAN_FILTERSEL_t FilterSel, uint32_t ID, uint32_t Mask)
{
	CAN_CTRL_t* Handle = __CAN_GET_HANDLE(Channel);
	
	/*接口参数检查*/
	if(!Handle)
		return CAN_RET_ERR_PARAM;		
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)	//ID筛选功能在官方驱动库写的不好用,自己重写!
	CAN_RESET_ENABLE();
	//滤波器选择
	M4_CAN->ACFCTRL_f.ACFADR = FilterSel;
	//向所选滤波器写入配置值	
	ID &= 0x1FFFFFFFu;
	Mask &= 0x1FFFFFFFu;
	M4_CAN->ACFCTRL_f.SELMASK = 0;		//0-ACF指向筛选器ID寄存器组
	M4_CAN->ACF = ID;
	M4_CAN->ACFCTRL_f.SELMASK = 1;		//1-ACF指向筛选器MASK寄存器组
	M4_CAN->ACF = Mask;					//29位MASK置0,表示ID所有位都比较
	M4_CAN->ACF_f.AIDEE = ((CanAllFrames >> 1ul) & 0x01u);
	//使能所选滤波器
	M4_CAN->ACFEN |= (uint8_t)(1ul << FilterSel);
	CAN_RESET_DISABLE();	
#endif
    return CAN_RET_OK;
}
/*使能ID筛选器*/
int8_t can_adapter_EnableFilter(uint8_t Channel, CAN_FILTERSEL_t FilterSel)
{
	CAN_CTRL_t* Handle = __CAN_GET_HANDLE(Channel);
	
	/*接口参数检查*/
	if(!Handle)
		return CAN_RET_ERR_PARAM;
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
	CAN_RESET_ENABLE();
	M4_CAN->ACFEN |= (uint8_t)(1ul << FilterSel);
	CAN_RESET_DISABLE();	
#endif
    return CAN_RET_OK;
}
/*禁止ID筛选器*/
int8_t can_adapter_DisableFilter(uint8_t Channel, CAN_FILTERSEL_t FilterSel)
{
	CAN_CTRL_t* Handle = __CAN_GET_HANDLE(Channel);
	
	/*接口参数检查*/
	if(!Handle)
		return CAN_RET_ERR_PARAM;
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
	CAN_RESET_ENABLE();
	M4_CAN->ACFEN &= (uint8_t)(~(1ul << FilterSel));
	CAN_RESET_DISABLE();	
#endif
    return CAN_RET_OK;
}
/*修改波特率*/
int8_t can_adapter_Set_BDR(uint8_t Channel, uint32_t BDR)
{
	CAN_CTRL_t* Handle = __CAN_GET_HANDLE(Channel);
	
	/*接口参数检查*/
	if(!Handle)
		return CAN_RET_ERR_PARAM;		
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
#endif
    return CAN_RET_OK;
}
/*使能硬件功能*/
int8_t can_adapter_Func_Sel(uint8_t Channel, uint16_t Func, uint8_t Enable)
{
	CAN_CTRL_t* Handle = __CAN_GET_HANDLE(Channel);
	
	/*接口参数检查*/
	if(!Handle)	
		return CAN_RET_ERR_PARAM;	
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    if(Func == CAN_FUNC_TX)
        CAN_IrqCmd(CanRxIrqEn, (en_functional_state_t)Enable);
    else if(Func == CAN_FUNC_TXE_IE)
        CAN_IrqCmd(CanRxIrqEn, (en_functional_state_t)Enable);
    else if(Func == CAN_FUNC_RXNE_IE)
        CAN_IrqCmd(CanRxIrqEn, (en_functional_state_t)Enable);
    else if(Func == CAN_FUNC_TC_IE)
        CAN_IrqCmd(CanRxIrqEn, (en_functional_state_t)Enable);
    else if(Func == CAN_FUNC_IDLE_IE)
        CAN_IrqCmd(CanRxIrqEn, (en_functional_state_t)Enable);
	else
    {
    }
	return CAN_RET_OK;
#endif
}
/*标准帧装载*/
int8_t can_adapter_LoadStdFrame(CAN_TSMT_FRAME_t* pFrame, uint32_t CAN_ID, uint8_t* pData, uint8_t Len)
{
	/*接口参数检查*/
	if(!pFrame || Len > 8)	
		return CAN_RET_ERR_PARAM;
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
	pFrame->StdID = CAN_ID;
	pFrame->Control_f.RTR = 0;	//1：遥控帧  0：数据帧
	pFrame->Control_f.IDE = 0;	//1：扩展帧  0：标准帧
	pFrame->Control_f.DLC = Len;
	pFrame->enBufferSel = CanPTBSel;//使用PTB发送
	for(uint8_t i=0; i<Len; i++)
	{
		pFrame->Data[i] = *pData;
		pData++;
	}
#endif
    return CAN_RET_OK;
}

/*扩展帧装载*/
int8_t can_adapter_LoadExtFrame(CAN_TSMT_FRAME_t* pFrame, uint32_t CAN_ID, uint8_t* pData, uint8_t Len)
{
	/*接口参数检查*/
	if(!pFrame || Len > 8)	
		return CAN_RET_ERR_PARAM;
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
	pFrame->ExtID = CAN_ID;
	pFrame->Control_f.RTR = 0;	//1：遥控帧  0：数据帧
	pFrame->Control_f.IDE = 1;	//1：扩展帧  0：标准帧
	pFrame->Control_f.DLC = Len;
	pFrame->enBufferSel = CanPTBSel;//使用PTB发送
	for(uint8_t i=0; i<Len; i++)
	{
		pFrame->Data[i] = *pData;
		pData++;
	}
#endif
    return CAN_RET_OK;
}
/*轮训发送*/
int8_t can_adapter_Transmit_Polling(uint8_t Channel, const CAN_TSMT_FRAME_t* pFrame, uint8_t Len)
{
    CAN_CTRL_t* Handle = __CAN_GET_HANDLE(Channel);
	
	/*接口参数检查*/
	if(!Handle)	
		return CAN_RET_ERR_PARAM;
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
	Handle->Recorder.TxFrameCnt++;
	CAN_SetFrame((CAN_TSMT_FRAME_t*)pFrame);	//不强转会报const警告,不明白
	CAN_TransmitCmd(CanPTBTxCmd);
#endif
    return CAN_RET_OK;
}
/*中断发送*/
int8_t can_adapter_Transmit_IT(uint8_t Channel, const CAN_TSMT_FRAME_t* pFrame, uint8_t Len)
{
    CAN_CTRL_t* Handle = __CAN_GET_HANDLE(Channel);
	
	/*接口参数检查*/
	if(!Handle || !Len)	
		return CAN_RET_ERR_PARAM;
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
	Handle->Recorder.TxFrameCnt++;
	CAN_SetFrame((CAN_TSMT_FRAME_t*)pFrame);	//不强转会报const警告,不明白
	CAN_TransmitCmd(CanPTBTxCmd);
#endif
    return CAN_RET_OK;
}
/*轮训接收*/
int8_t can_adapter_Receive_Polling(uint8_t Channel, CAN_RCV_FRAME_t *pFrame, uint8_t Len)
{
    CAN_CTRL_t* Handle = __CAN_GET_HANDLE(Channel);
	
	/*接口参数检查*/
	if(!Handle)
		return CAN_RET_ERR_PARAM;
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
	if(M4_CAN->RCTRL_f.RSSTAT != CanRxBufEmpty)
	{
		CAN_Receive(pFrame);
		return CAN_RET_OK;
	}
#endif
    return CAN_RET_ERR_NODATA;
}
/*中断接收*/
int8_t can_adapter_Receive_IT(uint8_t Channel, CAN_RCV_FRAME_t *pFrame, uint8_t Len)
{
    CAN_CTRL_t* Handle = __CAN_GET_HANDLE(Channel);
	
	/*接口参数检查*/
	if(!Handle)
		return CAN_RET_ERR_PARAM;
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
#endif
    return CAN_RET_OK;
}
/*总线故障关闭恢复*/
int8_t can_adapter_BusRecover(uint8_t Channel)
{
    CAN_CTRL_t* Handle = __CAN_GET_HANDLE(Channel);
	CAN_RCV_FRAME_t Frame = {0};
	/*接口参数检查*/
	if(!Handle)
		return CAN_RET_ERR_PARAM;
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
	if(M4_CAN->CFG_STAT_f.RESET == 1 && Handle->RxIdleTime > Handle->RxTimeOut)
	{
		CAN_RESET_ENABLE();
		CAN_RESET_DISABLE();
		while(M4_CAN->RCTRL_f.RSSTAT != CanRxBufEmpty)	//清除RB中已有数据
		{
			CAN_Receive(&Frame);
		}
	}
#endif
	return CAN_RET_OK;
}
/********************************函数定义************************************
*函数名:

*函数功能描述: CAN控制器-控制器相关操作

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*CAN控制器-初始化*/
int8_t can_hInit(CAN_CTRL_t *Handle, CAN_INS_t Ins, uint8_t Channel)
{
	/*参数检查*/
    if(Channel >= MCU_CAN_NB || !Ins || !Handle)
        return CAN_RET_ERR_INIT;
    /*将Channel与数组元素(对应1个句柄)绑定,实现通过Channel访问对应的结构体实例*/
    CanHandle[Channel] = Handle;
    /*句柄初始化*/
	memset(Handle, 0, sizeof(CAN_CTRL_t));
    Handle->Ins = Ins;
	Handle->RxTimeOut = CAN_RX_TIMEOUT;
    return CAN_RET_OK;
}
/*CAN控制器-获取已接的帧*/
/*注: 参数Data可以是0, Data==0表示销毁已接收数据不输出到外部缓存*/
int8_t can_Get_Receive(uint8_t Channel, CAN_RCV_FRAME_t* pFrame, uint8_t Len)
{
    CAN_CTRL_t* Handle = __CAN_GET_HANDLE(Channel);
//	CAN_RCV_FRAME_t Ignore = {0};
	/*接口参数检查*/
	if(!Handle || !Len)
		return CAN_RET_ERR_PARAM;
    uint8_t ReFrameNb = 0;
	if(Handle->Rear != Handle->Head)	//有新数据到达ReBuffer
	{
		if(Handle->Rear >= Handle->Head)
			ReFrameNb = Handle->Rear - Handle->Head;
		else
			ReFrameNb = (CAN_RX_BUF_SIZE + Handle->Rear - Handle->Head);
		if(pFrame)
		{
			for(uint8_t i = 0; i < (ReFrameNb < Len ? ReFrameNb : Len); i++)//剩余数据比需要获取的数据少,将剩余数据全部读走
			{
				memcpy(pFrame, &Handle->ReBuffer[Handle->Head], sizeof(CAN_RCV_FRAME_t));
				if(++Handle->Head >= CAN_RX_BUF_SIZE)
					Handle->Head = 0;
			}
		}else
		{
			Handle->Head = Handle->Rear;
		}
		return CAN_RET_OK;
    }
    return CAN_RET_ERR_NODATA;
}
/*CAN控制器-1ms定时应用*/
int8_t can_Timer_1ms(uint8_t Channel)
{
    CAN_CTRL_t* Handle = __CAN_GET_HANDLE(Channel);
	
	/*接口参数检查*/
	if(!Handle)
		return CAN_RET_ERR_PARAM;
	/*接收空闲计时*/
    if(Handle->RxIdleTime < 60000)
		Handle->RxIdleTime++;
	return CAN_RET_OK;
}
/********************************函数定义************************************
*函数名:

*函数功能描述: CAN控制器-中断回调函数

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*CAN控制器-接收回调函数*/
int8_t can_Rx_Callback(uint8_t Channel)
{
    CAN_CTRL_t* Handle = __CAN_GET_HANDLE(Channel);
	uint8_t i = 0;
	
	/*接口参数检查*/
	if(!Handle)
		return CAN_RET_ERR_PARAM;
    if(can_adapter_Get_SR(Handle, CAN_RXNE))
    {
        can_adapter_Clr_SR(Handle, CAN_RXNE);
	#if RX_FIFO_MODE	//f460的can的RB是一个深度10的FIFO,若采用中断方式接收必须把RB中数据全读出来,原因详见《华大F460芯片CAN调试笔记-230331》!
		/*须判断RB不为空才读取: 若发生这种情况,ISR执行can_adapter_Clr_SR(Handle, CAN_RXNE)后又收到新数据(rx中断标志再次置位),此时执行
		  while(can_adapter_RB_Not_Empty(Handle))将所有数据(包括新数据)从RB读出,退出ISR会直接再次触发中断,但此时RB中新的数据已经不存在了*/
		while(can_adapter_RB_Not_Empty(Handle))
		{
			can_adapter_Read_RDR(Handle, &Handle->ReBuffer[Handle->Rear]);
			if(++Handle->Rear >= CAN_RX_BUF_SIZE)
				Handle->Rear = 0;
			Handle->Recorder.RxFrameCnt++;
			Handle->RxIdleTime = 0;
			i++;
		}
		if(i)	
			return CAN_RET_OK;
	#else	//只取一包数据
		if(can_adapter_RB_Not_Empty(Handle))
		{
			can_adapter_Read_RDR(Handle, &Handle->ReBuffer[Handle->Rear]);
			if(++Handle->Rear >= CAN_RX_BUF_SIZE)
				Handle->Rear = 0;
			Handle->Recorder.RxFrameCnt++;
			Handle->RxIdleTime = 0;
			return CAN_RET_OK;
		}
	#endif
    }
    return CAN_RET_BUSY;
}
/*CAN控制器-错误回调函数*/
int8_t can_Err_Callback(uint8_t Channel)
{
    CAN_CTRL_t* Handle = __CAN_GET_HANDLE(Channel);
    //CAN_RCV_FRAME_t ReData = {0};
	uint32_t CanErrorFlag = 0;
	
	/*接口参数检查*/
	if(!Handle)
		return CAN_RET_ERR_PARAM;
	CanErrorFlag = can_adapter_ClrErrOccured(Handle);
    if(CanErrorFlag != 0)
    {
		can_adapter_Clr_RB(Channel);
		Handle->Recorder.ErrCnt++;
        return CAN_RET_OK;
    }
    return CAN_RET_BUSY;
}
/*CAN控制器-发送缓冲区空回调函数*/
int8_t can_TXE_Callback(uint8_t Channel)
{
    CAN_CTRL_t* Handle = __CAN_GET_HANDLE(Channel);
	
	/*接口参数检查*/
	if(!Handle)
		return CAN_RET_ERR_PARAM;
    if(can_adapter_Get_SR(Handle, CAN_TXE))
    {
        can_adapter_Clr_SR(Handle, CAN_TXE);
        can_adapter_IT_Send(Handle);
        return CAN_RET_OK;
    }
    return CAN_RET_BUSY;
}
/*CAN控制器-发送完成回调函数*/
int8_t can_TC_Callback(uint8_t Channel)
{
    CAN_CTRL_t* Handle = __CAN_GET_HANDLE(Channel);
	
	/*接口参数检查*/
	if(!Handle)
		return CAN_RET_ERR_PARAM;
    if(can_adapter_Get_SR(Handle, CAN_TC))
    {
        can_adapter_Clr_SR(Handle, CAN_TC);
        can_adapter_Func_Sel(Channel, CAN_FUNC_TC_IE, CAN_FUNC_DISABLE);
        return CAN_RET_OK;
    }
    return CAN_RET_BUSY;
}
/********************************函数定义************************************
*函数名:

*函数功能描述: CAN控制器-测试

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
static int8_t s_TestCAN = 0;
void can_Test(void)
{
    static CAN_TSMT_FRAME_t s_TxData = {0};
    static CAN_RCV_FRAME_t s_RxData = {0};

	static uint8_t s_Data8[8] = {0};
	
    if(s_TestCAN == -1)
    {
        s_TestCAN = 0;
		memset(&s_Data8, 0, sizeof s_RxData);
        memset(&s_RxData, 0, sizeof s_RxData);
    }
    if(s_TestCAN == 1)
    {
        //s_TestCAN = 0;
		for(uint8_t i=0; i<8; i++)
			s_Data8[i]++;
		can_adapter_LoadStdFrame(&s_TxData, 0x70, s_Data8, 8);
        can_adapter_Transmit_Polling(CAN1, &s_TxData, 10);
    }
    if(s_TestCAN == 2)
    {
        //s_TestCAN = 0;
		if(can_Get_Receive(CAN1, &s_RxData, 1) == CAN_RET_OK)
		{
			can_adapter_LoadStdFrame(&s_TxData, s_RxData.StdID, s_RxData.Data, s_RxData.Cst.Control_f.DLC);
			can_adapter_Transmit_Polling(CAN1, &s_TxData, 1);
		}
    }
	if(s_TestCAN == 3)
    {
        //s_TestCAN = 0;
		if(can_adapter_Receive_Polling(CAN1, &s_RxData, 1) == CAN_RET_OK)
		{
			__nop();
		}
    }
#if (MCU_TYPE == MCU_TYPE_HC32_F4)
	if(s_TestCAN == 4)
    {
        s_TestCAN = 0;
		if(can_adapter_RB_Not_Empty(NULL))
		{
			can_adapter_Read_RDR(NULL, &s_RxData);
		}
    }
	if(s_TestCAN == 5)	//总线故障恢复
    {
        s_TestCAN = 0;
		CAN_RESET_ENABLE();
		CAN_RESET_DISABLE();
	}
	if(s_TestCAN == 6)
	{
		s_TestCAN = 0;
		__disable_irq();
	}
	if(s_TestCAN == 7)
	{
		s_TestCAN = 0;
		__enable_irq();
	}
	if(s_TestCAN == 8)
	{
		s_TestCAN = 0;
		CAN_IrqCmd(CanRxIrqEn, Enable);
	}
	if(s_TestCAN == 9)
	{
		s_TestCAN = 0;
		CAN_IrqCmd(CanRxIrqEn, Disable);
	}
#endif
}
