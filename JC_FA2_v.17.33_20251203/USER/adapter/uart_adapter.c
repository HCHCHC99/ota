/********************************文件说明*************************************
*文件名: uart_adapter.c

*作者: Yuchen Tan

*版本: V1.0.1

*功能简介:

*备注: 无

*修改履历:
------------------------------------V1.0.1------------------------------------
20230217:
1.删除不同数据收发模式(轮训\中断\DMA..)定义及模式设置接口,删除通用数据发送接口
  uart_Send.不同模式采用不同的接口(eg: 轮训发送用uart_adapter_Transmit_Polling()).
*****************************************************************************/

/*****************************文件包含(私有)**********************************
*
*备注: 无
*
*****************************************************************************/
#include "uart_adapter.h"
/*****************************宏定义(私有)************************************
*
*备注: 本文件中,不希望被外部使用或随意修改的宏在这里定义
*
*****************************************************************************/
/*串口空闲(接收超时)检测方式: 1-通过软件检测 0-通过硬件检测*/
#if (MCU_TYPE == MCU_TYPE_STM32)
#define IDLE_CHECK_SW       (0)
#else
#define IDLE_CHECK_SW       (1)
#endif

#ifdef IDLE_CHECK_SW
/*接收超时检测时间(单位: 1=250us)*/
#define RX_TIME_OUT         (10)
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
uint8_t g_RxData = 0;
int8_t s_TestUart = 0;
UART_CTRL_t* UartHandle[MCU_UART_NB];
/********************************函数定义************************************
*函数名:

*函数功能描述: 串口控制器-不同MCU串口底层驱动适配

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*使能硬件功能*/
void uart_adapter_Func_Sel(UART_CTRL_t* Handle, uint16_t Func, uint8_t Enable)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
    UART_HandleTypeDef  *huart = Handle->Ins.huart;
    if(Func == FUNC_TX)
    {
    }else if(Func == FUNC_RX)
    {
    }else if(Func == FUNC_TXE_IE)
    {
    }else if(Func == FUNC_RXNE_IE)
    {
        if(Enable)
        {
            __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);  //周工发现有这行代码程序一上电就会死在串口中断中
            HAL_UART_Receive_IT(huart, &g_RxData, 1);
        }else
        {
        }
    }else if(Func == FUNC_TC_IE)
    {
    }else if(Func == FUNC_IDLE_IE)
    {
        if(Enable)
            __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
        else
    }else
    {
    }
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    M4_USART_TypeDef *USARTx = Handle->Ins.USARTx;
    if(Func == UART_FUNC_TX)
        USART_FuncCmd(USARTx, UsartTx, (en_functional_state_t)Enable);
    else if(Func == UART_FUNC_RX)
        USART_FuncCmd(USARTx, UsartRx, (en_functional_state_t)Enable);
    else if(Func == UART_FUNC_TXE_IE)
        USART_FuncCmd(USARTx, UsartTxEmptyInt, (en_functional_state_t)Enable);
    else if(Func == UART_FUNC_RXNE_IE)
        USART_FuncCmd(USARTx, UsartRxInt, (en_functional_state_t)Enable);
    else if(Func == UART_FUNC_TC_IE)
        USART_FuncCmd(USARTx, UsartTxCmpltInt, (en_functional_state_t)Enable);
    else if(Func == UART_FUNC_IDLE_IE)
    {
        USART_FuncCmd(USARTx, UsartTimeOut, (en_functional_state_t)Enable);
        USART_FuncCmd(USARTx, UsartTimeOutInt, (en_functional_state_t)Enable);
    }else
    {
    }
#endif
}
/*读硬件状态标志*/
static uint32_t uart_adapter_Get_SR(UART_CTRL_t* Handle, uint8_t Status)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    M4_USART_TypeDef *USARTx = Handle->Ins.USARTx;
    if(Status == UART_IDLE)
        return USART_GetStatus(USARTx, UsartRxTimeOut);
    else if(Status == UART_TC)
        return USART_GetStatus(USARTx, UsartTxComplete);
    else if(Status == UART_TXE)
        return USART_GetStatus(USARTx, UsartTxEmpty);
    else if(Status == UART_RXNE)
        return USART_GetStatus(USARTx, UsartRxNoEmpty);
    else if(Status == UART_FE)
        return USART_GetStatus(USARTx, UsartFrameErr);
    else if(Status == UART_PE)
        return USART_GetStatus(USARTx, UsartParityErr);
    else if(Status == UART_OVR)
        return USART_GetStatus(USARTx, UsartOverrunErr);
    else
        return 0;
#endif
}
/*清硬件状态标志*/
static void uart_adapter_Clr_SR(UART_CTRL_t* Handle, uint8_t Status)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    M4_USART_TypeDef *USARTx = Handle->Ins.USARTx;
    if(Status == UART_IDLE)
        USART_ClearStatus(USARTx, UsartRxTimeOut);
    else if(Status == UART_FE)
        USART_ClearStatus(USARTx, UsartFrameErr);
    else if(Status == UART_PE)
        USART_ClearStatus(USARTx, UsartParityErr);
    else if(Status == UART_OVR)
        USART_ClearStatus(USARTx, UsartOverrunErr);
    else
    {
    }
#endif
}
/*清硬件错误寄存器*/
static uint8_t uart_adapter_ClrErrOccured(UART_CTRL_t* Handle)
{
    uint8_t ErrOccur = 0;
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    M4_USART_TypeDef *USARTx = Handle->Ins.USARTx;
    if (Set == USART_GetStatus(USARTx, UsartFrameErr))
    {
        ErrOccur = 1;
        USART_ClearStatus(USARTx, UsartFrameErr);
    }
    if (Set == USART_GetStatus(USARTx, UsartParityErr))
    {
        ErrOccur = 1;
        USART_ClearStatus(USARTx, UsartParityErr);
    }
    if (Set == USART_GetStatus(USARTx, UsartOverrunErr))
    {
        ErrOccur = 1;
        USART_ClearStatus(USARTx, UsartOverrunErr);
        Handle->Recorder.OverRunCnt++;
    }
#endif
    return ErrOccur;
}
/*读硬件接收寄存器*/
static uint8_t uart_adapter_Read_RDR(UART_CTRL_t* Handle)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    return (uint8_t)USART_RecData(Handle->Ins.USARTx);
#endif
}
/*写硬件发送寄存器*/
static void uart_adapter_Write_TDR(UART_CTRL_t* Handle, uint8_t Data)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    USART_SendData(Handle->Ins.USARTx, Data);
#endif
}
/*修改波特率*/
static int8_t uart_adapter_Change_BDR(UART_CTRL_t* Handle, uint32_t BDR)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
    UART_HandleTypeDef  *huart = Handle->Ins.huart;
    if(huart.Init.BaudRate != BDR)
    {
        HAL_UART_DeInit(&huart);
        huart.Instance = huart;
        huart.Init.BaudRate = BDR;
        huart.Init.WordLength = UART_WORDLENGTH_8B;
        huart.Init.StopBits = UART_STOPBITS_1;
        huart.Init.Parity = UART_PARITY_NONE;
        huart.Init.Mode = UART_MODE_TX_RX;
        huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
        huart.Init.OverSampling = UART_OVERSAMPLING_16;
        huart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
        huart.Init.ClockPrescaler = UART_PRESCALER_DIV1;
        huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
        if (HAL_UART_Init(&huart) != HAL_OK)
        {
            Error_Handler();
        }
    }
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
//  M4_USART_TypeDef *USARTx = Handle->Ins.USARTx;
//    stc_usart_uart_init_t stcInitCfg;

//  MEM_ZERO_STRUCT(stcInitCfg);
//
//    //stcInitCfg.enClkMode = UsartIntClkCkNoOutput;
//  stcInitCfg.enClkMode = UsartIntClkCkOutput;     //想用IDLE(接收超时)功能需要选UsartIntClkCkOutput
//  if(BDR <= 2400)
//      stcInitCfg.enClkDiv = UsartClkDiv_64;       //1200bps
//  else if(BDR <= 19200)
//      stcInitCfg.enClkDiv = UsartClkDiv_16;       //9600bps
//  else
//      stcInitCfg.enClkDiv = UsartClkDiv_4;        //115200bps
//    stcInitCfg.enDataLength = UsartDataBits8;
//    stcInitCfg.enDirection = UsartDataLsbFirst;
//    stcInitCfg.enStopBit = UsartOneStopBit;
//    stcInitCfg.enParity = UsartParityNone;
//    stcInitCfg.enSampleMode = UsartSampleBit8;
//    stcInitCfg.enDetectMode = UsartStartBitFallEdge;
//    stcInitCfg.enHwFlow = UsartRtsEnable;
//    if(Ok != USART_UART_Init(USARTx, &stcInitCfg))
//    {
//        while (1);
//    }
//  if(Ok != USART_SetBaudrate(USARTx, BDR))
//    {
//        while (1);
//    }
#endif
    return UART_RET_OK;
}
/*中断发送处理*/
static void uart_adapter_IT_Send(UART_CTRL_t *Handle)
{
    uint8_t SendData = 0;
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    //注: F460和STM32的中断发送有较大区别,F460即使TXE标志已置1,也必须写TDR才能触发TXE中断.\
    因此触发TXE中断时,第1字节已从发送寄存器移至移位寄存器.中断发N个字节,只会触发N-1次TXE中断,和参考手册上的中断发送时序图不一样)
    if(Handle->ByteToTransmit)
    {
        SendData = *Handle->TransmitData;
        USART_SendData(Handle->Ins.USARTx, SendData);
        Handle->ByteToTransmit--;
        Handle->TransmitData++;
    }
    if(!Handle->ByteToTransmit)
    {
        Handle->TransmitData = 0;
        USART_FuncCmd(Handle->Ins.USARTx, UsartTxEmptyInt, Disable);
        USART_FuncCmd(Handle->Ins.USARTx, UsartTxCmpltInt, Enable);
    }
#endif
}
/*阻塞式轮训发送*/
int8_t uart_adapter_Transmit_Polling(uint8_t Channel, const uint8_t *Data, uint8_t Len)
{
    UART_CTRL_t* Handle = uart_Get_Handle(Channel);

    Handle->TxState = UC_TX_BUSY;
#if (MCU_TYPE == MCU_TYPE_STM32)
    HAL_UART_Transmit(Ins->huart, Data, Len, Len*10);
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    while(Len)
    {
        if(Set == USART_GetStatus(Handle->Ins.USARTx, UsartTxEmpty))
        {
            USART_SendData(Handle->Ins.USARTx, *Data);
            Data++;
            Len--;
            Handle->Recorder.TxByteCnt++;
        }
    }
    while(Reset == USART_GetStatus(Handle->Ins.USARTx, UsartTxComplete));
#endif
    Handle->TxState = UC_READY;
    Handle->StateFlag |= UART_TC;
    Handle->Recorder.TxPackCnt++;
    return UART_RET_OK;
}
/*中断发送*/
/*华大F460中断发送似乎有点问题*/
int8_t uart_adapter_Transmit_IT(uint8_t Channel, const uint8_t *Data, uint8_t Len)
{
    UART_CTRL_t* Handle = uart_Get_Handle(Channel);

    if(!Len)
        return UART_RET_ERR_PARAM;
    Handle->TxState = UC_TX_BUSY;
    Handle->ByteToTransmit = Len;
    Handle->TransmitData = Data;
#if (MCU_TYPE == MCU_TYPE_STM32)
    HAL_UART_Transmit_IT(Ins->huart, Data, Len);
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    USART_FuncCmd(Handle->Ins.USARTx, UsartTxAndTxEmptyInt, Enable);
    USART_FuncCmd(Handle->Ins.USARTx, UsartTxCmpltInt, Disable);
    USART_SendData(Handle->Ins.USARTx, *Data);  //TI(TXE)中断发生条件: TEIE使能&&数据移入移位寄存器中.故需先写一个数据.否则无法触发TI(TXE)中断
    Handle->ByteToTransmit--;
    Handle->TransmitData++;
    Handle->Recorder.TxByteCnt++;
#endif
    return UART_RET_OK;
}
/*阻塞式轮训接收*/
int8_t uart_adapter_Receive_Polling(uint8_t Channel, uint8_t *Data, uint8_t Len)
{
    UART_CTRL_t* Handle = uart_Get_Handle(Channel);

    Handle->RxState = UC_RX_BUSY;
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    while(Len)
    {
        if (Set == USART_GetStatus(Handle->Ins.USARTx, UsartRxNoEmpty))
        {
            *Data = USART_RecData(Handle->Ins.USARTx);
            Data++;
            Len--;
        }
    }
#endif
    Handle->RxState = UC_READY;
    return UART_RET_OK;
}
/*中断接收*/
int8_t uart_adapter_Receive_IT(uint8_t Channel, uint8_t *Data, uint8_t Len)
{
    UART_CTRL_t* Handle = uart_Get_Handle(Channel);

    Handle->RxState = UC_RX_BUSY;
#if (MCU_TYPE == MCU_TYPE_STM32)
    HAL_UART_Receive_IT(Ins->huart, Data, Len);
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
#endif
    return UART_RET_OK;
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 串口控制器-控制器

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*串口控制器-控制器初始化*/
int8_t uart_hInit(UART_CTRL_t *Handle, UART_INS_t* Ins, uint32_t BDR)
{
    if(Ins->Channel >= MCU_UART_NB)
    {
        return UART_RET_ERR_INIT;
    }
    /*将Channel与数组元素(对应1个句柄)绑定,实现通过Channel访问对应的结构体实例*/
    UartHandle[Ins->Channel] = Handle;
    /*句柄初始化*/
    Handle->Ins = *Ins;
    Handle->BDR = BDR;
    uart_adapter_Change_BDR(Handle, BDR);
    Handle->StateFlag = 0;
    Handle->TxState = UC_READY;
    Handle->RxState = UC_READY;
    Handle->RxTimeoutCnt = 0xFF;
    Handle->TransmitData = 0;
    memset(Handle->ReceiveData, 0, RX_BUF_SIZE);
    Handle->BufHead[0] = 0;
    Handle->BufRear[0] = 0;
    Handle->ByteToTransmit = 0;
    Handle->ByteToReceive = 0;
    return UART_RET_OK;
}
/*串口控制器-获取控制器句柄*/
UART_CTRL_t* uart_Get_Handle(uint8_t Channel)
{
    return UartHandle[Channel];
}
/*串口控制器-获取状态标志*/
uint8_t uart_Get_State(uint8_t Channel, uint8_t State)
{
    UART_CTRL_t* Handle = uart_Get_Handle(Channel);

    return (Handle->StateFlag & State);
}
/*串口控制器-清除状态标志*/
void uart_Clr_State(uint8_t Channel, uint8_t State)
{
    UART_CTRL_t* Handle = uart_Get_Handle(Channel);

    Handle->StateFlag &= ~State;
}
/*串口控制器-发送就绪*/
BOOL uart_Transmit_Ready(uint8_t Channel)
{
    UART_CTRL_t* Handle = uart_Get_Handle(Channel);

    return (BOOL)(Handle->TxState == UC_READY);
}
/*串口控制器-使能接收*/
int8_t uart_Rx_Enable(uint8_t Channel)
{
    UART_CTRL_t* Handle = uart_Get_Handle(Channel);

    uart_adapter_Func_Sel(Handle, UART_FUNC_RX, UART_FUNC_ENABLE);
    uart_adapter_Func_Sel(Handle, UART_FUNC_RXNE_IE, UART_FUNC_ENABLE);
    return UART_RET_OK;
}
/*串口控制器-重置接收*/
int8_t uart_Rx_Reset(uint8_t Channel)
{
    UART_CTRL_t* Handle = uart_Get_Handle(Channel);

    Handle->RxTimeoutCnt = 0xFF;
    Handle->BufHead[0] = Handle->BufRear[0] = 0;
    Handle->StateFlag &= ~UART_IDLE;
    return UART_RET_OK;
}
/*串口控制器-修改波特率*/
int8_t uart_Change_BDR(uint8_t Channel, uint32_t BDR)
{
    UART_CTRL_t* Handle = uart_Get_Handle(Channel);

    Handle->BDR = BDR;
    return uart_adapter_Change_BDR(Handle, BDR);
}
/*串口控制器-获取已接收数据*/
/*注: 参数Data和Len可以是0, Data==0表示销毁已接收数据不输出到外部缓存, Len==0表示上层不需要获取收到多少字节*/
int8_t uart_Get_Receive(uint8_t Channel, uint8_t *Data, uint16_t *Len)
{
    UART_CTRL_t* Handle = uart_Get_Handle(Channel);
    uint8_t ReLen = 0;

    if(Handle->StateFlag & UART_IDLE)
    {
        Handle->StateFlag &= ~UART_IDLE;
        if(Handle->BufRear == Handle->BufHead)
        {   //切换波特率可能导致IDLE置1,须增加判断(Rear != Head)且不能与if(StateFlag & IDLE)合并,以保证IDLE置位就能立即清0!
            return UART_RET_ERR_NODATA;
        }
        if(Handle->BufRear[0] >= Handle->BufHead[0])
            ReLen = Handle->BufRear[0] - Handle->BufHead[0];
        else
            ReLen = (RX_BUF_SIZE + Handle->BufRear[0] - Handle->BufHead[0]);
        if(Data)
        {
            for(uint8_t i = 0; i < ReLen; i++)
            {
                Data[i] = Handle->ReceiveData[Handle->BufHead[0]];
                if(++Handle->BufHead[0] >= RX_BUF_SIZE)
                    Handle->BufHead[0] = 0;
            }
        }else
        {
            Handle->BufHead[0] = Handle->BufRear[0];
        }
        if(Len)
            *Len = ReLen;
        return UART_RET_OK;
    }
    return UART_RET_ERR_NODATA;
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 串口控制器-中断回调函数

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*串口控制器-空闲回调函数(
使用硬件空闲检测：在空闲中断中调用
使用软件空闲检测：在250us中断中调用)*/
uint8_t uart_Idle_Callback(uint8_t Channel)
{
    UART_CTRL_t* Handle = uart_Get_Handle(Channel);
    uint8_t IdleEventOccur = 0;

#if (IDLE_CHECK_SW == 1)
    if(Handle->RxTimeoutCnt < 100)
        Handle->RxTimeoutCnt++;
    if(Handle->RxTimeoutCnt >= RX_TIME_OUT && Handle->RxTimeoutCnt <= 100)
    {
        IdleEventOccur = 1;
        Handle->RxTimeoutCnt = 0xFF;
    }
#else
    if(uart_adapter_Get_SR(Handle, UART_IDLE))
    {
        IdleEventOccur = 1;
        uart_adapter_Clr_SR(Handle, UART_IDLE);
    }
#endif
    if(IdleEventOccur)
    {
        Handle->StateFlag |= UART_IDLE;
        Handle->RxState = UC_READY;
        Handle->Recorder.RxPackCnt++;
        uart_adapter_Receive_IT(Channel, &g_RxData, 1);
        return 1;
    }
    return 0;
}
/*串口控制器-接收回调函数*/
uint8_t uart_Rx_Callback(uint8_t Channel)
{
    UART_CTRL_t* Handle = uart_Get_Handle(Channel);
    uint8_t ReData = 0;
    if(uart_adapter_Get_SR(Handle, UART_RXNE))
    {
        Handle->RxTimeoutCnt = 0;
        uart_adapter_Clr_SR(Handle, UART_RXNE);
        ReData = uart_adapter_Read_RDR(Handle);
        Handle->ReceiveData[Handle->BufRear[0]] = ReData;
        if(++Handle->BufRear[0] >= RX_BUF_SIZE)
            Handle->BufRear[0] = 0;
        Handle->Recorder.RxByteCnt++;
        uart_adapter_Receive_IT(Channel, &g_RxData, 1);
        return 1;
    }
    return 0;
}
/*串口控制器-错误回调函数*/
uint8_t uart_Err_Callback(uint8_t Channel)
{
    UART_CTRL_t* Handle = uart_Get_Handle(Channel);
    //uint8_t ReData = 0;
    if(uart_adapter_ClrErrOccured(Handle))
    {
        Handle->Recorder.ErrCnt++;
        //(void)uart_adapter_Read_RDR(Handle);  //清接收寄存器
//      ReData = uart_adapter_Read_RDR(Handle);
//      Handle->ReceiveData[Handle->BufRear] = ReData;
//      if(++Handle->BufRear >= RX_BUF_SIZE)
//          Handle->BufRear = 0;
//      Handle->Recorder.RxByteCnt++;
        uart_adapter_Receive_IT(Channel, &g_RxData, 1);
        return 1;
    }
    return 0;
}
/*串口控制器-发送缓冲区空回调函数*/
uint8_t uart_TXE_Callback(uint8_t Channel)
{
    UART_CTRL_t* Handle = uart_Get_Handle(Channel);

    if(uart_adapter_Get_SR(Handle, UART_TXE))
    {
        uart_adapter_Clr_SR(Handle, UART_TXE);
        Handle->Recorder.TxByteCnt++;
        uart_adapter_IT_Send(Handle);
        return 1;
    }
    return 0;
}
/*串口控制器-发送完成回调函数*/
uint8_t uart_TC_Callback(uint8_t Channel)
{
    UART_CTRL_t* Handle = uart_Get_Handle(Channel);

    if(uart_adapter_Get_SR(Handle, UART_TC))
    {
        uart_adapter_Clr_SR(Handle, UART_TC);
        Handle->TxState = UC_READY;
        Handle->Recorder.TxPackCnt++;
        Handle->StateFlag |= UART_TC;
        uart_adapter_Func_Sel(Handle, UART_FUNC_TC_IE, UART_FUNC_DISABLE);
        return 1;
    }
    return 0;
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 串口控制器-测试

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
void uart_Test(void)
{
    static uint8_t s_TxData[10] = {1,2,3,4,5,6,7,8,9,10};
    static uint8_t s_RxData[10];
//    static int8_t s_TestUart = 0;
    static uint16_t s_ReLen = 0;
//	uart_adapter_Transmit_Polling(UART1, s_TxData, 10);
    if(s_TestUart == -1)
    {
        s_TestUart = 0;
        memset(s_RxData, 0, 10);
    }
    if(s_TestUart == 1)
    {
        s_TestUart = 0;
        uart_adapter_Transmit_Polling(UART1, s_TxData, 10);
    }
    if(s_TestUart == 2)
    {
        s_TestUart = 0;
        if(uart_Get_Receive(UART1, s_RxData, &s_ReLen) == UART_RET_OK)
        {
            uart_adapter_Transmit_Polling(UART1, s_RxData, s_ReLen);
        }
    }
}
