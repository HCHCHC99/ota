/********************************文件说明*************************************
*文件名: uart_adapter.h

*作者: Yuchen Tan

*版本: V1.0.1

*功能简介:

*备注: 无

*修改履历:

*****************************************************************************/
#ifndef UART_ADAPTER_H_
#define UART_ADAPTER_H_

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
/*mcu串口索引号*/
#define UART1           (0)
#define UART2           (1)
#define UART3           (2)
#define UART4           (3)
#define MCU_UART_NB     (4)

/*接收循环缓存参数定义*/
#define RX_BUF_SIZE         (50)        //缓存字节数
#define RX_BUF_PARSE_NB     (2)         //缓存头尾索引组数量(每组用于多协议中的一种协议解析(Todo))

/*串口硬件功能*/
#define UART_FUNC_ENABLE    (1)
#define UART_FUNC_DISABLE   (0)
#define UART_FUNC_TX        (1<<0)      //发送器使能
#define UART_FUNC_RX        (1<<1)      //接收器使能
#define UART_FUNC_TXE_IE    (1<<2)      //发送寄存器空中断使能
#define UART_FUNC_RXNE_IE   (1<<3)      //接收寄存器非空中断使能
#define UART_FUNC_TC_IE     (1<<4)      //发送完成中断使能
#define UART_FUNC_IDLE_IE   (1<<5)      //接收空闲中断使能

/*串口硬件状态标志*/
#define UART_IDLE           (1<<0)      //空闲(接收完成/接收超时)
#define UART_TC             (1<<1)      //发送完成
#define UART_TXE            (1<<2)      //发送寄存器空
#define UART_RXNE           (1<<3)      //接收寄存器非空
#define UART_FE             (1<<4)      //帧错误
#define UART_NE             (1<<5)      //噪声错误
#define UART_PE             (1<<6)      //奇偶校验错误
#define UART_OVR            (1<<7)      //接收过载

/*串口功能返回值*/
#define UART_RET_OK         (1)
#define UART_RET_BUSY       (0)
#define UART_RET_ERR_INIT   (-1)
#define UART_RET_ERR_PARAM  (-2)
#define UART_RET_ERR_NODATA (-3)

/*串口控制器状态*/
#define UC_READY            (0)         //控制器就绪
#define UC_TX_BUSY          (1<<0)      //发送中
#define UC_RX_BUSY          (1<<1)      //接收中
/**************************数据类型及结构定义(公开)***************************
*
*备注: 需要被外部使用的数据结构及类型在这里定义
*
*****************************************************************************/
/*与MCU平台相关的抽象类型定义*/
#if (MCU_TYPE == MCU_TYPE_STM32)
/*数据类型定义-UART实例*/
typedef struct
{
    UART_HandleTypeDef  *huart; /*UART外设寄实例*/
    uint8_t             Channel;    /*UART外设(索引号)*/
}UART_INS_t;
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
/*数据类型定义-UART实例*/
typedef struct
{
    M4_USART_TypeDef    *USARTx;    /*UART外设寄实例*/
    uint8_t             Channel;    /*UART外设(索引号)*/
}UART_INS_t;
#endif

/*数据类型定义-UART控制器工作记录*/
typedef struct
{
    uint16_t    TxPackCnt;          //发包计数
    uint16_t    RxPackCnt;          //收包计数
    uint16_t    TxByteCnt;          //发字节计数
    uint16_t    RxByteCnt;          //收字节计数
    uint16_t    ErrCnt;             //错误计数
    uint16_t    OverRunCnt;         //接收过载错误
}UART_RECORDER_t;

/*数据类型定义-UART控制器*/
typedef struct
{
    /*UART控制器封装的MCU-UART外设实例*/
    UART_INS_t      Ins;
    /*UART波特率*/
    uint32_t        BDR;
    /*UART控制器状态*/
    uint8_t         StateFlag;      //状态标志
    uint8_t         TxState;        //发送控制器状态
    uint8_t         RxState;        //接收控制器状态
    uint8_t         RxTimeoutCnt;   //接收超时计数器(用于实现接收空闲检测)
    /*UART控制器数据缓冲区*/
    const uint8_t   *TransmitData;  //指向待发送数据存储地址
    uint8_t         ReceiveData[RX_BUF_SIZE];
    uint8_t         BufHead[RX_BUF_PARSE_NB];
    uint8_t         BufRear[RX_BUF_PARSE_NB];
    /*UART控制器数据缓冲区*/
    uint8_t         ByteToTransmit;
    uint8_t         ByteToReceive;
    /*UART控制器诊断记录*/
    UART_RECORDER_t Recorder;
}UART_CTRL_t;
/*****************************函数声明(公开)**********************************
*
*备注: 需要被外部使用的接口函数在这里声明
*
*****************************************************************************/
/*串口控制器*/
int8_t uart_hInit(UART_CTRL_t *Handle, UART_INS_t* Ins, uint32_t BDR);
UART_CTRL_t* uart_Get_Handle(uint8_t Channel);
void uart_adapter_Func_Sel(UART_CTRL_t *Handle, uint16_t Func, uint8_t Enable);
uint8_t uart_Get_State(uint8_t Channel, uint8_t State);
void uart_Clr_State(uint8_t Channel, uint8_t State);
BOOL uart_Transmit_Ready(uint8_t Channel);
int8_t uart_Change_BDR(uint8_t Channel, uint32_t BDR);
int8_t uart_Get_Receive(uint8_t Channel, uint8_t *Data, uint16_t *Len);
/*数据收发接口*/
int8_t uart_adapter_Transmit_Polling(uint8_t Channel, const uint8_t *Data, uint8_t Len);
int8_t uart_adapter_Transmit_IT(uint8_t Channel, const uint8_t *Data, uint8_t Len);
int8_t uart_adapter_Receive_Polling(uint8_t Channel, uint8_t *Data, uint8_t Len);
int8_t uart_adapter_Receive_IT(uint8_t Channel, uint8_t *Data, uint8_t Len);
/*串口控制器-中断回调函数*/
uint8_t uart_Idle_Callback(uint8_t Channel);
uint8_t uart_Rx_Callback(uint8_t Channel);
uint8_t uart_Err_Callback(uint8_t Channel);
uint8_t uart_TXE_Callback(uint8_t Channel);
uint8_t uart_TC_Callback(uint8_t Channel);
/*串口控制器-测试*/
void uart_Test(void);
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
