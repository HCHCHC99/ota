/********************************文件说明*************************************
*文件名: modbus.h

*作者: Yuchen Tan

*版本: V1.0.0

*功能简介:

*备注: 无

*修改履历:

*****************************************************************************/
#ifndef MODBUS_H_
#define MODBUS_H_

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
/*MODBUS节点定义*/
/*节点类型*/
#define MB_NODE_HOST            (0)
#define MB_NODE_SLAVE           (1)
/*从节点地址*/
#define MB_NODE_SLAVE_ADDR      (0x01)

/*MODBUS报文数据及格式定义*/
/*1-地址*/
#define MB_ADDR_BD              (0x00)  //广播地址
#define MB_ADDR_SC_MAX          (247)   //单播节点地址最大值
/*2-功能码*/
#define MB_FUNC_READ_COIL           (0x01)  //读线圈
#define MB_FUNC_READ_DISCRETE       (0x02)  //读离散量输入
#define MB_FUNC_READ_HOLDING_REG    (0x03)  //读保持寄存器
#define MB_FUNC_READ_INPUT_REG      (0x04)  //读输入寄存器
#define MB_FUNC_WRITE_1_COIL        (0x05)  //写单个线圈
#define MB_FUNC_WRITE_1_REG         (0x06)  //写单个寄存器
#define MB_FUNC_WRITE_COILS         (0x0F)  //写多个线圈
#define MB_FUNC_WRITE_REGS          (0x10)  //写多个寄存器
#define MB_FUNC_READ_FILE           (0x14)  //读文件
#define MB_FUNC_WRITE_FILE          (0x15)  //写文件
#define MB_FUNC_ATTACH_ERR          (0x80)  //功能码错误附属位
/*3-字段长度(modbus报文最大长度==256byte)*/
#define MB_MSG_ADDR_SIZE            (1)
#define MB_MSG_CRC_SIZE             (2)
#define MB_MSG_SIZE_MAX             (256)
#define MB_MSG_SIZE_MIN             (3)
#define MB_MSG_DATA_SIZE_MAX        (253)   //MB_MSG_SIZE_MAX - MB_MSG_CRC_SIZE - MB_MSG_ADDR_SIZE
/*4-请求消息错误类型*/
#define MB_MSG_ERR_CRC              (1)
#define MB_MSG_ERR_ACCESS           (2)
#define MB_MSG_ERR_SET_VALUE        (3)
/*5-标准MODBUS消息中的字段位置/尺寸定义*/
//消息字段-公共
#define MB_ADU_OFFSET_ADDR              (0)     //地址偏移
#define MB_PDU_OFFSET_FUNC              (1)     //功能码/差错码
#define MB_PDU_OFFSET_ERR               (2)     //异常值
//消息字段-03功能码(读保持寄存器)
#define MB_PDU_OFFSET_FUNC03_REGADDR    (2)     //寄存器读取起始地址(请求)
#define MB_PDU_OFFSET_FUNC03_REGNB      (4)     //寄存器读取数量(请求)
#define MB_PDU_OFFSET_FUNC03_BYTENB     (2)     //应答字节数(应答)
#define MB_PDU_OFFSET_FUNC03_REGVALUE   (3)     //寄存器值(应答)
//消息字段-06功能码(写单个寄存器)
#define MB_PDU_OFFSET_FUNC06_REGADDR    (2)     //寄存器写入起始地址(请求/应答)
#define MB_PDU_OFFSET_FUNC06_REGVALUE   (4)     //写入寄存器值(请求/应答)
//消息字段-16功能码(写多个寄存器)
#define MB_PDU_OFFSET_FUNC16_REGADDR    (2)     //寄存器写入起始地址(请求/应答)
#define MB_PDU_OFFSET_FUNC16_REGNB      (4)     //寄存器写入数量(请求/应答)
#define MB_PDU_OFFSET_FUNC16_BYTENB     (6)     //寄存器写入数据字节数(请求)
#define MB_PDU_OFFSET_FUNC16_REGVALUE   (7)     //寄存器写入数据(请求)

/*MODBUS相关时间定义(ms)*/
#define MB_SP_PERIOD            (200)   //主机单播轮训周期
#define MB_SA_TIMEOUT           (1000)  //主机单播等待从机应答超时
#define MB_BP_PERIOD            (300)   //主机广播轮训周期
#define MB_BSTD_TIME            (500)   //主机广播等待从机转换延时(标准规定取所有从机中延时最长的)

/*MODBUS接口返回值*/
#define MB_RET_OK               (1)
#define MB_RET_ERR_PARAM        (-1)    //函数参数错误(形参值越界,空指针)
#define MB_RET_ERR_CRC          (-2)    //帧校验错误
#define MB_RET_ERR_REG          (-3)    //寄存器范围或值错误
#define MB_RET_ERR_FUNC         (-4)    //功能码错误

/*MODBUS通讯收发状态标志定义*/
#define MB_XF_SEND_CPLT         (1<<0)
#define MB_XF_RCV_CPLT          (1<<1)
#define MB_XF_MASK_ALL          (0xFF)
/**************************数据类型及结构定义(公开)***************************
*
 备注: 需要被外部使用的数据结构及类型在这里定义
*
*****************************************************************************/
/*MODBUS-数据发送回调函数定义*/
typedef int8_t (*pfDataSend_t)(uint8_t Interface, uint8_t* SendData, uint8_t Len);
typedef int8_t (*pfHandlerCB_t)(uint8_t* RcvData, uint8_t Len);

/*MODBUS-RTU寄存器键值对定义*/
typedef struct
{
    uint16_t    RegAddr;            //寄存器地址(键)
    uint16_t    RegValue;           //寄存器值
}MB_REG_KVP_t;

/*MODBUS-RTU数据包格式定义*/
typedef struct
{
    uint8_t     Msg[MB_MSG_SIZE_MAX];   //消息缓存
    uint8_t     MsgLen;                 //消息长度(从地址到CRC的字节数,该字段非标准MODBUS消息中的字段,仅用于方便程序处理)
}MB_RTU_MSG_t;

/*MODBUS节点状态类型定义*/
typedef enum
{
    MB_READY = 0,           /*就绪(主-可发送数据, 从-接收数据)*/

    MB_HOST_WAIT_TD,        /*主节点等待从节点转换延时*/
    MB_HOST_WAIT_ACK,       /*主节点等待从节点应答消息*/
    MB_HOST_MSG_HANDLE,     /*主节点处理从节点应答消息*/

    MB_SLAVE_CHECK_REQ,     /*从节点检查主节点请求消息*/
    MB_SLAVE_ACK,           /*从节点向主节点发送应答消息*/
}MB_STATE_t;

/*MODBUS节点控制器类型定义*/
typedef struct
{
    /*节点信息*/
    uint8_t             NodeType;   /*0-主节点, 1-从节点*/
    uint8_t             SlaveAddr;  /*节点地址(从)*/
    uint8_t             LastSlaveAddr;/*上一次节点地址*/
    /*状态控制*/
    MB_STATE_t          State;      /*消息传输状态*/
    uint8_t             TimerEn;    /*计数器使能*/
    uint16_t            Timer;      /*计数器*/
    uint8_t             XfFlag;     /*数据收发状态标志*/
    uint8_t             MsgErr;     /*消息错误指示*/
    /*当前收发消息缓存*/
    MB_RTU_MSG_t        MsgSend;    /*发送消息*/
    MB_RTU_MSG_t        MsgRcv;     /*接收消息*/
    /*寄存器键值对索引表*/
    uint16_t            RegNb;      //索引表中的寄存器个数
    MB_REG_KVP_t        *RegTable;  //索引表
    /*回调函数组定义*/
    pfDataSend_t        pfSlaveAck; //从机应答消息发送
    pfHandlerCB_t       pfCB03;     //读保持寄存器回调函数
    pfHandlerCB_t       pfCB06;     //写单个寄存器回调函数
    pfHandlerCB_t       pfCB16;     //写多个寄存器回调函数
}MB_CTRL_t;
/*****************************函数声明(公开)**********************************
*
*备注: 需要被外部使用的接口函数在这里声明
*
*****************************************************************************/
/*寄存器索引表操作*/
uint16_t modbus_Read_1Reg(MB_REG_KVP_t *RegTable, uint16_t RegAddr);
void modbus_Write_1Reg(MB_REG_KVP_t *RegTable, uint16_t RegAddr, uint16_t Value);
/*modbus协议栈-初始化*/
int8_t modbus_hInit(MB_CTRL_t* MBCtrl, uint8_t NodeType, uint8_t SlaveAddr, MB_REG_KVP_t *RegTable);
int8_t modbus_Reg_CallBack_Ack(MB_CTRL_t* MBCtrl, pfDataSend_t pf);
int8_t modbus_Reg_CallBack_FuncHandlerCB(MB_CTRL_t* MBCtrl, pfHandlerCB_t pf, uint8_t Func);
/*modbus协议栈-RTU格式数据打包/解包*/
int8_t modbus_rtu_Pack(MB_CTRL_t* MBCtrl, const uint8_t *UpkMsg, uint8_t UpkMsgLen);
int8_t modbus_rtu_Unpack(MB_CTRL_t* MBCtrl, uint8_t *PkMsg, uint8_t PkMsgLen);
/*modbus协议栈-功能处理*/
void modbus_Func_Handler(MB_CTRL_t* MBCtrl, uint8_t Interface);
/*modbus协议栈-节点数据收发控制*/
uint8_t modbus_Get_XfFlag(MB_CTRL_t* MBCtrl, uint8_t XfFlag);
void modbus_Set_XfFlag(MB_CTRL_t* MBCtrl, uint8_t XfFlag);
void modbus_Clr_XfFlag(MB_CTRL_t* MBCtrl, uint8_t XfFlag);
BOOL modbus_Send_Ready(MB_CTRL_t* MBCtrl);
void modbus_Timer(MB_CTRL_t* MBCtrl);
void modbus_Host_Controller(MB_CTRL_t* MBCtrl);
void modbus_Slave_Controller(MB_CTRL_t* MBCtrl);
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
