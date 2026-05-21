/********************************文件说明*************************************
*文件名: i2c_adapter.h

*功能: 

*主要内容：
[1]
[2]
[3]

*备注: 
*****************************************************************************/
#ifndef I2C_ADAPTER_H_
#define I2C_ADAPTER_H_

/*****************************文件包含(公开)**********************************
*
*备注:无
*
*****************************************************************************/
#include "base_types.h"
#include "i2c.h"
/*****************************宏定义(公开)************************************
*
*备注:无
*
*****************************************************************************/
#define DEFAULT_SLAVE_ADDR	(0x00)
/************************数据类型及结构定义(公开)*****************************
*
*备注:无
*
*****************************************************************************/
/*I2C控制器可执行命令(工作模式)定义*/
typedef enum
{
	E_I2C_CMD_NULL,					//无动作指令
	E_I2C_CMD_MASTER_SEND,			//主机发送
	E_I2C_CMD_MASTER_REVEIVE,		//主机接收
	E_I2C_CMD_MASTER_SEND_RECEIVE,	//主机发送+接收	
	E_I2C_CMD_SLAVE_SEND,			//从机发送(Todo)
	E_I2C_CMD_SLAVE_REVEIVE,		//从机接收(Todo)
}I2C_CMD;

/*I2C控制器状态定义*/
typedef enum
{
	E_I2C_GENERATE_START,			//主机产生启动信号
	E_I2C_GENERATE_RESTART,			//主机产生重复启动信号
	E_I2C_WAIT_SEND_START,			//等待主机完成启动信号发送
	E_I2C_SEND_SLAVEADDR_W,			//主机发送首个字节(7位从机地址+读写位(写))
	E_I2C_SEND_SLAVEADDR_R,			//主机发送首个字节(7位从机地址+读写位(读))	
	E_I2C_WAIT_SEND_SLAVEADDR_ACK,	//等待从机对主机发送首个字节的应答
	E_I2C_SEND_DATA,				//主机发送数据
	E_I2C_RECEIVE_DATA,				//主机接收数据(使能)
	E_I2C_RECEIVING_DATA,			//主机接收数据(接收中)
	E_I2C_WAIT_SEND_DATA_ACK,		//等待从机对主机发送数据的应答
	E_I2C_GENERATE_STOP,			//主机产生停止信号
	E_I2C_WAIT_SEND_STOP,			//等待主机完成停止信号发送	
}I2C_STATE;

/*I2C控制器句柄定义*/
typedef struct
{
	/*I2C控制器句柄对应的MCU-I2C通道名*/		
	en_i2c_channel_t	Channel;
	/*I2C控制器当前执行的动作*/	
	I2C_CMD				CurrentCmd;
	/*I2C控制器工作状态*/
	I2C_STATE			State;
	/*I2C控制器等待计时(用于主机等待从机应答,等待停止信号)*/
	boolean_t			WaitTimerEn;
	uint16_t			WaitTimerCnt;
	/*I2C控制器从机地址*/
	stc_i2c_addr_t		SlaveAddr;
	/*I2C控制器数据缓冲区*/
	uint8_t				*SendData;
	uint8_t				*ReceiveData;
	/*I2C控制器数据缓冲区*/
	uint8_t				ByteToSend;
	uint8_t				ByteToReceive;
}h_I2C_CTRL;
/*****************************函数声明(公开)**********************************
*
*备注:在.h中声明提供给外部文件调用的接口函数
*
*****************************************************************************/
/*I2C适配模块-定时应用入口*/
void i2c_adapter_Timer(void);
/*I2C适配模块-后台应用入口*/
void i2c_adapter_Loop_Task(void);
/*I2C适配模块-初始化应用入口*/
void i2c_adapter_Init(void);
/*****************************变量声明(全局)**********************************
*
*备注:在.h中将本.c中需要给其他模块使用的全局变量用extern声明(尽量不用,会带来耦合)
*
*****************************************************************************/


#endif
