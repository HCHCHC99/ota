/********************************文件说明*************************************
*文件名: i2c_adapter.c

*功能: I2C适配器(介于应用功能和MCU的I2C底层驱动之间的中间层)

*主要内容：
[1]重新封装MCU的I2C驱动函数,并提供更简单易用的接口给上层应用；
[2]
[3]

*备注: 
*****************************************************************************/

/******************************文件包含(私有)*********************************
*
*备注: 无
*
*****************************************************************************/
#include "i2c_adapter.h"
#include "i2c.h"
#include "main.h"
#include "icm40608.h"
/*******************************宏定义(私有)**********************************
*
*备注:无
*
*****************************************************************************/
/*I2C控制中的定时检测(注:不要设置为1,防止定时器的边界效应导致实际时间缩短,功能误判)*/
#define WAIT_SLAVE_ACK		(2)		//等待从机应答信号超时(单位：1=250us)
#define WAIT_IIC_STOP		(2)		//等待IIC停止信号完成(单位：1=250us)
#define WAIT_SLAVE_SEND		(2)		//等待从机发送数据(单位：1=250us)(需根据波特率计算1MHz波特率对应10us)

/*I2C读写位定义*/
#define WR_BIT_WRITE		(0X00)
#define WR_BIT_READ			(0X01)

/*I2C测试功能开关-与ICM40608六轴传感器通讯*/
#define USE_I2C_TEST		(1)
/************************数据类型及结构定义(私有)*****************************
*
*备注:无
*
*****************************************************************************/

/*****************************函数声明(私有)**********************************
*
*备注:在.c文件中声明不开放给外部使用的(非接口)函数
*
*****************************************************************************/
/*主机发送模式状态控制*/
static void i2c_adapter_MasterWrite_StateMachine(h_I2C_CTRL *pI2CCtrl);
/*主机接收模式状态控制*/
static void i2c_adapter_MasterRead_StateMachine(h_I2C_CTRL *pI2CCtrl);
/*主机发送+接收模式状态控制*/
static void i2c_adapter_MasterWriteRead_StateMachine(h_I2C_CTRL *pI2CCtrl);
/********************************变量定义*************************************
*
*备注:下列有些变量未加static的原因是为了仿真方便看, 并不是故意定义成全局变量的
*
*****************************************************************************/
h_I2C_CTRL	hI2CCtrl;
M0P_I2C_TypeDef* I2C1_Reg = ((M0P_I2C_TypeDef *)0x40004400UL);
uint8_t		g_TestI2CCmd = 0;

#if (USE_I2C_TEST == 1)
#define ICM40608_I2C_ADDR	(0XD0>>1)
uint8_t	g_I2CWriteData[20];
uint8_t	g_I2CReadData[20];
IMU_SENSOR_DATA	g_OgData;	//六轴传感器原始数据
#endif
/********************************变量引用*************************************
*
*备注:外部数据可通过传参的方式传给本.c文件中定义的接口,降低与外部的耦合(尽量不用extern引用其他模块的全局变量)
*
*****************************************************************************/

/********************************函数引用*************************************
*
*备注:外部接口应通过文件包含方式给本.c文件使用,降低与外部的耦合(尽量不用extern直接引用其他模块的函数)
*
*****************************************************************************/

/********************************函数定义************************************
*函数名: 

*函数功能描述: I2C适配模块-控制器句柄初始化

*函数参数: 无

*函数返回值: 无

*备注: 
*****************************************************************************/
static void i2c_adapter_hInit(h_I2C_CTRL* pI2CCtrl, en_i2c_channel_t Channel)
{
	pI2CCtrl->Channel = Channel;
	pI2CCtrl->CurrentCmd = E_I2C_CMD_NULL;
	pI2CCtrl->State = E_I2C_GENERATE_START;
	pI2CCtrl->WaitTimerEn = FALSE;
	pI2CCtrl->WaitTimerCnt = 0;

	pI2CCtrl->SlaveAddr.Addr = DEFAULT_SLAVE_ADDR;
	pI2CCtrl->SlaveAddr.Gc = FALSE;
	pI2CCtrl->SendData = NULL;
	pI2CCtrl->ReceiveData = NULL;
	pI2CCtrl->ByteToSend = 0;
	pI2CCtrl->ByteToReceive = 0;
}
/********************************函数定义************************************
*函数名: 

*函数功能描述: I2C适配模块-接收应答超时控制

*函数参数: 无

*函数返回值: 无

*备注: 
*****************************************************************************/
static void i2c_adapter_ReTimeout_Timer_Start(h_I2C_CTRL* pI2CCtrl)
{
	pI2CCtrl->WaitTimerCnt = 0;	
	pI2CCtrl->WaitTimerEn = 1;
}

static void i2c_adapter_ReTimeout_Timer_Run(h_I2C_CTRL* pI2CCtrl)
{
	if(pI2CCtrl->WaitTimerEn == 1)
	{	
		pI2CCtrl->WaitTimerCnt++;
	}
}

static void i2c_adapter_ReTimeout_Timer_Stop(h_I2C_CTRL* pI2CCtrl)
{
	pI2CCtrl->WaitTimerEn = 0;
	pI2CCtrl->WaitTimerCnt = 0;
}
/********************************函数定义************************************
*函数名: 

*函数功能描述: I2C数据传输控制接口

*函数参数: 
* *pI2CCtrl：I2C控制器句柄
* SlaveAddr：I2C需要访问的从机地址(7位左移前的值)
* *pWData：发送数据指针
* *pRData：接收数据指针
* WDataLength：发送数据长度
* RDataLength：接收数据长度

*函数返回值: 无

*备注: 
*****************************************************************************/
void i2c_adapter_MasterWrite(h_I2C_CTRL* pI2CCtrl, uint8_t SlaveAddr, uint8_t *pWData, uint32_t WDataLength)
{
	if(pI2CCtrl->CurrentCmd == E_I2C_CMD_NULL)
	{
		pI2CCtrl->SlaveAddr.Addr = SlaveAddr;
		pI2CCtrl->SlaveAddr.Gc = FALSE;
		I2C_WriteSlaveAddr(pI2CCtrl->Channel, &pI2CCtrl->SlaveAddr);	//设置目标从机地址用于???		
		pI2CCtrl->CurrentCmd = E_I2C_CMD_MASTER_SEND;
		pI2CCtrl->State = E_I2C_GENERATE_START;
		pI2CCtrl->SendData = pWData;
		pI2CCtrl->ByteToSend = WDataLength;
		while(pI2CCtrl->CurrentCmd != E_I2C_CMD_NULL)
		{
			i2c_adapter_MasterWrite_StateMachine(pI2CCtrl);
		}
	}
}

void i2c_adapter_MasterRead(h_I2C_CTRL* pI2CCtrl, uint8_t SlaveAddr, uint8_t *pRData, uint32_t RDataLength)
{
	if(pI2CCtrl->CurrentCmd == E_I2C_CMD_NULL)
	{
		pI2CCtrl->SlaveAddr.Addr = SlaveAddr;
		pI2CCtrl->SlaveAddr.Gc = FALSE;
		I2C_WriteSlaveAddr(pI2CCtrl->Channel, &pI2CCtrl->SlaveAddr);	//设置目标从机地址用于???		
		pI2CCtrl->CurrentCmd = E_I2C_CMD_MASTER_REVEIVE;
		pI2CCtrl->State = E_I2C_GENERATE_START;
		pI2CCtrl->ReceiveData = pRData;
		pI2CCtrl->ByteToReceive = RDataLength;
		while(pI2CCtrl->CurrentCmd != E_I2C_CMD_NULL)
		{
			i2c_adapter_MasterRead_StateMachine(pI2CCtrl);
		}
	}
}

void i2c_adapter_MasterWriteRead(h_I2C_CTRL* pI2CCtrl, uint8_t SlaveAddr, uint8_t *pWData, uint32_t WDataLength, uint8_t *pRData, uint32_t RDataLength)
{
	if(pI2CCtrl->CurrentCmd == E_I2C_CMD_NULL)
	{
		pI2CCtrl->SlaveAddr.Addr = SlaveAddr;
		pI2CCtrl->SlaveAddr.Gc = FALSE;
		I2C_WriteSlaveAddr(pI2CCtrl->Channel, &pI2CCtrl->SlaveAddr);	//设置目标从机地址用于???		
		pI2CCtrl->CurrentCmd = E_I2C_CMD_MASTER_SEND_RECEIVE;
		pI2CCtrl->State = E_I2C_GENERATE_START;
		pI2CCtrl->SendData = pWData;
		pI2CCtrl->ByteToSend = WDataLength;
		pI2CCtrl->ReceiveData = pRData;
		pI2CCtrl->ByteToReceive = RDataLength;
		while(pI2CCtrl->CurrentCmd != E_I2C_CMD_NULL)
		{
			i2c_adapter_MasterWriteRead_StateMachine(pI2CCtrl);
		}
	}	
}
/********************************函数定义************************************
*函数名: 

*函数功能描述: I2C适配模块-I2C读写控制状态机

*函数参数: 无

*函数返回值: 无

*备注: 
* 1.I2C_ClearIrq()应理解为用于触发下一个动作,而不是用于清除状态 \
*   调用I2C_ClearIrq()就会执行下一个动作,因此必须放在数据装填后!!!.
* 2.产生启动信号不需要(也不能)使用I2C_ClearIrq()清零I2Cx_CR.si来触发产生起始\
	信号动作,而产生重复起始信号必须清零I2Cx_CR.si才能触发产生重复起始信号动作!!!.
* 3.获取mcu的I2C状态语句"HWState = I2C_GetState(Channel);"不要放在状态机函数\
	头部,防止在中断产生时,I2C状态寄存器值变化,但HWState保存的是变化前的值,从而\
	导致程序误判I2C状态寄存器值异常,跳入E_I2C_GENERATE_STOP状态,终止I2C操作.
	解决办法: 
	[1]可以定义uint8_t *HWState = &(I2C状态寄存器),这样每次执行*HWState都是新的值.
	[2]在中断标志置位判断满足时再调用HWState = I2C_GetState(Channel)更新HWState的值.
* 4.每当状态寄存器I2Cx_STAT的值变化(除变为0XF8外),中断标志I2Cx_CR.si都会置1.
*****************************************************************************/
static void i2c_adapter_MasterWrite_StateMachine(h_I2C_CTRL *pI2CCtrl)
{
	en_i2c_channel_t Channel = pI2CCtrl->Channel;	
	uint8_t 	HWState;	//MCU的IIC外设硬件状态,和状态机的pI2CCtrl->State是2个概念!
	uint8_t 	Data;
	
	/**/
	if(pI2CCtrl->CurrentCmd != E_I2C_CMD_MASTER_SEND)
	{
		return;
	}
	/*IIC控制器-主机发送功能*/
	switch(pI2CCtrl->State)
	{
		case E_I2C_GENERATE_START:			
		#if (1)		//置位I2Cx_CR.sta,当总线空闲时,主机发出START信号
			I2C_SetFunc(Channel, I2cStart_En);
		#endif
			pI2CCtrl->State = E_I2C_WAIT_SEND_START;
			break;
		
		case E_I2C_WAIT_SEND_START:
			if(I2C_GetIrq(Channel) == 1)
			{
				HWState = I2C_GetState(Channel);
				I2C_ClearFunc(Channel, I2cStart_En);
				if(HWState == 0X08)		/*0X08-(已发送起始信号)*/
				{
					pI2CCtrl->State = E_I2C_SEND_SLAVEADDR_W;
				}else
				{
					pI2CCtrl->State = E_I2C_GENERATE_STOP;
				}
			}
			break;
		
		case E_I2C_SEND_SLAVEADDR_W:
			Data = (pI2CCtrl->SlaveAddr.Addr << 1) | WR_BIT_WRITE;
			I2C_WriteByte(Channel, Data);
			I2C_ClearIrq(Channel);
			i2c_adapter_ReTimeout_Timer_Start(pI2CCtrl);
			pI2CCtrl->State = E_I2C_WAIT_SEND_SLAVEADDR_ACK;
			break;
		
		case E_I2C_WAIT_SEND_SLAVEADDR_ACK:
			if(I2C_GetIrq(Channel) == 1)
			{
				HWState = I2C_GetState(Channel);
				i2c_adapter_ReTimeout_Timer_Stop(pI2CCtrl);
				if(HWState == 0X18)		/*0X18-(从机应答SLA+W)*/
				{
					pI2CCtrl->State = E_I2C_SEND_DATA;
				}else 	/*0X20-(从机未应答SLA+W) || 0X38-(主机丢失仲裁)*/
				{
					pI2CCtrl->State = E_I2C_GENERATE_STOP;				
				}
			}else
			{
				/*等待从机ACK中...*/
				if(pI2CCtrl->WaitTimerCnt >= WAIT_SLAVE_ACK)
				{
					pI2CCtrl->State = E_I2C_GENERATE_STOP;
				}
			}
			break;
			
		case E_I2C_SEND_DATA:
			if(pI2CCtrl->ByteToSend > 0)
			{
				Data = *(pI2CCtrl->SendData);
				I2C_WriteByte(Channel, Data);
				I2C_ClearIrq(Channel);
				pI2CCtrl->ByteToSend--;
				pI2CCtrl->SendData++;
				i2c_adapter_ReTimeout_Timer_Start(pI2CCtrl);
				pI2CCtrl->State = E_I2C_WAIT_SEND_DATA_ACK;
			}else
			{
				pI2CCtrl->State = E_I2C_GENERATE_STOP;
			}
			break;
		
		case E_I2C_WAIT_SEND_DATA_ACK:
			if(I2C_GetIrq(Channel) == 1)
			{
				HWState = I2C_GetState(Channel);
				i2c_adapter_ReTimeout_Timer_Stop(pI2CCtrl);
				if(HWState == 0X28)		/*0X28-(从机应答DATA)*/
				{
					pI2CCtrl->State = E_I2C_SEND_DATA;
				}else 	/*0X30-(从机未应答DATA) || 0X38-(主机丢失仲裁)*/
				{
					pI2CCtrl->State = E_I2C_GENERATE_STOP;
				}
			}else
			{
				/*等待从机ACK中...*/
				if(pI2CCtrl->WaitTimerCnt >= WAIT_SLAVE_ACK)
				{
					pI2CCtrl->State = E_I2C_GENERATE_STOP;
				}
			}			
			break;
		
		case E_I2C_GENERATE_STOP:
			i2c_adapter_ReTimeout_Timer_Stop(pI2CCtrl);	//注：统一在此关闭因异常跳入E_I2C_GENERATE_STOP状态之前已开启的计时
			i2c_adapter_ReTimeout_Timer_Start(pI2CCtrl);		
		#if (1)		//先置位I2Cx_CR.sto再清零si位,主机发出STOP信号
			I2C_SetFunc(Channel, I2cStop_En);
			I2C_ClearIrq(Channel);
		#endif
			pI2CCtrl->State = E_I2C_WAIT_SEND_STOP;
			break;
		
		case E_I2C_WAIT_SEND_STOP:
			/*等待停止信号发送完成中...(注：停止信号发送完成不会触发Irq!,只能通过延时等待)*/
			if(pI2CCtrl->WaitTimerCnt >= WAIT_IIC_STOP)
			{
				i2c_adapter_ReTimeout_Timer_Stop(pI2CCtrl);
				I2C_ClearFunc(Channel, I2cStop_En);
				pI2CCtrl->CurrentCmd = E_I2C_CMD_NULL;				
			}
			break;
		
		default:
			break;
	}
}

/*主机接收模式状态控制*/
static void i2c_adapter_MasterRead_StateMachine(h_I2C_CTRL *pI2CCtrl)
{

}

/*主机发送+接收模式状态控制*/
static void i2c_adapter_MasterWriteRead_StateMachine(h_I2C_CTRL *pI2CCtrl)
{
	en_i2c_channel_t Channel = pI2CCtrl->Channel;	
	uint8_t 	HWState;	//MCU的IIC外设硬件状态,和状态机的pI2CCtrl->State是2个概念!
	uint8_t 	Data;
	
	/**/
	if(pI2CCtrl->CurrentCmd != E_I2C_CMD_MASTER_SEND_RECEIVE)
	{
		return;
	}
	/*IIC控制器-主机发送功能*/
	switch(pI2CCtrl->State)
	{
		case E_I2C_GENERATE_START:			
		#if (1)		//置位I2Cx_CR.sta,当总线空闲时,主机发出START信号
			I2C_SetFunc(Channel, I2cStart_En);
		#endif
			pI2CCtrl->State = E_I2C_WAIT_SEND_START;
			break;
		
		case E_I2C_GENERATE_RESTART:
		#if (1)		//置位I2Cx_CR.sta,当总线空闲时,主机发出START信号
			I2C_SetFunc(Channel, I2cStart_En);
			I2C_ClearIrq(Channel);		//不能省略,见函数说明描述备注1,2
		#endif
			pI2CCtrl->State = E_I2C_WAIT_SEND_START;			
			break;
		
		case E_I2C_WAIT_SEND_START:
			if(I2C_GetIrq(Channel) == 1)
			{
				HWState = I2C_GetState(Channel);
				I2C_ClearFunc(Channel, I2cStart_En);
				if(HWState == 0X08)		/*0X08-(已发送起始信号)*/
				{
					pI2CCtrl->State = E_I2C_SEND_SLAVEADDR_W;
				}else if(HWState == 0X10)	/*0X10-(已发送重复起始信号)*/
				{
					pI2CCtrl->State = E_I2C_SEND_SLAVEADDR_R;
				}else
				{
					pI2CCtrl->State = E_I2C_GENERATE_STOP;
				}
			}
			break;
		
		case E_I2C_SEND_SLAVEADDR_W:
			Data = (pI2CCtrl->SlaveAddr.Addr << 1) | WR_BIT_WRITE;
			I2C_WriteByte(Channel, Data);
			I2C_ClearIrq(Channel);
			i2c_adapter_ReTimeout_Timer_Start(pI2CCtrl);
			pI2CCtrl->State = E_I2C_WAIT_SEND_SLAVEADDR_ACK;
			break;

		case E_I2C_SEND_SLAVEADDR_R:
			Data = (pI2CCtrl->SlaveAddr.Addr << 1) | WR_BIT_READ;
			I2C_WriteByte(Channel, Data);
			I2C_ClearIrq(Channel);
			i2c_adapter_ReTimeout_Timer_Start(pI2CCtrl);
			pI2CCtrl->State = E_I2C_WAIT_SEND_SLAVEADDR_ACK;
			break;
		
		case E_I2C_WAIT_SEND_SLAVEADDR_ACK:
			if(I2C_GetIrq(Channel) == 1)
			{
				HWState = I2C_GetState(Channel);
				i2c_adapter_ReTimeout_Timer_Stop(pI2CCtrl);
				if(HWState == 0X18)		/*0X18-(从机应答SLA+W)*/
				{
					pI2CCtrl->State = E_I2C_SEND_DATA;
				}else if(HWState == 0X40)	/*0X40-(从机应答SLA+R)*/
				{
					pI2CCtrl->State = E_I2C_RECEIVE_DATA;				
				}else 	/*0X20-(从机未应答SLA+W) || 0X48-(从机未应答SLA+R) || 0X38-(主机丢失仲裁)*/
				{
					pI2CCtrl->State = E_I2C_GENERATE_STOP;				
				}
			}else
			{
				/*等待从机ACK中...*/
				if(pI2CCtrl->WaitTimerCnt >= WAIT_SLAVE_ACK)
				{
					pI2CCtrl->State = E_I2C_GENERATE_STOP;
				}
			}
			break;
			
		case E_I2C_SEND_DATA:
			if(pI2CCtrl->ByteToSend > 0)
			{
				Data = *(pI2CCtrl->SendData);
				I2C_WriteByte(Channel, Data);
				I2C_ClearIrq(Channel);
				pI2CCtrl->ByteToSend--;
				pI2CCtrl->SendData++;
				i2c_adapter_ReTimeout_Timer_Start(pI2CCtrl);
				pI2CCtrl->State = E_I2C_WAIT_SEND_DATA_ACK;
			}else
			{
				pI2CCtrl->State = E_I2C_GENERATE_RESTART;	//产生重复起始信号,用于数据读取
			}
			break;
			
		case E_I2C_RECEIVE_DATA:
			i2c_adapter_ReTimeout_Timer_Start(pI2CCtrl);
			if(pI2CCtrl->ByteToReceive > 1)	//读到倒数第2个数据时,关闭ACK功能
			{
				I2C_SetFunc(Channel, I2cAck_En);
				I2C_ClearIrq(Channel);		//清零si后,开始接收数据,若已使能主机ACK则主机产生应答
				pI2CCtrl->State = E_I2C_RECEIVING_DATA;
			}else
			{
				I2C_ClearFunc(Channel, I2cAck_En);
				I2C_ClearIrq(Channel);		//清零si后,开始接收数据,若已禁止主机NACK则主机产生非应答
				pI2CCtrl->State = E_I2C_RECEIVING_DATA;
			}
			break;

		case E_I2C_RECEIVING_DATA:
			if(I2C_GetIrq(Channel) == 1)
			{
				HWState = I2C_GetState(Channel);
				if(HWState == 0X50)		/*0X50-(主机已接收数据并返回ACK)*/
				{
					i2c_adapter_ReTimeout_Timer_Stop(pI2CCtrl);
					*(pI2CCtrl->ReceiveData) = I2C_ReadByte(Channel);
					pI2CCtrl->ByteToReceive--;				
					pI2CCtrl->ReceiveData++;
					pI2CCtrl->State = E_I2C_RECEIVE_DATA;
				}else if(HWState == 0X58)/*0X58-(主机已接收数据并返回NACK)*/
				{
					i2c_adapter_ReTimeout_Timer_Stop(pI2CCtrl);
					*(pI2CCtrl->ReceiveData) = I2C_ReadByte(Channel);
					pI2CCtrl->ByteToReceive = 0;
					pI2CCtrl->State = E_I2C_GENERATE_STOP;
				}else
				{
					/*等待从机发送数据中...*/
					if(pI2CCtrl->WaitTimerCnt >= WAIT_SLAVE_SEND)
					{
						pI2CCtrl->State = E_I2C_GENERATE_STOP;
					}			
				}
			}
			break;
			
		case E_I2C_WAIT_SEND_DATA_ACK:
			if(I2C_GetIrq(Channel) == 1)
			{
				HWState = I2C_GetState(Channel);				
				i2c_adapter_ReTimeout_Timer_Stop(pI2CCtrl);
				if(HWState == 0X28)		/*0X28-(从机应答DATA)*/
				{
					pI2CCtrl->State = E_I2C_SEND_DATA;
				}else 	/*0X30-(从机未应答DATA) || 0X38-(主机丢失仲裁)*/
				{
					pI2CCtrl->State = E_I2C_GENERATE_STOP;
				}
			}else
			{
				/*等待从机ACK中...*/
				if(pI2CCtrl->WaitTimerCnt >= WAIT_SLAVE_ACK)
				{
					pI2CCtrl->State = E_I2C_GENERATE_STOP;
				}
			}
			break;
		
		case E_I2C_GENERATE_STOP:
			i2c_adapter_ReTimeout_Timer_Stop(pI2CCtrl);	//注：统一在此关闭因异常跳入E_I2C_GENERATE_STOP状态之前已开启的计时
		#if (1)		//先置位I2Cx_CR.sto再清零si位,主机发出STOP信号
			I2C_SetFunc(Channel, I2cStop_En);
			I2C_ClearIrq(Channel);
		#endif
			i2c_adapter_ReTimeout_Timer_Start(pI2CCtrl);
			pI2CCtrl->State = E_I2C_WAIT_SEND_STOP;
			break;
		
		case E_I2C_WAIT_SEND_STOP:
			/*等待停止信号发送完成中...(注：停止信号发送完成不会触发Irq!,只能通过延时等待)*/
			if(pI2CCtrl->WaitTimerCnt >= WAIT_IIC_STOP)
			{
				i2c_adapter_ReTimeout_Timer_Stop(pI2CCtrl);
				I2C_ClearFunc(Channel, I2cStop_En);
				pI2CCtrl->CurrentCmd = E_I2C_CMD_NULL;				
			}
			break;
		
		default:
			break;
	}
}
/********************************函数定义************************************
*函数名: 

*函数功能描述: I2C适配模块-模块功能测试

*函数参数: 无

*函数返回值: 无

*备注: 
*****************************************************************************/
static void i2c_adapter_Test(void)
{
#if (USE_I2C_TEST == 1)
	if(g_TestI2CCmd == 1)
	{	/*icm40608六轴传感器初始化*/
		g_I2CWriteData[0] = PWR_MGMT0;
		g_I2CWriteData[1] = 0x0F;
		i2c_adapter_MasterWrite(&hI2CCtrl, ICM40608_I2C_ADDR, g_I2CWriteData, 2);
		
		g_I2CWriteData[0] = INTF_CONFIG1;
		g_I2CWriteData[1] = 0x90;
		i2c_adapter_MasterWrite(&hI2CCtrl, ICM40608_I2C_ADDR, g_I2CWriteData, 2);
		
		g_I2CWriteData[0] = GYRO_ACCEL_CONFIG0;
		g_I2CWriteData[1] = 0x77;
		i2c_adapter_MasterWrite(&hI2CCtrl, ICM40608_I2C_ADDR, g_I2CWriteData, 2);
		
		g_I2CWriteData[0] = SELF_TEST_CONFIG;
		g_I2CWriteData[1] = 0x00;
		i2c_adapter_MasterWrite(&hI2CCtrl, ICM40608_I2C_ADDR, g_I2CWriteData, 2);
		
		g_I2CWriteData[0] = GYRO_CONFIG0;
		g_I2CWriteData[1] = 0x07;
		i2c_adapter_MasterWrite(&hI2CCtrl, ICM40608_I2C_ADDR, g_I2CWriteData, 2);

		g_I2CWriteData[0] = ACCEL_CONFIG0;
		g_I2CWriteData[1] = 0x07;
		i2c_adapter_MasterWrite(&hI2CCtrl, ICM40608_I2C_ADDR, g_I2CWriteData, 2);
		
		g_TestI2CCmd = 0;
	}else if(g_TestI2CCmd == 2)
	{	/*icm40608六轴原始数据单次读取*/
		g_I2CWriteData[0] = TEMP_OUT_H;
		i2c_adapter_MasterWriteRead(&hI2CCtrl, ICM40608_I2C_ADDR, g_I2CWriteData, 1, g_I2CReadData, (GYRO_ZOUT_L - TEMP_OUT_H + 1));
		g_OgData.TEMP = (g_I2CReadData[0]<<8)|g_I2CReadData[1];
		g_OgData.Ax = (g_I2CReadData[2]<<8)|g_I2CReadData[3];
		g_OgData.Ay = (g_I2CReadData[4]<<8)|g_I2CReadData[5];
		g_OgData.Az = (g_I2CReadData[6]<<8)|g_I2CReadData[7];
		g_OgData.Gx = (g_I2CReadData[8]<<8)|g_I2CReadData[9];
		g_OgData.Gy = (g_I2CReadData[10]<<8)|g_I2CReadData[11];
		g_OgData.Gz = (g_I2CReadData[12]<<8)|g_I2CReadData[13];		
		g_TestI2CCmd = 0;
	}else if(g_TestI2CCmd == 3)
	{	/*icm40608六轴原始数据连续读取*/
		g_I2CWriteData[0] = TEMP_OUT_H;
		i2c_adapter_MasterWriteRead(&hI2CCtrl, ICM40608_I2C_ADDR, g_I2CWriteData, 1, g_I2CReadData, (GYRO_ZOUT_L - TEMP_OUT_H + 1));
		g_OgData.TEMP = (g_I2CReadData[0]<<8)|g_I2CReadData[1];
		g_OgData.Ax = (g_I2CReadData[2]<<8)|g_I2CReadData[3];
		g_OgData.Ay = (g_I2CReadData[4]<<8)|g_I2CReadData[5];
		g_OgData.Az = (g_I2CReadData[6]<<8)|g_I2CReadData[7];
		g_OgData.Gx = (g_I2CReadData[8]<<8)|g_I2CReadData[9];
		g_OgData.Gy = (g_I2CReadData[10]<<8)|g_I2CReadData[11];
		g_OgData.Gz = (g_I2CReadData[12]<<8)|g_I2CReadData[13];		
		//g_TestI2CCmd = 0;
	}else
	{
	
	}
#endif
}
/********************************函数定义************************************
*函数名: 

*函数功能描述: I2C适配模块-定时应用入口

*函数参数: 无

*函数返回值: 无

*备注: 250us定时调用1次
*****************************************************************************/
void i2c_adapter_Timer(void)
{	
	i2c_adapter_ReTimeout_Timer_Run(&hI2CCtrl);
}
/********************************函数定义************************************
*函数名: 

*函数功能描述: I2C适配模块-后台应用入口

*函数参数: 无

*函数返回值: 无

*备注: 
*****************************************************************************/
void i2c_adapter_Loop_Task(void)
{
	i2c_adapter_Test();
}
/********************************函数定义************************************
*函数名: 

*函数功能描述: I2C适配模块-初始化应用入口

*函数参数: 无

*函数返回值: 无

*备注: 
*****************************************************************************/
void i2c_adapter_Init(void)
{
	i2c_adapter_hInit(&hI2CCtrl, I2C1);
}
