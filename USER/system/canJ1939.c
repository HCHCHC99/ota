/********************************文件说明*************************************
*文件名: canj1939.c

*作者: JingWen Zhou

*版本: V1.0.0

*功能简介: CANJ1939协议
 
*备注: 无

*修改履历: 

*****************************************************************************/
#include "canj1939.h"
#include "can_adapter.h"
#include "system.h"

CAN_J1939_CTRL j1939_ctrl;

/*解析收到的CANJ1939报文*/
int8_t cj_RcvHandle(void)
{
	CAN_RCV_FRAME_t RcvMsg = {0};
	CAN_TSMT_FRAME_t SendMsg = {0};
	CAN_J1939_CTRL *ctrl = &j1939_ctrl; 
	uint32_t ext_id;
	uint16_t ovcthhmA;
	int32_t MotorSpeed;
	int32_t	SpdStartAcc;
	int32_t	SpdStopAcc;
	int32_t GotoHallPos;
	float GotoPosMm;
	float start_s;
	float stop_s;
	int8_t Ret = ERR_OK;
	
	if (can_Get_Receive(CAN1, &RcvMsg, 1) == CAN_RET_OK)	//接收到CAN报文
	{
		RcvMsg.StdID = RcvMsg.ExtID & EXT_ID_MASK;
		memcpy(&ext_id, &RcvMsg.ExtID, sizeof(ext_id));
		if (RCV_CANID == ext_id)
		{
			ctrl->para[PARA_CTRL_CMD] = RcvMsg.Data[0];
//			ctrl->para[PARA_CUR_SET] = RcvMsg.Data[2];
//			ctrl->para[PARA_SPD_SET] = RcvMsg.Data[3];
//			ctrl->para[PARA_SOFTSTART] = RcvMsg.Data[4];
//			ctrl->para[PARA_SOFTSTOP] = RcvMsg.Data[5];
			/*收到报文，清除计时*/
			ctrl->cansignalcnt = 0;
//			/*设置过流值*/
//			if (ctrl->para[PARA_CUR_SET] <= CUR_SET_MAX)
//			{
//				ovcthhmA = (ctrl->para[PARA_CUR_SET] / 4) * 1000;
//				mc_app_Write_Param(MOTOR1, e_map_ocp_thh, ovcthhmA);
//			}else if (CUR_SET_DEFAUIT_CMD == ctrl->para[PARA_CUR_SET])
//			{
//				mc_app_Write_Param(MOTOR1, e_map_ocp_thh, CUR_SET_DEFAUIT_VALUE);
//			}else
//			{
//				Ret = ERR_CUR;
//			}
//			
//			/*设置速度--百分比*/
//			if (ctrl->para[PARA_SPD_SET] <= SPD_SET_99_5)
//			{
//				MotorSpeed = PRM_MAX * (ctrl->para[PARA_SPD_SET] * 0.005);
//				system_set_MotorSpeed(MotorSpeed);
//			}
//			else if (ctrl->para[PARA_SPD_SET] >= SPD_SET_100_MIN && ctrl->para[PARA_SPD_SET] <= SPD_SET_100_MAX)
//			{
//				MotorSpeed = PRM_MAX;
//				system_set_MotorSpeed(MotorSpeed);
//			}
//			else if (ctrl->para[PARA_SPD_SET] == SPD_SET_DEFAUIT)
//			{
//				MotorSpeed = PRM_MAX * SPD_SET_DEFAUIT_PCT;
//				system_set_MotorSpeed(MotorSpeed);
//			}else
//			{
//				Ret = ERR_SPD;
//			}
//			
//			/*设置启动时间 加速度a = (V-V0)/△t*/
//			if (ctrl->para[PARA_SOFTSTART] <= PARA_SOFT_MAX)
//			{
//				start_s = ((float)ctrl->para[PARA_SOFTSTART] / 1000 *50);
//				/*限制斜坡时间大于300ms，以免损坏结构*/
//				if (start_s < 0.3)
//				{
//					start_s = 0.3;
//				}			
//				SpdStartAcc = (MotorSpeed - M_START_RPM_UP) / start_s;	
//				mc_app_Write_Param(MOTOR1, e_map_start_acc, SpdStartAcc);
//			}
//			else if (ctrl->para[PARA_SOFTSTART] == PARA_SOFT_DEFAUIT_1 || ctrl->para[PARA_SOFTSTART] == PARA_SOFT_DEFAUIT_2)
//			{
//				start_s = 1.5;//单位s
//				SpdStartAcc = (MotorSpeed - M_START_RPM_UP) / start_s;
//				mc_app_Write_Param(MOTOR1, e_map_start_acc, SpdStartAcc);
//			}else
//			{
//				Ret = ERR_START;
//			}
//			/*设置停止时间*/	
//			if (ctrl->para[PARA_SOFTSTOP] <= PARA_SOFT_MAX)
//			{	
//				stop_s = ((float)ctrl->para[PARA_SOFTSTOP] / 1000 * 50);
//				/*限制斜坡时间大于300ms，以免损坏结构*/
//				if (stop_s < 0.3)
//				{
//					stop_s = 0.3;
//				}
//				
//				SpdStopAcc = (M_SLOWSTOP_RPM - MotorSpeed) / stop_s;			
//				mc_app_Write_Param(MOTOR1, e_map_stop_acc, SpdStopAcc);
//			}
//			else if (ctrl->para[PARA_SOFTSTOP] == PARA_SOFT_DEFAUIT_1 || ctrl->para[PARA_SOFTSTOP] == PARA_SOFT_DEFAUIT_2)
//			{
//				stop_s = 1.5;//单位s
//				SpdStopAcc = (M_SLOWSTOP_RPM - MotorSpeed) / stop_s;			
//				mc_app_Write_Param(MOTOR1, e_map_stop_acc, SpdStopAcc);
//			}else
//			{
//				Ret = ERR_STOP;
//			}
			
			
			/*控制命令*/
//			if(ERR_OK == Ret)
//			{
				switch(ctrl->para[PARA_CTRL_CMD])
				{
//					case CTRL_CMD_CLR:
//					{
//						system_set_CanCmd(e_can_clrfault);

//					}
//					break;
					
					case CTRL_CMD_OUT:
					{					
						
						system_set_CanCmd(e_can_up);
					}
					break;
					
					case CTRL_CMD_IN:
					{
						system_set_CanCmd(e_can_dn);					
					}
					break;
					
//					case CTRL_CMD_STOP:
//					{
//						/*第一次上电写入停止离开初始化状态，进入故障状态*/
////						system_set_PowerState(TRUE);
//						/*停止*/
//						system_set_CanCmd(e_can_stop);					
//					}
					break;
					
					default:
					{
//						if (ctrl->para[PARA_CTRL_CMD] <= CTRL_GOTO_MAX)
//						{
//							system_set_CanCmd(e_can_goto);
//							GotoPosMm = (float)ctrl->para[PARA_CTRL_CMD] /10;
//							/*设置目标位置*/
//							/*限制行程*/
//							GotoHallPos = system_Column_Pos_2_HallData( GotoPosMm );
//							system_set_GotoHallPos(GotoHallPos);
//						}else
//						{
//							Ret = ERR_CMD;
//						}
					}
					break;
				}
//			}
//			else
//			{
//			
//			}
		}
	}	
}

void cj_upd_timer(void)
{
	j1939_ctrl.updcnt ++;
	j1939_ctrl.cansignalcnt ++;
}

//CANJ1939自动发送：ID：左：18FF0084h 右：18FF0085h
void cj_initiate_send(void )
{
	CAN_TSMT_FRAME_t SendMsg = {0};
	CAN_J1939_CTRL *ctrl = &j1939_ctrl; 
	
//	ext_id.priority = 0x06;
//	ext_id.pdu_format = SEND_ADD;
//	ext_id.pdu_specific = HOST_ADDR;
//	ext_id.source_address = DEVICE_ADDR_L;
	 
	SendMsg.ExtID = SEND_CANID;
	
	SendMsg.Data[0] = ctrl->para[PARA_STAT_FLAG];
	SendMsg.Data[1] = ctrl->para[PARA_ERR_CODE];	
	SendMsg.Data[2] = 0x00;
	SendMsg.Data[3] = 0x00;
	SendMsg.Data[4] = 0x00;
	SendMsg.Data[5] = 0x00;
	SendMsg.Data[6] = 0x00;/*Byte7:bit4-7为保留位，一直写F*/
	SendMsg.Data[7] = 0x00;
	
	if (ctrl->updcnt >= 100)
	{
		ctrl->updcnt = 0;
		can_adapter_LoadExtFrame(&SendMsg, SendMsg.ExtID, SendMsg.Data, 8);
		can_adapter_Transmit_Polling(CAN1, &SendMsg, 8);
	}
	
}


/*更新J1939参数字典*/
void update_para(void)
{
	CAN_J1939_CTRL *ctrl = &j1939_ctrl; 
//	int32_t motor_cur;
//	int32_t motor_rpm;
//	uint16_t motor_spd;

	/*位置更新*/
//	if (FALSE == system_get_ZeroFound())
//	{
//		ctrl->para[PARA_POS] = POS_LOST;
//	}
//	else if ((system_get_FaultFlag() & FAULT_POS) == 0)
//	{		
//		ctrl->para[PARA_POS] = system_get_columnPosMM() * 10;//单位0.1mm/bit
//	}
	
	/*电流更新*/
//	if ((system_get_FaultFlag() & FAULT_M1_OVC) == 0)
//	{
//		mc_app_Read_Param(MOTOR1, e_map_current, &motor_cur);
//		ctrl->para[PARA_CUR] = (motor_cur * 4)/1000;		//motor_cur单位为mA,0.25A/bit
//	}
//	else
//	{
//		ctrl->para[PARA_CUR] = CUR_FAULT;
//	}

//	if (system_get_RunDir() == DIR_DOWN)
//	{
//		ctrl->para[PARA_STAT_FLAG] |= STAT_FLAG_RUNNING_IN;
//	}else
//	{
//		ctrl->para[PARA_STAT_FLAG] &= ~STAT_FLAG_RUNNING_IN;
//	}
	
//	if (system_get_RunDir() == DIR_UP)
//	{
//		ctrl->para[PARA_STAT_FLAG] |= STAT_FLAG_RUNNING_OUT;
//	}else
//	{
//		ctrl->para[PARA_STAT_FLAG] &= ~STAT_FLAG_RUNNING_OUT;
//	}	
	
//	if(system_get_FaultFlag() & FAULT_M1_OVC)
//	{
//		ctrl->para[PARA_STAT_FLAG] |= STAT_FLAG_OVER_CUR;		
//	}else
//	{
//		ctrl->para[PARA_STAT_FLAG] &= ~STAT_FLAG_OVER_CUR;
//	}
	
	/*运行过程中检测CAN信号*/	
//	if (system_get_RunDir() == DIR_STOP)
//	{
//		ctrl->cansignalcnt = 0;
//	}else
//	{
//		if (ctrl->cansignalcnt >= CAN_TIMEOUT_VALUE)
//		{
//			system_set_FaultFlag(FAULT_CAN_RCV);
//		}
//	}

	/*状态标志*/
	if ((system_get_LimitFlag() & M1_BTM))
	{
		ctrl->para[PARA_STAT_FLAG] |= STAT_FLAG_ESS_IN;
	}else
	{
		ctrl->para[PARA_STAT_FLAG] &= ~STAT_FLAG_ESS_IN;//清除状态
	}
	
	if ((system_get_LimitFlag() & M1_TOP))
	{
		ctrl->para[PARA_STAT_FLAG] |= STAT_FLAG_ESS_OUT;
	}else
	{
		ctrl->para[PARA_STAT_FLAG] &= ~STAT_FLAG_ESS_OUT;
	}
	if(system_get_LockedFlag() & M_LOCKED_FLAG)
	{
		ctrl->para[PARA_STAT_FLAG] |= STAT_FLAG_IN_OVER;
	}else
	{
		ctrl->para[PARA_STAT_FLAG] &= ~STAT_FLAG_IN_OVER;
	}

	/*故障码*/	
	if((system_get_FaultFlag() & FAULT_M1_OVC) && (system_get_BrakeState() != 1)) //0
	{
		ctrl->para[PARA_ERR_CODE] |= OVC_ERR;		
	}
	if (system_get_FaultFlag() & FAULT_M1_HAB)//1
	{
		ctrl->para[PARA_ERR_CODE] |= HALL_ERR;
	}
	if (system_get_FaultFlag() & FAULT_OVT)//2
	{
		ctrl->para[PARA_ERR_CODE] |= TEM_ERR;
	}
	if (system_get_FaultFlag() & FAULT_OVV)//3
	{
		ctrl->para[PARA_ERR_CODE] |= OVER_VOL;
	}
	if (system_get_FaultFlag() & FAULT_UDV)//4
	{
		ctrl->para[PARA_ERR_CODE] |= UNDER_VOL;
	}
	
	if(system_get_FaultFlag() == 0)
	{
		ctrl->para[PARA_ERR_CODE] = NO_ERR;
	}


	/*刚上电：Power on Block State*/
	/*Failed to Keep CAN signal alive:运行状态下3S未收到报文*/
	/*速度*/
//	mc_app_Read_Param(MOTOR1, e_map_fdbkspd, &motor_rpm);
//	motor_spd = system_MotorRPM_2_ColumnSpeed(motor_rpm) * 10;//0.1mm/s/bit
//	ctrl->para[PARA_SPD] = motor_spd;
//	
//	/*输入电压状态--保留*/
//	ctrl->para[PARA_INPUT] = 0xFF;
	
}

