/********************************文件说明*************************************
*文件名: mc_spd.c

*作者: Yuchen Tan

*版本: V1.0.3

*功能简介:
*1.电机速度闭环控制;
*2.电机加减速斜坡控制-闭环/开环;

*备注:

*修改履历:
------------------------------------V1.0.1------------------------------------
20220614:
1.修复bug.在mc_spd_OpenLoop_Controller()的if(pSpdOL->TargetDCPercent == pSpdOL->
InitDCPercent)中加pSpdOL->DCRampOverFlag = TRUE,防止开环启动瞬间调用停止,因
mc_spd_Is_OpenLoop_Ramp_Over()永远返回FALSE,导致电机停不下来(eg: 刚好发生过热保护).
100%复现方式:(删掉代码pSpdOL->DCRampOverFlag = TRUE)
step1: 上电后先正常调用一次开环启动+停止(可让(pSpdOL->TargetDCPercent == pSpdOL->
InitDCPercent)条件满足;
step2: 将过热标志置位再次启动(模拟调用开环启动后,瞬间调用开环停止);
注: 同样的方式测闭环没有此问题!因为闭环停止时调用mc_spd_Ramp_Controller_Enable()
会将步骤设置为pHandle->RampCtrlState = E_CTRL_INIT;随后在mc_spd_Ramp_Controller()的
case E_CTRL_INIT中会在条件if(pHandle->TargetSpeed != pHandle->InitialSpeed)满足时执
行pHandle->RampCtrlState = E_SPEED_CHANGE_DONE;保证mc_spd_Ramp_Is_Ramp_Over()返回
TRUE并执行mc_app_Set_Single_Motor_Cmd(pMC->Motor, e_mac_stop, 0);
------------------------------------V1.0.2------------------------------------
20220712: 将宏定义SPD_LOOP_PID_MODE从mc_common.h移动到mc_spd.h中.
------------------------------------V1.0.3------------------------------------
20220922: 补充/修改开环速度控制器代码,实现电机开环运行实时修改目标占空比功能;
*****************************************************************************/

/*****************************文件包含(私有)**********************************
*
*备注: 无
*
*****************************************************************************/
#include "mc_spd.h"
#include "pid.h"
/*****************************宏定义(私有)************************************
*
*备注: 本文件中,不希望被外部使用或随意修改的宏在这里定义
*
*****************************************************************************/
/*马达速度斜坡控制器参数(单位:rps)*/
#define DEFAULT_MIN_SPEED       (0)     /*默认最小输出速率*/
#define DEFAULT_MAX_SPEED       (10000) /*默认最大输出速率*/
#define DEFAULT_ACCELERATION    (3000)  /*默认加速率*/
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
/*以下均为测试用变量*/
uint8_t     g_TestMCSpdCmd = 0;
/********************************函数定义************************************
*函数名:

*函数功能描述: 马达控制-电机速度控制-速度闭环控制

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*速度环控制器句柄初始化*/
void mc_spd_CloseLoop_hInit(SPD_CL_Handle_t *pSpdCL, PID_ZL_Handle_t *pPIDZL, PID_WZ_Handle_t *pPIDWZ, SPD_RAMP_Handle_t *pSpdRamp)
{
	memset(pSpdCL, 0, sizeof(SPD_CL_Handle_t));
    pSpdCL->MaxSpeed = DEFAULT_MAX_SPEED;
    pSpdCL->MinSpeed = DEFAULT_MIN_SPEED;

    if(pPIDZL != NULL)
    {
        pSpdCL->pSpdLoopPIDZL = pPIDZL;
        pid_PID_ZL_hInit(pPIDZL);
    }
    if(pPIDWZ != NULL)
    {
        pSpdCL->pSpdLoopPIDWZ = pPIDWZ;
        pid_PID_WZ_hInit(pPIDWZ);
    }
    if(pSpdRamp != NULL)
    {
        pSpdCL->pSpdRamp = pSpdRamp;
        mc_spd_Ramp_Controller_hInit(pSpdRamp);
    }
}
/*重置速度环控制器*/
void mc_spd_Reset_CloseLoop_Controller(SPD_CL_Handle_t *pSpdCL)
{
    pSpdCL->PeriodTimer = 0;

    pSpdCL->FdbkSpeed = 0;
    pSpdCL->TargetSpeed = 0;

    pSpdCL->Output = 0;
}
/*设置速度环控制器运行周期*/
void mc_spd_Set_CloseLoop_Period(SPD_CL_Handle_t *pSpdCL, uint16_t Period)
{
    pSpdCL->Period = Period;
}
/*使能速度环控制器*/
void mc_spd_CloseLoop_Controller_Enable(SPD_CL_Handle_t *pSpdCL)
{
    pSpdCL->SpdCLCtrlEn = TRUE;
}
/*禁止速度环控制器*/
void mc_spd_CloseLoop_Controller_Disable(SPD_CL_Handle_t *pSpdCL)
{
    pSpdCL->SpdCLCtrlEn = FALSE;
}
/*速度环控制器运行周期计数器(1ms调用一次)*/
void mc_spd_CloseLoop_Controller_Timer(SPD_CL_Handle_t *pSpdCL)
{
    if(pSpdCL->SpdCLCtrlEn == TRUE)
    {
        pSpdCL->PeriodTimer++;
    }else
    {
        pSpdCL->PeriodTimer = 0;
    }
}
/*设置速度环控制器最大输出速度*/
void mc_spd_Set_CloseLoop_MaxSpeed(SPD_CL_Handle_t* pSpdCL, MOTOR_SPD_t MaxSpeed)
{
    pSpdCL->MaxSpeed = MaxSpeed;
}
/*设置速度环控制器最小输出速度*/
void mc_spd_Set_CloseLoop_MinSpeed(SPD_CL_Handle_t* pSpdCL, MOTOR_SPD_t MinSpeed)
{
    pSpdCL->MinSpeed = MinSpeed;
}
/*设置速度环控制器目标速度*/
void mc_spd_Set_CloseLoop_TargetSpeed(SPD_CL_Handle_t *pSpdCL, MOTOR_SPD_t TargetSpeed)
{
	int16_t Sign = (TargetSpeed >= 0) ? 1 : (-1);
	MOTOR_SPD_t AbsTargetSpeed = abs(TargetSpeed);
	
    if(AbsTargetSpeed > pSpdCL->MaxSpeed)
    {
        pSpdCL->TargetSpeed = pSpdCL->MaxSpeed * Sign;
    }else if(AbsTargetSpeed < pSpdCL->MinSpeed)
    {
        pSpdCL->TargetSpeed = pSpdCL->MinSpeed * Sign;
    }else
    {
        pSpdCL->TargetSpeed = TargetSpeed;
    }
}
/*获取速度环控制器目标速度*/
MOTOR_SPD_t mc_spd_Get_CloseLoop_TargetSpeed(const SPD_CL_Handle_t *pSpdCL)
{
    return pSpdCL->TargetSpeed;
}
/*获取速度环控制器反馈速度*/
MOTOR_SPD_t mc_spd_Get_CloseLoop_RealSpeed(const SPD_CL_Handle_t *pSpdCL)
{
    return pSpdCL->FdbkSpeed;
}
/*速度环控制器*/
void mc_spd_CloseLoop_Controller(SPD_CL_Handle_t *pSpdCL, MOTOR_SPD_t FdbkSpeed)
{
    if(pSpdCL->SpdCLCtrlEn == TRUE)
    {
        if(pSpdCL->PeriodTimer >= pSpdCL->Period)
        {
            //Gpio_ToggleOutputIO(GPIO_TEST1_PORT, GPIO_TEST1_PIN);
            pSpdCL->PeriodTimer = 0;
            pSpdCL->FdbkSpeed = FdbkSpeed;
        #if (SPD_LOOP_PID_MODE == 1)    //位置式PID
            pSpdCL->Output = pid_PID_WZ_Controller(pSpdCL->pSpdLoopPIDWZ, (pSpdCL->TargetSpeed - FdbkSpeed));
        #else       //增量式PID
            pSpdCL->Output = pid_PID_ZL_Controller(pSpdCL->pSpdLoopPIDZL, (pSpdCL->TargetSpeed - FdbkSpeed));
        #endif
        }
    }
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 马达控制-电机速度控制-速度斜坡控制器

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*速度斜坡控制器-句柄初始化*/
void mc_spd_Ramp_Controller_hInit(SPD_RAMP_Handle_t* pHandle)
{
	memset(pHandle, 0, sizeof(SPD_RAMP_Handle_t));
    pHandle->RampCtrlState = E_SPEED_CHANGE_DONE;
    pHandle->Acceleration = DEFAULT_ACCELERATION;
}
/*速度斜坡控制器-控制器参数重设状态控制*/
static void mc_spd_Ramp_Controller_ParamChange(SPD_RAMP_Handle_t* pHandle)
{
    if(pHandle->RampCtrlEn == TRUE)
    {
        pHandle->RampCtrlState = E_CTRL_PARAM_CHANGE;
    }
}
/*设置加速度率*/
void mc_spd_Ramp_Set_Acceleration(SPD_RAMP_Handle_t* pHandle, MOTOR_SPD_t Acceleration)
{
    if(Acceleration != pHandle->Acceleration)
    {
        pHandle->Acceleration = Acceleration;
        mc_spd_Ramp_Controller_ParamChange(pHandle);
    }
}
/*设置最终目标速度*/
void mc_spd_Ramp_Set_TargetSpeed(SPD_RAMP_Handle_t* pHandle, MOTOR_SPD_t TargetSpeed)
{
    if(TargetSpeed != pHandle->TargetSpeed)
    {
        pHandle->TargetSpeed = TargetSpeed;
        mc_spd_Ramp_Controller_ParamChange(pHandle);
    }
}
/*设置起始速度*/
void mc_spd_Ramp_Set_InitialSpeed(SPD_RAMP_Handle_t* pHandle, MOTOR_SPD_t InitialSpeed)
{
    if(InitialSpeed != pHandle->InitialSpeed)
    {
        pHandle->InitialSpeed = InitialSpeed;
        mc_spd_Ramp_Controller_ParamChange(pHandle);
    }
    /*设置初始速度后,过程速度也要同步更新
    (Todo: 思考什么情况需要更新ProcessSpeed,且此变量最好在斜坡控制器内部修改)*/
    pHandle->ProcessSpeed = pHandle->InitialSpeed;
}
/*获取过程速度*/
MOTOR_SPD_t mc_spd_Ramp_Get_ProcessSpeed(const SPD_RAMP_Handle_t* pHandle)
{
    return pHandle->ProcessSpeed;
}
/*速度斜坡控制器-控制重置*/
void mc_spd_Reset_Ramp_Controller(SPD_RAMP_Handle_t* pHandle)
{
    pHandle->RampCtrlState = E_SPEED_CHANGE_DONE;
    pHandle->RampCtrlTimer = 0;
    pHandle->RampCtrlTimerEn = 0;

    pHandle->AccelerationTime = 0;
}
/*速度斜坡控制器-启动*/
void mc_spd_Ramp_Controller_Enable(SPD_RAMP_Handle_t* pHandle)
{
    pHandle->RampCtrlEn = TRUE;
    pHandle->RampCtrlState = E_CTRL_INIT;
}
/*速度斜坡控制器-关闭*/
void mc_spd_Ramp_Controller_Disable(SPD_RAMP_Handle_t* pHandle)
{
    pHandle->RampCtrlEn = FALSE;
}
/*速度斜坡控制器-速度变化是否完成*/
BOOL mc_spd_Ramp_Is_Ramp_Over(SPD_RAMP_Handle_t* pHandle)
{
    if(pHandle->RampCtrlState == E_SPEED_CHANGE_DONE)
    {
        return TRUE;
    }
    return FALSE;
}
/*速度斜坡控制器*/
void mc_spd_Ramp_Controller(SPD_RAMP_Handle_t* pHandle)
{
    MOTOR_SPD_t ProcessRPM = 0;
    MOTOR_SPD_t Acc = 0;
    uint8_t SpeedChangeDone = 0;
    MOTOR_SPD_t DeltaRPM = 0;

    if(pHandle->RampCtrlEn == FALSE)
    {
        return;
    }
    switch(pHandle->RampCtrlState)
    {
        case E_SPEED_CHANGE_DONE:

            break;

        case E_CTRL_INIT:
            if(pHandle->TargetSpeed != pHandle->InitialSpeed)
            {
                pHandle->RampCtrlTimer = 0;
                pHandle->RampCtrlTimerEn = 1;
                pHandle->RampCtrlState = E_CHANGING_SPEED;
                DeltaRPM = pHandle->TargetSpeed - pHandle->InitialSpeed;
				pHandle->AccelerationTime = abs(DeltaRPM * 1000 / pHandle->Acceleration);
            }else
            {
                pHandle->ProcessSpeed = pHandle->TargetSpeed;
                pHandle->RampCtrlState = E_SPEED_CHANGE_DONE;
                pHandle->RampCtrlTimer = 0;
                pHandle->RampCtrlTimerEn = 0;
            }
            break;

        case E_CHANGING_SPEED:  /*速度变化中*/
            if(pHandle->TargetSpeed >= pHandle->InitialSpeed)
            {   /*最终速度 > 初始速度, 加速度为正*/
                Acc = abs(pHandle->Acceleration);
            }else
            {   /*最终速度 < 初始速度, 加速度为负*/
                Acc = abs(pHandle->Acceleration) * (-1);
            }
        #if 1
            /*一次加速度曲线：v(t) = v0 + a*t */
            ProcessRPM = Acc * pHandle->RampCtrlTimer;
            ProcessRPM /= 1000;
            ProcessRPM += pHandle->InitialSpeed;
        #else
            /*二次加速度曲线：v(t) = v0 + (a/T) * t^2 (T是加速总时间)*/
            if(pHandle->TargetSpeed * pHandle->InitialSpeed >= 0)   /*目前只支持末速度与初速度同方向的处理*/
            {
                if(Cal_Abs(pHandle->TargetSpeed) >= Cal_Abs(pHandle->InitialSpeed))
                {   /*速率增加*/
                    ProcessRPM = pHandle->RampCtrlTimer * pHandle->RampCtrlTimer;
                    ProcessRPM /= pHandle->AccelerationTime;
                    ProcessRPM *= Acc;
                    ProcessRPM /= 1000;
                    ProcessRPM += pHandle->InitialSpeed;
                }else
                {   /*速率减小*/
                    if(pHandle->AccelerationTime >= pHandle->RampCtrlTimer)
                    {
                        ProcessRPM = pHandle->AccelerationTime - pHandle->RampCtrlTimer;
                        ProcessRPM *= ProcessRPM;
                        ProcessRPM /= pHandle->AccelerationTime;
                        ProcessRPM *= (Acc * (-1));
                        ProcessRPM /= 1000;
                        ProcessRPM += pHandle->TargetSpeed;
                    }else
                    {
                        ProcessRPM = pHandle->TargetSpeed;
                    }
                }
            }
        #endif
            /*速度按照曲线变化完成,结束*/
            if(Acc > 0)
            {
                /*加速正转 or 减速反转*/
                if(ProcessRPM >= pHandle->TargetSpeed)
                    SpeedChangeDone = 1;
            }else if(Acc < 0)
            {
                /*减速正转 or 加速反转*/
                if(ProcessRPM <= pHandle->TargetSpeed)
                    SpeedChangeDone = 1;
            }else
            {
                /*如果加速度为0,则SpeedChangeDone无法置位*/
            }
            if(SpeedChangeDone)
            {
                ProcessRPM = pHandle->TargetSpeed;
                pHandle->RampCtrlState = E_SPEED_CHANGE_DONE;
                pHandle->RampCtrlTimer = 0;
                pHandle->RampCtrlTimerEn = 0;
            }
            pHandle->ProcessSpeed = ProcessRPM;
            break;

        case E_CTRL_PARAM_CHANGE:
            pHandle->InitialSpeed = pHandle->ProcessSpeed;
            pHandle->RampCtrlTimer = 0;
            pHandle->RampCtrlState = E_CTRL_INIT;
            break;

        default:
            break;
    }
}
/*马达速度控制器-定时控制(1ms调用一次)*/
void mc_spd_Ramp_Controller_Timer(SPD_RAMP_Handle_t* pHandle)
{
    if(pHandle->RampCtrlTimerEn == 1)
    {
        if(pHandle->RampCtrlTimer < 60000)
        {
            pHandle->RampCtrlTimer++;
        }
    }else
    {
        pHandle->RampCtrlTimer = 0;
    }
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 马达控制-电机速度控制-速度开环斜坡控制器

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*速度开环控制器句柄初始化*/
void mc_spd_OpenLoop_hInit(SPD_OL_Handle_t *pSpdOL)
{
	memset(pSpdOL, 0, sizeof(SPD_OL_Handle_t));
}
/*重置速度开环控制器*/
void mc_spd_Reset_OpenLoop_Controller(SPD_OL_Handle_t *pSpdOL)
{
    pSpdOL->AccelerationTimer = 0;
    pSpdOL->DCRampOverFlag = FALSE;
}
/*使能速度开环控制器*/
void mc_spd_OpenLoop_Controller_Enable(SPD_OL_Handle_t *pSpdOL)
{
    pSpdOL->SpdOLCtrlEn = TRUE;
}
/*禁止速度开环控制器*/
void mc_spd_OpenLoop_Controller_Disable(SPD_OL_Handle_t *pSpdOL)
{
    pSpdOL->SpdOLCtrlEn = FALSE;
}
/*速度开环控制器计数器(1ms调用一次)*/
void mc_spd_OpenLoop_Controller_Timer(SPD_OL_Handle_t *pSpdOL)
{
    if(pSpdOL->SpdOLCtrlEn == TRUE)
    {
        pSpdOL->AccelerationTimer++;
    }else
    {
        pSpdOL->AccelerationTimer = 0;
    }
}
/*设置速度开环控制器占空比模值*/
void mc_spd_Set_OpenLoop_DutyCycleModValue(SPD_OL_Handle_t *pSpdOL, int16_t DCModValue)
{
    pSpdOL->DCModValue = DCModValue;
}
/*设置速度开环控制器最大输出占空比*/
void mc_spd_Set_OpenLoop_MaxDutyCycle(SPD_OL_Handle_t *pSpdOL, int16_t MaxDutyCycle)
{
    pSpdOL->MaxDutyCycle = MaxDutyCycle;
}
/*设置速度开环控制器最小输出占空比*/
void mc_spd_Set_OpenLoop_MinDutyCycle(SPD_OL_Handle_t *pSpdOL, int16_t MinDutyCycle)
{
    pSpdOL->MinDutyCycle = MinDutyCycle;
}
/*设置速度开环控制器目标占空比*/
void mc_spd_Set_OpenLoop_TargetDutyCycle(SPD_OL_Handle_t *pSpdOL, int16_t TargetDCPercent)
{
	int16_t Sign = (TargetDCPercent >= 0) ? 1 : (-1);
	int16_t AbsTargetDCPercent = abs(TargetDCPercent);
	
    if(AbsTargetDCPercent > pSpdOL->MaxDutyCycle)
    {
        TargetDCPercent = pSpdOL->MaxDutyCycle * Sign;
    }else if(AbsTargetDCPercent < pSpdOL->MinDutyCycle)
    {
        TargetDCPercent = pSpdOL->MinDutyCycle * Sign;
    }else
    {
    }
    if(pSpdOL->TargetDCPercent != TargetDCPercent)
    {
        pSpdOL->TargetDCPercent = TargetDCPercent;
        mc_spd_Reset_OpenLoop_Controller(pSpdOL);
    }
}
/*设置速度开环控制器初始占空比*/
void mc_spd_Set_OpenLoop_InitDutyCycle(SPD_OL_Handle_t *pSpdOL, int16_t InitDCPercent)
{
    if(pSpdOL->InitDCPercent != InitDCPercent)
    {
        pSpdOL->InitDCPercent = InitDCPercent;
        mc_spd_Reset_OpenLoop_Controller(pSpdOL);
    }
	pSpdOL->ProcessDCPercent = pSpdOL->InitDCPercent;/*设置初始值后,需同步更新当前值*/
}
/*设置速度开环控制器占空比加速度(百分比/s)*/
void mc_spd_Set_OpenLoop_Acceleration(SPD_OL_Handle_t *pSpdOL, int16_t Acceleration)
{
    if(pSpdOL->Acceleration != Acceleration)
    {
        pSpdOL->Acceleration = Acceleration;
        mc_spd_Reset_OpenLoop_Controller(pSpdOL);
    }
}
/*速度开环控制器-占空比斜坡是否完成*/
BOOL mc_spd_Is_OpenLoop_Ramp_Over(SPD_OL_Handle_t* pSpdOL)
{
    if(pSpdOL->SpdOLCtrlEn == TRUE)
    {
        return pSpdOL->DCRampOverFlag;
    }
    return FALSE;
}
/*速度开环控制器*/
void mc_spd_OpenLoop_Controller(SPD_OL_Handle_t *pSpdOL)
{
    uint16_t DC_AddP1_TimeMs = 0;	//根据当前加速度,计算占空比加1%的时间
    int16_t DC_Output;   //占空比值

    if(pSpdOL->SpdOLCtrlEn == TRUE)
    {
		DC_AddP1_TimeMs = 1000 / abs(pSpdOL->Acceleration);
		if(pSpdOL->AccelerationTimer >= DC_AddP1_TimeMs)
		{
			pSpdOL->AccelerationTimer = 0;
			if(pSpdOL->TargetDCPercent == pSpdOL->ProcessDCPercent)
			{
				pSpdOL->DCRampOverFlag = TRUE;
			}else
			{
				if(pSpdOL->ProcessDCPercent < pSpdOL->TargetDCPercent)
				{
					pSpdOL->ProcessDCPercent++; 
				}else
				{
					pSpdOL->ProcessDCPercent--; 
				}		
			}
		}
		DC_Output = pSpdOL->ProcessDCPercent * (pSpdOL->DCModValue / 100);
        pSpdOL->Output = DC_Output;
    }
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 马达控制-电机速度控制-模块功能测试

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
void mc_spd_Test(SPD_CL_Handle_t *pSpdCL, SPD_RAMP_Handle_t* pHandle)
{
    if(g_TestMCSpdCmd == 1)
    {
        g_TestMCSpdCmd = 0;
    }
}
