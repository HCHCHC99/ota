/********************************文件说明*************************************
*文件名: mc_hall.c

*作者: Yuchen Tan

*版本: V2.0.0

*功能简介: 电机HALL信号相关功能组件
*1.hall脉冲计数;
*2.hall异常检测;
*3.hall测速-M法/T法;

*备注:

*修改履历:
*20220419: HALL_Handle_t的HALL信号改为数组形式,数组元素数对应宏定义HALL_NB.
*20220530: GPIO操作适配已有MCU.
------------------------------------V1.0.3------------------------------------
*20220611: 增加宏定义HALL_NB_MAX用于定义电机hall传感器数组的元素总数,电机实
际使用的hall传感器数用宏定义HALL_NB表示.
------------------------------------V1.0.4------------------------------------
*20220709:
1.HALL计步器增加单HALL功能,调整结构体HALL_PEDOMETER_Handle_t定义,适配1-3个HALL传感器.
2.HALL异常检测器增加异常HALL电平的监测功能.
*20220718:
1.修改适配不同MCU的预编译命令书写错误.
2.HALL异常检测接口改为250us调用一次(无刷电机HALL异常状态检测须在ms级完成,否则导致
  不换相等效于MOS短路)吧,并基于250us调整HALL状态异常检测阈值;
3.优化hall停止,hall反向检测的判断阈值;
*20220921: GPIO代码改用gpio_adapter.h提供的类型定义及接口;
*20221013:
1.删除开启/关闭HALL脉冲计步器的接口;
2.重写Hall电平状态更新接口mc_hall_Get_HallState();
  重写HallData更新接口mc_hall_Update_HallData();
------------------------------------V1.0.5------------------------------------
20230220: 配合mc_config.h文件的V1.0.2修改,详见mc_config.h修改履历;
------------------------------------V2.0.0------------------------------------
20230406: 适配电机动态自锁功能,优化M法\T法测速器代码;
------------------------------------V2.0.1------------------------------------
20230911: 增加适配hall方向可设置的代码;
*****************************************************************************/

/*****************************文件包含(私有)**********************************
*
*备注: 无
*
*****************************************************************************/
#include "mc_hall.h"
/*****************************宏定义(私有)************************************
*
*备注: 本文件中,不希望被外部使用或随意修改的宏在这里定义
*
*****************************************************************************/
/*计算HALL脉宽的定时器溢出周期*/
#define TIMER_OVF_PERIOD    (60000)     //单位:us

/*HALL初始值*/
#define DEFAULT_HALLDATA    (0)

/*HALL脉宽检测前屏蔽个数*/
#define HALL_MEASUER_PRESHIELD      (1)     //屏蔽前N个捕捉到的HALL信号

/*HALL异常检测相关时间*/
#define HALL_CHECK_ABN_PRESHIELD_TIME       (500)   //单位:ms
#define HALL_CHECK_ABN_PERIOD               (100)   //单位:ms
#define HALL_CHECK_ABN_ILLEGAL_STATE_TIME   (2)     //单位:ms
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
static void mc_hall_Pedometer_hInit(HALL_PEDOMETER_Handle_t *pHP, uint8_t HallNb);
static void mc_hall_SpdMeasure_M_hInit(HALL_SPDMEAS_M_Handle_t *pHSMM);
static void mc_hall_SpdMeasure_T_hInit(HALL_SPDMEAS_T_Handle_t *pHSMT);
static void mc_hall_AbnChecker_hInit(HALL_ABN_CHECKER_Handle_t *pHAC, uint8_t HallNb);
/********************************变量定义*************************************
*
*备注: 不要在.h中定义变量,防止多个.c包含这个.h,导致编译后报错变量重复定义
*
*****************************************************************************/
/*HALL状态表(单HALL反馈. 正转/反转：0,1,0*/
const uint8_t g_HallStateTbl_1[2] = {0, 1};
/*HALL状态表(双HALL反馈/90°电角度相位差. 正转：0,1,3,2,0;  反转：0,2,3,1,0)*/
const uint8_t g_HallStateTbl_2[4] = {0, 1, 3, 2};
/*HALL状态表(三HALL反馈/120°电角度相位差. 正转：4,5,1,3,2,6;  反转：4,6,2,3,1,5)*/
const uint8_t g_HallStateTbl_3[6] = {4, 6, 2, 3, 1, 5};

/*用于HALL速度测量(T法)的定时器大周期计数器*/
uint16_t    g_HallSpdMeasureTimer = 0;

/*以下均为测试用变量*/
uint8_t     g_TestMCHallCmd = 0;
uint16_t    g_TestTimerCounter = 0;
/********************************函数定义************************************
*函数名:

*函数功能描述: 马达控制-电机HALL-句柄初始化

*函数参数: 无

*函数返回值: 无

*备注:
*1.mc_hall_hInit()中pHall->Hall[HALL_X] = Hall[N]采用逆序赋值的目的:
 使pHall->pHP->State的值与mc_hall_hInit()中定义的HALL序列排布方向一致!
 (eg: 定义HALL序列是ABC,当A==1,B==1,C==0时,pHP->State的值是0B0110而不是0B0011)
*****************************************************************************/
void mc_hall_hInit(HALL_Handle_t *pHall,
                    HALL_t *Hall,
                    uint8_t HallNb,
                    uint8_t PoleNb,
                    uint8_t SequenceSel,
                    HALL_PEDOMETER_Handle_t *pHP,
                    HALL_SPDMEAS_M_Handle_t *pHSMM,
                    HALL_SPDMEAS_T_Handle_t *pHSMT,
                    HALL_ABN_CHECKER_Handle_t *pHAC)
{
    if(!pHall)
        return;
    /*句柄初始化*/
    pHall->PoleNb = PoleNb;

    if(Hall != NULL)
    {
        if(HallNb < 1 || HallNb > 3)
        {
            return;
        }
        pHall->HallNb = HallNb;
        if(HallNb == 1)
        {
            pHall->SequenceSel = SequenceSel % 1;//单HALL有1种HALL排序(A)
            pHall->Hall[HALL_A] = Hall[0];
        }else if(HallNb == 2)
        {
            pHall->SequenceSel = SequenceSel % 2;//双HALL有2种HALL排序(AB/BA)
            if(pHall->SequenceSel == 0)
            {   //AB
                pHall->Hall[HALL_A] = Hall[1];
                pHall->Hall[HALL_B] = Hall[0];
            }else   //(pHall->SequenceSel == 1)
            {   //BA
                pHall->Hall[HALL_B] = Hall[1];
                pHall->Hall[HALL_A] = Hall[0];
            }
        }else //(HallNb == 3)
        {
            pHall->SequenceSel = SequenceSel % 6;//三HALL有6种HALL排序(ABC/BCA/CAB/ACB/CBA/BAC)
            if(pHall->SequenceSel == 0)
            {   //ABC
                pHall->Hall[HALL_A] = Hall[2];
                pHall->Hall[HALL_B] = Hall[1];
                pHall->Hall[HALL_C] = Hall[0];
            }else if(pHall->SequenceSel == 1)
            {   //BCA
                pHall->Hall[HALL_B] = Hall[2];
                pHall->Hall[HALL_C] = Hall[1];
                pHall->Hall[HALL_A] = Hall[0];
            }else if(pHall->SequenceSel == 2)
            {   //CAB
                pHall->Hall[HALL_C] = Hall[2];
                pHall->Hall[HALL_A] = Hall[1];
                pHall->Hall[HALL_B] = Hall[0];
            }else if(pHall->SequenceSel == 3)
            {   //ACB
                pHall->Hall[HALL_A] = Hall[2];
                pHall->Hall[HALL_C] = Hall[1];
                pHall->Hall[HALL_B] = Hall[0];
            }else if(pHall->SequenceSel == 4)
            {   //CBA
                pHall->Hall[HALL_C] = Hall[2];
                pHall->Hall[HALL_B] = Hall[1];
                pHall->Hall[HALL_A] = Hall[0];
            }else //(pHall->SequenceSel == 5)
            {   //BAC
                pHall->Hall[HALL_B] = Hall[2];
                pHall->Hall[HALL_A] = Hall[1];
                pHall->Hall[HALL_C] = Hall[0];
            }
        }
    }
    if(pHP != NULL)
    {
        pHall->pHP = pHP;
        mc_hall_Pedometer_hInit(pHP, HallNb);
    }
    if(pHSMM != NULL)
    {
        pHall->pHSMM = pHSMM;
        mc_hall_SpdMeasure_M_hInit(pHSMM);
    }
    if(pHSMT != NULL)
    {
        pHall->pHSMT = pHSMT;
        mc_hall_SpdMeasure_T_hInit(pHSMT);
    }
    if(pHAC != NULL)
    {
        pHall->pHAC = pHAC;
        mc_hall_AbnChecker_hInit(pHAC, HallNb);
    }
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 马达控制-电机HALL-HALL信号脉冲步数

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*句柄初始化*/
static void mc_hall_Pedometer_hInit(HALL_PEDOMETER_Handle_t *pHP, uint8_t HallNb)
{
    pHP->State = 0;
    pHP->PrevState = 0;
    if(HallNb == 1)
    {
        pHP->StateNb = 2;
        pHP->StateTbl = g_HallStateTbl_1;
        pHP->OneHallSetDir = HALL_DIR_DECREASE; //静止状态可能会下滑,故初始化默认给定反方向
    }else if(HallNb == 2)
    {
        pHP->StateNb = 4;
        pHP->StateTbl = g_HallStateTbl_2;
    }else if(HallNb == 3)
    {
        pHP->StateNb = 6;
        pHP->StateTbl = g_HallStateTbl_3;
    }else
    {
        //do nothing
    }
    pHP->HallData = DEFAULT_HALLDATA;
}
/*获取电机位置*/
MOTOR_POS_t mc_hall_Get_HallData(HALL_PEDOMETER_Handle_t *pHP)
{
    return pHP->HallData;
}
/*设置电机位置*/
void mc_hall_Set_HallData(HALL_PEDOMETER_Handle_t *pHP, MOTOR_POS_t HallData)
{
    pHP->HallData = HallData;
}
/*获取电机HALL状态*/
void mc_hall_Set_OneHallDir(HALL_PEDOMETER_Handle_t *pHP, HALL_DIR_t Dir)
{
    pHP->OneHallSetDir = Dir;
}
/*获取电机HALL状态*/
uint8_t mc_hall_Get_HallState(HALL_Handle_t *pHall)
{
    HALL_t Hall;
    uint8_t HallState = 0;
    /*根据HALL传感器引脚电平,解析HALL状态*/
    for(uint8_t i = 0; i < pHall->HallNb; i++)
    {
        Hall = pHall->Hall[i];
        HallState |= (((uint8_t)gpio_adapter_Read_Pin(Hall.HallPort, Hall.HallPin)) << i);  //逆序
    }
    pHall->pHP->State = HallState;
    return HallState;
}
/*电机HALL脉冲计步器*/
void mc_hall_Update_HallData(HALL_Handle_t *pHall)
{
    uint8_t iNext = 0;
    uint8_t iPrev = 0;
    HALL_PEDOMETER_Handle_t *pHP = pHall->pHP;
    uint8_t HallStateNb = pHall->pHP->StateNb;
    int8_t Inc = 0;
    uint8_t HallState = pHP->State;

    /*根据HALL状态变化进行脉冲计步*/
    for(uint8_t i=0; i<HallStateNb; i++)
    {
        if(HallState == pHP->StateTbl[i])
        {
            iNext = (i + 1) % HallStateNb;
            iPrev = (i + HallStateNb - 1) % HallStateNb;
            break;
        }
    }
    if(pHall->HallNb == 1)
    {
        if(pHP->PrevState != HallState)
        {
            if(pHP->OneHallSetDir == HALL_DIR_INCREASE)
                Inc = 1;
            else
                Inc = -1;
        }
    }else
    {
        if(pHP->PrevState == pHP->StateTbl[iPrev])
            Inc = 1;
        else if(pHP->PrevState == pHP->StateTbl[iNext])
            Inc = -1;
    }
	if(pHall->HallDirectionSel == 1)
	{
		Inc = -Inc;			
	}
    pHP->HallData += Inc;
    pHP->PrevState = HallState;
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 马达控制-电机HALL-HALL信号测速(M法)

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*句柄初始化*/
static void mc_hall_SpdMeasure_M_hInit(HALL_SPDMEAS_M_Handle_t *pHSMM)
{
    memset(pHSMM, 0, sizeof(HALL_SPDMEAS_M_Handle_t));
    pHSMM->MeasureSpdEn_M = FALSE;
    pHSMM->WindowWidth = HALLSMP_WINDOW_WIDTH_M;
}
/*计算窗口平移式HALL脉冲数采样器窗口时间*/
/*SpdDivision: 将1s分成SpdDivision份单位时间*/
void mc_hall_SpeedMeasure_M_Cal_WindowWidth(HALL_Handle_t *pHall, MOTOR_SPD_t TargetRPM, uint16_t SpdDivision)
{
    MOTOR_POS_t HallNbSmp = 0;
    HALL_SPDMEAS_M_Handle_t *pHSMM = pHall->pHSMM;

    HallNbSmp = TargetRPM * (HPP * pHall->PoleNb);
    HallNbSmp /= 60;
    pHSMM->WindowWidth = (uint16_t)(HallNbSmp / SpdDivision);
}
/*获取窗口平移式HALL脉冲数采样器窗口时间*/
uint16_t mc_hall_Get_M_WindowWidth(HALL_SPDMEAS_M_Handle_t *pHSMM)
{
    return pHSMM->WindowWidth;
}
/*重置窗口平移式HALL脉冲数采样器*/
void mc_hall_Reset_SpeedMeasure_M(HALL_Handle_t *pHall)
{
    HALL_SPDMEAS_M_Handle_t *pHSMM = pHall->pHSMM;

    pHSMM->WindowCnt = 0;
    pHSMM->HallNbSmpCnt = 0;
	pHSMM->HallNbSmpDoneCnt = 0;
	memset(pHSMM->HallNbSmp, 0, sizeof(pHSMM->HallNbSmp));
    pHSMM->PrevHallData = pHall->pHP->HallData;
    pHSMM->MeasureSpd_M = 0;
}
/*使能窗口平移式HALL脉冲数采样器*/
void mc_hall_SpeedMeasure_M_Enable(HALL_SPDMEAS_M_Handle_t *pHSMM)
{
    pHSMM->MeasureSpdEn_M = TRUE;
}
/*禁止窗口平移式HALL脉冲数采样器*/
void mc_hall_SpeedMeasure_M_Disable(HALL_SPDMEAS_M_Handle_t *pHSMM)
{
    pHSMM->MeasureSpdEn_M = FALSE;
}
/*HALL信号测速-M法(注:1ms定时调用1次)*/
void mc_hall_SpeedMeasure_M(HALL_Handle_t *pHall)
{
    MOTOR_POS_t CurrentHallData = 0;
    MOTOR_POS_t HallNb = 0;    //连续HALLSMP_BUF_SIZE_M个采样窗口内的HALL数量之和(步)
    int32_t HallFs = 0;    //根据HallNb计算出的1s内的HALL数量(步)

    HALL_SPDMEAS_M_Handle_t *pHSMM = pHall->pHSMM;

    if(pHSMM->MeasureSpdEn_M == TRUE)
    {
        if(++pHSMM->WindowCnt >= pHSMM->WindowWidth)
        {
            //gpio_adapter_Toggle_Pin(TEST1_GPIO_Port, TEST1_Pin);
            pHSMM->WindowCnt = 0;
            /*计算采样窗口内的HALL脉冲数*/
            CurrentHallData = pHall->pHP->HallData;
            /*窗口平移式HALL脉冲数采样器采样点存储*/
            pHSMM->HallNbSmp[pHSMM->HallNbSmpCnt] = CurrentHallData - pHSMM->PrevHallData;
            pHSMM->HallNbSmpCnt++;
            if(pHSMM->HallNbSmpCnt >= HALLSMP_BUF_SIZE_M)
            {
                pHSMM->HallNbSmpCnt = 0;
            }
			if(pHSMM->HallNbSmpDoneCnt < HALLSMP_BUF_SIZE_M)
            {			
				pHSMM->HallNbSmpDoneCnt++;
            }
            /*根据采样点,还原连续HALLSMP_BUF_SIZE_M个采样窗口内的HALL数量(步)*/
            for(uint8_t i = 0; i < pHSMM->HallNbSmpDoneCnt; i++)
            {
				HallNb += pHSMM->HallNbSmp[i];
            }
            HallNb *= HALLSMP_BUF_SIZE_M;
            HallNb /= pHSMM->HallNbSmpDoneCnt;
            /*HALL数量(步)转化为马达转速*/
            HallFs = (HallNb * 1000) / (HALLSMP_BUF_SIZE_M * pHSMM->WindowWidth);
            //pHSMM->MeasureSpd_M = HallFs * (60 / HPP) / pHall->PoleNb;        //n=60f/p ---- f(HALL周期频率)=HallFs(HALL步数频率)/4
            pHSMM->MeasureSpd_M = (HallFs * 60) / (pHall->PoleNb * HPP);        //n=60f/p ---- f(HALL周期频率)=HallFs(HALL步数频率)/4
            /**/
            pHSMM->PrevHallData = CurrentHallData;
        }
    }
}
/*获取M法的测量速度*/
MOTOR_SPD_t mc_hall_Get_MeasureSpeed_M(HALL_SPDMEAS_M_Handle_t *pHSMM)
{
    return pHSMM->MeasureSpd_M;
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 马达控制-电机HALL-HALL信号测速(T法)

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*句柄初始化*/
static void mc_hall_SpdMeasure_T_hInit(HALL_SPDMEAS_T_Handle_t *pHSMT)
{
	memset(pHSMT, 0, sizeof(HALL_SPDMEAS_T_Handle_t));
    pHSMT->MeasureSpdEn_T = FALSE;
}
/*读取用于计算HALL脉宽的定时器计数器值*/
static uint16_t mc_hall_Get_SpeedMeasure_T_TimerCnt(void)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
    //return timer_Get_Counter(&htim17);
    return 0;
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
    return Bt_M0_Cnt16Get(TIM1);
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    return 0;
#endif
}
/*用于计算HALL脉宽的大周期计数器(1ms调用一次)*/
void mc_hall_SpeedMeasure_T_Timer(void)
{
    g_HallSpdMeasureTimer++;
}
/*重置窗口平移式HALL脉宽采样器*/
void mc_hall_Reset_SpeedMeasure_T(HALL_SPDMEAS_T_Handle_t *pHSMT)
{
    pHSMT->HallWidthSmpCnt = 0;
	pHSMT->HallWidthSmpDoneCnt = 0;
	memset(pHSMT->HallWidthSmp, 0, sizeof(pHSMT->HallWidthSmp));
	
    pHSMT->PreShieldCnt = 0;
    pHSMT->TC1 = mc_hall_Get_SpeedMeasure_T_TimerCnt();
    pHSMT->TP1 = g_HallSpdMeasureTimer;
    pHSMT->TC0 = pHSMT->TC1;
    pHSMT->TP0 = pHSMT->TP1;
    pHSMT->MeasureSpd_T = 0;
    pHSMT->PrevMeasureSpd_T = 0;
}
/*使能窗口平移式HALL脉宽采样器*/
void mc_hall_SpeedMeasure_T_Enable(HALL_SPDMEAS_T_Handle_t *pHSMT)
{
    pHSMT->MeasureSpdEn_T = TRUE;
}
/*禁止窗口平移式HALL脉宽采样器*/
void mc_hall_SpeedMeasure_T_Disable(HALL_SPDMEAS_T_Handle_t *pHSMT)
{
    pHSMT->MeasureSpdEn_T = FALSE;
}
/*计算2个HALL有效边沿间的时间*/
static uint32_t mc_hall_Cal_SpeedMeasure_T_Time(HALL_SPDMEAS_T_Handle_t *pHSMT)
{
    uint32_t MeasureHallPulseWidth = 0;

    pHSMT->TC0 = pHSMT->TC1;
    pHSMT->TP0 = pHSMT->TP1;
    pHSMT->TC1 = mc_hall_Get_SpeedMeasure_T_TimerCnt();
    pHSMT->TP1 = g_HallSpdMeasureTimer;
    if(pHSMT->TP1 >= pHSMT->TP0)
    {
        MeasureHallPulseWidth = (uint32_t)(pHSMT->TP1 - pHSMT->TP0) * TIMER_OVF_PERIOD;
        MeasureHallPulseWidth += pHSMT->TC1;
        MeasureHallPulseWidth -= pHSMT->TC0;
    }else   /*g_HallSpdMeasureTimer累加计数溢出*/
    {
        MeasureHallPulseWidth = (uint32_t)pHSMT->TP1 + (65536 - pHSMT->TP0);
        MeasureHallPulseWidth *= TIMER_OVF_PERIOD;
        MeasureHallPulseWidth += pHSMT->TC1;
        MeasureHallPulseWidth -= pHSMT->TC0;
    }
    return MeasureHallPulseWidth;
}
/*HALL信号测速-T法(注:HALL1和HALL2脉冲信号跳变时都需调用)*/
/*Todo：1.目前HALL信号有异常会导致MeasureSpd_T的值无法被更新,需要想个办法解决.
2. 目前MeasureSpd_T的值恒为正,无法表示转速的正负*/
void mc_hall_SpeedMeasure_T(HALL_Handle_t *pHall)
{
    uint32_t MeasureHallPulseWidth = 0;
    uint32_t MeasureHallFrq = 0;        //每分钟的HALL信号数
    uint32_t MeasureRPM = 0;
    HALL_SPDMEAS_T_Handle_t *pHSMT = pHall->pHSMT;

    if(pHSMT->MeasureSpdEn_T == TRUE)
    {
        //gpio_adapter_Toggle_Pin(GPIO_TEST1_PORT, GPIO_TEST1_PIN);
        /*HALL脉宽计算*/
        MeasureHallPulseWidth = mc_hall_Cal_SpeedMeasure_T_Time(pHSMT);
        if(pHSMT->PreShieldCnt < HALL_MEASUER_PRESHIELD)
        {
            pHSMT->PreShieldCnt++;
        }else
        {
            pHSMT->PrevMeasureSpd_T = pHSMT->MeasureSpd_T;
            /*窗口平移式HALL脉宽采样器采样点存储*/
            pHSMT->HallWidthSmp[pHSMT->HallWidthSmpCnt] = MeasureHallPulseWidth;
            pHSMT->HallWidthSmpCnt++;
            if(pHSMT->HallWidthSmpCnt >= HALLSMP_BUF_SIZE_T)
            {
                pHSMT->HallWidthSmpCnt = 0;
            }
			if(pHSMT->HallWidthSmpDoneCnt < HALLSMP_BUF_SIZE_T)
            {
				pHSMT->HallWidthSmpDoneCnt++;
            }
            /*根据采样点还原HALL周期宽度(每个机械周期的HALL信号宽度(几对极就是几个HALL周期的求和))*/
            MeasureHallPulseWidth = 0;
            for(uint8_t i = 0; i < pHSMT->HallWidthSmpDoneCnt; i++)
            {
				MeasureHallPulseWidth += pHSMT->HallWidthSmp[i];
            }
            MeasureHallPulseWidth *= HALLSMP_BUF_SIZE_T;
            MeasureHallPulseWidth /= pHSMT->HallWidthSmpDoneCnt;
            /*HALL脉宽转化为马达转速*/
            if(MeasureHallPulseWidth != 0)
            {
            #if 0   //对应MeasureHallPulseWidth是电周期的情况
                MeasureHallFrq = 60000000 / MeasureHallPulseWidth;  //60(s)*1000000(us/s)  n=60f/p
                MeasureRPM = MeasureHallFrq / pHall->PoleNb;
            #else   //对应MeasureHallPulseWidth是机械周期的情况
                MeasureHallFrq = 60000000 / MeasureHallPulseWidth;  //60(s)*1000000(us/s)  n=60f/p
                MeasureRPM = MeasureHallFrq;
            #endif
                pHSMT->MeasureSpd_T = (MOTOR_SPD_t)MeasureRPM;
            }else
            {
                //如果采集值异常,则MeasureSpd_T值不更新(保持上次的值)
            }
        }
    }
}
/*获取T法的测量速度*/
MOTOR_SPD_t mc_hall_Get_MeasureSpeed_T(HALL_SPDMEAS_T_Handle_t *pHSMT)
{
    return pHSMT->MeasureSpd_T;
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 马达控制-电机HALL-HALL信号异常检测

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*句柄初始化*/
static void mc_hall_AbnChecker_hInit(HALL_ABN_CHECKER_Handle_t *pHAC, uint8_t HallNb)
{
	memset(pHAC, 0, sizeof(HALL_ABN_CHECKER_Handle_t));
    pHAC->SetDir = HALL_DIR_INCREASE;
    pHAC->HallDataA = DEFAULT_HALLDATA;
    pHAC->HallDataB = DEFAULT_HALLDATA;
    pHAC->HallAbnType = HALL_ABN_NO_ABN;
    if(HallNb == 1)
    {
        pHAC->LegalStateMin = 0;
        pHAC->LegalStateMax = 1;
    }else if(HallNb == 2)
    {
        pHAC->LegalStateMin = 0;
        pHAC->LegalStateMax = 3;
    }else if(HallNb == 3)
    {
        pHAC->LegalStateMin = 1;
        pHAC->LegalStateMax = 6;
    }else
    {
        //do nothing
    }
}
/*使能HALL异常监测*/
void mc_hall_Check_Hall_Abnormal_Enable(HALL_ABN_CHECKER_Handle_t *pHAC)
{
    pHAC->CheckAbnEn = TRUE;
}
/*禁止HALL异常监测*/
void mc_hall_Check_Hall_Abnormal_Disable(HALL_ABN_CHECKER_Handle_t *pHAC)
{
    pHAC->CheckAbnEn = FALSE;
}
/*初始化HALL异常监测的参数*/
/*param:
MinSpeed: 电机设置最小转速(RPM)
MaxSpeed: 电机设置最大转速(RPM)
HPR: 电机每圈有效HALL信号数*/
void mc_hall_Check_Hall_Abnormal_Init(HALL_ABN_CHECKER_Handle_t *pHAC, HALL_DIR_t Dir, MOTOR_SPD_t MinSpeed, MOTOR_SPD_t MaxSpeed, int32_t Hpr)
{
    int32_t Halls1, Halls2;
    int32_t Div_1s = 1000 / HALL_CHECK_ABN_PERIOD;

    Halls1 = MinSpeed * Hpr / (60 * Div_1s);
    Halls2 = MaxSpeed * Hpr / (60 * Div_1s);

    pHAC->SetDir = Dir;
    pHAC->ReverseTHH = 0;
    pHAC->TooFewTHH = Halls1 / 3 + 1;   //最小速度的0.33倍, +1防止算出来是0
    pHAC->TooManyTHH = (Halls2<<3) / 6; //最大速度的1.33倍
}
/*重置HALL异常监测器*/
void mc_hall_Reset_Check_Hall_Abnormal(HALL_ABN_CHECKER_Handle_t *pHAC)
{
    pHAC->CheckAbnTimer = 0;
    pHAC->CheckAbnShieldTimer = 0;
    pHAC->IllegalStateCnt = 0;
    pHAC->HallAbnType = HALL_ABN_NO_ABN;
}
/*HALL异常监测(定时250us调用1次)*/
void mc_hall_Check_Hall_Abnormal(HALL_Handle_t *pHall)
{
    int32_t HallChangeAbs = 0;
    HALL_ABN_CHECKER_Handle_t *pHAC = pHall->pHAC;
    uint8_t HallState = 0;
    /*HALL异常采样周期计时*/
    if(pHAC->CheckAbnEn == TRUE)
    {
        /*HALL异常采样启动屏蔽计时*/
        if(pHAC->CheckAbnShieldTimer < 60000)
        {
            pHAC->CheckAbnShieldTimer++;
        }
        if(pHAC->CheckAbnShieldTimer >= 4 * HALL_CHECK_ABN_PRESHIELD_TIME)
        {
            pHAC->CheckAbnTimer++;
        }
    }else
    {
        return;
    }
    /*HALL电平状态异常判断*/
    HallState = pHall->pHP->State;
    if(HallState >= pHAC->LegalStateMin && HallState <= pHAC->LegalStateMax)
    {
        pHAC->IllegalStateCnt = 0;
    }else
    {
        //if(++pHAC->IllegalStateCnt >= 4 * HALL_CHECK_ABN_ILLEGAL_STATE_TIME)
        if(++pHAC->IllegalStateCnt >= 5)
        {
            pHAC->HallAbnType |= HALL_ABN_ILLEGAL_STATE;
        }
    }
    /*采集A,B点HALL值用于HALL异常判断*/
    if(pHAC->CheckAbnTimer == 4 * HALL_CHECK_ABN_PERIOD)
    {
        pHAC->HallDataA = pHall->pHP->HallData;
    }
    if(pHAC->CheckAbnTimer == 8 * HALL_CHECK_ABN_PERIOD)
    {
        pHAC->HallDataB = pHall->pHP->HallData;
        HallChangeAbs = pHAC->HallDataB - pHAC->HallDataA;
        if(HallChangeAbs < 0)
            HallChangeAbs = HallChangeAbs * (-1);
        /*判断HALL变化方向与预设方向是否一致*/
        if(pHAC->SetDir == HALL_DIR_INCREASE)
        {
            if(pHAC->HallDataB - pHAC->HallDataA < (int32_t)pHAC->ReverseTHH)
            {
                pHAC->HallAbnType |= HALL_ABN_REVERSE;
            }
        }else   /*pHall->SetDir == HALL_DIR_DECREASE*/
        {
            if(pHAC->HallDataB - pHAC->HallDataA > (int32_t)pHAC->ReverseTHH)
            {
                pHAC->HallAbnType |= HALL_ABN_REVERSE;
            }
        }
        /*判断HALL数量是否正确*/
        if(HallChangeAbs > pHAC->TooManyTHH)
        {
            pHAC->HallAbnType |= HALL_ABN_TOO_MANY;
        }
        if(HallChangeAbs < pHAC->TooFewTHH)
        {
            pHAC->HallAbnType |= HALL_ABN_TOO_FEW;
        }
        pHAC->CheckAbnTimer = 0;
    }
}
/*清零HALL异常*/
void mc_hall_Clear_Hall_Abnormal(HALL_ABN_CHECKER_Handle_t *pHAC)
{
    pHAC->HallAbnType = HALL_ABN_NO_ABN;
}
/*获取HALL异常类型*/
uint8_t mc_hall_Get_Hall_Abnormal(HALL_ABN_CHECKER_Handle_t *pHAC)
{
    return pHAC->HallAbnType;
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 马达控制-电机HALL-模块功能测试

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
void mc_hall_Test(HALL_Handle_t *pHall)
{
	/*T法测速*/
    if(g_TestMCHallCmd == 1)
    {
        g_TestMCHallCmd = 0;
        mc_hall_SpeedMeasure_T_Enable(pHall->pHSMT);
    }
    if(g_TestMCHallCmd == 2)
    {
        g_TestMCHallCmd = 0;
        mc_hall_SpeedMeasure_T_Disable(pHall->pHSMT);
    }
    if(g_TestMCHallCmd == 4)
    {
        g_TestMCHallCmd = 0;
        g_TestTimerCounter = mc_hall_Get_SpeedMeasure_T_TimerCnt();
    }
    if(g_TestMCHallCmd == 5)
    {
        g_TestMCHallCmd = 0;
        mc_hall_Reset_SpeedMeasure_T(pHall->pHSMT);
    }
	/*M法测速*/
	if(g_TestMCHallCmd == 8)
    {
        g_TestMCHallCmd = 0;
        mc_hall_Reset_SpeedMeasure_M(pHall);
    }
	/*hall异常检测*/
    if(g_TestMCHallCmd == 6)
    {
        g_TestMCHallCmd = 0;
        mc_hall_Check_Hall_Abnormal_Enable(pHall->pHAC);
    }
    if(g_TestMCHallCmd == 7)
    {
        g_TestMCHallCmd = 0;
        mc_hall_Check_Hall_Abnormal_Disable(pHall->pHAC);
    }
}
