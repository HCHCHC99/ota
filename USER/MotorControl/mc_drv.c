/********************************文件说明*************************************
*文件名: mc_drv.c

*作者: Yuchen Tan

*版本: V2.0.0

*功能简介: H桥驱动器

*备注:
1.电机方向顺时针(CW)/逆时针(CCW)定义均为从电机出轴端看入的转向.

*修改履历:
------------------------------------V1.0.1------------------------------------
*20220419: MC_DRV_Handle_t的MOS引脚改为数组形式,数组元素数对应宏定义DRIVER_PHASE_NB.
------------------------------------V1.0.2------------------------------------
*20220419:
1.MOS操作函数的传参改为结构体指针,接口内部适配不同MCU类型;
2.MOS类型定义提高抽象度,不分MOS_H,MOS_L.
------------------------------------V1.0.3------------------------------------
*20220709: 驱动器关闭时,在DRIVER_STATE_STOP_CLOSE_3HL状态把3路下桥也一起关闭,并
且等待时间加长到100ms后再开启刹车(3路下桥开启)
*20220713: MOS操作函数中,补充适配HC32F460/HC32F030/130的MCU.
------------------------------------V1.0.4------------------------------------
*20220716:
1.通配2相驱动器(H桥-驱动有刷电机)和3相驱动器(3相逆变器-驱动BLDC).
2.删除未使用的驱动器主句柄成员SleepCmd和WakeUpCmd.
3.修改函数mc_drv_MOS_H_Close(),改HAL_TIM_PWM_Stop()为HAL_TIM_PWM_Start(),否则
  HAL_TIM_PWM_Stop()调用完成后,引脚输出的是高电平而不是低电平!
*20220921:
1.GPIO代码改用gpio_adapter.h提供的类型定义及接口;
2.TIMER(部分)代码改用timer_adapter.h提供的类型定义及接口;
*20221021:
1.禁止在主循环中调用换向函数(若和中断中的换向函数重入,可能导致mainLoop中的换相换错);
*20221025:
1.电机停止时,给一个刹车时间STOP_BRAKE_TIME(100ms)再跳转到IDLE状态,使应用层获取
  到DRIVER_STATE_IDLE状态的一瞬间电机已基本完全静止;
------------------------------------V1.0.5------------------------------------
20230220: 配合mc_config.h文件的V1.0.2修改,详见mc_config.h修改履历;
20230309: MOS的PWM引脚操作改用timer_adapter模块提供的抽象接口.
------------------------------------V2.0.0------------------------------------
20230406: 适配电机动态自锁功能,驱动器根据PWMDuty的±符号控制占空比输出;
------------------------------------V2.0.1------------------------------------
20230911: 增加适配驱动方向可设置的代码;
20231109: 增加适配不同驱动电路+预驱,上下桥端口输出模式的代码(DRV_OUTPUT_TYPE);
*****************************************************************************/

/*****************************文件包含(私有)**********************************
*
*备注: 无
*
*****************************************************************************/
#include "mc_drv.h"
#include "delay_adapter.h"
/*****************************宏定义(私有)************************************
*
*备注: 本文件中,不希望被外部使用或随意修改的宏在这里定义
*
*****************************************************************************/
/*驱动器参数定义*/
#define BOOT_CHARGE_TIME        (10)    //自举电容充电时间(单位：ms)
#define MOS_OP_DELAY            (5)     //MOS开通关闭等待延时(单位：ms)
#define STOP_RELEASE_TIME       (20)   //停止开启刹车前的自由释放时间(单位：ms)(时间越长,电机停止的惯性过冲距离越长!)
#define STOP_BRAKE_TIME         (100)   //停止进入IDLE前的刹车持续时间(单位：ms)

/*模块功能配置参数*/
#define MOS_OUTPUT_ENABLE       (1)     //MOS操作函数刚刚写好后,先不开启NMOS输出,逻辑调通后再输出

/*默认最大最小占空比*/
#define DEFAULT_MAX_PWM_DUTY    (PWM_OUTPUT_FULLSCALE)
#define DEFAULT_MIN_PWM_DUTY    (0)

/*电机驱动模块-模块功能测试方式定义*/
#define MC_DRV_TEST_TYPE        (0)     /*0-测电机驱动API及状态机功能, 1-只测电机驱动端口信号*/
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
static inline void mc_drv_MOS_L_Open(MOS_t* MosL, int16_t PWMDuty);
static inline void mc_drv_MOS_L_Close(MOS_t* MosL);
static void mc_drv_MOS_H_Open(MOS_t* MosH, int16_t PWMDuty);
static void mc_drv_MOS_H_Close(MOS_t* MosH);
static void mc_drv_Update_PWM_Duty(MC_DRV_Handle_t *pDrv, int16_t PWMDuty);
/********************************变量定义*************************************
*
*备注: 不要在.h中定义变量,防止多个.c包含这个.h,导致编译后报错变量重复定义
*
*****************************************************************************/
/*驱动器状态跳转表定义*/
const uint16_t g_DrvStateJumpTbl[DRIVER_STATE_NB] =
{
    DRIVER_STATE_IDLE_START,                //legal state jump from state DRIVER_STATE_IDLE
    DRIVER_STATE_START_CHARGE_BOOT,         //legal state jump from state DRIVER_STATE_IDLE_START
    DRIVER_STATE_START_WAIT_CHARGE_BOOT,    //legal state jump from state DRIVER_STATE_START_CHARGE_BOOT
    DRIVER_STATE_START_CLOSE_3L,            //legal state jump from state DRIVER_STATE_START_WAIT_CHARGE_BOOT
    DRIVER_STATE_START_WAIT_CLOSE_3L,       //legal state jump from state DRIVER_STATE_START_CLOSE_3L
    DRIVER_STATE_START_OPEN_MOS,            //legal state jump from state DRIVER_STATE_START_WAIT_CLOSE_3L
    DRIVER_STATE_RUN,                       //legal state jump from state DRIVER_STATE_START_OPEN_MOS
    DRIVER_STATE_STOP_CLOSE_3HL,            //legal state jump from state DRIVER_STATE_RUN
    DRIVER_STATE_STOP_WAIT_CLOSE_3HL,       //legal state jump from state DRIVER_STATE_STOP_CLOSE_3HL
    DRIVER_STATE_STOP_OPEN_3L,              //legal state jump from state DRIVER_STATE_STOP_WAIT_CLOSE_3HL
    DRIVER_STATE_STOP_BRAKE,                //legal state jump from state DRIVER_STATE_STOP_OPEN_3L
    DRIVER_STATE_IDLE                       //legal state jump from state DRIVER_STATE_STOP_BRAKE
};

/* ”转子位置(HALL扇区)-相状态” 映射表(数组索引表示扇区对应的hall值.值的高4位表示上桥状态,低4位表示下桥状态(1开0关,位序低A高C))*/
/*3相驱动器换相表*/
/*  正转:3:AC 2:AB    6:CB    4:CA    5:BA    1:BC(hall序列: 451326)
    反转:4:AC 5:AB    1:CB    3:CA    2:BA    6:BC(hall序列: 462315)
    注: 转子在同一个扇区时,正转和反转给的电压矢量刚好相反(3:AC(正) 3:CA(反))!*/
const uint8_t g_3PhaseCommutationTable[2][8] =
{
    {0x00, 0x24, 0x12, 0x14, 0x41, 0x21, 0x42, 0x00},   //CW
    {0x00, 0x42, 0x21, 0x41, 0x14, 0x12, 0x24, 0x00}    //CCW
};
/*2相驱动器换相表(实际不用换相,采用换相表目的是和3相驱动器代码统一)*/
/*  正转: 0:AB    1:AB    3:AB    2:AB(hall序列: 0132)
    反转: 0:BA    1:BA    3:BA    2:BA(hall序列: 0231)*/
const uint8_t g_2PhaseCommutationTable[2][4] =
{
    {0x12, 0x12, 0x12, 0x12},   //CW
    {0x21, 0x21, 0x21, 0x21}    //CCW
};

/*以下为测试用变量*/
uint8_t		g_TestMotorCmd = 0;
int16_t		g_TestSetPWMDuty = 0;
/********************************函数定义************************************
*函数名:

*函数功能描述: 马达控制-MOS三相逆变驱动器-句柄操作

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*换向器初始化*/
 void mc_drv_Commutator_hInit(COMMUTATOR_t *Commutator, uint8_t PhaseNb, uint8_t PhaseDirectionSel)
{
    memset(Commutator, 0, sizeof(COMMUTATOR_t));
    if(PhaseNb == 2)
    {
		if(PhaseDirectionSel == 0)
		{
			Commutator->CommutationTableFO = g_2PhaseCommutationTable[0];
			Commutator->CommutationTableRO = g_2PhaseCommutationTable[1];
		}else
		{
			Commutator->CommutationTableFO = g_2PhaseCommutationTable[1];
			Commutator->CommutationTableRO = g_2PhaseCommutationTable[0];
		}
    }else //(PhaseNb == 3)
    {
		if(PhaseDirectionSel == 0)
		{
			Commutator->CommutationTableFO = g_3PhaseCommutationTable[0];
			Commutator->CommutationTableRO = g_3PhaseCommutationTable[1];
		}else
		{
			Commutator->CommutationTableFO = g_3PhaseCommutationTable[1];
			Commutator->CommutationTableRO = g_3PhaseCommutationTable[0];
		}
    }
}
/*驱动器初始化*/
void mc_drv_hInit(MC_DRV_Handle_t *pDrv, uint16_t Tag, MOS_t *MosH, MOS_t *MosL, uint8_t PhaseNb, uint8_t PhaseSequence, COMMUTATOR_t *Commutator, uint8_t PhaseDirectionSel)
{
    if(!pDrv || !MosH || !MosL || !Commutator || (PhaseNb < 2 || PhaseNb > 3))
        return;

    /*句柄初始化*/
	memset(pDrv, 0, sizeof(MC_DRV_Handle_t));
    pDrv->PhaseNb = PhaseNb;

    /*mos初始化*/
    if(PhaseNb == 2)
    {
        pDrv->PhaseSequenceSel = PhaseSequence % 2; //双相有2种相排序(AB/BA)
        if(pDrv->PhaseSequenceSel == 0)
        {   //AB
            pDrv->MosH[DRIVER_PHASE_A] = MosH[0];
            pDrv->MosL[DRIVER_PHASE_A] = MosL[0];
            pDrv->MosH[DRIVER_PHASE_B] = MosH[1];
            pDrv->MosL[DRIVER_PHASE_B] = MosL[1];
        }else   //(pDrv->PhaseSequenceSel == 1)
        {   //BA
            pDrv->MosH[DRIVER_PHASE_B] = MosH[0];
            pDrv->MosL[DRIVER_PHASE_B] = MosL[0];
            pDrv->MosH[DRIVER_PHASE_A] = MosH[1];
            pDrv->MosL[DRIVER_PHASE_A] = MosL[1];
        }
    }else //(PhaseNb == 3)
    {
        pDrv->PhaseSequenceSel = PhaseSequence % 6; //三相有6种相排序(ABC/BCA/CAB/ACB/CBA/BAC)
        if(pDrv->PhaseSequenceSel == 0)
        {   //ABC
            pDrv->MosH[DRIVER_PHASE_A] = MosH[0];
            pDrv->MosL[DRIVER_PHASE_A] = MosL[0];
            pDrv->MosH[DRIVER_PHASE_B] = MosH[1];
            pDrv->MosL[DRIVER_PHASE_B] = MosL[1];
            pDrv->MosH[DRIVER_PHASE_C] = MosH[2];
            pDrv->MosL[DRIVER_PHASE_C] = MosL[2];
        }else if(pDrv->PhaseSequenceSel == 1)
        {   //BCA
            pDrv->MosH[DRIVER_PHASE_B] = MosH[0];
            pDrv->MosL[DRIVER_PHASE_B] = MosL[0];
            pDrv->MosH[DRIVER_PHASE_C] = MosH[1];
            pDrv->MosL[DRIVER_PHASE_C] = MosL[1];
            pDrv->MosH[DRIVER_PHASE_A] = MosH[2];
            pDrv->MosL[DRIVER_PHASE_A] = MosL[2];
        }else if(pDrv->PhaseSequenceSel == 2)
        {   //CAB
            pDrv->MosH[DRIVER_PHASE_C] = MosH[0];
            pDrv->MosL[DRIVER_PHASE_C] = MosL[0];
            pDrv->MosH[DRIVER_PHASE_A] = MosH[1];
            pDrv->MosL[DRIVER_PHASE_A] = MosL[1];
            pDrv->MosH[DRIVER_PHASE_B] = MosH[2];
            pDrv->MosL[DRIVER_PHASE_B] = MosL[2];
        }else if(pDrv->PhaseSequenceSel == 3)
        {   //ACB
            pDrv->MosH[DRIVER_PHASE_A] = MosH[0];
            pDrv->MosL[DRIVER_PHASE_A] = MosL[0];
            pDrv->MosH[DRIVER_PHASE_C] = MosH[1];
            pDrv->MosL[DRIVER_PHASE_C] = MosL[1];
            pDrv->MosH[DRIVER_PHASE_B] = MosH[2];
            pDrv->MosL[DRIVER_PHASE_B] = MosL[2];
        }else if(pDrv->PhaseSequenceSel == 4)
        {   //CBA
            pDrv->MosH[DRIVER_PHASE_C] = MosH[0];
            pDrv->MosL[DRIVER_PHASE_C] = MosL[0];
            pDrv->MosH[DRIVER_PHASE_B] = MosH[1];
            pDrv->MosL[DRIVER_PHASE_B] = MosL[1];
            pDrv->MosH[DRIVER_PHASE_A] = MosH[2];
            pDrv->MosL[DRIVER_PHASE_A] = MosL[2];
        }else //(pDrv->PhaseSequenceSel == 5)
        {   //BAC
            pDrv->MosH[DRIVER_PHASE_B] = MosH[0];
            pDrv->MosL[DRIVER_PHASE_B] = MosL[0];
            pDrv->MosH[DRIVER_PHASE_A] = MosH[1];
            pDrv->MosL[DRIVER_PHASE_A] = MosL[1];
            pDrv->MosH[DRIVER_PHASE_C] = MosH[2];
            pDrv->MosL[DRIVER_PHASE_C] = MosL[2];
        }
    }

    /*换向器初始化*/
    if(Commutator != NULL)
    {
        pDrv->Commutator = Commutator;
        mc_drv_Commutator_hInit(Commutator, PhaseNb, PhaseDirectionSel);
    }

    pDrv->Tag = Tag;
    pDrv->State = DRIVER_STATE_IDLE;
	
    pDrv->MaxPWMDuty = DEFAULT_MAX_PWM_DUTY;
    pDrv->MinPWMDuty = DEFAULT_MIN_PWM_DUTY;

    /*电机驱动器控制接口初始化(下桥导通提供自锁)*/
    //tyc: 奇怪问题：F460必须要先调用mc_drv_MOS_H_Open()开一下占空比,再调用mc_drv_MOS_H_Close()才能使PWM通道关闭(输出0),\
                     如果不这样做,调用mc_drv_MOS_H_Close()后PWM通道仍然输出1.
    for(uint8_t i=0; i < PhaseNb; i++)
    {
        mc_drv_MOS_H_Open(&pDrv->MosH[i], 1);
        mc_drv_MOS_H_Close(&pDrv->MosH[i]);
    }
    /*打开下桥提供刹车自锁(和DRIVER_STATE_IDLE状态下的上下桥mos开关状态对应)*/
    for(uint8_t i=0; i < PhaseNb; i++)
    {
        mc_drv_MOS_H_Close(&pDrv->MosH[i]);
        mc_drv_MOS_L_Open(&pDrv->MosL[i], PWM_OUTPUT_FULLSCALE);
    }
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 马达控制-MOS三相逆变驱动器-MOS开关控制

*函数参数: 无

*函数返回值: 无

*备注:
1.MOS预驱芯片SDH21263特性: HIN-1开0关(上桥),LIN-0开1关(下桥)
*****************************************************************************/
/*下桥MOS导通*/
static inline void mc_drv_MOS_L_Open(MOS_t* MosL, int16_t PWMDuty)
{
#if (MOS_OUTPUT_ENABLE == 1)
#if (DRV_OUTPUT_TYPE == DO_H_PWM_L_NULL)
	//do nothing
#elif (DRV_OUTPUT_TYPE == DO_H_PWM_L_IO)
    gpio_adapter_Reset_Pin(MosL->MosPort, MosL->MosPin);
#elif (DRV_OUTPUT_TYPE == DO_H_PWM_L_NPWM)
	
#endif
#endif
}
/*下桥MOS关闭*/
static inline void mc_drv_MOS_L_Close(MOS_t* MosL)
{
#if (DRV_OUTPUT_TYPE == DO_H_PWM_L_NULL)
	//do nothing
#elif (DRV_OUTPUT_TYPE == DO_H_PWM_L_IO)
    gpio_adapter_Set_Pin(MosL->MosPort, MosL->MosPin);
#elif (DRV_OUTPUT_TYPE == DO_H_PWM_L_NPWM)

#endif
}
/*上桥MOS导通*/
static void mc_drv_MOS_H_Open(MOS_t* MosH, int16_t PWMDuty)
{
#if (MOS_OUTPUT_ENABLE == 1)
	TIMER_CHANNAL_t PWMCH = {.TimerIns = MosH->MosPWMTimer, .Channal = MosH->MosPWMCh};
	timer_Ch_PWM_OutPut(&PWMCH, abs(PWMDuty));	//给timer通道寄存器的值是正的!
#endif
}
/*上桥MOS关闭*/
static void mc_drv_MOS_H_Close(MOS_t* MosH)
{
	TIMER_CHANNAL_t PWMCH = {.TimerIns = MosH->MosPWMTimer, .Channal = MosH->MosPWMCh};
	timer_Ch_PWM_OutPut_Disable(&PWMCH);
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 马达控制-MOS三相逆变驱动器-MOS开关延时控制

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*MOS开关延时计数器启动*/
static void mc_drv_Delay_Timer_Start(MC_DRV_Handle_t *pDrv)
{
    pDrv->DelayTimerCnt = 0;
    pDrv->DelayTimerEn = 1;
}
/*MOS开关延时计数器*/
void mc_drv_Delay_Timer_Run(MC_DRV_Handle_t *pDrv)
{
    if(pDrv->DelayTimerEn == 1)
    {
        pDrv->DelayTimerCnt++;
    }
}
/*MOS开关延时计数器停止*/
static void mc_drv_Delay_Timer_Stop(MC_DRV_Handle_t *pDrv)
{
    pDrv->DelayTimerEn = 0;
    pDrv->DelayTimerCnt = 0;
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 马达控制-MOS三相逆变驱动器-驱动器命令及设置接口

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*驱动器命令*/
void mc_drv_SetCmd(MC_DRV_Handle_t *pDrv, mc_drv_cmd_t Cmd)
{
    pDrv->Cmd = Cmd;
}
/*驱动器设置占空比*/
/*注: 速度环的输出可能是负的,因此PWMValue,PWMDuty必须是带符号int类型不能是uint类型,防止负数反向溢出*/
void mc_drv_SetPWMDuty(MC_DRV_Handle_t *pDrv, int16_t PWMValue)
{
#if (REVERSE_OUTPUT_EN == 1)
	int16_t Sign = (PWMValue >= 0) ? 1 : (-1);
	int16_t AbsPWMValue = abs(PWMValue);
	if(AbsPWMValue > pDrv->MaxPWMDuty)
        pDrv->PWMDuty = pDrv->MaxPWMDuty * Sign;
    else if(AbsPWMValue < pDrv->MinPWMDuty)
        pDrv->PWMDuty = pDrv->MinPWMDuty * Sign;
    else
        pDrv->PWMDuty = PWMValue;
#else
	if(pDrv->SetDir == e_forward)
	{
		if(PWMValue < pDrv->MinPWMDuty)
			pDrv->PWMDuty = pDrv->MinPWMDuty;
		else if(PWMValue > pDrv->MaxPWMDuty)
			pDrv->PWMDuty = pDrv->MinPWMDuty;
		else
			pDrv->PWMDuty = PWMValue;
	}else if(pDrv->SetDir == e_reverse)
	{
		if(PWMValue > pDrv->MinPWMDuty * (-1))
			pDrv->PWMDuty = pDrv->MinPWMDuty * (-1);
		else if(PWMValue < pDrv->MaxPWMDuty * (-1))
			pDrv->PWMDuty = pDrv->MaxPWMDuty * (-1);
		else
			pDrv->PWMDuty = PWMValue;
	}
#endif
#if (MOTOR_TYPE == MOTOR_TYPE_DC)   //注: 无刷电机不能在此调用mc_drv_Update_PWM_Duty(),因为此函数中有换相操作(不能重入),同时段只能存在1个此函数的调用.
    if(pDrv->State == DRIVER_STATE_RUN)
    {
		mc_drv_Set_Motor_Rotor_Sector(pDrv, pDrv->Commutator->RotorSector);
        mc_drv_Update_PWM_Duty(pDrv, pDrv->PWMDuty);
    }
#endif
}
/*获取驱动器占空比*/
int16_t mc_drv_GetPWMDuty(MC_DRV_Handle_t *pDrv)
{
    return pDrv->PWMDuty;
}
/*设置驱动器最大输出*/
void mc_drv_SetPWMDutyMax(MC_DRV_Handle_t *pDrv, int16_t MaxPWMValue)
{
    pDrv->MaxPWMDuty = MaxPWMValue;
}
/*设置驱动器最小输出*/
void mc_drv_SetPWMDutyMin(MC_DRV_Handle_t *pDrv, int16_t MinPWMValue)
{
    pDrv->MinPWMDuty = MinPWMValue;
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 马达控制-MOS三相逆变驱动器-驱动器换向控制

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*设置马达转子所在扇区(Hall状态更新时刻调用,电机运行过程中不可重入!)*/
void mc_drv_Set_Motor_Rotor_Sector(MC_DRV_Handle_t *pDrv, uint8_t RotorSector)
{
    uint8_t NextPhaseState = 0;
    uint8_t OpenH = 0;  //默认强制全部输出0
    uint8_t OpenL = 0;  //默认强制全部输出0

    if((pDrv->PhaseNb == 2 && RotorSector > 3) || (pDrv->PhaseNb == 3 && RotorSector > 7))
    {
        OpenH = OpenL = 0;      /*非法扇区参数,强制全部输出0*/
    }else
    {
        /*根据转子位置和换相对应表,控制驱动器上下桥开关*/
        if(pDrv->PWMDuty >= 0)
            NextPhaseState = pDrv->Commutator->CommutationTableFO[RotorSector];
        else
            NextPhaseState = pDrv->Commutator->CommutationTableRO[RotorSector];
        OpenH = (NextPhaseState>>4) & 0x0F;
        OpenL = (NextPhaseState) & 0x0F;
        if(OpenH & OpenL)
        {
            OpenH = OpenL = 0;  //换向表不对导致计算出上下桥直通的情况,强制全部输出0
        }
    }
    pDrv->Commutator->OpenH = OpenH;
    pDrv->Commutator->OpenL = OpenL;
    pDrv->Commutator->CloseHL = ~(OpenH | OpenL) & 0x0F;
    pDrv->Commutator->RotorSector = RotorSector;
}
/*驱动器根据转子所在扇区控制对应MOS输出*/ //hz foc
static void mc_drv_Update_PWM_Duty(MC_DRV_Handle_t *pDrv, int16_t PWMDuty)
{
    /*关闭不导通相的一组上下桥*/
    for(uint8_t i = 0; i < pDrv->PhaseNb; i++)
    {
        if((pDrv->Commutator->CloseHL>>i) & 0x01)
        {
            mc_drv_MOS_L_Close(&pDrv->MosL[i]);
            mc_drv_MOS_H_Close(&pDrv->MosH[i]);
        }
    }
    /*开启导通相的对角上下桥*/
    for(uint8_t i = 0; i < pDrv->PhaseNb; i++)
    {
        if((pDrv->Commutator->OpenH>>i) & 0x01)
        {
            mc_drv_MOS_L_Close(&pDrv->MosL[i]);
            mc_drv_MOS_H_Open(&pDrv->MosH[i], PWMDuty);
        }
        if((pDrv->Commutator->OpenL>>i) & 0x01)
        {
            mc_drv_MOS_H_Close(&pDrv->MosH[i]);
            mc_drv_MOS_L_Open(&pDrv->MosL[i], PWM_OUTPUT_FULLSCALE);
        }
    }
}
/*驱动器换向(换向后需保持当前输出占空比不变,HALL信号跳变时刻调用)*/
void mc_drv_Commutation(MC_DRV_Handle_t *pDrv)
{
    if(pDrv->State == DRIVER_STATE_RUN)
    {
        mc_drv_Update_PWM_Duty(pDrv, pDrv->PWMDuty);
    }
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 马达控制-MOS三相逆变驱动器-状态机

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*获取驱动器状态机状态*/
uint16_t mc_drv_Get_State(MC_DRV_Handle_t *pDrv)
{
    return pDrv->State;
}
/*状态机状态跳变判断*/
static void mc_drv_State_Jump(MC_DRV_Handle_t *pDrv, uint16_t NextState)
{
    uint16_t CurrentState = pDrv->State;
    uint8_t StateIndex = 0;

    /*找到当前状态对应的索引*/
    for(StateIndex = 1; StateIndex < DRIVER_STATE_NB; StateIndex++)
    {
        if(!(CurrentState >> StateIndex))
        {
            break;
        }
    }
    /*判断目标跳转是已定义的合法状态跳转*/
    if(NextState & g_DrvStateJumpTbl[StateIndex - 1])
    {
        pDrv->State = NextState;
    }
}
/*状态机动作*/
void mc_drv_StateMachine(MC_DRV_Handle_t *pDrv)
{
#if (MC_DRV_TEST_TYPE == 0)// hz/*0-测电机驱动API及状态机功能, 1-只测电机驱动端口信号*/
    switch(pDrv->State)
    {
        case DRIVER_STATE_IDLE://hz 
            if(pDrv->Cmd == e_mdc_output_forward)
            {
                pDrv->Cmd = e_mdc_none;
				pDrv->SetDir = e_forward;
                mc_drv_State_Jump(pDrv, DRIVER_STATE_IDLE_START);
                return;
            }
			if(pDrv->Cmd == e_mdc_output_reverse)
            {
                pDrv->Cmd = e_mdc_none;
				pDrv->SetDir = e_reverse;
                mc_drv_State_Jump(pDrv, DRIVER_STATE_IDLE_START);
                return;
            }
            if(pDrv->Cmd == e_mdc_stop)
            {
                pDrv->Cmd = e_mdc_none;
				pDrv->SetDir = e_nooutput;
                return;
            }
            break;

        case DRIVER_STATE_IDLE_START://hz 
            mc_drv_State_Jump(pDrv, DRIVER_STATE_START_CHARGE_BOOT);
            break;

        case DRIVER_STATE_START_CHARGE_BOOT: //hz
			for(uint8_t i = 0; i < pDrv->PhaseNb; i++)
			{
				mc_drv_MOS_L_Open(&pDrv->MosL[DRIVER_PHASE_A + i], 100);
			}
			mc_drv_Delay_Timer_Start(pDrv);
			mc_drv_State_Jump(pDrv, DRIVER_STATE_START_WAIT_CHARGE_BOOT);
            break;

        case DRIVER_STATE_START_WAIT_CHARGE_BOOT:// hz
            if(pDrv->DelayTimerCnt >= BOOT_CHARGE_TIME)
            {
                mc_drv_Delay_Timer_Stop(pDrv);
                mc_drv_State_Jump(pDrv, DRIVER_STATE_START_CLOSE_3L);
            }
            break;

        case DRIVER_STATE_START_CLOSE_3L://hz
            for(uint8_t i = 0; i < pDrv->PhaseNb; i++)
            {
                mc_drv_MOS_L_Close(&pDrv->MosL[DRIVER_PHASE_A + i]);
            }
            mc_drv_Delay_Timer_Start(pDrv);
            mc_drv_State_Jump(pDrv, DRIVER_STATE_START_WAIT_CLOSE_3L);
            break;

        case DRIVER_STATE_START_WAIT_CLOSE_3L://hz
            if(pDrv->DelayTimerCnt >= MOS_OP_DELAY)
            {
                mc_drv_Delay_Timer_Stop(pDrv);
                mc_drv_State_Jump(pDrv, DRIVER_STATE_START_OPEN_MOS);
            }
            break;

        case DRIVER_STATE_START_OPEN_MOS://hz
			mc_drv_Set_Motor_Rotor_Sector(pDrv, pDrv->Commutator->RotorSector);
			mc_drv_Update_PWM_Duty(pDrv, pDrv->PWMDuty);//hz
			mc_drv_State_Jump(pDrv, DRIVER_STATE_RUN);
            break;

        case DRIVER_STATE_RUN:
			if(pDrv->Cmd == e_mdc_output_forward)
			{
				pDrv->Cmd = e_mdc_none;				
				pDrv->SetDir = e_forward;
			}
			if(pDrv->Cmd == e_mdc_output_reverse)
			{
				pDrv->Cmd = e_mdc_none;				
				pDrv->SetDir = e_reverse;
			}
            if(pDrv->Cmd == e_mdc_stop)
            {
                pDrv->Cmd = e_mdc_none;
				pDrv->SetDir = e_nooutput;
                mc_drv_State_Jump(pDrv, DRIVER_STATE_STOP_CLOSE_3HL);
                return;
            }
            break;

        case DRIVER_STATE_STOP_CLOSE_3HL://hz 关闭所有mos
            for(uint8_t i = 0; i < pDrv->PhaseNb; i++)
            {
                /*同时关闭上下桥,让驱动器处于无输出状态(电机自由停止)*/
                mc_drv_MOS_H_Close(&pDrv->MosH[DRIVER_PHASE_A + i]);
                mc_drv_MOS_L_Close(&pDrv->MosL[DRIVER_PHASE_A + i]);
            }
            mc_drv_Delay_Timer_Start(pDrv);
            mc_drv_State_Jump(pDrv, DRIVER_STATE_STOP_WAIT_CLOSE_3HL);
            break;

        case DRIVER_STATE_STOP_WAIT_CLOSE_3HL:
            if(pDrv->DelayTimerCnt >= STOP_RELEASE_TIME)    //让驱动器处于无输出状态持续的时间长一点再进入刹车,抑制反电动势
            {
                mc_drv_Delay_Timer_Stop(pDrv);
                mc_drv_State_Jump(pDrv, DRIVER_STATE_STOP_OPEN_3L);
            }
            break;

        case DRIVER_STATE_STOP_OPEN_3L:
            for(uint8_t i = 0; i < pDrv->PhaseNb; i++)
            {
                mc_drv_MOS_L_Open(&pDrv->MosL[DRIVER_PHASE_A + i], PWM_OUTPUT_FULLSCALE);
            }
            mc_drv_Delay_Timer_Start(pDrv);
            mc_drv_State_Jump(pDrv, DRIVER_STATE_STOP_BRAKE);
            break;

        case DRIVER_STATE_STOP_BRAKE:
            if(pDrv->DelayTimerCnt >= STOP_BRAKE_TIME)  //至少刹车100ms再进入IDLE,抑制停止惯性
            {
                mc_drv_Delay_Timer_Stop(pDrv);
                mc_drv_State_Jump(pDrv, DRIVER_STATE_IDLE);
            }
            break;

        default:
            break;
    }
#endif
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 马达控制-MOS三相逆变驱动器-模块功能测试

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
void mc_drv_Test(MC_DRV_Handle_t *pDrv1, MC_DRV_Handle_t *pDrv2)
{
#if (MC_DRV_TEST_TYPE == 1)
    /*MOSL(GPIO)端口测试*/
    if(g_TestMotorCmd == 1)
    {
        g_TestMotorCmd = 0;

        /*先关闭上桥*/
        for(uint8_t i=0; i<pDrv1->PhaseNb; i++)
        {
            mc_drv_MOS_H_Close(&pDrv1->MosH[DRIVER_PHASE_A + i]);
        }
        /*测试下桥开关*/
        for(uint8_t i=0; i<pDrv1->PhaseNb; i++)
        {
            mc_drv_MOS_L_Open(&pDrv1->MosL[DRIVER_PHASE_A + i],0);
            mc_drv_MOS_L_Close(&pDrv1->MosL[DRIVER_PHASE_A + i]);
        }
    }
    /*MOSH(PWM)端口测试*/
    if(g_TestMotorCmd == 2)
    {
        g_TestMotorCmd = 0;

        /*先关闭下桥*/
        for(uint8_t i=0; i<pDrv1->PhaseNb; i++)
        {
            mc_drv_MOS_L_Close(&pDrv1->MosL[DRIVER_PHASE_A + i]);
        }
        /*测试上桥开关*/
        for(uint8_t i=0; i<pDrv1->PhaseNb; i++)
        {
            mc_drv_MOS_H_Open(&pDrv1->MosH[DRIVER_PHASE_A + i], 400);
            delay_ms(3000);
            mc_drv_MOS_H_Close(&pDrv1->MosH[DRIVER_PHASE_A + i]);
        }
    }
    /*MOSH(PWM)端口测试-上电输占空比,给硬件调试*/
    if(g_TestMotorCmd == 3)
    {
        g_TestMotorCmd = 0;

        /*先关闭下桥*/
        for(uint8_t i=0; i<pDrv1->PhaseNb; i++)
        {
            mc_drv_MOS_L_Close(&pDrv1->MosL[DRIVER_PHASE_A + i]);
        }
        /*测试上桥开关*/
        for(uint8_t i=0; i<pDrv1->PhaseNb; i++)
        {
            mc_drv_MOS_H_Open(&pDrv1->MosH[DRIVER_PHASE_A + i], 400);
            delay_ms(1000);
        }
    }
    /*逐相开通,给硬件调试*/
    if(g_TestMotorCmd == 4)
    {
        //g_TestMotorCmd = 0;

        /*先关闭下桥*/
        for(uint8_t i=0; i<pDrv1->PhaseNb; i++)
        {
            mc_drv_MOS_L_Close(&pDrv1->MosL[DRIVER_PHASE_A + i]);
        }
        /*逐相开通(无刷电机)*/
        if(pDrv1->PhaseNb == 3)
        {
            mc_drv_MOS_H_Close(&pDrv1->MosH[DRIVER_PHASE_C]);
            mc_drv_MOS_L_Close(&pDrv1->MosL[DRIVER_PHASE_C]);
            mc_drv_MOS_H_Open(&pDrv1->MosH[DRIVER_PHASE_A], 400);   //A+ B-
            mc_drv_MOS_L_Open(&pDrv1->MosL[DRIVER_PHASE_B]);
            delay_ms(5000);
            mc_drv_MOS_H_Close(&pDrv1->MosH[DRIVER_PHASE_A]);
            mc_drv_MOS_L_Close(&pDrv1->MosL[DRIVER_PHASE_A]);
            mc_drv_MOS_H_Open(&pDrv1->MosH[DRIVER_PHASE_B], 400);   //B+ C-
            mc_drv_MOS_L_Open(&pDrv1->MosL[DRIVER_PHASE_C]);
            delay_ms(5000);
            mc_drv_MOS_H_Close(&pDrv1->MosH[DRIVER_PHASE_B]);
            mc_drv_MOS_L_Close(&pDrv1->MosL[DRIVER_PHASE_B]);
            mc_drv_MOS_H_Open(&pDrv1->MosH[DRIVER_PHASE_C], 400);   //C+ A-
            mc_drv_MOS_L_Open(&pDrv1->MosL[DRIVER_PHASE_A]);
            delay_ms(5000);
        }
    }
    /*逐相开通,给硬件调试*/
    if(g_TestMotorCmd == 5)
    {
        //g_TestMotorCmd = 0;

        /*逐相开通(无刷电机)*/
        mc_drv_MOS_H_Open(&pDrv1->MosH[DRIVER_PHASE_A], 400);
        mc_drv_MOS_H_Close(&pDrv1->MosH[DRIVER_PHASE_A]);
        mc_drv_MOS_H_Open(&pDrv1->MosH[DRIVER_PHASE_B], 400);
        mc_drv_MOS_H_Close(&pDrv1->MosH[DRIVER_PHASE_B]);
    }
#else
    /*以下测试指令用于测试1个马达驱动,状态机逻辑是否正确*/
    /*驱动器1*/
    if(g_TestMotorCmd == 1)
    {
        g_TestMotorCmd = 0;
        mc_drv_SetCmd(pDrv1, e_mdc_output_forward);
    }
	if(g_TestMotorCmd == 2)
    {
        g_TestMotorCmd = 0;
        mc_drv_SetCmd(pDrv1, e_mdc_output_reverse);
    }
    if(g_TestMotorCmd == 3)
    {
        g_TestMotorCmd = 0;
        mc_drv_SetCmd(pDrv1, e_mdc_stop);
    }
    if(g_TestMotorCmd == 4)
    {
        g_TestMotorCmd = 0;
        mc_drv_SetPWMDuty(pDrv1, g_TestSetPWMDuty);
    }
    /*驱动器2*/
    if(g_TestMotorCmd == 5)
    {
        g_TestMotorCmd = 0;
        mc_drv_SetCmd(pDrv2, e_mdc_output_forward);
    }
	if(g_TestMotorCmd == 6)
    {
        g_TestMotorCmd = 0;
        mc_drv_SetCmd(pDrv2, e_mdc_output_reverse);
    }
    if(g_TestMotorCmd == 7)
    {
        g_TestMotorCmd = 0;
        mc_drv_SetCmd(pDrv2, e_mdc_stop);
    }
    if(g_TestMotorCmd == 8)
    {
        g_TestMotorCmd = 0;
        mc_drv_SetPWMDuty(pDrv2, g_TestSetPWMDuty);
    }
    /*以下测试指令用于测试2个马达驱动器,状态机逻辑是否正确*/
    /*驱动器1+2*/
    if(g_TestMotorCmd == 9)
    {
        g_TestMotorCmd = 0;
        mc_drv_SetCmd(pDrv1, e_mdc_output_forward);
        mc_drv_SetCmd(pDrv2, e_mdc_output_forward);
    }
	if(g_TestMotorCmd == 10)
    {
        g_TestMotorCmd = 0;
        mc_drv_SetCmd(pDrv1, e_mdc_output_reverse);
        mc_drv_SetCmd(pDrv2, e_mdc_output_reverse);
    }
    if(g_TestMotorCmd == 11)
    {
        g_TestMotorCmd = 0;
        mc_drv_SetCmd(pDrv1, e_mdc_stop);
        mc_drv_SetCmd(pDrv2, e_mdc_stop);
    }
    if(g_TestMotorCmd == 12)
    {
        g_TestMotorCmd = 0;
        mc_drv_SetPWMDuty(pDrv1, g_TestSetPWMDuty);
        mc_drv_SetPWMDuty(pDrv2, g_TestSetPWMDuty);
    }
    if(g_TestMotorCmd == 13)
    {
        g_TestMotorCmd = 0;
        mc_drv_SetCmd(pDrv1, e_mdc_output_forward);
        mc_drv_SetCmd(pDrv2, e_mdc_stop);
    }
    if(g_TestMotorCmd == 14)
    {
        g_TestMotorCmd = 0;
        mc_drv_SetCmd(pDrv1, e_mdc_stop);
        mc_drv_SetCmd(pDrv2, e_mdc_output_forward);
    }
	if(g_TestMotorCmd == 15)
    {
        g_TestMotorCmd = 0;
        mc_drv_SetCmd(pDrv1, e_mdc_output_reverse);
        mc_drv_SetCmd(pDrv2, e_mdc_stop);
    }
    if(g_TestMotorCmd == 16)
    {
        g_TestMotorCmd = 0;
        mc_drv_SetCmd(pDrv1, e_mdc_stop);
        mc_drv_SetCmd(pDrv2, e_mdc_output_reverse);
    }
#endif
}
