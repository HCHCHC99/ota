/********************************文件说明*************************************
*文件名: mc_app.c

*作者: Yuchen Tan

*版本: V1.0.5

*功能简介:
*1.单电机应用功能程序(开环启停/闭环启停/运行至目标位置);
*2.多电机应用功能程序(同步启停/同步运行至目标位置);

*备注:

*修改履历:
------------------------------------V1.0.1------------------------------------
20220613:
1.HALL异常停止增加条件if(pMC->MCEnableFunc & HAP_EN),防止已有的HALL异常导致开环启动不了;
2.命名优化PMC_MMC(Multi Motor Control) -> PMGC(Motor Group Control);
------------------------------------V1.0.2------------------------------------
20220709:
1.增加适配HALL计步器使用单HALL的初始化代码.
2.增加适配HALL异常检测器检测HALL电平异常的初始化代码.
3.增加FaultFlag指示各种电机运行时异常,上层;
4.给应用层的接口统一为控制命令,设置参数,读取参数命令(接口功能未测试);
20220718:
1.HALL异常检测放到250us定时中断中调用(见mc_hall-V1.0.4-20220718修改日志)
------------------------------------V1.0.3------------------------------------
20220721:
1.MotorState从e_mac_stop->e_mas_idle的条件由无条件改为驱动器状态变为DRIVER_STATE_IDLE.
2.Todo: 电机命令接口内部改为缓存当前指令,并在状态机内部进行动作命令的加载执行!
3.同步指令MOTOR_SYNC_GOTO_TARGET_POS改为可连续响应不同位置,目标位置更新不需要停止.
20220810:
1.删除函数mc_app_Set_OpenLoop_Parameter()和mc_app_Set_CloseLoop_Parameter().
2.增加参数e_map_fdbkspd(R).
20220819:修复bug: 多电机控制器在e_mas_idle状态下直接调用MOTOR_SYNC_STOP,会导致g_UseMGCNb
自加1,当自加超过MGC_NB时,多电机控制器无法使用! g_UseMGCNb的使用属画蛇添足,删除即可!
20220824:
1.mc_app_Read_Param()接口补充e_map_ocp_thh参数;
2.mc_cur_Set_OVCTHH(handle, MOTOR_OVC_VALUE)从mc_app_Single_Motor_Controller()挪到
mc_app_Init()中,否则应用程序无法有效修改过流阈值.
------------------------------------V1.0.4------------------------------------
20220921:
1.删除过热保护检测相关代码(如需过热保护功能,请在应用层代码自行实现).
2.电机启动初始化状态的代码放到mc_app_Init()中;
20220922:
1.mc_app_Write_Param()和mc_app_Read_Param()接口增加参数,配合mc_spd,mc_pos改动,实现:
-电机开环运行实时调目标占空比;
-电机速度闭环,位置闭环运行实时调目标速度.
2.完善以下接口返回值:
int8_t mc_app_Set_Single_Motor_Cmd(uint16_t Motor, uint16_t Cmd, int32_t argv);
int8_t mc_app_Set_Multi_Motor_Cmd(uint16_t MotorGroup, uint16_t Cmd, int32_t argv);
int8_t mc_app_Write_Param(uint16_t Motor, uint16_t WhichParam, int32_t argv);
int8_t mc_app_Read_Param(uint16_t Motor, uint16_t WhichParam, int32_t* argv);
20221007:
1.针对多电机控制器的修改:
-重写接口mc_app_Get_MGC_State(uint16_t MotorGroup),实现多电机控制器状态获取;
-优化函数mc_app_Judege_MGC_Valid(uint16_t MotorGroup),实现参数MotorGroup非法判断;
-优化部分函数名称;
*20221021:
1.修改hall计步器状态更新和halldata值更新重入的问题,禁止在250us中断和hall捕捉中断
  中都调用hall状态和halldata更新函数(重入会导致halldata的值错误,产生丢步);
2.修改速度闭环/位置闭环运行时,直接切换电机方向可能导致误触发hall异常(切换方向后,没
  有等驱动器状态变为IDLE状态MotorCtrlStep就加上去了并开启hall异常检测,到达检测时间
  时,闭环输出的占空比还没加上去,电机没转起来导致误触发hall异常);
3.修改hall超速检测的速度阈值为(MAX_RPM + 1000),因为MAX_RPM是电机位置环正常运行
  中可能达到的速度.
4.调整无刷电机换相逻辑:
  启动XXXms内,在250us定时中断中换向;
  启动XXXms后,在hall边沿中断中换向(换相函数重入可能会导致换向换错);
------------------------------------V1.0.5------------------------------------
20230220->20230309:
1.配合mc_config.h文件的V1.0.2修改,详见mc_config.h修改履历;
2.移除mc_app_Init()中母线电压/温度采样中使用adc_adapter模块的相关代码,移到应用层.
  目前认为母线电压/温度不属于MotorControl功能模块的组件;
  (注: 后面增加mc_bv.c/h母线电压组件后可把bv采样代码移回MotorCoontrol.温度不需要)
20230228->20230309: 
1.修复重要bug: 增加g_MCBootInitCplt,防止mc_app_Init()执行完成前有空指针的风险.
------------------------------------V1.0.6------------------------------------
20230901:增加设置hall方向和drv方向的接口.
*****************************************************************************/

/*****************************文件包含(私有)**********************************
*
*备注: 无
*
*****************************************************************************/
#include "mc_app.h"
#include "mc_drv.h"
#include "mc_hall.h"
#include "mc_spd.h"
#include "mc_pos.h"
#include "mc_cur.h"
#include "main.h"
#include "adc_adapter.h"
#include "pid.h"
/*****************************宏定义(私有)************************************
*
*备注: 本文件中,不希望被外部使用或随意修改的宏在这里定义
*
*****************************************************************************/
/*同时最多执行的多电机控制任务数(理论上MGC_NB <= MOTOR_NB / 2)*/
#define MGC_NB          (MOTOR_NB)

/*电机控制器功能组件使能标志定义(位运算)*/
#define SCL_EN          (uint16_t)(1<<0)    //TRUE：速度闭环控制启用     FALSE: 速度开环控制
#define PCL_EN          (uint16_t)(1<<1)    //TRUE：位置闭环控制启用     FALSE: 位置闭环控制禁止
#define MGC_EN          (uint16_t)(1<<2)    //TRUE：多电机协同控制器启用   FALSE: TRUE：多电机协同控制器禁止
#define OCP_EN          (uint16_t)(1<<3)    //TRUE：过流保护启用           FALSE: 过流保护禁止
#define HAP_EN          (uint16_t)(1<<4)    //TRUE：HALL异常保护启用       FALSE: HALL异常保护禁止
#define MC_EN_ALL       (uint16_t)(0xFFFF)

/*无刷电机切向控制(单位: ms)*/
#define BLDC_SP_TIME    (350)   /*三相无刷电机刚启动时在250us定时中断中换向(会产生切向延时),随后在hall变化中断中换相,防止换相重入导致给相错误*/

#define MC_INACCESSIBLE (0xDEAD)
/**************************数据类型及结构定义(私有)***************************
*
*备注: 本文件中,不希望被外部使用的数据结构及类型在这里定义
*
*****************************************************************************/
/*电机控制接口给定参数结构定义*/
typedef struct
{
    int32_t             TargetMotorDir;     //1：上升  -1：下降  0：停止
//  uint16_t            MCFuncEnable;       //电机控制器组件功能开关选择
//  MOTOR_POS_t         TargetPos;          //目标位置
//  MC_CLP_Handle_t     CLP;                /*速度闭环运行参数*/
//  MC_OLP_Handle_t     OLP;                /*速度开环运行参数*/
}MC_Cmd_Param_t;

/*电机控制器结构定义*/
typedef struct
{
    uint16_t            Motor;              //电机索引标号(按位逻辑操作)
    mc_app_cmd_t		MotorCmd;           //电机目标动作指令(响应后清0)
	int32_t            	MotorCmdArgv;		//电机目标动作参数
    uint8_t				MotorCtrlStep;
    mc_app_sta_t		MotorState;         //电机当前动作状态
    uint8_t             MotorFault;         //电机故障汇总
    uint16_t            MotorMoveTime;
    /*MC控制器组件工作选择参数*/
    int32_t             MotorDir;           //1：上升  -1：下降  0：停止
    uint16_t            MCEnableFunc;       //电机控制器组件功能开关选择
    /*MC控制器控制参数(注: 易改变的成员不用指针,否则修改时只能改指针对应的实例.若使用不慎(用局部变量进行初始化)还会导致函数结束后地址被释放!)*/
    MC_CLP_Handle_t     CLP;                /*速度闭环运行参数*/
    MC_OLP_Handle_t     OLP;                /*速度开环运行参数*/
    MOTOR_POS_t         TargetPos;          /*目标位置*/
    BOOL                InheritSpdFlag;     //TRUE：继承当前速度控制参数   FALSE: 速度控制参数需重新初始化
    /*MC控制器各个子模块组件(注: 不易改变的成员用指针,声明全局变量与之绑定,方便访问和修改)*/
    MC_DRV_Handle_t     *pDrv;              /*电机驱动器句柄*/
    HALL_Handle_t       *pHall;             /*电机HALL信号处理器句柄*/
    SPD_CL_Handle_t     *pSpdCL;            /*电机速度闭环控制器句柄*/
    SPD_OL_Handle_t     *pSpdOL;            /*电机速度开环控制器句柄*/
    POS_CL_Handle_t     *pPosCL;            /*电机位置闭环控制器句柄*/
    CUR_Handle_t        *pCur;              /*电机电流采样控制器句柄*/
}MC_Handle_t;

/*多电机协同控制器结构定义*/
typedef struct
{
    uint16_t            MotorGroup;                 //多电机索引标号(按位逻辑操作)
    uint8_t             MotorGroupControlNb;        //同时控制的电机个数
    mc_app_sta_t		MotorGroupState;
    uint16_t            MotorGroupCtrlStep;
    /*MC多电机协同控制器工作参数*/
    int32_t             MotorGroupDir;              //电机运行方向
    MOTOR_POS_t         SyncTargetPos;              //同步目标位置
    MC_Handle_t         *pMC[MOTOR_NB];             //电机控制器句柄数组指针,用于索引关联的多个电机
    PID_WZ_Handle_t     *pSyncLoopPIDWZ;            //同步控制器差速控制环(做P控制)
}MGC_Handle_t;
/*****************************函数声明(私有)**********************************
*
*备注: 本文件中,不希望被外部调用的函数统一在这里声明
*
*****************************************************************************/
static MC_Handle_t* mc_app_Get_Handle(uint16_t Motor);
static MC_Handle_t* mc_app_MGC_Get_Specified_Pos(MGC_Handle_t *pMGC, uint8_t Type);
/********************************变量定义*************************************
*
*备注: 不要在.h中定义变量,防止多个.c包含这个.h,导致编译后报错变量重复定义
*
*****************************************************************************/
//MC组件初始化完成标志
//注: 防止mc_app_Init()初始化各个电机相关组件(其中有IO端口),应用程序先开启mcu中断外
//设,导致mc相关中断任务执行,结构体里面都是空指针.(eg: pHall->Hall[i]未初始化HALL引脚)
uint16_t                    g_MCBootInitCplt = MC_INACCESSIBLE;

/*MC所有组件定义*/
MC_Handle_t                 hMC[MOTOR_NB];

COMMUTATOR_t                hCmt[MOTOR_NB];
MC_DRV_Handle_t             hMCDrv[MOTOR_NB];

HALL_Handle_t               hMCHall[MOTOR_NB];
HALL_PEDOMETER_Handle_t     hMCHallPedometer[MOTOR_NB];
HALL_SPDMEAS_M_Handle_t     hMCHallSpdMeasureM[MOTOR_NB];
HALL_SPDMEAS_T_Handle_t     hMCHallSpdMeasureT[MOTOR_NB];
HALL_ABN_CHECKER_Handle_t   hMCHallAbnChecker[MOTOR_NB];

SPD_OL_Handle_t             hMCSpeedOL[MOTOR_NB];
SPD_CL_Handle_t             hMCSpeedCL[MOTOR_NB];
PID_ZL_Handle_t             hMCSpeedPID_ZL[MOTOR_NB];
PID_WZ_Handle_t             hMCSpeedPID_WZ[MOTOR_NB];
SPD_RAMP_Handle_t           hMCSpeedRamp[MOTOR_NB];

POS_CL_Handle_t             hMCPosCL[MOTOR_NB];
PID_WZ_Handle_t             hMCPosLoopPID_WZ[MOTOR_NB];
POS_RAMP_Handle_t           hMCPosRamp[MOTOR_NB];

CUR_Handle_t                hMCCur[MOTOR_NB];
ADC_CH_CTRL_t               hADCChannalMCurrent[MOTOR_NB];

MGC_Handle_t                hMGC[MGC_NB];
PID_WZ_Handle_t             hMGCSyncLoop[MGC_NB];

/*以下为测试用变量*/
uint8_t     g_TestToggle = 0;
uint16_t    g_TestMCAppCmd = 0;
int32_t		g_TestTargetSpeed = 0;
MOTOR_POS_t g_TestTargetPos = 0;
MOTOR_POS_t g_WatchMotorCurrentPos[MOTOR_NB];
MOTOR_SPD_t g_WatchMotorCurrentSpd[MOTOR_NB];
MOTOR_SPD_t g_WatchMotorTargetSpd[MOTOR_NB];
MOTOR_POS_t g_WatchPosDValue = 0;
//int32_t g_TestLoopDiv = SPD_LOOP_PID_WZ_DIV;
//int32_t   g_TestLoopKp = 0;
//int32_t   g_TestLoopKi = 0;
//int32_t   g_TestLoopKd = 0;
/********************************函数定义************************************
*函数名:

*函数功能描述: 马达控制-马达功能控制器-单电机控制器

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*单电机控制器-句柄初始化*/
static void mc_app_hInit(   MC_Handle_t         *pMC,
                            uint16_t            Motor,
                            MC_CLP_Handle_t     *CLP,
                            MC_OLP_Handle_t     *OLP,
                            MC_DRV_Handle_t     *pDrv,
                            HALL_Handle_t       *pHall,
                            SPD_CL_Handle_t     *pSpdCL,
                            SPD_OL_Handle_t     *pSpdOL,
                            POS_CL_Handle_t     *pPosCL,
                            CUR_Handle_t        *pCur)
{
    pMC->Motor = Motor;
    pMC->MotorState = e_mas_idle;
    pMC->MotorCtrlStep = 0;
    pMC->MotorFault = MOTOR_NO_FAULT;

    pMC->MotorDir = DIR_STOP;
    pMC->MCEnableFunc = 0;

    pMC->CLP = *CLP,
    pMC->OLP = *OLP,

    pMC->TargetPos = 0;
    pMC->InheritSpdFlag = FALSE;

    pMC->pDrv = pDrv;
    pMC->pHall = pHall;
    pMC->pSpdCL = pSpdCL;
    pMC->pSpdOL = pSpdOL;
    pMC->pPosCL = pPosCL;
    pMC->pCur = pCur;
}
/*单电机控制器-获取句柄(Motor必须是单个电机的索引)*/
static MC_Handle_t* mc_app_Get_Handle(uint16_t Motor)
{
    for(uint8_t i=0; i<MOTOR_NB; i++)
    {
        if(Motor & (MOTOR1<<i))
        {
            return &hMC[i];
        }
    }
    return NULL;
}
/*单电机控制器-电机运行计数器*/
static void mc_app_MotorRun_Timer(MC_Handle_t *pMC)
{
    if(pMC->MotorState == e_mas_idle)
    {
        pMC->MotorMoveTime = 0;
    }else
    {
        if(pMC->MotorMoveTime < 60000)
        {
            pMC->MotorMoveTime++;
        }
    }
}
/*单电机控制器-获取控制器状态*/
mc_app_sta_t mc_app_Get_State(uint16_t Motor)
{
    MC_Handle_t *pMC = mc_app_Get_Handle(Motor);
    return pMC->MotorState;
}
/*单电机控制器-置位控制器组件使能开关*/
static void mc_app_Set_MCFlag(MC_Handle_t *pMC, uint16_t EnFuncMask)
{
    pMC->MCEnableFunc |= EnFuncMask;
}
/*单电机控制器-清除控制器组件使能开关*/
static void mc_app_Clr_MCFlag(MC_Handle_t *pMC, uint16_t EnFuncMask)
{
    pMC->MCEnableFunc &= (~EnFuncMask);
}
/*写电机参数*/
int8_t mc_app_Write_Param(uint16_t Motor, mc_app_param_t WhichParam, int32_t argv)
{
    MC_Handle_t *pMC = mc_app_Get_Handle(Motor);

    /*参数合法判断*/
    if(!pMC)
        return MC_RET_ERR_MOTOR;
    if(WhichParam >= map_ValidCheck)
        return MC_RET_PARAM_NOT_ACCESS;
    /*写参数*/
    if(WhichParam == e_map_fault)     //清除故障操作,argv为任意值均可
    {
        pMC->MotorFault = 0;
        mc_cur_Clear_OVC_Flag(pMC->pCur);
        mc_hall_Clear_Hall_Abnormal(pMC->pHall->pHAC);
    }else if(WhichParam == e_map_fdbkpos)
    {
        mc_hall_Set_HallData(pMC->pHall->pHP, (MOTOR_POS_t)argv);
    }else if(WhichParam == e_map_targetspd)
    {
        pMC->CLP.TargetSpd = (MOTOR_SPD_t)argv;
        mc_spd_Ramp_Set_TargetSpeed(pMC->pSpdCL->pSpdRamp, (MOTOR_SPD_t)argv * pMC->MotorDir);
        mc_pos_Set_Ramp_TargetSpeed(pMC->pPosCL->pPosRamp, (MOTOR_SPD_t)argv * pMC->MotorDir);
    }else if(WhichParam == e_map_acc)
    {
        pMC->CLP.SpdUpAcc = (MOTOR_SPD_t)argv;
        mc_spd_Ramp_Set_Acceleration(pMC->pSpdCL->pSpdRamp, (MOTOR_SPD_t)argv * pMC->MotorDir);
    }else if(WhichParam == e_map_startspd)
    {
        pMC->CLP.StartSpd = (MOTOR_SPD_t)argv;
    }else if(WhichParam == e_map_stopspd)
    {
        pMC->CLP.StopSpd = (MOTOR_SPD_t)argv;
    }else if(WhichParam == e_map_ol_targetdc)
    {
        pMC->OLP.TargetDCPercent = (int16_t)argv;
        mc_spd_Set_OpenLoop_TargetDutyCycle(pMC->pSpdOL, (int16_t)argv * pMC->MotorDir);
    }else if(WhichParam == e_map_drvoutput_max)
    {
        mc_drv_SetPWMDutyMax(pMC->pDrv, (int16_t)argv);
    }else if(WhichParam == e_map_ocp_thh)
    {
        mc_cur_Set_OVCTHH(pMC->pCur, (uint16_t)argv);
    }else
    {
        return MC_RET_PARAM_NOT_ACCESS;
    }
    return MC_RET_OK;
}
/*读电机参数*/
int8_t mc_app_Read_Param(uint16_t Motor, mc_app_param_t WhichParam, int32_t* argv)
{
    MC_Handle_t *pMC = mc_app_Get_Handle(Motor);

    /*参数合法判断*/
    if(!pMC)
        return MC_RET_ERR_MOTOR;
    if(WhichParam >= map_ValidCheck)
        return MC_RET_PARAM_NOT_ACCESS;
    /*读参数*/
    if(WhichParam == e_map_state)
    {
        *argv = pMC->MotorState;
    }else if(WhichParam == e_map_dir)
    {
        *argv = pMC->MotorDir;
    }else if(WhichParam == e_map_fault)
    {
        *argv = pMC->MotorFault;
    }else if(WhichParam == e_map_fdbkpos)
    {
        *argv = mc_hall_Get_HallData(pMC->pHall->pHP);
    }else if(WhichParam == e_map_targetspd)
    {
        *argv = pMC->CLP.TargetSpd;
    }else if(WhichParam == e_map_fdbkspd)
    {
        *argv = mc_hall_Get_MeasureSpeed_M(pMC->pHall->pHSMM);
    }else if(WhichParam == e_map_acc)
    {
        *argv = pMC->CLP.SpdUpAcc;
    }else if(WhichParam == e_map_startspd)
    {
        *argv = pMC->CLP.StartSpd;
    }else if(WhichParam == e_map_stopspd)
    {
        *argv = pMC->CLP.StopSpd;
    }else if(WhichParam == e_map_ol_targetdc)
    {
        *argv = pMC->OLP.TargetDCPercent;
    }else if(WhichParam == e_map_current)
    {
        *argv = mc_cur_Get_Current_Value(pMC->pCur);
    }else if(WhichParam == e_map_ocp_thh)
    {
        *argv = mc_cur_Get_OVCTHH(pMC->pCur);
    }else
    {
        return MC_RET_PARAM_NOT_ACCESS;
    }
    return MC_RET_OK;
}
/*单电机控制器-单电机动作命令接口(含控制参数初始化)*/
int8_t mc_app_Set_Single_Motor_Cmd(uint16_t Motor, mc_app_cmd_t Cmd, int32_t argv)
{
    MC_Handle_t *pMC = mc_app_Get_Handle(Motor);

    if(!pMC)
        return MC_RET_ERR_MOTOR;
	if(Cmd > e_mac_goto_targetpos)
		return MC_RET_ERR_CMD;
	pMC->MotorCmd = Cmd;
	pMC->MotorCmdArgv = argv;
    return MC_RET_OK;
}
/*单电机控制器-单电机动作命令接口(含控制参数初始化)*/
static int8_t mc_app_Load_Single_Motor_Cmd(MC_Handle_t *pMC)
{
	uint16_t Cmd = pMC->MotorCmd;        //电机目标动作指令(响应后清0)
	int32_t argv = pMC->MotorCmdArgv;    //电机目标动作参数
	
	if(Cmd == 0)
		return MC_RET_ERR_CMD;
    if(e_mas_idle == pMC->MotorState || e_mas_pseudo_idle == pMC->MotorState)
    {
        if(Cmd == e_mac_start_closeloop)
        {
			pMC->MotorCmd = e_mac_none;
            pMC->MotorState = (mc_app_sta_t)Cmd;
            pMC->MotorCtrlStep = 0;
            pMC->MotorDir = argv;
            mc_app_Clr_MCFlag(pMC, MC_EN_ALL);      //hz 禁用所有功能
            mc_app_Set_MCFlag(pMC, SCL_EN | OCP_EN | HAP_EN); //hz  速度闭环控制启用 | 过流保护启用 | HALL异常保护启用
        }else if(Cmd == e_mac_start_openloop)
        {
			pMC->MotorCmd = e_mac_none;
            pMC->MotorState = (mc_app_sta_t)Cmd;
            pMC->MotorCtrlStep = 0;
            pMC->MotorDir = argv;
            mc_app_Clr_MCFlag(pMC, MC_EN_ALL);
            mc_app_Set_MCFlag(pMC, OCP_EN | HAP_EN);
        }else if(Cmd == e_mac_goto_targetpos)
        {
			pMC->MotorCmd = e_mac_none;
            pMC->MotorState = (mc_app_sta_t)Cmd;
            pMC->MotorCtrlStep = 0;
            pMC->TargetPos = (MOTOR_POS_t)argv;
            mc_app_Clr_MCFlag(pMC, MC_EN_ALL);
        #if 0
            mc_app_Set_MCFlag(pMC, SCL_EN | OCP_EN | HAP_EN);
        #else
            mc_app_Set_MCFlag(pMC, SCL_EN | PCL_EN | OCP_EN | HAP_EN);
        #endif
        }else
        {
            return MC_RET_CMD_NOT_EXEC;
        }
    }else if(e_mas_stopping == pMC->MotorState)
    {
        //此状态不能响应任何马达动作的命令
        return MC_RET_CMD_NOT_EXEC;
    }else if(e_mas_start_closeloop == pMC->MotorState)
    {
        if(Cmd == e_mac_stop || Cmd == e_mac_stop_closeloop)
        {
			pMC->MotorCmd = e_mac_none;
            pMC->MotorState = (mc_app_sta_t)Cmd;
            pMC->MotorCtrlStep = 0;
        }else if(Cmd == e_mac_stop_openloop)
        {
			pMC->MotorCmd = e_mac_none;
            pMC->MotorState = (mc_app_sta_t)Cmd;
            pMC->MotorCtrlStep = 0;
            mc_app_Clr_MCFlag(pMC, MC_EN_ALL);      //失能闭环控制的功能
            mc_app_Set_MCFlag(pMC, OCP_EN);
        }else if(Cmd == e_mac_start_closeloop)
        {
            if(pMC->MotorDir != argv)   //运行中切换方向
            {
				pMC->MotorState = e_mas_stopping;
                pMC->MotorCtrlStep = 0;
            }
        }else
        {
            return MC_RET_CMD_NOT_EXEC;
        }
    }else if(e_mas_stop_closeloop == pMC->MotorState || e_mas_stop_openloop == pMC->MotorState)
    {
        if(Cmd == e_mac_stop)
        {
			pMC->MotorCmd = e_mac_none;
            pMC->MotorState = (mc_app_sta_t)Cmd;
            pMC->MotorCtrlStep = 0;
        }else
        {
            return MC_RET_CMD_NOT_EXEC;
        }
    }else if(e_mas_start_openloop == pMC->MotorState)
    {
        if(Cmd == e_mac_stop || Cmd == e_mac_stop_openloop)
        {
			pMC->MotorCmd = e_mac_none;
            pMC->MotorState = (mc_app_sta_t)Cmd;
            pMC->MotorCtrlStep = 0;
        }else if(Cmd == e_mac_start_openloop)
        {
            if(pMC->MotorDir != argv)   //运行中切换方向
            {
				pMC->MotorState = e_mas_stopping;
                pMC->MotorCtrlStep = 0;
            }
        }else
        {
            return MC_RET_CMD_NOT_EXEC;
        }
    }else if(e_mas_goto_targetpos == pMC->MotorState)
    {
        if(Cmd == e_mac_stop || Cmd == e_mac_stop_closeloop)
        {
			pMC->MotorCmd = e_mac_none;
            pMC->MotorState = (mc_app_sta_t)Cmd;
            pMC->MotorCtrlStep = 0;
        }else if(Cmd == e_mac_stop_openloop)
        {
			pMC->MotorCmd = e_mac_none;
            pMC->MotorState = (mc_app_sta_t)Cmd;
            pMC->MotorCtrlStep = 0;
            mc_app_Clr_MCFlag(pMC, MC_EN_ALL);      //失能闭环控制的功能
            mc_app_Set_MCFlag(pMC, OCP_EN);
        }else if(Cmd == e_mac_goto_targetpos)
        {
            MOTOR_POS_t CurrentPos = mc_hall_Get_HallData(pMC->pHall->pHP);
            //情况1.新目标位置==原目标位置: 不做任何改变;
            //情况2.新目标位置!=原目标位置,但不会导致电机运行方向改变: 1.更新目标位置控制器特征点; 2.继承当前参数(速度,电流,HALL)并同步至速度斜坡控制器.
            //情况3.新目标位置!=原目标位置,且会导致电机运行方向改变: 1.更新目标位置控制器特征点; 2.重置所有电机重启需要初始化的参数(速度,电流,HALL).
            if(argv != pMC->TargetPos)
            {
                if((argv > CurrentPos && pMC->MotorDir == DIR_UP) || (argv < CurrentPos && pMC->MotorDir == DIR_DOWN))
                {
                    pMC->InheritSpdFlag = TRUE;
					pMC->MotorCmd = e_mac_none;
                }else	//情况3
				{
					pMC->MotorState = e_mas_stopping;
					//pMC->MotorCmd = e_mac_none;	保留MotorCmd的值
				}
                pMC->MotorCtrlStep = 0;
                pMC->TargetPos = (MOTOR_POS_t)argv;
            }
        }else
        {
            return MC_RET_CMD_NOT_EXEC;
        }
    }else
    {
        //do nothing
    }
    return MC_RET_OK;
}
/*状态机内部的停止*/
static inline void mc_app_Motor_Stop(MC_Handle_t *pMC)
{
	pMC->MotorState = e_mas_stopping;
	pMC->MotorCtrlStep = 0;
}
/*单电机控制器-控制器*/
static void mc_app_Single_Motor_Controller(MC_Handle_t *pMC)
{
    MOTOR_SPD_t TargetSpeed, HallSpeed;
    #if 1
    MOTOR_POS_t CurrentPos = mc_hall_Get_HallData(pMC->pHall->pHP);
    MOTOR_SPD_t CurrentSpeed = mc_hall_Get_MeasureSpeed_M(pMC->pHall->pHSMM);/*ToTest: 用CurrentSpeed作为InitSpeed,若果和TargetSpeed的符号相反时是否会引发异常!*/
#else
	MOTOR_SPD_t CurrentSpeed = mc_spd_Get_CloseLoop_TargetSpeed(pMC->pSpdCL);/*速度环的目标速度可认为是当前的速度*/
#endif
	int16_t	CurrentDutyCycle = 0;

	/*加载当前Cmd并执行*/
	mc_app_Load_Single_Motor_Cmd(pMC); // 非异常则MotorCtrlStep = 0，若异常则MotorCtrlStep不变

    uint8_t CtrlStep = pMC->MotorCtrlStep; // 为什么要用CtrlStep,可能因为
	
	/*单电机状态控制*/
    switch(pMC->MotorState)
    {
        case e_mas_idle:
            //mc_hall_Set_OneHallDir(pMC->pHall->pHP, HALL_DIR_DECREASE);
            break;

        case e_mas_goto_targetpos:
            if(CtrlStep == 0)
            {
                if(pMC->TargetPos > CurrentPos + 5)
                {   /*上升*/
                    pMC->MotorDir = DIR_UP;
                    mc_drv_SetCmd(pMC->pDrv, e_mdc_output_forward);
                    mc_hall_Set_OneHallDir(pMC->pHall->pHP, HALL_DIR_INCREASE);
                }else if(pMC->TargetPos < CurrentPos - 5)
                {   /*下降*/
                    pMC->MotorDir = DIR_DOWN;
                    mc_drv_SetCmd(pMC->pDrv, e_mdc_output_reverse);
                    mc_hall_Set_OneHallDir(pMC->pHall->pHP, HALL_DIR_DECREASE);
                }else
                {   /*已在目标位置无需动作,结束*/
					mc_app_Motor_Stop(pMC);
                    return;
                }
                pMC->MotorCtrlStep++;
            }else if(CtrlStep == 1)
            {
                if(mc_drv_Get_State(pMC->pDrv) == DRIVER_STATE_RUN)
                {
                    pMC->MotorCtrlStep++;
                    mc_drv_Set_Motor_Rotor_Sector(pMC->pDrv, pMC->pHall->pHP->State);
                    if(pMC->InheritSpdFlag == TRUE) /*方向不变*/
                    {
                        pMC->InheritSpdFlag = FALSE;
                        /*位置控制器初始化*/
                        mc_pos_Ramp_SetInput(pMC->pPosCL->pPosRamp, CurrentSpeed, pMC->CLP.StopSpd * pMC->MotorDir, pMC->CLP.TargetSpd * pMC->MotorDir, pMC->CLP.SpdUpAcc * pMC->MotorDir, CurrentPos, pMC->TargetPos);
                        mc_pos_Reset_Ramp_Controller(pMC->pPosCL->pPosRamp);
                    #if 0   //跑位置环不需要速度斜坡控制器
                        /*速度斜坡控制初始化*/
                        mc_spd_Reset_Ramp_Controller(pMC->pSpdCL->pSpdRamp);
                        mc_spd_Ramp_Controller_Enable(pMC->pSpdCL->pSpdRamp);
                        mc_spd_Ramp_Set_Acceleration(pMC->pSpdCL->pSpdRamp, pMC->CLP.SpdUpAcc * pMC->MotorDir);
                        mc_spd_Ramp_Set_TargetSpeed(pMC->pSpdCL->pSpdRamp, SPD_SWITCH_HPS_TO_RPM(pMC->pPosCL->pPosRamp->vP1));
                        mc_spd_Ramp_Set_InitialSpeed(pMC->pSpdCL->pSpdRamp, CurrentSpeed);
                    #endif
                    }else   /*静止启动 || 切换方向*/
                    {
                        pMC->MotorMoveTime = 0;
						mc_drv_SetPWMDuty(pMC->pDrv, 0);
                        /*位置控制器初始化-位置斜坡控制器*/
                        mc_pos_Ramp_SetInput(pMC->pPosCL->pPosRamp, pMC->CLP.StartSpd * pMC->MotorDir, pMC->CLP.StopSpd * pMC->MotorDir, pMC->CLP.TargetSpd * pMC->MotorDir, pMC->CLP.SpdUpAcc * pMC->MotorDir, CurrentPos, pMC->TargetPos);
                        mc_pos_Ramp_Controller_Enable(pMC->pPosCL->pPosRamp);
                        mc_pos_Reset_Ramp_Controller(pMC->pPosCL->pPosRamp);
                    #if 0
                        /*速度斜坡控制初始化*/
                        mc_spd_Reset_Ramp_Controller(pMC->pSpdCL->pSpdRamp);
                        mc_spd_Ramp_Controller_Enable(pMC->pSpdCL->pSpdRamp);
                        mc_spd_Ramp_Set_Acceleration(pMC->pSpdCL->pSpdRamp, pMC->CLP.SpdUpAcc * pMC->MotorDir);
                        mc_spd_Ramp_Set_TargetSpeed(pMC->pSpdCL->pSpdRamp, SPD_SWITCH_HPS_TO_RPM(pMC->pPosCL->pPosRamp->vP1));
                        mc_spd_Ramp_Set_InitialSpeed(pMC->pSpdCL->pSpdRamp, pMC->CLP.StartSpd * pMC->MotorDir);
                    #endif
                        if(pMC->MCEnableFunc & PCL_EN)
                        {
                            /*位置控制器初始化-位置环*/
                            mc_pos_CloseLoop_Controller_Enable(pMC->pPosCL);
                            mc_pos_Reset_CloseLoop_Controller(pMC->pPosCL);
                            pid_Reset_PID_WZ_Controller(pMC->pPosCL->pPosLoopPIDWZ);
                        }
                        /*速度环-PID控制器初始化*/
                    #if (SPD_LOOP_PID_MODE == 0)    //增量式PID
                        pid_Reset_PID_ZL_Controller(pMC->pSpdCL->pSpdLoopPIDZL);
                    #else                           //位置式PID
                        pid_Reset_PID_WZ_Controller(pMC->pSpdCL->pSpdLoopPIDWZ);
                    #endif
                        /*速度环-速度环控制器初始化*/
                        mc_spd_Reset_CloseLoop_Controller(pMC->pSpdCL);
                        mc_spd_CloseLoop_Controller_Enable(pMC->pSpdCL);
                        /*HALL-脉宽检测初始化(M法)*/
                        mc_hall_Reset_SpeedMeasure_M(pMC->pHall);
                        /*HALL-异常检测初始化*/
                        mc_hall_Reset_Check_Hall_Abnormal(pMC->pHall->pHAC);
                        mc_hall_Check_Hall_Abnormal_Enable(pMC->pHall->pHAC);
                        if(pMC->MotorDir == DIR_UP)
                            mc_hall_Check_Hall_Abnormal_Init(pMC->pHall->pHAC, HALL_DIR_INCREASE, M_MIN_RPM, M_MAX_RPM + 1000, HPR);
                        if(pMC->MotorDir == DIR_DOWN)
                            mc_hall_Check_Hall_Abnormal_Init(pMC->pHall->pHAC, HALL_DIR_DECREASE, M_MIN_RPM, M_MAX_RPM + 1000, HPR);
                        /*电流采样初始化*/
                        mc_cur_Current_Sample_Enable(pMC->pCur);
                        mc_cur_Reset_Current_Sample(pMC->pCur);
                        /*过流保护初始化*/
                        mc_cur_OVC_Enable(pMC->pCur);
                        mc_cur_Reset_OVC(pMC->pCur);
                        mc_cur_Set_OVC_Shield_Time(pMC->pCur, OVC_SHIELD_TIME);
                        pMC->MotorFault = MOTOR_NO_FAULT;
                    }
                }
            }else if(CtrlStep == 2)
            {
                /*到位判断-到达P2点减速(适用于Type1-Type3所有情况)*/
                if(pMC->MCEnableFunc & PCL_EN)
                {
                    //位置环的速度由位置斜坡控制器自行计算,不需要设置,直接下一步
                    pMC->MotorCtrlStep++;
                }else
                {
                    if(TRUE == mc_pos_If_RunTo_TargetPos(pMC->pPosCL->pPosRamp->P2, CurrentPos, pMC->MotorDir))
                    {
                        mc_spd_Ramp_Set_TargetSpeed(pMC->pSpdCL->pSpdRamp, pMC->CLP.StopSpd);
                        pMC->MotorCtrlStep++;
                    }
                }
            }else if(CtrlStep == 3)
            {
                /*到位判断-到达PT点停止*/
                if(TRUE == mc_pos_If_StopAt_TargetPos(pMC->pPosCL->pPosRamp->PT, CurrentPos, pMC->MotorDir, CurrentSpeed))
                {
					mc_app_Motor_Stop(pMC);
                    return;
                }
            }else
            {
            }
            if(CtrlStep >= 2 && CtrlStep <= 3)
            {
                /*目标速度来源于位置环(集成斜坡控制器)控制器*/
                mc_pos_CloseLoop_Controller(pMC->pPosCL, CurrentPos);
                mc_spd_Ramp_Controller(pMC->pSpdCL->pSpdRamp);
                /*更新目标速度,给速度环控制器输入*/
                if(pMC->MCEnableFunc & PCL_EN)
                {
                    TargetSpeed = (MOTOR_SPD_t)mc_pos_CloseLoop_Get_Output(pMC->pPosCL);
                }else
                {
                    TargetSpeed = mc_spd_Ramp_Get_ProcessSpeed(pMC->pSpdCL->pSpdRamp);
                }
                /**/
                if((pMC->MCEnableFunc & MGC_EN) == 0)
                {
                    mc_spd_Set_CloseLoop_TargetSpeed(pMC->pSpdCL, TargetSpeed);
                }
                HallSpeed = mc_hall_Get_MeasureSpeed_M(pMC->pHall->pHSMM);
                mc_spd_CloseLoop_Controller(pMC->pSpdCL, HallSpeed);
                /*更新占空比*/
                mc_drv_SetPWMDuty(pMC->pDrv, pMC->pSpdCL->Output);
                /*电流采样及过流保护功能执行*/
                mc_cur_Sample(pMC->pCur);
                mc_cur_OVC_Protect(pMC->pCur);
                /*检测电机异常停止*/
                if(mc_hall_Get_Hall_Abnormal(pMC->pHall->pHAC) != HALL_ABN_NO_ABN)
                {
                    pMC->MotorFault |= MOTOR_FAULT_HALL;
					mc_app_Motor_Stop(pMC);
                    return;
                }
                if(mc_cur_Get_OVC_Flag(pMC->pCur) == TRUE)
                {
                    pMC->MotorFault |= MOTOR_FAULT_OVC;
					mc_app_Motor_Stop(pMC);
                    return;
                }
            }
            break;

        case e_mas_start_openloop:
        case e_mas_start_closeloop:
            if(CtrlStep == 0)
            {
                /*电机驱动器-初始化*/
				mc_drv_SetPWMDuty(pMC->pDrv, 0);
                if(pMC->MotorDir == DIR_UP)
                {
                    mc_drv_SetCmd(pMC->pDrv, e_mdc_output_forward);
                    mc_hall_Set_OneHallDir(pMC->pHall->pHP, HALL_DIR_INCREASE);
                }
                if(pMC->MotorDir == DIR_DOWN)
                {
                    mc_drv_SetCmd(pMC->pDrv, e_mdc_output_reverse);
                    mc_hall_Set_OneHallDir(pMC->pHall->pHP, HALL_DIR_DECREASE);
                }
                pMC->MotorCtrlStep++;
            }else if(CtrlStep == 1)
            {
                if(mc_drv_Get_State(pMC->pDrv) == DRIVER_STATE_RUN)
                {
                    pMC->MotorCtrlStep++;
                    /*速度闭环控制-初始化*/
                    mc_drv_Set_Motor_Rotor_Sector(pMC->pDrv, pMC->pHall->pHP->State);
                    pMC->MotorMoveTime = 0;
                    if(pMC->MCEnableFunc & SCL_EN)
                    {
                        /*速度环-PID控制器初始化*/
                    #if (SPD_LOOP_PID_MODE == 0)    //增量式PID
                        pid_Reset_PID_ZL_Controller(pMC->pSpdCL->pSpdLoopPIDZL);
                    #else                           //位置式PID
                        pid_Reset_PID_WZ_Controller(pMC->pSpdCL->pSpdLoopPIDWZ);
                    #endif
                        /*速度环-速度环控制器初始化*/
                        mc_spd_Reset_CloseLoop_Controller(pMC->pSpdCL);
                        mc_spd_CloseLoop_Controller_Enable(pMC->pSpdCL);
                        /*HALL-脉宽检测初始化(M法)*/
                        mc_hall_Reset_SpeedMeasure_M(pMC->pHall);
                        /*速度斜坡控制初始化*/
                        mc_spd_Reset_Ramp_Controller(pMC->pSpdCL->pSpdRamp);
                        mc_spd_Ramp_Controller_Enable(pMC->pSpdCL->pSpdRamp);
                        mc_spd_Ramp_Set_Acceleration(pMC->pSpdCL->pSpdRamp, pMC->CLP.SpdUpAcc * pMC->MotorDir);
                        mc_spd_Ramp_Set_TargetSpeed(pMC->pSpdCL->pSpdRamp, pMC->CLP.TargetSpd * pMC->MotorDir);
                        mc_spd_Ramp_Set_InitialSpeed(pMC->pSpdCL->pSpdRamp, pMC->CLP.StartSpd * pMC->MotorDir);
                    }
                    /*速度开环控制-初始化*/
                    else
                    {
                        /*速度控制-速度开环控制器初始化*/
                        mc_spd_Reset_OpenLoop_Controller(pMC->pSpdOL);
                        mc_spd_OpenLoop_Controller_Enable(pMC->pSpdOL);
                        mc_spd_Set_OpenLoop_Acceleration(pMC->pSpdOL, pMC->OLP.SpdUpAcc * pMC->MotorDir);
                        mc_spd_Set_OpenLoop_TargetDutyCycle(pMC->pSpdOL, pMC->OLP.TargetDCPercent * pMC->MotorDir);
                        mc_spd_Set_OpenLoop_InitDutyCycle(pMC->pSpdOL, pMC->OLP.InitDCPercent * pMC->MotorDir);
                    }
                    /*HALL异常检测-初始化*/
                    if(pMC->MCEnableFunc & HAP_EN)
                    {
                        mc_hall_Reset_Check_Hall_Abnormal(pMC->pHall->pHAC);
                        mc_hall_Check_Hall_Abnormal_Enable(pMC->pHall->pHAC);
                        if(pMC->MotorDir == DIR_UP)
                            mc_hall_Check_Hall_Abnormal_Init(pMC->pHall->pHAC, HALL_DIR_INCREASE, M_MIN_RPM, M_MAX_RPM + 1000, HPR);
                        if(pMC->MotorDir == DIR_DOWN)
                            mc_hall_Check_Hall_Abnormal_Init(pMC->pHall->pHAC, HALL_DIR_DECREASE, M_MIN_RPM, M_MAX_RPM + 1000, HPR);
                    }
                    /*电流采样-初始化*/
                    mc_cur_Current_Sample_Enable(pMC->pCur);
                    mc_cur_Reset_Current_Sample(pMC->pCur);
                    /*过流保护-初始化*/
                    if(pMC->MCEnableFunc & OCP_EN)
                    {
                        mc_cur_OVC_Enable(pMC->pCur);
                        mc_cur_Reset_OVC(pMC->pCur);
                        mc_cur_Set_OVC_Shield_Time(pMC->pCur, OVC_SHIELD_TIME);
                    }
                    pMC->MotorFault = MOTOR_NO_FAULT;
                }
            }else
            {
                /*速度闭环控制任务*/
                if(pMC->MCEnableFunc & SCL_EN)
                {
                    /*速度斜坡控制*/
                    mc_spd_Ramp_Controller(pMC->pSpdCL->pSpdRamp);
                    /*速度环控制*/
                    HallSpeed = mc_hall_Get_MeasureSpeed_M(pMC->pHall->pHSMM);
                    TargetSpeed = mc_spd_Ramp_Get_ProcessSpeed(pMC->pSpdCL->pSpdRamp);
                    if((pMC->MCEnableFunc & MGC_EN) == 0)
                    {
                        mc_spd_Set_CloseLoop_TargetSpeed(pMC->pSpdCL, TargetSpeed);
                    }
                    mc_spd_CloseLoop_Controller(pMC->pSpdCL, HallSpeed);
                    /*更新占空比*/
                    mc_drv_SetPWMDuty(pMC->pDrv, pMC->pSpdCL->Output);
                }
                /*速度开环控制任务*/
                else
                {
                    /*速度斜坡控制*/
                    mc_spd_OpenLoop_Controller(pMC->pSpdOL);
                    /*更新占空比*/
                    mc_drv_SetPWMDuty(pMC->pDrv, pMC->pSpdOL->Output);
                }
                /*HALL异常检测任务*/
                if(pMC->MCEnableFunc & HAP_EN)
                {
                    if(mc_hall_Get_Hall_Abnormal(pMC->pHall->pHAC) != HALL_ABN_NO_ABN)
                    {
                        pMC->MotorFault |= MOTOR_FAULT_HALL;
						mc_app_Motor_Stop(pMC);
                        return;
                    }
                }
                /*电流采样任务*/
                mc_cur_Sample(pMC->pCur);
                /*过流保护任务*/
                mc_cur_OVC_Protect(pMC->pCur);
                if(mc_cur_Get_OVC_Flag(pMC->pCur) == TRUE)
                {
                    pMC->MotorFault |= MOTOR_FAULT_OVC;
					mc_app_Motor_Stop(pMC);
                    return;
                }
            }
            break;

        case e_mas_stop_openloop:
        case e_mas_stop_closeloop:
            if(CtrlStep == 0)
            {
                pMC->MotorCtrlStep++;
                /*速度闭环控制-初始化*/
                if(pMC->MCEnableFunc & SCL_EN)
                {
                    /*速度斜坡控制初始化*/
                    mc_spd_Reset_Ramp_Controller(pMC->pSpdCL->pSpdRamp);
                    mc_spd_Ramp_Controller_Enable(pMC->pSpdCL->pSpdRamp);
                    mc_spd_Ramp_Set_Acceleration(pMC->pSpdCL->pSpdRamp, pMC->CLP.SpdUpAcc * pMC->MotorDir);
                    mc_spd_Ramp_Set_TargetSpeed(pMC->pSpdCL->pSpdRamp, pMC->CLP.StopSpd * pMC->MotorDir);
                    mc_spd_Ramp_Set_InitialSpeed(pMC->pSpdCL->pSpdRamp, CurrentSpeed);
                }
                /*速度开环控制-初始化*/
                else
                {
                    /*速度控制-速度开环控制器初始化*/
                    mc_spd_Reset_OpenLoop_Controller(pMC->pSpdOL);
                    mc_spd_OpenLoop_Controller_Enable(pMC->pSpdOL);
                    mc_spd_Set_OpenLoop_Acceleration(pMC->pSpdOL, pMC->OLP.SpdDownAcc * pMC->MotorDir);
                    mc_spd_Set_OpenLoop_TargetDutyCycle(pMC->pSpdOL, pMC->OLP.FinalDutyCycle * pMC->MotorDir);
                    CurrentDutyCycle = mc_drv_GetPWMDuty(pMC->pDrv);
                    mc_spd_Set_OpenLoop_InitDutyCycle(pMC->pSpdOL, (CurrentDutyCycle * 100) / PWM_OUTPUT_FULLSCALE);
                }
            }else
            {
                /*速度闭环控制任务*/
                if(pMC->MCEnableFunc & SCL_EN)
                {
                    /*速度斜坡控制*/
                    mc_spd_Ramp_Controller(pMC->pSpdCL->pSpdRamp);
                    /*速度环控制*/
                    HallSpeed = mc_hall_Get_MeasureSpeed_M(pMC->pHall->pHSMM);
                    TargetSpeed = mc_spd_Ramp_Get_ProcessSpeed(pMC->pSpdCL->pSpdRamp);
                    if((pMC->MCEnableFunc & MGC_EN) == 0)
                    {
                        mc_spd_Set_CloseLoop_TargetSpeed(pMC->pSpdCL, TargetSpeed);
                    }
                    mc_spd_CloseLoop_Controller(pMC->pSpdCL, HallSpeed);
                    /*更新占空比*/
                    mc_drv_SetPWMDuty(pMC->pDrv, pMC->pSpdCL->Output);
                    /*闭环正常停止判断*/
                    if(mc_spd_Ramp_Is_Ramp_Over(pMC->pSpdCL->pSpdRamp) == TRUE)
                    {
                        mc_app_Motor_Stop(pMC);
                        return;
                    }
                }
                /*速度开环控制任务*/
                else
                {
                    /*速度斜坡控制*/
                    mc_spd_OpenLoop_Controller(pMC->pSpdOL);
                    /*更新占空比*/
                    mc_drv_SetPWMDuty(pMC->pDrv, pMC->pSpdOL->Output);
                    /*开环正常停止判断*/
                    if(mc_spd_Is_OpenLoop_Ramp_Over(pMC->pSpdOL) == TRUE)
                    {
                        mc_app_Motor_Stop(pMC);
                        return;
                    }
                }
                /*HALL异常检测任务*/
                if(pMC->MCEnableFunc & HAP_EN)
                {
                    if(mc_hall_Get_Hall_Abnormal(pMC->pHall->pHAC) != HALL_ABN_NO_ABN)
                    {
                        pMC->MotorFault |= MOTOR_FAULT_HALL;
                        mc_app_Motor_Stop(pMC);
                        return;
                    }
                }
                /*电流采样任务*/
                mc_cur_Sample(pMC->pCur);
                /*过流保护任务*/
                mc_cur_OVC_Protect(pMC->pCur);
                if(mc_cur_Get_OVC_Flag(pMC->pCur) == TRUE)
                {
                    pMC->MotorFault |= MOTOR_FAULT_OVC;
                    mc_app_Motor_Stop(pMC);
                    return;
                }
            }
            break;

        case e_mas_stopping:
            if(CtrlStep == 0)
            {
                pMC->MotorCtrlStep++;
                pMC->MotorDir = DIR_STOP;
                mc_drv_SetCmd(pMC->pDrv, e_mdc_stop);
                /*HALL-异常检测*/
                mc_hall_Check_Hall_Abnormal_Disable(pMC->pHall->pHAC);
                /*速度控制-速度斜坡控制器*/
                mc_spd_Reset_Ramp_Controller(pMC->pSpdCL->pSpdRamp);
                mc_spd_Ramp_Controller_Disable(pMC->pSpdCL->pSpdRamp);
                /*速度控制-速度闭环控制器*/
                mc_spd_CloseLoop_Controller_Disable(pMC->pSpdCL);
                /*速度控制-速度开环控制器*/
                mc_spd_OpenLoop_Controller_Disable(pMC->pSpdOL);
                /*位置控制-位置闭环控制器*/
                mc_pos_Ramp_Controller_Disable(pMC->pPosCL->pPosRamp);
                mc_pos_CloseLoop_Controller_Disable(pMC->pPosCL);
                /*电流采样及过流保护*/
                mc_cur_Current_Sample_Disable(pMC->pCur);
                mc_cur_OVC_Disable(pMC->pCur);
            }else
            {
                if(mc_drv_Get_State(pMC->pDrv) == DRIVER_STATE_IDLE)
                {
					pMC->MotorCtrlStep = 0;
					pMC->MotorState = e_mas_pseudo_idle;
                }
            }
            break;

		case e_mas_pseudo_idle:
			//收到不响应的命令需要直接跳转到idle状态
			if(pMC->MotorCmd == e_mac_none || pMC->MotorCmd == e_mac_stop || \
				pMC->MotorCmd == e_mac_stop_openloop || pMC->MotorCmd == e_mac_stop_closeloop)
			{
				pMC->MotorState = e_mas_idle;	
			}
			break;
		
        default:
            break;
    }
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 马达控制-马达功能控制器-多电机控制器

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*多电机控制器-句柄初始化*/
static void mc_app_MGC_hInit(MGC_Handle_t *pMGC, uint16_t MotorGroup, mc_app_sta_t MotorGroupState)
{
    uint8_t  MotorNb = 0;
    /*电机组拆分成对应的单电机句柄给到多电机控制器*/
    for(uint8_t i=0; i<MOTOR_NB; i++)
    {
        if(MotorGroup & (1<<i))
        {
            pMGC->pMC[MotorNb] = mc_app_Get_Handle(1<<i);
            MotorNb++;
        }
    }
    pMGC->MotorGroup = MotorGroup;
    pMGC->MotorGroupControlNb = MotorNb;
    pMGC->MotorGroupState = MotorGroupState;
    pMGC->MotorGroupCtrlStep = 0;
    pMGC->MotorGroupDir = DIR_STOP;
    //Todo: malloc分配内存失败时的处理(目前默认不会失败)
    pMGC->pSyncLoopPIDWZ = (PID_WZ_Handle_t*)malloc(sizeof(PID_WZ_Handle_t));
    if(pMGC->pSyncLoopPIDWZ != NULL)
    {
        pid_PID_WZ_hInit(pMGC->pSyncLoopPIDWZ);
    }
}
/*多电机控制器-句柄反初始化*/
static void mc_app_MGC_hDeinit(MGC_Handle_t *pMGC)
{
    pMGC->MotorGroup = 0;
    pMGC->MotorGroupControlNb = 0;
    pMGC->MotorGroupState = e_mas_idle;
    pMGC->MotorGroupCtrlStep = 0;
    pMGC->MotorGroupDir = DIR_STOP;
    for(uint8_t i=0; i<MOTOR_NB; i++)
    {
        pMGC->pMC[i] = NULL;
    }
    //注: 因为mc_app_MGC_hInit()在应用程序中会被随机调用,因此mc_app_MGC_hDeinit()必须释放这部分内存,否则堆内存会被逐渐消耗殆尽(内存泄漏)!
    if(pMGC->pSyncLoopPIDWZ != NULL)
    {
        free(pMGC->pSyncLoopPIDWZ);
    }
}
/*多电机控制器-根据MotorGroup获取控制器句柄*/
static MGC_Handle_t* mc_app_Get_MGC(uint16_t MotorGroup)
{
    for(uint8_t i=0; i<MGC_NB; i++)
    {
        if(hMGC[i].MotorGroup == MotorGroup)
        {
            return &hMGC[i];
        }
    }
    return NULL;
}
/*多电机控制器-获取可使用的控制器句柄*/
static MGC_Handle_t* mc_app_New_MGC(void)
{
    uint8_t i = 0;

    for (i = 0; i< MGC_NB; i++)
    {
        if(hMGC[i].MotorGroupState == e_mas_idle)
        {
            return &hMGC[i];
        }
    }
    return NULL;
}
/*多电机控制器-释放当前控制器句柄*/
static void mc_app_Release_MGC(MGC_Handle_t *pMGC)
{
    if(pMGC != NULL)
    {
        mc_app_MGC_hDeinit(pMGC);
    }
}
/*多电机控制器-判断当前控制命令的目标电机组是否可响应动作命令*/
/*返回值：
@ 0: 无效的电机组;
@ -1: 当前电机组中,存在已被多电机控制器使用的电机;
@ 1: 当前电机组未被多电机控制器使用(可响应新的电机组命令);
@ 2: 当前电机组已被多电机控制器使用(可响应新的电机组命令or维持当前动作)*/
static int8_t mc_app_Judege_MGC_Valid(uint16_t MotorGroup)
{
    int8_t  result = 1;     //目标控制命令可被响应
    uint8_t MotorNb = 0;

    for(uint8_t i=0; i<MOTOR_NB; i++)
    {
        if(MotorGroup & (MOTOR1<<i))
            MotorNb++;
    }
    if(MotorNb < 2)     //MotorGroup参数错误(1个电机)
        return 0;
    for(uint8_t i=0; i<MGC_NB; i++)
    {
        if((hMGC[i].MotorGroup & MotorGroup) != 0)
        {
            if(hMGC[i].MotorGroup == MotorGroup)//完全重合,说明当前电机组已在执行某个MGC(多电机协同控制)任务.
            {
                return 2;
            }else       //部分相同,说明MotorGroup中某些电机正在执行其他MGC命令).
            {
                result = -1;
            }
        }
    }
    return result;
}
/*多电机控制器-获取控制器状态*/
mc_app_sta_t mc_app_Get_MGC_State(uint16_t MotorGroup)
{
    MGC_Handle_t *pMGC = 0;
    int8_t  ThisMGCType = mc_app_Judege_MGC_Valid(MotorGroup);

    if(ThisMGCType == 1)
    {
        return e_mas_idle;
    }else if(ThisMGCType == 2)
    {
        pMGC = mc_app_Get_MGC(MotorGroup);
        return pMGC->MotorGroupState;
    }
    return e_mas_unknown;  //无效的电机组
}
/*多电机控制器-多电机控制器动作命令接口(含控制参数初始化)*/
int8_t mc_app_Set_Multi_Motor_Cmd(uint16_t MotorGroup, mc_app_cmd_t Cmd, int32_t argv)
{
    MGC_Handle_t *pMGC = 0;
    int8_t  ThisMGCType = mc_app_Judege_MGC_Valid(MotorGroup);
	
	if(Cmd > e_mac_sync_goto_targetpos || Cmd < e_mac_sync_start)
		return MC_RET_ERR_CMD;
    /*判断当前电机组是否可相应多电机控制器控制命令*/
    if(ThisMGCType == 1)
    {
        pMGC = mc_app_New_MGC();    //目标电机组未被协同控制,可响应当前命令,不会引起电机动作冲突
        if(!pMGC)
            return MC_RET_CMD_NOT_EXEC; //ThisMGCType == 1条件下pMGC的值可能为空(eg:没有空闲的多电机控制器可用),不响应此电机组的协同控制命令
    }else if(ThisMGCType == 2)
    {
        pMGC = mc_app_Get_MGC(MotorGroup);      //该电机组控制器已在正常运行中.
    }else
    {
        return MC_RET_CMD_NOT_EXEC;     //不响应此电机组的协同控制命令
    }
    /*命令设置给目标电机组对应的多电机控制器*/
    if(e_mas_idle == pMGC->MotorGroupState)
    {
        if(Cmd == e_mac_sync_start)
        {
            mc_app_MGC_hInit(pMGC, MotorGroup, (mc_app_sta_t)Cmd);
            pMGC->MotorGroupDir = argv;
        }else if(Cmd == e_mac_sync_goto_targetpos)
        {
            mc_app_MGC_hInit(pMGC, MotorGroup, (mc_app_sta_t)Cmd);
            pMGC->SyncTargetPos = (MOTOR_POS_t)argv;
        }else
        {
            return MC_RET_CMD_NOT_EXEC;
        }
    }else if(e_mas_sync_start == pMGC->MotorGroupState)
    {
        if(Cmd == e_mac_sync_stop)
        {
            pMGC->MotorGroupState = (mc_app_sta_t)Cmd;
            pMGC->MotorGroupCtrlStep = 0;
        }else if(Cmd == e_mac_sync_start)
        {
            if(pMGC->MotorGroupDir != argv) //运行中切换方向
            {
                pMGC->MotorGroupCtrlStep = 0;
                pMGC->MotorGroupDir = argv;
            }
        }else
        {
            return MC_RET_CMD_NOT_EXEC;
        }
    }else if(e_mas_sync_stop == pMGC->MotorGroupState)
    {
        //此状态不响应任何多电机控制命令
        return MC_RET_CMD_NOT_EXEC;
    }else if(e_mas_sync_goto_targetpos == pMGC->MotorGroupState)
    {
        if(Cmd == e_mac_sync_stop)
        {
            pMGC->MotorGroupState = (mc_app_sta_t)Cmd;
            pMGC->MotorGroupCtrlStep = 0;
        }else if(Cmd == e_mac_sync_goto_targetpos)
        {
            if(argv != pMGC->SyncTargetPos)
            {
                pMGC->MotorGroupCtrlStep = 0;
                pMGC->SyncTargetPos = (MOTOR_POS_t)argv;
            }
        }else
        {
            return MC_RET_CMD_NOT_EXEC;
        }
    }else
    {
        //do nothing
    }
    return MC_RET_OK;
}
/*多电机控制器-获取多电机中的主电机*/
static MC_Handle_t* mc_app_MGC_Get_SyncMaster(MGC_Handle_t *pMGC)
{
    uint8_t Num = pMGC->MotorGroupControlNb;//同时控制的电机个数
    int32_t Dir = pMGC->MotorGroupDir;
    MC_Handle_t *pMCSysnMaster = NULL;
    MC_Handle_t *pMCThis = NULL;
    MOTOR_POS_t CurrentPos, CurrentPosMin, CurrentPosMax;

    /*查找首个运行中的马达,并预设为主机*/
    for(uint8_t i=0; i<Num; i++)
    {
        if(mc_app_Get_State(pMGC->pMC[i]->Motor) != e_mas_idle)
        {
            pMCSysnMaster = pMGC->pMC[i];
            CurrentPosMin = mc_hall_Get_HallData(pMCSysnMaster->pHall->pHP);
            CurrentPosMax = CurrentPosMin;
            break;
        }
    }
    /*查找所有运行中的马达,根据升降方向及位置高低,按照相应规则产生主机*/
    for(uint8_t i=0; i<Num; i++)
    {
        pMCThis = pMGC->pMC[i];
        if(mc_app_Get_State(pMGC->pMC[i]->Motor) != e_mas_idle)
        {
            CurrentPos = mc_hall_Get_HallData(pMCThis->pHall->pHP);
            if(Dir == DIR_UP)
            {
                if(CurrentPos < CurrentPosMin)  /*上升过程中,高度最低的立柱是同步控制的主机,其他从机做减速控制*/
                {
                    pMCSysnMaster = pMCThis;
                    CurrentPosMin = CurrentPos;
                }
            }else if(Dir == DIR_DOWN)
            {
                if(CurrentPos > CurrentPosMax)  /*下降过程中,高度最高的立柱是同步控制的主机,其他从机做减速控制*/
                {
                    pMCSysnMaster = pMCThis;
                    CurrentPosMax = CurrentPos;
                }
            }else
            {
            }
        }
    }
    return pMCSysnMaster;
}
/*多电机控制器-获取指定位置*/
//Type: 1-最低位置  2-中间位置  3-最高位置
static MC_Handle_t* mc_app_MGC_Get_Specified_Pos(MGC_Handle_t *pMGC, uint8_t Type)
{
    uint8_t Num = pMGC->MotorGroupControlNb;//同时控制的电机个数
    MC_Handle_t *pMC = NULL;
    MC_Handle_t *pMCTemp = NULL;
    MC_Handle_t *pMCTable[MOTOR_NB];
    MOTOR_POS_t Pos1, Pos2;

    if(Num == 0)
    {
        return NULL;
    }else if(Num == 1)
    {
        pMC = pMGC->pMC[0];
    }else
    {
        for(uint8_t i=0; i<Num; i++)
        {
            pMCTable[i] = pMGC->pMC[i];
        }
        /*按照高度从小到大排序*/
        for(uint8_t i=0; i<Num; i++)
        {
            for(uint8_t j=i+1; j<Num; j++)
            {
                Pos1 = mc_hall_Get_HallData(pMCTable[i]->pHall->pHP);
                Pos2 = mc_hall_Get_HallData(pMCTable[j]->pHall->pHP);
                if(Pos1 > Pos2)
                {
                    pMCTemp = pMCTable[i];
                    pMCTable[i] = pMCTable[j];
                    pMCTable[j] = pMCTemp;
                }
            }
        }
        if(Type == 1)
        {
            pMC = pMCTable[0];
        }else if(Type == 2)
        {
            pMC = pMCTable[Num/2];
        }else if(Type == 3)
        {
            pMC = pMCTable[Num-1];
        }else
        {
        }
    }
    return pMC;
}
/*多电机控制器-等待所有电机停止*/
static BOOL mc_app_Multi_Motor_WaitingAllStop(MGC_Handle_t *pMGC)
{
    uint8_t i = 0;
    uint8_t MotorNb = pMGC->MotorGroupControlNb;
    MC_Handle_t *pMC = NULL;
    uint8_t StopNb = 0;

    if(!pMGC)
        return FALSE;
    for(i=0; i<MotorNb; i++)
    {
        pMC = pMGC->pMC[i];
        if(mc_app_Get_State(pMC->Motor) != e_mas_idle)
        {
            mc_app_Motor_Stop(pMC);
        }else
        {
            if(++StopNb == MotorNb)
            {
                return TRUE;
            }
        }
    }
    return FALSE;
}
/*多电机控制器-判断新的同步目标位置是否有效*/
static int32_t mc_app_Multi_Motor_Is_SyncPos_Valid(MGC_Handle_t *pMGC)
{
#if 1
    uint8_t MotorNb = pMGC->MotorGroupControlNb;
    MC_Handle_t *pMC = NULL;
    MOTOR_POS_t CurrentPos = 0;
    uint8_t  GoUpNb = 0;
    uint8_t  GoDnNb = 0;

    if(!pMGC)
        return FALSE;
    for(uint8_t i=0; i<MotorNb; i++)
    {
        pMC = pMGC->pMC[i];
        CurrentPos = mc_hall_Get_HallData(pMC->pHall->pHP);
        if(pMGC->SyncTargetPos > CurrentPos + 5)
        {   /*上升*/
            GoUpNb++;
        }
        if(pMGC->SyncTargetPos < CurrentPos - 5)
        {   /*下降*/
            GoDnNb++;
        }
    }
    if(GoUpNb == MotorNb)
    {
        return DIR_UP;
    }else
    {
        if(GoDnNb == MotorNb)
        {
            return DIR_DOWN;
        }
    }
#else
    MC_Handle_t  *pMCHighestPos = NULL;
    MC_Handle_t  *pMCLowestPos = NULL;
    MOTOR_POS_t  HighestPos = 0, LowestPos = 0;

    pMCLowestPos = mc_app_MGC_Get_Specified_Pos(pMGC, 1);
    pMCHighestPos = mc_app_MGC_Get_Specified_Pos(pMGC, 3);
    LowestPos = mc_hall_Get_HallData(pMCLowestPos->pHall->pHP);
    HighestPos = mc_hall_Get_HallData(pMCHighestPos->pHall->pHP);
    if(pMGC->SyncTargetPos > (HighestPos + 5))
    {
        return DIR_UP;
    }
    if(pMGC->SyncTargetPos < (LowestPos - 5))
    {
        return DIR_DOWN;
    }
#endif
    return DIR_STOP;
}
/*多电机控制器-多电机位置同步控制器*/
static BOOL mc_app_Multi_Motor_PosSyncCtrl(MGC_Handle_t *pMGC)
{
    uint8_t i = 0;
    uint8_t MotorNb = pMGC->MotorGroupControlNb;
    MC_Handle_t *pMCSyncMaster, *pMCThis;
    MOTOR_POS_t SyncMasterPos, ThisPos;
    MOTOR_SPD_t SpdDValue, CurrentSpd;

    pMCSyncMaster = mc_app_MGC_Get_SyncMaster(pMGC);
    if(pMCSyncMaster != NULL)   //注:mc_app_MGC_Get_SyncMaster()返回NULL说明所有同步运行马达已停止!
    {
        for(i=0; i<MotorNb; i++)
        {
            pMCThis = pMGC->pMC[i];
        #if 1
            mc_app_Set_MCFlag(pMCThis, MGC_EN); //使能MGC_EN,防止同步速度被单电机控制器修改(多电机控制器基于单电机控制器运行)
            /*计算和主电机的高度差*/
            if(pMCSyncMaster != pMCThis)
            {
                SyncMasterPos = mc_hall_Get_HallData(pMCSyncMaster->pHall->pHP);
                ThisPos = mc_hall_Get_HallData(pMCThis->pHall->pHP);
                /*从机做减速控制(同步策略: 快的减速)*/
                /*主机选取的原则决定了(SyncMasterPos - ThisPos)的值: DIR_UP时必<=0,DIR_DOWN时必> 0*/
                SpdDValue = (MOTOR_SPD_t)pid_PID_WZ_Controller(pMGC->pSyncLoopPIDWZ, (SyncMasterPos - ThisPos));
                SpdDValue = abs(SpdDValue);
            #if 1
                CurrentSpd = mc_spd_Ramp_Get_ProcessSpeed(pMCThis->pSpdCL->pSpdRamp);
            #else   //用真实速度做减速,可能会导致同步环KP值较大时,立柱相互比较时越比越慢,待调试验证
                CurrentSpd = mc_hall_Get_MeasureSpeed_M(pMCSyncMaster->pHall->pHSMM);
            #endif
                if(CurrentSpd > SpdDValue)
                {
                    mc_spd_Set_CloseLoop_TargetSpeed(pMCThis->pSpdCL, (CurrentSpd - SpdDValue));
                }else
                {
                    mc_spd_Set_CloseLoop_TargetSpeed(pMCThis->pSpdCL, M_MIN_RPM);
                }
            }else
            {
                //主机速度保持不变
                CurrentSpd = mc_spd_Ramp_Get_ProcessSpeed(pMCSyncMaster->pSpdCL->pSpdRamp);
                mc_spd_Set_CloseLoop_TargetSpeed(pMCSyncMaster->pSpdCL, CurrentSpd);
            #if 1   //主机是否考虑适当加速

            #endif
            }
        #else
            //mc_app_Set_MCFlag(pMCThis, MGC_EN);   //使能MGC_EN,防止同步速度被单电机控制器修改(多电机控制器基于单电机控制器运行)
            /*计算和主电机的高度差*/
            if(pMCSyncMaster != pMCThis)
            {
                SyncMasterPos = mc_hall_Get_HallData(pMCSyncMaster->pHall->pHP);
                ThisPos = mc_hall_Get_HallData(pMCThis->pHall->pHP);
                /*从机做减速控制(同步策略: 快的减速)*/
                /*主机选取的原则决定了(SyncMasterPos - ThisPos)的值: DIR_UP时必<=0,DIR_DOWN时必> 0*/
                if(pMGC->MotorGroupDir == DIR_UP)
                {
                    mc_pos_CloseLoop_Set_TargetPos_Offset(pMCThis->pPosCL, (SyncMasterPos - ThisPos));
                }else
                {
                    if(pMGC->MotorGroupDir == DIR_DOWN)
                    {
                        mc_pos_CloseLoop_Set_TargetPos_Offset(pMCThis->pPosCL, (ThisPos - SyncMasterPos));
                    }
                }
            }else
            {
                //主机速度保持不变
                mc_pos_CloseLoop_Set_TargetPos_Offset(pMCThis->pPosCL, 0);
            }
        #endif
        }
    }else   //所有马达已停止(eg: 发生异常)
    {
        return FALSE;
    }
    return TRUE;
}
/*多电机控制器*/
static void mc_app_MotorGroup_Controller(MGC_Handle_t *pMGC)
{
    uint16_t CtrlStep = pMGC->MotorGroupCtrlStep;
    uint8_t MotorNb = pMGC->MotorGroupControlNb;
    MC_Handle_t *pMC;
    MOTOR_POS_t CurrentPos = 0;
    MOTOR_SPD_t CurrentSpeed = 0;
    static uint8_t s_SlowFlag[MOTOR_NB] = {0};
    /**/
    switch(pMGC->MotorGroupState)
    {
        case e_mas_sync_start:
            if(CtrlStep == 0)
            {
                pMGC->MotorGroupCtrlStep++;
                /*电机组启动同步运行相关初始化*/
                for(uint8_t i=0; i<MotorNb; i++)
                {
                    pMC = pMGC->pMC[i];
                    mc_app_Set_Single_Motor_Cmd(pMC->Motor, e_mac_start_closeloop, pMGC->MotorGroupDir);
                }
                /*同步环初始化*/
                pid_Reset_PID_WZ_Controller(pMGC->pSyncLoopPIDWZ);
                pid_PID_WZ_Set_Kp(pMGC->pSyncLoopPIDWZ, 75);    //只能用P控制不能用PI控制,免去每个电机和主电机都需要独立存储积分项.
                pid_PID_WZ_Set_Output_Max(pMGC->pSyncLoopPIDWZ, M_TARGRT_RPM - M_START_RPM_UP); //最大差速
                pid_PID_WZ_Set_OutputDiv(pMGC->pSyncLoopPIDWZ, 1);
            }else if(CtrlStep == 1)
            {
                /*位置同步控制*/
                if(FALSE == mc_app_Multi_Motor_PosSyncCtrl(pMGC))
                {
                    pMGC->MotorGroupCtrlStep++;
                    return;
                }
            }else if(CtrlStep == 2)
            {
                /*停止电机组当前动作*/
                if(TRUE == mc_app_Multi_Motor_WaitingAllStop(pMGC))
                {
                    mc_app_Release_MGC(pMGC);
                }
            }else
            {
            }
            break;

        case e_mas_sync_stop:
            if(CtrlStep == 0)
            {
                pMGC->MotorGroupCtrlStep++;
                /*慢停止初始化*/
                for(uint8_t i = 0; i < MotorNb; i++)
                {
                    pMC = pMGC->pMC[i];
                    mc_spd_Ramp_Set_TargetSpeed(pMC->pSpdCL->pSpdRamp, pMC->CLP.StopSpd);
                }
            }else if(CtrlStep == 1)
            {
                /*位置同步控制*/
                if(FALSE == mc_app_Multi_Motor_PosSyncCtrl(pMGC))
                {
                    pMGC->MotorGroupCtrlStep++;
                    return;
                }else
                {
                    /*慢停止完成判断*/
                    uint8_t Match = 0;
                    for(uint8_t i = 0; i < MotorNb; i++)
                    {
                        pMC = pMGC->pMC[i];
                        if(mc_spd_Ramp_Is_Ramp_Over(pMC->pSpdCL->pSpdRamp) == TRUE)
                        {
                            mc_app_Motor_Stop(pMC);
                            Match++;
                        }
                    }
                    if(Match == MotorNb)
                    {
                        pMGC->MotorGroupCtrlStep++;
                        return;
                    }
                }
            }else if(CtrlStep == 2)
            {
                /*停止电机组当前动作*/
                if(TRUE == mc_app_Multi_Motor_WaitingAllStop(pMGC))
                {
                    mc_app_Release_MGC(pMGC);
                }
            }else
            {
            }
            break;

        case e_mas_sync_goto_targetpos:
            if(CtrlStep == 0)
            {
                /*判断新的目标位置是否有效*/
                pMGC->MotorGroupDir = mc_app_Multi_Motor_Is_SyncPos_Valid(pMGC);
                if(pMGC->MotorGroupDir == DIR_STOP)
                {
                    pMGC->MotorGroupCtrlStep = 2;
                }else
                {
                    pMGC->MotorGroupCtrlStep++;
                    /*同步环初始化*/
                    pid_Reset_PID_WZ_Controller(pMGC->pSyncLoopPIDWZ);
                    pid_PID_WZ_Set_Kp(pMGC->pSyncLoopPIDWZ, 75);    //只能用P控制不能用PI控制,免去每个电机和主电机都需要独立存储积分项
                    pid_PID_WZ_Set_Output_Max(pMGC->pSyncLoopPIDWZ, M_TARGRT_RPM - M_START_RPM_UP); //最大差速
                    pid_PID_WZ_Set_OutputDiv(pMGC->pSyncLoopPIDWZ, 1);
                    /*电机组同步运行相关初始化*/
                    for(uint8_t i=0; i<MotorNb; i++)
                    {
                        pMC = pMGC->pMC[i];
                        s_SlowFlag[i] = 0;
                        CurrentPos = mc_hall_Get_HallData(pMC->pHall->pHP);
                        if(pMC->MotorDir == pMGC->MotorGroupDir)
                        //保持同方向运行(但目标位置更新)
                        {
                            CurrentSpeed = mc_spd_Get_CloseLoop_TargetSpeed(pMC->pSpdCL);/*速度环的目标速度可认为是当前的速度*/
                            /*借助位置斜坡控制器,获得速度曲线*/
                            mc_pos_Ramp_SetInput(pMC->pPosCL->pPosRamp, CurrentSpeed, pMC->CLP.StopSpd, pMC->CLP.TargetSpd, pMC->CLP.SpdUpAcc, CurrentPos, pMGC->SyncTargetPos);
                            /*速度斜坡控制初始化*/
                            mc_spd_Reset_Ramp_Controller(pMC->pSpdCL->pSpdRamp);
                            mc_spd_Ramp_Controller_Enable(pMC->pSpdCL->pSpdRamp);
                            mc_spd_Ramp_Set_Acceleration(pMC->pSpdCL->pSpdRamp, pMC->CLP.SpdUpAcc);
                            mc_spd_Ramp_Set_TargetSpeed(pMC->pSpdCL->pSpdRamp, SPD_SWITCH_HPS_TO_RPM(pMC->pPosCL->pPosRamp->vP1));
                            mc_spd_Ramp_Set_InitialSpeed(pMC->pSpdCL->pSpdRamp, CurrentSpeed);
                        }else
                        //静止启动 || 需要切换方向
                        {
                            /*借助位置斜坡控制器,获得速度曲线*/
                            mc_pos_Ramp_SetInput(pMC->pPosCL->pPosRamp, pMC->CLP.StartSpd, pMC->CLP.StopSpd, pMC->CLP.TargetSpd, pMC->CLP.SpdUpAcc, CurrentPos, pMGC->SyncTargetPos);
                        }
                        mc_app_Set_Single_Motor_Cmd(pMC->Motor, e_mac_start_closeloop, pMGC->MotorGroupDir);
                    }
                }
            }else if(CtrlStep == 1)
            {
                /*位置同步控制*/
                if(FALSE == mc_app_Multi_Motor_PosSyncCtrl(pMGC))
                {
                    pMGC->MotorGroupCtrlStep++;
                    return;
                }else
                {
                    uint8_t Match = 0;
                    for(uint8_t i = 0; i < MotorNb; i++)
                    {
                        pMC = pMGC->pMC[i];
                        CurrentPos = mc_hall_Get_HallData(pMC->pHall->pHP);
                        if(! s_SlowFlag[i])
                        {
                            /*判断是否到达减速位置*/
                            if(TRUE == mc_pos_If_RunTo_TargetPos(pMC->pPosCL->pPosRamp->P2, CurrentPos, pMC->MotorDir))
                            {
                                mc_spd_Ramp_Set_TargetSpeed(pMC->pSpdCL->pSpdRamp, pMC->CLP.StopSpd);
                                s_SlowFlag[i] = 1;
                            }
                        }else
                        {
                            /*慢停止完成判断*/
                            CurrentSpeed = mc_spd_Get_CloseLoop_TargetSpeed(pMC->pSpdCL);/*速度环的目标速度可认为是当前的速度*/
                            if(mc_pos_If_StopAt_TargetPos(pMC->pPosCL->pPosRamp->PT, CurrentPos, pMC->MotorDir, CurrentSpeed) == TRUE)
                            {
								mc_app_Motor_Stop(pMC);
                                Match++;
                            }
                        }
                    }
                    if(Match == MotorNb)
                    {
                        pMGC->MotorGroupCtrlStep++;
                        return;
                    }
                }
            }else if(CtrlStep == 2)
            {
                /*停止电机组当前动作*/
                if(TRUE == mc_app_Multi_Motor_WaitingAllStop(pMGC))
                {
                    mc_app_Release_MGC(pMGC);
                }
            }else
            {
            }
            break;

        default:
            break;
    }
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 马达控制-电机应用功能-模块入口函数

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*后台入口*/
void mc_app_Loop_Task(void)
{
	if(MC_INACCESSIBLE == g_MCBootInitCplt)
        return;
    /*M1-M4*/
    for(uint8_t i=0; i<MOTOR_NB; i++)
    {
        mc_drv_StateMachine(&hMCDrv[i]); //hz 先进行当前底层状态的判断，响应命令并更新硬件状态。
        mc_app_Single_Motor_Controller(&hMC[i]); //hz 后进行应用层电机的控制，依赖于驱动器的最新状态。？：第一次可能会乱，等后续获取数据，可进入正常循环
        //mc_hall_Test(&hMCHall[i]);
        //mc_spd_Test(&hMCSpeedCL[i], &hMCSpeedRamp[i]);
    }
    /*M1-M4*/
    mc_drv_Test(&hMCDrv[0], &hMCDrv[1]);
    mc_app_Test(MOTOR1, MOTOR2);
    /*common*/
#if (MOTOR_NB >= 2)
    for(uint8_t i=0; i<MGC_NB; i++)
    {
        mc_app_MotorGroup_Controller(&hMGC[i]);
    }
#endif
    //Ddl_Delay1ms(10); //测试结果：SystemCoreClock = 128MHz,马达5000PRM,目标位置运行停止位置滞后20步
    //Ddl_Delay1us(1000);
    //Ddl_Delay1us(100);
}
/*250us定时调用1次*/
void mc_app_Timer_250us(void)
{
    if(MC_INACCESSIBLE == g_MCBootInitCplt)
        return;
    /*M1-M4*/
    for(uint8_t i=0; i<MOTOR_NB; i++)
    {
    #if (MOTOR_TYPE == MOTOR_TYPE_DC)
        /*M1-M4 更新转子位置*/
		uint8_t HallState = mc_hall_Get_HallState(&hMCHall[i]);
        mc_drv_Set_Motor_Rotor_Sector(&hMCDrv[i], HallState);
        mc_hall_Update_HallData(&hMCHall[i]);
    #elif (MOTOR_TYPE == MOTOR_TYPE_BLDC)
        if(hMC[i].MotorMoveTime < BLDC_SP_TIME)
        {
            mc_drv_Commutation(&hMCDrv[i]);
        }
    #endif
        mc_hall_Check_Hall_Abnormal(&hMCHall[i]);
    }
}
/*1ms定时调用1次*/
void mc_app_Timer_1ms(void)
{
    if(MC_INACCESSIBLE == g_MCBootInitCplt)
        return;
    /*common*/
    mc_hall_SpeedMeasure_T_Timer();
    adc_adapter_SampleInterval_Timer();

    /*M1-M4*/
    for(uint8_t i=0; i<MOTOR_NB; i++)
    {
        mc_drv_Delay_Timer_Run(&hMCDrv[i]);
        mc_hall_SpeedMeasure_M(&hMCHall[i]);
        mc_spd_CloseLoop_Controller_Timer(&hMCSpeedCL[i]);
        mc_spd_OpenLoop_Controller_Timer(&hMCSpeedOL[i]);
        mc_spd_Ramp_Controller_Timer(&hMCSpeedRamp[i]);
        mc_pos_Ramp_Controller_Timer(&hMCPosRamp[i]);
        mc_pos_CloseLoop_Controller_Timer(&hMCPosCL[i]);
        mc_cur_OVC_Shield_Timer(&hMCCur[i]);
        mc_app_MotorRun_Timer(&hMC[i]);
    }
}
/*HALL信号边沿触发任务*/
void mc_app_Trigger_Task_HallEdge(void)
{
    if(MC_INACCESSIBLE == g_MCBootInitCplt)
        return;
    //HAL_GPIO_TogglePin(GPIO_TEST2_GPIO_Port, GPIO_TEST2_Pin);
    TEST_TOGGLE(g_TestToggle);
    for(uint8_t i=0; i<MOTOR_NB; i++)
    {
    #if (MOTOR_TYPE == MOTOR_TYPE_BLDC)
        /*M1-M4 更新转子位置*/
        uint8_t HallState = mc_hall_Get_HallState(&hMCHall[i]);
        mc_drv_Set_Motor_Rotor_Sector(&hMCDrv[i], HallState);
        mc_hall_Update_HallData(&hMCHall[i]);
        if(hMC[i].MotorMoveTime >= BLDC_SP_TIME)
        {
            mc_drv_Commutation(&hMCDrv[i]);
        }
    #endif
    }
}
/*Hall方向选择*/
int8_t mc_app_ModifyHallDir(uint8_t Motor, uint8_t Dir)
{
	MC_Handle_t* pMotor = mc_app_Get_Handle(Motor);
	if(!pMotor)
		return -1;	//非法参数
	HALL_Handle_t *pHall = pMotor->pHall;
	pHall->HallDirectionSel = Dir;
	return 1;
}
/*驱动器方向选择*/
int8_t mc_app_ModifyDrvDir(uint8_t Motor, uint8_t Dir)
{
	MC_Handle_t* pMotor = mc_app_Get_Handle(Motor);
	if(!pMotor)
		return -1;	//非法参数
	MC_DRV_Handle_t *pDrv = pMotor->pDrv;
	/*换向器初始化*/
    if(pDrv->Commutator != NULL)
    {
        mc_drv_Commutator_hInit(pDrv->Commutator, pDrv->PhaseNb, Dir);
    }
	return 1;
}

/*初始化入口*/
void mc_app_Init(void)
{
#if (MOTOR_NB >= 1)
    /*hall模块初始化-M1*/
    HALL_t M1_Hall[HALL_NB] =
    {
#if (HALL_NB >= 1)
    {EXINT_M1_HALLA_PORT, EXINT_M1_HALLA_PIN},
#endif
#if (HALL_NB >= 2)
    {EXINT_M1_HALLB_PORT, EXINT_M1_HALLB_PIN},
#endif
#if (HALL_NB >= 3)
    {EXINT_M1_HALLC_PORT, EXINT_M1_HALLC_PIN},
#endif
    };
    mc_hall_hInit(&hMCHall[0], M1_Hall, HALL_NB, POLE_NB, 0, &hMCHallPedometer[0], &hMCHallSpdMeasureM[0], &hMCHallSpdMeasureT[0], &hMCHallAbnChecker[0]);
    hMCHall[0].pHP->PrevState = mc_hall_Get_HallState(&hMCHall[0]);
	mc_hall_SpeedMeasure_M_Enable(hMCHall[0].pHSMM);
	
    /*drv模块初始化-M1*/
    MOS_t   M1_MosL[DRIVER_PHASE_NB] =
    {
    {PWMCH_M1_UL_PORT, PWMCH_M1_UL_PIN, PWMCH_M1_UL_TIMER, PWMCH_M1_UL_CH, PWMCH_M1_UL_FUNC_GPIO, PWMCH_M1_UL_FUNC_PWM},
    {PWMCH_M1_VL_PORT, PWMCH_M1_VL_PIN, PWMCH_M1_VL_TIMER, PWMCH_M1_VL_CH, PWMCH_M1_VL_FUNC_GPIO, PWMCH_M1_VL_FUNC_PWM},
#if (DRIVER_PHASE_NB == 3)
    {PWMCH_M1_WL_PORT, PWMCH_M1_WL_PIN, PWMCH_M1_WL_TIMER, PWMCH_M1_WL_CH, PWMCH_M1_WL_FUNC_GPIO, PWMCH_M1_WL_FUNC_PWM},
#endif
    };
    MOS_t   M1_MosH[DRIVER_PHASE_NB] =
    {
    {PWMCH_M1_UH_PORT, PWMCH_M1_UH_PIN, PWMCH_M1_UH_TIMER, PWMCH_M1_UH_CH, PWMCH_M1_UH_FUNC_GPIO, PWMCH_M1_UH_FUNC_PWM},
    {PWMCH_M1_VH_PORT, PWMCH_M1_VH_PIN, PWMCH_M1_VH_TIMER, PWMCH_M1_VH_CH, PWMCH_M1_VH_FUNC_GPIO, PWMCH_M1_VH_FUNC_PWM},
#if (DRIVER_PHASE_NB == 3)
    {PWMCH_M1_WH_PORT, PWMCH_M1_WH_PIN, PWMCH_M1_WH_TIMER, PWMCH_M1_WH_CH, PWMCH_M1_WH_FUNC_GPIO, PWMCH_M1_WH_FUNC_PWM},
#endif
    };
#if (DRV_TYPE == DRV_TYPE_HB_WITH_PD)//hz
    mc_drv_hInit(&hMCDrv[0], MOTOR1, M1_MosH, M1_MosL, DRIVER_PHASE_NB, 2, &hCmt[0], 0);
#elif (DRV_TYPE == DRV_TYPE_PNMOS)
    mc_drv_hInit(&hMCDrv[0], MOTOR1, &M1_MosH[0], &M1_MosH[1], &M1_MosL[0], &M1_MosL[1]);
#endif
#endif

#if (MOTOR_NB >= 2)
    /*hall模块初始化-M2*/
    HALL_t M2_Hall[HALL_NB] =
    {
#if (HALL_NB >= 1)
    {EXINT_M2_HALLA_PORT, EXINT_M2_HALLA_PIN},
#endif
#if (HALL_NB >= 2)
    {EXINT_M2_HALLB_PORT, EXINT_M2_HALLB_PIN},
#endif
#if (HALL_NB >= 3)
    {EXINT_M2_HALLC_PORT, EXINT_M2_HALLC_PIN},
#endif
    };
    mc_hall_hInit(&hMCHall[1], M2_Hall, HALL_NB, POLE_NB, 0, &hMCHallPedometer[1], &hMCHallSpdMeasureM[1], &hMCHallSpdMeasureT[1], &hMCHallAbnChecker[1]);
    hMCHall[1].pHP->PrevState = mc_hall_Get_HallState(&hMCHall[1]);

    /*drv模块初始化-M2*/
    MOS_t   M2_MosL[DRIVER_PHASE_NB] =
    {
    {PWMCH_M2_UL_PORT, PWMCH_M2_UL_PIN, PWMCH_M2_UL_TIMER, PWMCH_M2_UL_CH, PWMCH_M2_UL_FUNC_GPIO, PWMCH_M2_UL_FUNC_PWM},
    {PWMCH_M2_VL_PORT, PWMCH_M2_VL_PIN, PWMCH_M2_VL_TIMER, PWMCH_M2_VL_CH, PWMCH_M2_VL_FUNC_GPIO, PWMCH_M2_VL_FUNC_PWM},
#if (DRIVER_PHASE_NB == 3)
    {PWMCH_M2_WL_PORT, PWMCH_M2_WL_PIN, PWMCH_M2_WL_TIMER, PWMCH_M2_WL_CH, PWMCH_M2_WL_FUNC_GPIO, PWMCH_M2_WL_FUNC_PWM},
#endif
    };
    MOS_t   M2_MosH[DRIVER_PHASE_NB] =
    {
    {PWMCH_M2_UH_PORT, PWMCH_M2_UH_PIN, PWMCH_M2_UH_TIMER, PWMCH_M2_UH_CH, PWMCH_M2_UH_FUNC_GPIO, PWMCH_M2_UH_FUNC_PWM},
    {PWMCH_M2_VH_PORT, PWMCH_M2_VH_PIN, PWMCH_M2_VH_TIMER, PWMCH_M2_VH_CH, PWMCH_M2_VH_FUNC_GPIO, PWMCH_M2_VH_FUNC_PWM},
#if (DRIVER_PHASE_NB == 3)
    {PWMCH_M2_WH_PORT, PWMCH_M2_WH_PIN, PWMCH_M2_WH_TIMER, PWMCH_M2_WH_CH, PWMCH_M2_WH_FUNC_GPIO, PWMCH_M2_WH_FUNC_PWM},
#endif
    };
    #if 0   //3相驱动器
    mc_drv_hInit(&hMCDrv[1], MOTOR2, M2_MosH, M2_MosL, DRIVER_PHASE_NB, 4, &hCmt[1]);
    #else   //npmos驱动器
    mc_drv_hInit(&hMCDrv[1], MOTOR2, &M2_MosH[0], &M2_MosH[1], &M2_MosL[0], &M2_MosL[1]);
    #endif
#endif
    for(uint8_t i=0; i<MOTOR_NB; i++)
    {
        mc_drv_SetPWMDutyMax(&hMCDrv[i], (PWM_OUTPUT_FULLSCALE / 100) * MAX_PWM_PERCENT);
        mc_drv_SetPWMDutyMin(&hMCDrv[i], (PWM_OUTPUT_FULLSCALE / 100) * MIN_PWM_PERCENT);
        mc_drv_Set_Motor_Rotor_Sector(&hMCDrv[i], hMCHall[i].pHP->State);
    }

    /*spd模块及关联pid子模块初始化(M1-M4)(注意先后顺序,pid子模块需放到后面,否则会被覆盖)*/
    for(uint8_t i=0; i<MOTOR_NB; i++)
    {
        mc_spd_CloseLoop_hInit(&hMCSpeedCL[i], &hMCSpeedPID_ZL[i], &hMCSpeedPID_WZ[i], &hMCSpeedRamp[i]);
        mc_spd_Set_CloseLoop_Period(&hMCSpeedCL[i], SPD_LOOP_PERIOD);
        mc_spd_Set_CloseLoop_MaxSpeed(&hMCSpeedCL[i], M_MAX_RPM);
        mc_spd_Set_CloseLoop_MinSpeed(&hMCSpeedCL[i], M_MIN_RPM);
        mc_spd_Ramp_Controller_hInit(&hMCSpeedRamp[i]);
        mc_spd_OpenLoop_hInit(&hMCSpeedOL[i]);
        mc_spd_Set_OpenLoop_DutyCycleModValue(&hMCSpeedOL[i], PWM_OUTPUT_FULLSCALE);
        mc_spd_Set_OpenLoop_MaxDutyCycle(&hMCSpeedOL[i], M_MAX_DC);
        mc_spd_Set_OpenLoop_MinDutyCycle(&hMCSpeedOL[i], M_MIN_DC);
    #if (SPD_LOOP_PID_MODE == 0)    //增量式PID
        pid_PID_ZL_Set_IncrementDiv(&hMCSpeedPID_ZL[i], SPD_LOOP_PID_ZL_DIV);
        pid_PID_ZL_Set_KpKiKd(&hMCSpeedPID_ZL[i], 20, 3, 5);
		pid_PID_ZL_Set_Output_Min(&hMCSpeedPID_ZL[i], 0);											//最小占空比
		pid_PID_ZL_Set_Output_Max(&hMCSpeedPID_ZL[i], PWM_OUTPUT_FULLSCALE * MAX_PWM_PERCENT / 100);//最大占空比
    #else                           //位置式PID
        pid_PID_WZ_Set_OutputDiv(&hMCSpeedPID_WZ[i], SPD_LOOP_PID_WZ_DIV);
        pid_PID_WZ_Set_KpKiKd(&hMCSpeedPID_WZ[i], 50, 3, 0);
        //pid_PID_WZ_Set_KpKiKd(pMC->pSpdCL->pSpdLoopPIDWZ, 50, 3, 100);
        pid_PID_WZ_Set_IntegralTerm_Max(&hMCSpeedPID_WZ[i], (PWM_OUTPUT_FULLSCALE << SPD_LOOP_PID_WZ_DIV));//积分项限幅对应最大输出(因为稳态比例项为0,全靠积分项)
		pid_PID_WZ_Set_Output_Min(&hMCSpeedPID_WZ[i], 0);											//最小占空比
		pid_PID_WZ_Set_Output_Max(&hMCSpeedPID_WZ[i], PWM_OUTPUT_FULLSCALE * MAX_PWM_PERCENT / 100);//最大占空比
	#endif
    }

    /*cur模块初始化(M1-M4)*/
    uint8_t ADChannelTbl[MCU_AD_CHANNEL_NB] = {ADCH_M1_IM_ADCH, ADCH_M2_IM_ADCH, ADCH_M3_IM_ADCH, ADCH_M4_IM_ADCH};
    for(uint8_t i=0; i<MOTOR_NB; i++)
    {
        mc_cur_hInit(&hMCCur[i], ADChannelTbl[i]);
//        mc_cur_Set_OVCTHH(&hMCCur[i], MOTOR_OVC_VALUE);
    }

    /*pos模块及关联pid子模块初始化(M1-M4)*/
    for(uint8_t i=0; i<MOTOR_NB; i++)
    {
        mc_pos_CloseLoop_hInit(&hMCPosCL[i], &hMCPosLoopPID_WZ[i], &hMCPosRamp[i]);
        mc_pos_Set_CloseLoop_Period(&hMCPosCL[i], 20);
        pid_PID_WZ_Set_OutputDiv(&hMCPosLoopPID_WZ[i], 6);
        pid_PID_WZ_Set_KpKiKd(&hMCPosLoopPID_WZ[i], 200, 0, 0);
        pid_PID_WZ_Set_Output_Min(&hMCPosLoopPID_WZ[i], 0);         //最小速度增量
        pid_PID_WZ_Set_Output_Max(&hMCPosLoopPID_WZ[i], SPD_SWITCH_RPM_TO_HPS(M_MAX_RPM - M_MIN_RPM));
        //pid_PID_WZ_Set_Output_Max(&hMCPosLoopPID_WZ[i], SPD_SWITCH_RPM_TO_HPS(500));  //最大速度增量不能大的原因: 下降自锁不够,防止位置环累计误差太快太大.
    }

    /*app模块初始化(M1-M4)(必须放在其他模块初始化完成后)*/
    /*common*/
    MC_CLP_Handle_t CLP = {M_START_RPM_UP, M_TARGRT_RPM, M_SLOWSTOP_RPM, M_START_ACCELERATION, M_STOP_ACCELERATION};
    MC_OLP_Handle_t OLP = {M_START_DC_UP, M_TARGRT_DC, M_SLOWSTOP_DC, M_START_DC_ACCELERATION, M_STOP_DC_ACCELERATION};
    for(uint8_t i=0; i<MOTOR_NB; i++)
    {
        mc_app_hInit(&hMC[i], (MOTOR1 << i), &CLP, &OLP, &hMCDrv[i], &hMCHall[i], &hMCSpeedCL[i], &hMCSpeedOL[i], &hMCPosCL[i], &hMCCur[i]);
    }

    /*adc_adapter模块初始化(M1-M4)(与mc相关的部分)*/
#if (MOTOR_NB >= 1)
    ADCH_t  M1_IM_ADCH = {ADCH_M1_IM_ADC,   ADCH_M1_IM_ADCH,    ADCH_M1_IM_PORT,    ADCH_M1_IM_PIN};
    adc_adapter_hInit(&hADCChannalMCurrent[0], &M1_IM_ADCH);
    adc_adapter_Set_Channal_SmpTime(ADCH_M1_IM_ADCH, CUR_SMP_WINDOW);
    adc_adapter_Set_Channal_SmpIntervalTime(ADCH_M1_IM_ADCH, CUR_SMP_PERIOD);
    adc_adapter_Channel_Enable(ADCH_M1_IM_ADCH);
#endif
#if (MOTOR_NB >= 2)
    ADCH_t  M2_IM_ADCH = {ADCH_M2_IM_ADC,   ADCH_M2_IM_ADCH,    ADCH_M2_IM_PORT,    ADCH_M2_IM_PIN};
    adc_adapter_hInit(&hADCChannalMCurrent[1], &M2_IM_ADCH);
    adc_adapter_Set_Channal_SmpTime(ADCH_M2_IM_ADCH, CUR_SMP_WINDOW);
    adc_adapter_Set_Channal_SmpIntervalTime(ADCH_M2_IM_ADCH, CUR_SMP_PERIOD);
    adc_adapter_Channel_Enable(ADCH_M2_IM_ADCH);
#endif

    g_MCBootInitCplt = 1;
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 马达控制-电机应用功能-模块测试

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
void mc_app_Test(uint16_t Motor1, uint16_t Motor2)
{
    MC_Handle_t *pMC1 = mc_app_Get_Handle(Motor1);
    MC_Handle_t *pMC2 = mc_app_Get_Handle(Motor2);

    for(uint8_t i=0; i<MOTOR_NB; i++)
    {
        MC_Handle_t *pMC = mc_app_Get_Handle(MOTOR1<<i);
        g_WatchMotorCurrentPos[i] = mc_hall_Get_HallData(pMC->pHall->pHP);
        g_WatchMotorCurrentSpd[i] = mc_spd_Get_CloseLoop_RealSpeed(pMC->pSpdCL);
        g_WatchMotorTargetSpd[i] = mc_spd_Get_CloseLoop_TargetSpeed(pMC->pSpdCL);
    }
#if (MOTOR_NB >=2)
    g_WatchPosDValue = g_WatchMotorCurrentPos[0] - g_WatchMotorCurrentPos[1];
#endif
#if (1)     /*单电机任务*/
    /*闭环控制命令-M1*/
    if(g_TestMCAppCmd == 101)
    {
        g_TestMCAppCmd = 0;
        mc_app_Set_Single_Motor_Cmd(Motor1, e_mac_start_closeloop, DIR_UP);
    }
    if(g_TestMCAppCmd == 102)
    {
        g_TestMCAppCmd = 0;
        mc_app_Set_Single_Motor_Cmd(Motor1, e_mac_start_closeloop, DIR_DOWN);
    }
    if(g_TestMCAppCmd == 103)
    {
        g_TestMCAppCmd = 0;
        mc_app_Set_Single_Motor_Cmd(Motor1, e_mac_stop_closeloop, 0);
    }
    if(g_TestMCAppCmd == 104)
    {
        g_TestMCAppCmd = 0;
        mc_app_Set_Single_Motor_Cmd(Motor1, e_mac_goto_targetpos, g_TestTargetPos);
    }
    if(g_TestMCAppCmd == 105)
    {
        g_TestMCAppCmd = 0;
        MOTOR_POS_t CurrentPos = mc_hall_Get_HallData(hMC[0].pHall->pHP);
        mc_app_Set_Single_Motor_Cmd(Motor1, e_mac_goto_targetpos, CurrentPos + g_TestTargetPos);
    }
	if(g_TestMCAppCmd == 106)
	{
		g_TestMCAppCmd = 0;
		mc_app_Write_Param(MOTOR1, e_map_targetspd, g_TestTargetSpeed);
	}
    /*闭环控制命令-M2*/
    if(g_TestMCAppCmd == 201)
    {
        g_TestMCAppCmd = 0;
        mc_app_Set_Single_Motor_Cmd(Motor2, e_mac_start_closeloop, DIR_UP);
    }
    if(g_TestMCAppCmd == 202)
    {
        g_TestMCAppCmd = 0;
        mc_app_Set_Single_Motor_Cmd(Motor2, e_mac_start_closeloop, DIR_DOWN);
    }
    if(g_TestMCAppCmd == 203)
    {
        g_TestMCAppCmd = 0;
        mc_app_Set_Single_Motor_Cmd(Motor2, e_mac_stop_closeloop, 0);
    }
    if(g_TestMCAppCmd == 204)
    {
        g_TestMCAppCmd = 0;
        mc_app_Set_Single_Motor_Cmd(Motor2, e_mac_goto_targetpos, g_TestTargetPos);
    }
    if(g_TestMCAppCmd == 205)
    {
        g_TestMCAppCmd = 0;
        MOTOR_POS_t CurrentPos = mc_hall_Get_HallData(hMC[1].pHall->pHP);
        mc_app_Set_Single_Motor_Cmd(Motor2, e_mac_goto_targetpos, CurrentPos + g_TestTargetPos);
    }

    /*开环控制命令-M1*/
    if(g_TestMCAppCmd == 111)
    {
        g_TestMCAppCmd = 0;
        mc_app_Set_Single_Motor_Cmd(Motor1, e_mac_start_openloop, DIR_UP);
    }
    if(g_TestMCAppCmd == 112)
    {
        g_TestMCAppCmd = 0;
        mc_app_Set_Single_Motor_Cmd(Motor1, e_mac_start_openloop, DIR_DOWN);
    }
    if(g_TestMCAppCmd == 113)
    {
        g_TestMCAppCmd = 0;
        mc_app_Set_Single_Motor_Cmd(Motor1, e_mac_stop_openloop, 0);
    }
	if(g_TestMCAppCmd == 116)
    {
        g_TestMCAppCmd = 0;
        mc_app_Write_Param(MOTOR1, e_map_ol_targetdc, g_TestTargetSpeed);
    }
    if(g_TestMCAppCmd == 199)
    {
        g_TestMCAppCmd = 0;
        mc_app_Set_Single_Motor_Cmd(Motor1, e_mac_stop, 0);
    }
    /*开环控制命令-M2*/
    if(g_TestMCAppCmd == 211)
    {
        g_TestMCAppCmd = 0;
        mc_app_Set_Single_Motor_Cmd(Motor2, e_mac_start_openloop, DIR_UP);
    }
    if(g_TestMCAppCmd == 212)
    {
        g_TestMCAppCmd = 0;
        mc_app_Set_Single_Motor_Cmd(Motor2, e_mac_start_openloop, DIR_DOWN);
    }
    if(g_TestMCAppCmd == 213)
    {
        g_TestMCAppCmd = 0;
        mc_app_Set_Single_Motor_Cmd(Motor2, e_mac_stop_openloop, 0);
    }
    if(g_TestMCAppCmd == 299)
    {
        g_TestMCAppCmd = 0;
        mc_app_Set_Single_Motor_Cmd(Motor2, e_mac_stop, 0);
    }
#endif

#if (2)     /*多电机任务*/
    /*闭环控制命令-M1*/
    if(g_TestMCAppCmd == 1201 || g_TestMCAppCmd == 2101)
    {
        g_TestMCAppCmd = 0;
        mc_app_Set_Multi_Motor_Cmd((Motor1|Motor2), e_mac_sync_start, DIR_UP);
    }
    if(g_TestMCAppCmd == 1202 || g_TestMCAppCmd == 2102)
    {
        g_TestMCAppCmd = 0;
        mc_app_Set_Multi_Motor_Cmd((Motor1|Motor2), e_mac_sync_start, DIR_DOWN);
    }
    if(g_TestMCAppCmd == 1203 || g_TestMCAppCmd == 2103)
    {
        g_TestMCAppCmd = 0;
        mc_app_Set_Multi_Motor_Cmd((Motor1|Motor2), e_mac_sync_stop, 0);
    }
    if(g_TestMCAppCmd == 1204 || g_TestMCAppCmd == 2104)
    {
        g_TestMCAppCmd = 0;
        mc_app_Set_Multi_Motor_Cmd((Motor1|Motor2), e_mac_sync_goto_targetpos, g_TestTargetPos);
    }
    if(g_TestMCAppCmd == 1205 || g_TestMCAppCmd == 2105)
    {
        g_TestMCAppCmd = 0;
        MOTOR_POS_t CurrentPos = mc_hall_Get_HallData(hMC[0].pHall->pHP);
        mc_app_Set_Multi_Motor_Cmd((Motor1|Motor2), e_mac_sync_goto_targetpos, CurrentPos + g_TestTargetPos);
    }
#endif
}
