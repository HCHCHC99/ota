/********************************文件说明*************************************
*文件名: mc_pos.c

*作者: Yuchen Tan

*版本: V1.0.4

*功能简介:
*1.电机运行到目标位置控制器(基于速度闭环);

*备注:
*1.基于速度闭环的原因:
[1].马达位置依赖于HALL传感器,有HALL就能实现速度闭环控制.
[2].对位置的控制若采用开环占空比纯属舍近求远,效果也不好.

*修改履历:
------------------------------------V1.0.2------------------------------------
20220810: mc_pos_If_StopAt_TargetPos()根据电机运行方向,增加不同的提前和滞后量.
------------------------------------V1.0.3------------------------------------
20220923:
1.位置闭环控制器增加mc_pos_Set_Ramp_TargetSpeed()接口,实现电机位置环运
行实时修改目标速度;
2.修复位置环斜坡控制器若干bug,详见《调试笔记-20220923》;
------------------------------------V1.0.4------------------------------------
20230329: 修复mc_pos_Ramp_Controller()中st,vt计算错误的bug;
*****************************************************************************/

/*****************************文件包含(私有)**********************************
*
*备注: 无
*
*****************************************************************************/
#include "mc_pos.h"
/*****************************宏定义(私有)************************************
*
*备注: 本文件中,不希望被外部使用或随意修改的宏在这里定义
*
*****************************************************************************/

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
/*马达运行到目标位置控制器-句柄初始化*/
static void mc_pos_TargetPos_hInit(POS_RAMP_Handle_t *pMTPC);
static void mc_pos_Ramp_Cal_Feature(POS_RAMP_Handle_t *pMTPC);
/********************************变量定义*************************************
*
*备注: 不要在.h中定义变量,防止多个.c包含这个.h,导致编译后报错变量重复定义
*
*****************************************************************************/

/********************************函数定义************************************
*函数名:

*函数功能描述: 马达控制-电机位置控制-位置判断

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*达到目标位置判断*/
BOOL mc_pos_If_StopAt_TargetPos(MOTOR_POS_t TargetPos, MOTOR_POS_t CurrentPos, int32_t Dir, MOTOR_SPD_t CurrentSpd)
{
    MOTOR_POS_t StopBuffer = 0;

    /*Todo: StopBuffer的值带载前后可能需要更加精确的调试*/
    //StopBuffer = CurrentSpd / 200;
    StopBuffer = abs(CurrentSpd) / 250;
    if(Dir != DIR_STOP)
    {
        if(Dir == DIR_UP)   /*上升*/
        {
            if(CurrentPos >= TargetPos - StopBuffer + 2)    //上升载重做负功,滞后几步停的更准
            {
                return TRUE;
            }
        }else   //DIR_DOWN  /*下降*/
        {
            if(CurrentPos <= TargetPos + StopBuffer + 2)    //下降载重做正功,提前几步停的更准
            {
                return TRUE;
            }
        }
    }
    return FALSE;
}
/*达到目标位置判断*/
BOOL mc_pos_If_RunTo_TargetPos(MOTOR_POS_t TargetPos, MOTOR_POS_t CurrentPos, int32_t Dir)
{
    if(Dir != DIR_STOP)
    {
        if(Dir == DIR_UP)   /*上升*/
        {
            if(CurrentPos >= TargetPos)
            {
                return TRUE;
            }
        }else   //DIR_DOWN  /*下降*/
        {
            if(CurrentPos <= TargetPos)
            {
                return TRUE;
            }
        }
    }
    return FALSE;
}
/*高于目标位置判断*/
BOOL mc_pos_If_Higher_Than_TargetPos(MOTOR_POS_t TargetPos, MOTOR_POS_t CurrentPos)
{
    if(CurrentPos > TargetPos)
    {
        return TRUE;
    }
    return FALSE;
}
/*低于目标位置判断*/
BOOL mc_pos_If_Below_TargetPos(MOTOR_POS_t TargetPos, MOTOR_POS_t CurrentPos)
{
    if(CurrentPos < TargetPos)
    {
        return TRUE;
    }
    return FALSE;
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 马达控制-电机位置控制-位置闭环斜坡控制器

*函数参数:

*函数返回值: 无

*备注:
*****************************************************************************/
/*位置闭环斜坡控制器-句柄初始化*/
static void mc_pos_TargetPos_hInit(POS_RAMP_Handle_t *pMTPC)
{
	memset(pMTPC, 0, sizeof(POS_RAMP_Handle_t));
}
/*位置闭环斜坡控制器-设置斜坡参数*/
void mc_pos_Ramp_SetInput(POS_RAMP_Handle_t *pMTPC, MOTOR_SPD_t v01, MOTOR_SPD_t v02, MOTOR_SPD_t vT, MOTOR_SPD_t a, MOTOR_POS_t PS, MOTOR_POS_t PT)
{
    MOTOR_SPD_t _vT = SPD_SWITCH_RPM_TO_HPS(vT);

    pMTPC->v01 = SPD_SWITCH_RPM_TO_HPS(v01);
    pMTPC->v02 = SPD_SWITCH_RPM_TO_HPS(v02);
    pMTPC->vT = _vT;
    pMTPC->a = SPD_SWITCH_RPM_TO_HPS(a);
    pMTPC->PS = PS;
    pMTPC->PT = PT;
    /*根据参数计算特征信息*/
    mc_pos_Ramp_Cal_Feature(pMTPC);
}
/*位置闭环斜坡控制器-设置位置环目标速度*/
void mc_pos_Set_Ramp_TargetSpeed(POS_RAMP_Handle_t *pMTPC, MOTOR_SPD_t vT)
{
    MOTOR_SPD_t _vT = SPD_SWITCH_RPM_TO_HPS(vT);

    if(pMTPC->vT != _vT)
    {
		//PORT_SetBits(TEST2_GPIO_Port, TEST2_Pin);		//result: excution time of the code between totally use 3.2us(hc32f460,clk==128MHz)
        pMTPC->v01 = pMTPC->vt; //理论上pMTPC->v01和pMTPC->PS选电机当前实际速度和位置效果更好,但不利于接口统一!
        pMTPC->vT = _vT;
        pMTPC->PS = pMTPC->Pt;
        mc_pos_Ramp_Cal_Feature(pMTPC);
        mc_pos_Reset_Ramp_Controller(pMTPC);	
		//PORT_ResetBits(TEST2_GPIO_Port, TEST2_Pin);	//result: excution time of the code between totally use 3.2us(hc32f460,clk==128MHz)
    }
}
/*位置闭环斜坡控制器-计算特征位置和速度*/
static void mc_pos_Ramp_Cal_Feature(POS_RAMP_Handle_t *pMTPC)
{
    MOTOR_POS_t S1 = 0, S3 = 0, S4 = 0;     //vt曲线面积(符号说明: 围成面积在直线y=0上方(+),围成面积在直线y=0下方(-))
    MOTOR_POS_t S1_ = 0, S3_ = 0;
    int32_t Dir = (pMTPC->PT >= pMTPC->PS) ? (1) : (-1);        /*电机运行方向*/
    int32_t a1 = (pMTPC->v01 <= pMTPC->vT) ? abs(pMTPC->a) : (abs(pMTPC->a) * (-1));	/*三段式/两段式速度变化-第1阶段加速度(v01到vT/vt)*/
    int32_t a2 = (pMTPC->vT <= pMTPC->v02) ? abs(pMTPC->a) : (abs(pMTPC->a) * (-1));	/*三段式/两段式速度变化-第2阶段加速度(vT/vt到v02)*/
    int32_t a3 = (pMTPC->v01 <= pMTPC->v02) ? abs(pMTPC->a) : (abs(pMTPC->a) * (-1));   /*一段式速度变化加速度(v01到v02)*/
	int32_t Temp = 0;
    /*非法参数屏蔽*/
    if(pMTPC->vT == 0)  //pMTPC->vT绝对不能等于0,详见《调试笔记-20220923》
        return;
    /**/
    pMTPC->sT = pMTPC->PT - pMTPC->PS;
    if(pMTPC->a == 0)
    {
        //预设加速度为0,无论目标行程是多少永远保持匀速运动
        pMTPC->Type = 4;
        pMTPC->s1 = pMTPC->s3 = 0;
        pMTPC->s2 = pMTPC->sT;
        pMTPC->P1 = pMTPC->PS;
        pMTPC->P2 = pMTPC->PT;
        pMTPC->vP1 = pMTPC->vP2 = pMTPC->vT;
        pMTPC->t1 = 0;
        pMTPC->t2 = pMTPC->t1 + (pMTPC->P2 - pMTPC->P1) * 1000 / pMTPC->vT;
        pMTPC->t3 = pMTPC->t2;
    }else
    {
        /*匀变速直线运动公式：st = (vt^2 - v0^2 / 2a) (其中vt, v0, a均可带符号,算出的st一致)*/
        S1 = (pMTPC->vT * pMTPC->vT - pMTPC->v01 * pMTPC->v01) / (a1 << 1);    //v01加速到vT阶段行程(绝对值)
        S3 = (pMTPC->v02 * pMTPC->v02 - pMTPC->vT * pMTPC->vT) / (a2 << 1);    //vT减速到v02阶段行程(绝对值)
        if(abs(pMTPC->sT) >= abs(S1 + S3))
        {
            //从v01加速到额定速度vT,持续一段时间,再从vT减速到v02(vP1 == vP2 == vT)
            pMTPC->Type = 1;
            pMTPC->s1 = S1;
            pMTPC->s3 = S3;
            pMTPC->s2 = pMTPC->sT - (pMTPC->s1 + pMTPC->s3);
            pMTPC->P1 = pMTPC->PS + pMTPC->s1;
            pMTPC->P2 = pMTPC->P1 + pMTPC->s2;
            pMTPC->vP1 = pMTPC->vP2 = pMTPC->vT;
            pMTPC->t1 = (pMTPC->vP1 - pMTPC->v01) * 1000 / a1;
		#if 0
			pMTPC->t2 = (pMTPC->P2 - pMTPC->P1) * 1000; 
            pMTPC->t2 /= pMTPC->vT;
            pMTPC->t2 += pMTPC->t1;
        #endif
			pMTPC->t2 = pMTPC->t1 + (pMTPC->P2 - pMTPC->P1) * 1000 / pMTPC->vT;	//NEED_NOTE: ((pMTPC->P2 - pMTPC->P1) * 1000) > 2147483647时会导致运算结果溢出,详见《电机驱动库调试笔记-20220923》
            pMTPC->t3 = pMTPC->t2 + (pMTPC->v02 - pMTPC->vP2) * 1000 / a2;
        }else
        {
            S4 = (pMTPC->v02 * pMTPC->v02 - pMTPC->v01 * pMTPC->v01) / (a3 << 1);  //v01变到v02阶段行程
            if(abs(pMTPC->sT) >= abs(S4))
            {
                //从v01加速到速度vt(vt<vT),立即开始减速,减到v02(无匀速阶段,P1和P2点重合,vP1 == vP2 != vT)
                pMTPC->Type = 2;
			#if 0	//下面算法仅在|a1| == |a2|时适用
				S1_ = abs(pMTPC->v02 * pMTPC->v02 - pMTPC->v01 * pMTPC->v01 + pMTPC->sT * (pMTPC->a << 1));
				S1_ /= pMTPC->a << 2;
				S3_ = abs(pMTPC->v01 * pMTPC->v01 - pMTPC->v02 * pMTPC->v02 + pMTPC->sT * (pMTPC->a << 1));
				S3_ /= pMTPC->a << 2;
				pMTPC->s1 = S1_;
				pMTPC->s2 = 0;
				pMTPC->s3 = S3_;
				pMTPC->P2 = pMTPC->P1 = pMTPC->PS + pMTPC->s1;
				pMTPC->vP2 = pMTPC->vP1 = sqrt(pMTPC->v01 * pMTPC->v01 + pMTPC->s1 * (a1 << 1)) * Dir;
				pMTPC->t2 = pMTPC->t1 = (pMTPC->vP1 - pMTPC->v01) * 1000 / pMTPC->a;
				pMTPC->t3 = pMTPC->t2 + (pMTPC->v02 - pMTPC->vP1) * 1000 / pMTPC->a;
			#endif
				Temp = pMTPC->sT * ((a1 * a2) << 1);
				Temp += a2 * pMTPC->v01 * pMTPC->v01;
				Temp -= a1 * pMTPC->v02 * pMTPC->v02;
				Temp /= (a2 - a1);
				pMTPC->vP2 = pMTPC->vP1 = sqrt(Temp) * Dir;
				S1_ = (pMTPC->vP1 * pMTPC->vP1 - pMTPC->v01 * pMTPC->v01) / (a1 << 1);
				S3_ = (pMTPC->v02 * pMTPC->v02 - pMTPC->vP2 * pMTPC->vP2) / (a2 << 1);
				pMTPC->s1 = S1_;
				pMTPC->s2 = 0;
				pMTPC->s3 = S3_;
				pMTPC->t2 = pMTPC->t1 = (pMTPC->vP1 - pMTPC->v01) * 1000 / a1;
				pMTPC->t3 = pMTPC->t2 + (pMTPC->v02 - pMTPC->vP1) * 1000 / a2;
            }else
            {
                //从v01直接变化到v02(不一定能达到v02)(P1和P2点均为PS)
                pMTPC->Type = 3;
                pMTPC->s1 = pMTPC->s2 = 0;
                pMTPC->s3 = pMTPC->sT;
                pMTPC->P1 = pMTPC->P2 = pMTPC->PS;
                pMTPC->vP2 = pMTPC->vP1 = pMTPC->v01;
                pMTPC->t1 = pMTPC->t2 = 0;
                pMTPC->t3 = (pMTPC->v02 - pMTPC->v01) * 1000 / a3;
            }
        }
    }
}
/*位置闭环斜坡控制器-重置控制器*/
void mc_pos_Reset_Ramp_Controller(POS_RAMP_Handle_t* pMTPC)
{
    pMTPC->PosRampTimer = 0;
}
/*位置闭环斜坡控制器-使能控制器*/
void mc_pos_Ramp_Controller_Enable(POS_RAMP_Handle_t* pMTPC)
{
    pMTPC->PosRampEn = TRUE;
}
/*位置闭环斜坡控制器-禁止控制器*/
void mc_pos_Ramp_Controller_Disable(POS_RAMP_Handle_t* pMTPC)
{
    pMTPC->PosRampEn = FALSE;
}
/*位置闭环斜坡控制器-斜坡完成?*/
BOOL mc_pos_Ramp_Is_Ramp_Over(POS_RAMP_Handle_t* pMTPC)
{
    return FALSE;
}
/*位置闭环斜坡控制器*/
/*注: pMTPC->sT太大会导致32位整形乘法计算溢出,解决方法:
1.用/div减小计算数(不能彻底解决(div↓: 除法截断误差小但溢出范围大; div↑: 溢出范围小但除法截断误差大));
    //(65535 * 65535)/2000000 = 4,294,836,225/2000000 = 2147
    //(6553 * 6553)/20000 = 4,294,1809/20000 = 2147
    //(655 * 655)/200 = 429,025/200 = 2145
2.用64位整形计算(不推荐使用(64位除法执行很慢));
3.用float浮点型计算(推荐做法);*/
void mc_pos_Ramp_Controller(POS_RAMP_Handle_t* pMTPC)
{
    float       vt = 0;
    float       st = 0;
    uint32_t    t = pMTPC->PosRampTimer;
    float       t_f = (float)pMTPC->PosRampTimer / 1000;
    float       t1_f = (float)pMTPC->t1 / 1000;
    float       t2_f = (float)pMTPC->t2 / 1000;
    float       temp1 = 0;
    float       a1 = (pMTPC->v01 <= pMTPC->vP1) ? abs(pMTPC->a) : (abs(pMTPC->a) * (-1));   /*v01到vP1加速度*/	//三段式/两段式速度变化-第1段变速加速度
    float       a2 = (pMTPC->vP2 <= pMTPC->v02) ? abs(pMTPC->a) : (abs(pMTPC->a) * (-1));   /*vP2到v02加速度*/	//三段式/两段式速度变化-第2段变速加速度
    float       a3 = (pMTPC->v01 <= pMTPC->v02) ? abs(pMTPC->a) : (abs(pMTPC->a) * (-1));   /*v01到v02加速度*/	//一段式速度变化加速度
	
    if(pMTPC->PosRampEn == FALSE || !pMTPC)
        return;
    /**/
    if(pMTPC->Type == 4)
    {
        vt = (float)pMTPC->vT;
        st = vt * t_f;
    }else
    {
        if(t <= pMTPC->t3)
        {
            if(pMTPC->Type == 1)			//三段式速度变化(v01 -> vT -> v02)
            {
                if(t <= pMTPC->t1)			//v01 -> vT
                {
					temp1 = (a1 * t_f * t_f) / 2;
                    st = (float)pMTPC->v01 * t_f + temp1;
                    vt = (float)pMTPC->v01 + a1 * t_f;
                }else if(t <= pMTPC->t2)	//vT
                {
                    vt = (float)pMTPC->vP1;
                    st = (float)pMTPC->s1 + (t_f - t1_f) * (float)pMTPC->vP1;
                }else						//vT -> v02
                {
                    vt = ((float)pMTPC->vP1 + a2 * (t_f - t2_f));
					temp1 = (vt * vt - (float)pMTPC->vP2 * (float)pMTPC->vP2) / (a2 * 2);
                    st = (float)pMTPC->s1 + (float)pMTPC->s2 + temp1;
                }
			}else if(pMTPC->Type == 2)		//两段式速度变化(v01 -> vT` -> v02)  (vT` != vT)
            {
                if(t <= pMTPC->t1)			//v01 -> vT`
                {
					temp1 = (a1 * t_f * t_f) / 2;
                    st = (float)pMTPC->v01 * t_f + temp1;
					vt = (float)pMTPC->v01 + a1 * t_f;
                }else						//vT` -> v02
                {
					vt = (float)pMTPC->vP2 + a2 * (t_f - t1_f);
                    temp1 = (vt * vt - (float)pMTPC->vP2 * (float)pMTPC->vP2) / (a2 * 2);
                    st = (float)pMTPC->s1 + temp1;
                }
            }else   /*pMTPC->Type == 3*/	//一段式速度变化(v01 -> v02)
            {
                if(t <= pMTPC->t3)
                {
                    if(pMTPC->v01 == pMTPC->v02)
                    {
                        vt = pMTPC->v01;
						st = vt * t_f;
                    }else
                    {
                        vt = (float)pMTPC->v01 + a3 * t_f;
						st = (vt * vt - (float)pMTPC->v01 * (float)pMTPC->v01) / (a3 * 2);
                    }
                }
            }
        }else
        {
            st = pMTPC->sT;
            vt = pMTPC->v02;
        }
    }
    pMTPC->Pt = pMTPC->PS + st;
    pMTPC->vt = (MOTOR_SPD_t)vt;
}
/*位置闭环斜坡控制器-斜坡控制器计数器(1ms调用一次)*/
void mc_pos_Ramp_Controller_Timer(POS_RAMP_Handle_t* pMTPC)
{
    if(pMTPC->PosRampEn == 1)
    {
        pMTPC->PosRampTimer++;
    }else
    {
        pMTPC->PosRampTimer = 0;
    }
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 马达控制-电机位置控制-位置闭环控制器

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*位置闭环控制器-句柄初始化*/
void mc_pos_CloseLoop_hInit(POS_CL_Handle_t *pPosCL, PID_WZ_Handle_t *pPIDWZ, POS_RAMP_Handle_t *pPosRamp)
{
	memset(pPosCL, 0, sizeof(POS_CL_Handle_t));
    if(pPIDWZ != NULL)
    {
        pPosCL->pPosLoopPIDWZ = pPIDWZ;
        pid_PID_WZ_hInit(pPIDWZ);
    }
    if(pPosRamp != NULL)
    {
        pPosCL->pPosRamp = pPosRamp;
        mc_pos_TargetPos_hInit(pPosRamp);
    }
}
/*位置闭环控制器-重置控制器*/
void mc_pos_Reset_CloseLoop_Controller(POS_CL_Handle_t *pPosCL)
{
    pPosCL->PeriodTimer = 0;

    pPosCL->FdbkPos = 0;
    pPosCL->TargetPos = 0;

    pPosCL->Output = 0;
}
/*位置闭环控制器-设置控制器运行周期*/
void mc_pos_Set_CloseLoop_Period(POS_CL_Handle_t *pPosCL, uint16_t Period)
{
    pPosCL->Period = Period;
}
/*位置闭环控制器-使能控制器*/
void mc_pos_CloseLoop_Controller_Enable(POS_CL_Handle_t *pPosCL)
{
    pPosCL->PosCLCtrlEn = TRUE;
}
/*位置闭环控制器-禁止控制器*/
void mc_pos_CloseLoop_Controller_Disable(POS_CL_Handle_t *pPosCL)
{
    pPosCL->PosCLCtrlEn = FALSE;
}
/*位置闭环控制器-控制器运行周期计数器(1ms调用一次)*/
void mc_pos_CloseLoop_Controller_Timer(POS_CL_Handle_t *pPosCL)
{
    if(pPosCL->PosCLCtrlEn == TRUE)
    {
        pPosCL->PeriodTimer++;
    }else
    {
        pPosCL->PeriodTimer = 0;
    }
}
/*位置闭环控制器-获取控制器输出*/
int32_t mc_pos_CloseLoop_Get_Output(POS_CL_Handle_t *pPosCL)
{
    return SPD_SWITCH_HPS_TO_RPM(pPosCL->Output);
}
/*位置闭环控制器*/
void mc_pos_CloseLoop_Controller(POS_CL_Handle_t *pPosCL, MOTOR_POS_t FdbkPos)
{
    MOTOR_SPD_t OutPutSpd = 0;

    if(pPosCL->PosCLCtrlEn == TRUE)
    {
        if(pPosCL->PeriodTimer >= pPosCL->Period)
        {
            pPosCL->PeriodTimer = 0;
            mc_pos_Ramp_Controller(pPosCL->pPosRamp);
            pPosCL->TargetPos = pPosCL->pPosRamp->Pt;
            pPosCL->FdbkPos = FdbkPos;
            OutPutSpd = pid_PID_WZ_Controller(pPosCL->pPosLoopPIDWZ, (pPosCL->TargetPos - FdbkPos));
            pPosCL->Output = pPosCL->pPosRamp->vt + OutPutSpd;	//注: 位置环输出速度必须>速度环最小速度,否则位置环在最低目标速运行时容易不断积累误差(低速控不稳导致的),故需给速度环一定的向下调速空间)
        }
    }
}
