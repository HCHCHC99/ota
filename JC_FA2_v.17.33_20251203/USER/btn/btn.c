/********************************文件说明*************************************
*文件名: btn.c

*作者: Yuchen Tan

*版本: V1.2.3

*功能简介:

*备注:

*修改履历:
------------------------------------V1.1.0------------------------------------
20220916:
1.采用单个按钮驱动状态机产生按钮事件(按下,弹起,长按,连按...),状态机的输入是IO引
脚值(本质上可抽象为1个0/1值).
2.后续兼容矩阵键盘的方式：将每个矩阵按键映射为1个0/1值。这样4*8矩阵按键值可仅用
1个32位整形变量表示(每位表示1个按键状态).
------------------------------------V1.2.0------------------------------------
20221114:
1.模块重构,分为2块：应用层-btn,驱动层-btn_drv;
2.开放给上层调用的接口参数统一用按钮设备的名称索引,而非结构体类型;
20221119:
3.修改btn_Match_Event()对于单个按钮连续调用,仅第一个调用能获取到对应事件的bug;
4.修改部分宏定义名称以使其更准确匹配代码含义!
------------------------------------V1.2.1------------------------------------
20221208:
1.增加按钮的群组式操作(状态,事件)!
------------------------------------V1.2.2------------------------------------
20230309:
1.使用#error替换btn.h的宏定义BTN_NB_MAX,实现在编译阶段的参数检查并报错超限!
2.按钮接口for循环索引的循环次数采用BTN_NB(实际按键数)替换BTN_NB_MAX(已删除,原先
  表示最大按键数,恒等于16),提升程序效率!
------------------------------------V1.2.3------------------------------------
20230321: btn_Init()修改: 在btn_drv_IOGen_Create(),btn_Create()之间,调用
btn_drv_IOGen_Update_Value(),保证对应的BTN_DRV_X(1,2,..)的IOInputValue值被更新为IO端口真
实值后再初始化btn,防止IOInputValue初始值和IO端口真实值不匹配导致btn生成错误的状态和事件!
*****************************************************************************/

/*****************************文件包含(私有)**********************************
*
*备注: 无
*
*****************************************************************************/
#include "btn.h"
#include "btn_drv.h"
#include "main.h"
#include "debug.h"
/*****************************宏定义(私有)************************************
*
*备注: 本文件中,不希望被外部使用或随意修改的宏在这里定义
*
*****************************************************************************/
/*按钮状态机任务运行选项*/
#define BTN_TASK_RUN_PERIOD (20)    //按钮状态机任务前台运行周期(单位: ms)
#define BTN_TASK_RUN_OPT    (1)     /*0-在后台(主循环)运行  1-在前台(定时中断)运行*/
/*0-在后台(主循环)运行(对系统后台程序效率无要求!)
  1-在前台(定时中断)运行(要求系统后台效率不能太低,否则若前台按钮事件产生过快会来不及被后台用户代码获取处理,导致事件丢失!)
按钮状态机在前台运行事件不丢失的理论条件: T_后台运行周期 < T_按钮时间产生周期 <= T_调用按钮任务前台周期BTN_TASK_RUN_PERIOD.
因此,若后台程序效率不高,可适当增大BTN_TASK_RUN_PERIOD*/

/*按钮控制时间定义(单位:ms)*/
#define LONGPRESS_TIME          (1000)  //长按判断时间
#define LONGPRESS_EVT_TICK      (500)   //长按状态下事件生成间隔
#define MULTPRESS_BREAK_TIME    (250)   //连按断开判断时间
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
BTN_HANDLE_t    *hBtn[BTN_NB];  //按钮句柄
BTN_MASK_t      g_BtnExistFlag = 0; //用来标志已创建实例的按钮句柄
/********************************函数定义************************************
*函数名:

*函数功能描述: 按钮驱动-按钮实例操作

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*创建按钮实例*/
int8_t btn_Create(uint8_t Index)
{
    /*参数诊断*/
    DEBUG_ASSERT(Index >= BTN_NB);

    /*创建按钮实例*/
    BTN_HANDLE_t* Handle = (BTN_HANDLE_t*)malloc(sizeof(BTN_HANDLE_t));
    if(!Handle) //注: malloc()失败返回值为0
        return RES_ERR_NEW_OBJ;
    hBtn[Index] = Handle;   //绑定
    g_BtnExistFlag |= (BTN_MASK_t)(1<<Index);

    /*按钮实例初始化*/
    memset(Handle, 0, sizeof(BTN_HANDLE_t));
    Handle->Index = Index;
    Handle->Mask = (BTN_MASK_t)(1<<Index);
    return RES_SUCCESS;
}
/*删除按钮实例*/
int8_t btn_Delete(uint8_t Index)
{
    /*参数诊断*/
    DEBUG_ASSERT(Index >= BTN_NB);

    /*删除按钮实例对应的堆内存块*/
    if(hBtn[Index] != 0)
        free(hBtn[Index]);
    hBtn[Index] = 0;   //解绑定
    g_BtnExistFlag &= (BTN_MASK_t)(~(1<<Index));
    return RES_SUCCESS;
}
/*根据索引获取按钮控制器*/
static BTN_HANDLE_t* btn_Get_Btn_By_Index(uint8_t Index)
{
	/*参数诊断*/
    DEBUG_ASSERT(Index >= BTN_NB);
	
    if(!(g_BtnExistFlag & (BTN_MASK_t)(1<<Index)))
        return 0;   //btn not created
    return hBtn[Index];
}
/*根据索引获取按钮控制器
 * Ret!=0的情况: Mask从低到高位首个不为0的bit对应的按钮控制器句柄(已定义);
 * Ret==0的情况: [1].Mask选中的按钮控制器句柄均未被Create; [2]Mask本身值为0; */
static BTN_HANDLE_t* btn_Get_Btn_By_Mask(BTN_MASK_t Mask, uint8_t *FoundIndex)
{
    uint8_t ValidMask = g_BtnExistFlag & Mask;
    if(!ValidMask)
        return 0;   //btn not created
    for(uint8_t Index = 0; Index < BTN_NB; Index++)
    {
        if(ValidMask & (BTN_MASK_t)(1<<Index))
        {
            if(FoundIndex)
                *FoundIndex = Index;
            return hBtn[Index];
        }
    }
    return 0;   //btn not found
}
/*按钮实例和按钮驱动绑定*/
int8_t btn_Bound_With_Driver(uint8_t Index, uint8_t DriverIndex)
{
    BTN_DRIVER_t Driver = {.BtnDriverGen = btn_drv_IOGen_Get_Handle(DriverIndex)};
    BTN_HANDLE_t* Handle = btn_Get_Btn_By_Index(Index);

    if(!Handle || !Driver.BtnDriverGen)
        return RES_ERR_PARAM;
    Handle->Driver = Driver;   //绑定
    return RES_SUCCESS;
}
/*设置按钮有效键值*/
int8_t btn_Set_ActiveValue(uint8_t Index, BTN_VALUE_t BtnValueMask, BTN_VALUE_t ActiveValue)
{
    BTN_HANDLE_t* Handle = btn_Get_Btn_By_Index(Index);

    Handle->BtnValueMask = BtnValueMask;
    Handle->ActiveValue = ActiveValue;
    return RES_SUCCESS;
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 按钮驱动-单个按钮事件/状态接口

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*获取按钮状态*/
BTN_STATE_t btn_Get_State(uint8_t Index)
{
    BTN_HANDLE_t* Handle = btn_Get_Btn_By_Index(Index);
    if(Handle)
        return Handle->BtnState;
    return E_BTN_STA_UNDEFINED;
}
/*设置按钮事件*/
static void btn_Set_Event(BTN_HANDLE_t* Handle, BTN_EVEVT_t Event, uint16_t argv)
{
    Handle->BtnEvent = Event;
    if(Event == E_BTN_EVT_LONG_PRESS || Event == E_BTN_EVT_LONG_UP)
    {
        Handle->LongPressTime = argv;
    }else
    {
    }
}
/*清除按钮事件*/
int8_t btn_Clr_Event(uint8_t Index)
{
    BTN_HANDLE_t* Handle = btn_Get_Btn_By_Index(Index);
    if(!Handle)
        return -1;
    Handle->BtnEvent = E_BTN_EVT_NONE;
    return 1;
}
/*获取按钮事件*/
int8_t btn_Get_Event(uint8_t Index, BTN_EVEVT_t *Event, uint16_t *argv)
{
    BTN_HANDLE_t* Handle = btn_Get_Btn_By_Index(Index);
    if(!Handle || !Event)
        return -1;
    *Event = Handle->BtnEvent;
    if(*Event == E_BTN_EVT_NONE)
        return 0;
    if(argv)
    {
        if(*Event == E_BTN_EVT_LONG_PRESS || *Event == E_BTN_EVT_LONG_UP)
        {
            *argv = Handle->LongPressTime;
        }else if(*Event == E_BTN_EVT_MULTI_PRESS || *Event == E_BTN_EVT_MULTI_UP)
        {
            *argv = Handle->MuitiPress;
        }else
        {
        }
    }
    Handle->BtnEvent = E_BTN_EVT_NONE;
    return 1;
}
/*匹配按钮事件*/
BOOL btn_Match_Event(uint8_t Index, BTN_EVEVT_t Event, uint16_t argv)
{
    BTN_HANDLE_t* Handle = btn_Get_Btn_By_Index(Index);
    if(!Handle)
        return FALSE;
    if(Handle->BtnEvent == E_BTN_EVT_NONE)
        return FALSE;
    if(Event == E_BTN_EVT_LONG_PRESS)
    {
        if(Handle->LongPressTime >= argv && Handle->BtnEvent == Event)
        {
            Handle->BtnEvent = E_BTN_EVT_NONE;
            return TRUE;
        }
    }else if(Event == E_BTN_EVT_MULTI_PRESS)
    {
        if(Handle->MuitiPress >= argv && Handle->BtnEvent == Event)
        {
            Handle->BtnEvent = E_BTN_EVT_NONE;
            return TRUE;
        }
    }else
    {
        if(Handle->BtnEvent == Event)
        {
            Handle->BtnEvent = E_BTN_EVT_NONE;
            return TRUE;
        }
    }
    return FALSE;
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 按钮驱动-按钮群组事件/状态接口

*函数参数:
@Mask: 选定的目标Btn句柄的Mask值集合
@State: 选定的状态
@Event: 选定的事件
@argv: 选定事件的对应参数(长按事件->时间, 连按时间->次数)

*函数返回值:
*btn_Match_GroupState: 形参Mask选定范围内的所有BtnState与形参State一致的按键句柄的Mask值集合
*btn_Clr_GroupEvent: 形参Mask选定范围内的所有已创建的按键句柄的Mask值集合
*btn_Match_GroupEvent: 形参Mask选定范围内的所有BtnEvent与形参Event,argv对应的按键句柄的Mask值集合

*备注:
*****************************************************************************/
/*匹配群组按钮状态*/
BTN_MASK_t btn_Match_GroupState(BTN_MASK_t Mask, BTN_STATE_t State)
{
    BTN_MASK_t RetMask = 0;
    BTN_HANDLE_t* Handle;
    BTN_MASK_t ValidMask = Mask & g_BtnExistFlag;

    if(!ValidMask)
        return 0;
    for(uint8_t Index = 0; Index < BTN_NB; Index++)
    {
        if(ValidMask & (BTN_MASK_t)(1<<Index))
        {
            Handle = hBtn[Index];
            if(Handle->BtnState == State)
                RetMask |= (BTN_MASK_t)(1<<Index);
        }
    }
    return RetMask;
}
/*清除群组按钮事件*/
BTN_MASK_t btn_Clr_GroupEvent(BTN_MASK_t Mask)
{
    BTN_MASK_t RetMask = 0;
    BTN_HANDLE_t* Handle;
    BTN_MASK_t ValidMask = Mask & g_BtnExistFlag;

    if(!ValidMask)
        return 0;
    for(uint8_t Index = 0; Index < BTN_NB; Index++)
    {
        if(ValidMask & (BTN_MASK_t)(1<<Index))
        {
            Handle = hBtn[Index];
            Handle->BtnEvent = E_BTN_EVT_NONE;
            RetMask |= (BTN_MASK_t)(1<<Index);
        }
    }
    return RetMask;
}
/*匹配群组按钮事件*/
BTN_MASK_t btn_Match_GroupEvent(BTN_MASK_t Mask, BTN_EVEVT_t Event, uint16_t argv)
{
    BTN_MASK_t RetMask = 0;
    BTN_HANDLE_t* Handle;
    BTN_MASK_t ValidMask = Mask & g_BtnExistFlag;
	
    if(!ValidMask)
        return 0;
    for(uint8_t Index = 0; Index < BTN_NB; Index++)
    {
        if(ValidMask & (BTN_MASK_t)(1<<Index))
        {
            Handle = hBtn[Index];
            if(Handle->BtnEvent == E_BTN_EVT_NONE)
                continue;
            if(Event == E_BTN_EVT_LONG_PRESS)
            {
                if(Handle->LongPressTime >= argv && Handle->BtnEvent == Event)
                {
                    Handle->BtnEvent = E_BTN_EVT_NONE;
                    RetMask |= (BTN_MASK_t)(1<<Index);
                }
            }else if(Event == E_BTN_EVT_MULTI_PRESS)
            {
                if(Handle->MuitiPress >= argv && Handle->BtnEvent == Event)
                {
                    Handle->BtnEvent = E_BTN_EVT_NONE;
                    RetMask |= (BTN_MASK_t)(1<<Index);
                }
            }else
            {
                if(Handle->BtnEvent == Event)
                {
                    Handle->BtnEvent = E_BTN_EVT_NONE;
                    RetMask |= (BTN_MASK_t)(1<<Index);
                }
            }
        }
    }
    return RetMask;
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 按钮驱动-状态控制器

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*按钮控制器任务定时器(定时调用(suggest period: 1-5ms)) */
void btn_State_Timer(void)
{
    BTN_HANDLE_t* Handle = 0;
    for(uint8_t Index = 0; Index < BTN_NB; Index++)
    {
        if(g_BtnExistFlag & (BTN_MASK_t)(1<<Index))
        {
            Handle = hBtn[Index];
            Handle->BtnValue = btn_drv_IOGen_Get_Value(Handle->Driver) & Handle->BtnValueMask;

            /*按钮生效翻转判断(用于连按检测)*/
            if(Handle->BtnValue != Handle->BtnValuePrev)
            {
                Handle->Toggle = TRUE;
                Handle->BtnValuePrev = Handle->BtnValue;
            }

            /*统计按钮有效按下时间*/
            if(Handle->BtnValue == Handle->ActiveValue)
            {
                if(Handle->ActiveTime < 60000)
                    Handle->ActiveTime++;
                Handle->InactiveTime = 0;
            }else
            {
                Handle->ActiveTime = 0;
                if(Handle->InactiveTime < 60000)
                    Handle->InactiveTime++;
            }
        }
    }
}
/*按钮控制状态机(主循环调用 or 定时调用(suggest period: 1-5ms))*/
void btn_State_Controller(void)
{
    BTN_HANDLE_t* Handle = 0;
    BTN_VALUE_t BtnValue = 0;
    BTN_STATE_t State = (BTN_STATE_t)0;
    for(uint8_t Index = 0; Index < BTN_NB; Index++)
    {
        if(g_BtnExistFlag & (BTN_MASK_t)(1<<Index))
        {
            Handle = hBtn[Index];
            BtnValue = Handle->BtnValue & Handle->BtnValueMask; //给状态机当前周期的判断值需用临时变量保存,防止在其他地方变化!
            State = Handle->BtnState;
            /*按钮状态机*/
            switch(State)
            {
                case E_BTN_STA_RELEASED:
                    if(BtnValue == Handle->ActiveValue)
                    {
                        Handle->BtnState = E_BTN_STA_FIRSTPRESSHOLD;
                        btn_Set_Event(Handle, E_BTN_EVT_PRESS, 0);
                    }
                    break;

                case E_BTN_STA_FIRSTPRESSHOLD:
                    if(BtnValue != Handle->ActiveValue)    //首次按键松开
                    {
                        Handle->BtnState = E_BTN_STA_FIRSTUP;
                        btn_Set_Event(Handle, E_BTN_EVT_UP, 0);
                    }else
                    {
                        if(Handle->ActiveTime >= LONGPRESS_TIME)   //首次按键转为长按
                        {
                            Handle->BtnState = E_BTN_STA_LONGPRESS;
                            Handle->LongPressEvtTime = 0;
                        }
                    }
                    break;

                case E_BTN_STA_LONGPRESS:
                    if(BtnValue != Handle->ActiveValue)
                    {
                        Handle->BtnState = E_BTN_STA_RELEASED;
                        btn_Set_Event(Handle, E_BTN_EVT_LONG_UP, 0);
                    }else
                    {
                        if(abs((int)Handle->ActiveTime - (int)Handle->LongPressEvtTime) >= LONGPRESS_EVT_TICK)
                        {
                            Handle->LongPressEvtTime = Handle->ActiveTime;
                            btn_Set_Event(Handle, E_BTN_EVT_LONG_PRESS, Handle->ActiveTime);
                        }
                    }
                    break;

                case E_BTN_STA_FIRSTUP:
                    if(BtnValue == Handle->ActiveValue)    //再次按下
                    {
                        Handle->BtnState = E_BTN_STA_MULTIPRESS_LOOP;
                        Handle->MuitiPress = 1;
                    }else
                    {
                        if(Handle->InactiveTime >= MULTPRESS_BREAK_TIME)
                            Handle->BtnState = E_BTN_STA_RELEASED;
                    }
                    break;

                case E_BTN_STA_MULTIPRESS_LOOP:
                    if(Handle->Toggle)
                    {
                        Handle->Toggle = FALSE;
                        if(BtnValue == Handle->ActiveValue)
                        {
                            Handle->MuitiPress++;
                            btn_Set_Event(Handle, E_BTN_EVT_MULTI_PRESS, 0);
                        }else
                        {
                            btn_Set_Event(Handle, E_BTN_EVT_MULTI_UP, 0);
                        }
                    }else
                    {
                        if(Handle->ActiveTime >= LONGPRESS_TIME)
                        {
                            Handle->BtnState = E_BTN_STA_LONGPRESS;
                            Handle->LongPressEvtTime = 0;
                        }else
                        {
                            if(Handle->InactiveTime >= MULTPRESS_BREAK_TIME)
                                Handle->BtnState = E_BTN_STA_RELEASED;
                        }
                    }
                    break;

                default:
                    break;
            }
        }
    }
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 按钮驱动-应用入口

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*初始化应用入口*/
int8_t btn_Init(void)
{
    BTN_IO_t IOInit[2] = {{GPIO_IN1_KUP_PORT, GPIO_IN1_KUP_PIN},\
                        {GPIO_IN2_KDOWN_PORT, GPIO_IN2_KDOWN_PIN}};
    /*按钮信号驱动始化*/
    btn_drv_IOGen_Create(BTN_DRV_1, IOInit, sizeof IOInit / sizeof IOInit[0]);
	//IOInputValue值更新为真实值后再初始化按钮,防止IOInputValue值和端口不匹配,btn生成错误的状态和事件!
	while(btn_drv_IOGen_Update_Value(BTN_DRV_1) != RES_SUCCESS);	//此函数未在1ms中断调用,按键消抖时间不准!

	/*按钮初始化*/
	if(btn_Create(BTN_1) == RES_SUCCESS)
	{
		btn_Bound_With_Driver(BTN_1, BTN_DRV_1);
		btn_Set_ActiveValue(BTN_1, (1<<0), 0);
	}
	if(btn_Create(BTN_2) == RES_SUCCESS)
	{
		btn_Bound_With_Driver(BTN_2, BTN_DRV_1);
		btn_Set_ActiveValue(BTN_2, (1<<1), 0);
	}
//	if(btn_Create(BTN_3) == RES_SUCCESS)
//	{
//		btn_Bound_With_Driver(BTN_3, BTN_DRV_1);
//		btn_Set_ActiveValue(BTN_3, (1<<2), 0);
//	}
    return RES_SUCCESS;
}
/*后台应用入口(要求: 调用的所有函数参数必须是抽象的设备名称索引)*/
void btn_Loop_Task(void)
{
#if (BTN_TASK_RUN_OPT == 0)
    btn_State_Controller();
#endif
    btn_Test();
}
/*定时应用入口(1ms调用1次)*/
void btn_Timer_Task_1ms(void)
{
    static uint8_t s_Period = 0;

    for(uint8_t i = BTN_DRV_1; i < BTN_DRV_NB ; i++)
    {
        btn_drv_IOGen_Update_Value(i);
    }
    btn_State_Timer();
    if(++s_Period >= BTN_TASK_RUN_PERIOD)
    {
        s_Period = 0;
#if (BTN_TASK_RUN_OPT == 1)
        btn_State_Controller();
#endif
    }
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 按钮驱动-模块测试

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*打印按钮事件*/
static void btn_Event_Printf(uint8_t Index, BTN_EVEVT_t Evt, uint16_t *argv)
{
//    if(Index == BTN_1)
//        debug_Printf_String("BTN1: ");
//    else if(Index == BTN_2)
//        debug_Printf_String("BTN2: ");
//    else if(Index == BTN_3)
//        debug_Printf_String("BTN3: ");
//    else if(Index == BTN_4)
//        debug_Printf_String("BTN4: ");
//    else if(Index == BTN_5)
//        debug_Printf_String("BTN5: ");
//    else if(Index == BTN_6)
//        debug_Printf_String("BTN6: ");

//    if(Evt == E_BTN_EVT_PRESS)
//    {
//        debug_Printf_String("Press");
//    }else if(Evt == E_BTN_EVT_LONG_PRESS)
//    {
//        debug_Printf_String("LongPress, time(ms):");
//        debug_Printf_Data_InString(argv, 1, 2);
//    }else if(Evt == E_BTN_EVT_LONG_UP)
//    {
//        debug_Printf_String("LongUp,    time(ms):");
//        debug_Printf_Data_InString(argv, 1, 2);
//    }else if(Evt == E_BTN_EVT_UP)
//    {
//        debug_Printf_String("Up");
//    }else if(Evt == E_BTN_EVT_MULTI_PRESS)
//    {
//        debug_Printf_String("MultiPress, times:");
//        debug_Printf_Data_InString(argv, 1, 2);
//    }else if(Evt == E_BTN_EVT_MULTI_UP)
//    {
//        debug_Printf_String("MultiUp,    times:");
//        debug_Printf_Data_InString(argv, 1, 2);
//    }
}
/*模块测试*/
void btn_Test(void)
{
#if 1
    BTN_EVEVT_t CurrentEvent;
    uint16_t argv = 0;
    if(btn_Get_Event(BTN_1, &CurrentEvent, &argv))
        btn_Event_Printf(BTN_1, CurrentEvent, &argv);
    if(btn_Get_Event(BTN_2, &CurrentEvent, &argv))
        btn_Event_Printf(BTN_2, CurrentEvent, &argv);
    if(btn_Get_Event(BTN_3, &CurrentEvent, &argv))
        btn_Event_Printf(BTN_3, CurrentEvent, &argv);
    if(btn_Get_Event(BTN_4, &CurrentEvent, &argv))
        btn_Event_Printf(BTN_4, CurrentEvent, &argv);
    if(btn_Get_Event(BTN_5, &CurrentEvent, &argv))
        btn_Event_Printf(BTN_5, CurrentEvent, &argv);
    if(btn_Get_Event(BTN_6, &CurrentEvent, &argv))
        btn_Event_Printf(BTN_6, CurrentEvent, &argv);
    if(btn_Get_Event(BTN_7, &CurrentEvent, &argv))
        btn_Event_Printf(BTN_7, CurrentEvent, &argv);
#else
    if(btn_Match_Event(BTN_1, E_BTN_EVT_PRESS, 0))
        btn_Event_Printf(BTN_1, E_BTN_EVT_PRESS, 0);
    if(btn_Match_Event(BTN_1, E_BTN_EVT_UP, 0))
        btn_Event_Printf(BTN_1, E_BTN_EVT_UP, 0);
    if(btn_Match_Event(BTN_1, E_BTN_EVT_MULTI_PRESS, 0))
        btn_Event_Printf(BTN_1, E_BTN_EVT_MULTI_PRESS, 0);
    if(btn_Match_Event(BTN_1, E_BTN_EVT_MULTI_UP, 0))
        btn_Event_Printf(BTN_1, E_BTN_EVT_MULTI_UP, 0);
    if(btn_Match_Event(BTN_1, E_BTN_EVT_LONG_PRESS, 0))
        btn_Event_Printf(BTN_1, E_BTN_EVT_LONG_PRESS, 0);
    if(btn_Match_Event(BTN_1, E_BTN_EVT_LONG_UP, 0))
        btn_Event_Printf(BTN_1, E_BTN_EVT_LONG_UP, 0);
#endif
}
