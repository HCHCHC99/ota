/********************************�ļ�˵��*************************************
*�ļ���: system.c

*����: Yuchen Tan

*���: V1.0.0

*���ܼ��:

*��ע: ��

*�޸�����:
------------------------------------V1.0.1------------------------------------
20231106
1��������λ�����ù��ܣ�ͨ����λ�����õ�����ܺ�ϵͳ����

*****************************************************************************/

/*****************************�ļ�����(˽��)**********************************
*
*��ע: ��
*
*****************************************************************************/
#include "system.h"
#include "main.h"
#include "mc_app.h"
#include "btn.h"
#include "adc_adapter.h"
#include "uart_adapter.h"
#include "flash_adapter.h"
#include "gpio_adapter.h"
#include "timer_adapter.h"
#include "delay_adapter.h"
#include "sys_cfg.h"
#include "modbus.h"    
#include "queue.h"
#include "can_adapter.h"
#include "co.h"
#include "co_sdo.h"  
#include "crc.h"

#include "hz_timer.h"
#include "mc_app.h"
#include "mc_cur.h"
#include "motor_adc.h"
#include "canJ1939.h"
#include "uds_diagnostic.h"
#include "isotp_transport.h"
#include "rtt_log.h"
//#include "debug.h"
/*****************************�궨��(˽��)************************************
*
*��ע: ���ļ���,��ϣ�����ⲿʹ�û������޸ĵĺ������ﶨ��
*
*****************************************************************************/
/*�������*/
#define SW_VERSION				(1)
#define SW_MINI_VERSION			(1)

/*ע: FLAG,VALUE���뵥��������������,��ֹVALUE��һ��λ�����ֵ,չ�������*/
#define SYSTEM_SET_FLAG(FLAG, VALUE)    ((FLAG) |= (VALUE))
#define SYSTEM_CLR_FLAG(FLAG, VALUE)    ((FLAG) &= ~(VALUE))
#define SYSTEM_GET_FLAG(FLAG, VALUE)    ((FLAG) & (VALUE))
#define SYSTEM_MATCH_FLAG(FLAG, VALUE)  ((FLAG) == (VALUE))

/*ϵͳLED����*/
#define LED1                    (0)
#define LED2                    (1)

/*ϵͳ��������*/
#define KEY_MOTOR_DOWN          (BTN_2)     //����½����м�
#define KEY_MOTOR_UP            (BTN_1)     //����������м�

/*�����λ��׼����*/
#define MOTOR_POS_TOP           (0x01)
#define MOTOR_POS_BTM           (0x10)

/*ϵͳ���������־*/
#define M1_UP                   (0x01)
#define M1_DOWN                 (0x01<<8)
#define M1_MOVE                 (M1_UP | M1_DOWN)
#define M2_UP                   (0x02)
#define M2_DOWN                 (0x02<<8)
#define M2_MOVE                 (M2_UP | M2_DOWN)

//
/*��λ��־*/
#define M1_TOP                  (0x01)
#define M1_BTM                  (0x01<<8)
#define M2_TOP                  (0x02)
#define M2_BTM                  (0x02<<8)
#define M_LOCKED_FLAG			(0x01)


/*ϵͳ��������ָʾ����˸ʱ��*/
#define SYSTEM_RUN_LED_TIME		500		
/*��λHALLֵ����*/
//#define TOP_POS_HALLDATA       		(MAX_ROUTE)
/*��λHALLֵ����*/
#define BTM_POS_HALLDATA       		(0)
/*��λHALLֵ����*/
//#define CAN_ZERO_POS_HALLDATA       (500000)


/*�Ƹ��г�λ����������*/
#define COLUMN_POS_ZERO         (0)     //��λλ��(��λ��ɵ�λ��)
#define COLUMN_POS_SET_BTM      (1)     //������������λ
#define COLUMN_POS_TOP_LIMIT    (2)     //��������г�
#define COLUMN_POS_SET_TOP      (3)     //������������λ
#define COLUMN_POS_MEMORY1      (4)     //����λ��1
#define COLUMN_POS_MEMORY2      (5)     //����λ��2
#define COLUMN_POS_MEMORY3      (6)     //����λ��3
#define COLUMN_POS_MEMORY4      (7)     //����λ��4
#define COLUMN_POS_NB           (8)

/*FLASH�����־λ*/
#define FS_MOTOR_SATE           (0x01)
#define FS_MOTOR_LIMIT          (0x02)
#define FS_SYS_PARAM			(0x08)
#define	FS_SYS_CONFIG			(0x10)
#define FS_MOTOR_CONFIG			(0x20)
#define	FS_BOOT_CONFIG			(0x40)


/*FLASH���������*/
#define FSA_MOTOR_STATE         BLOCK_L1_ADDR   //����λ��
#define FSA_MOTOR_LIMIT         BLOCK_L2_ADDR   //������λ
#define FSA_SYS_PARAM     		BLOCK_L3_ADDR   //���ϵͳ����(CanOpen�ڵ���Ϣ)

/*ģ��D����������ֵ����*/
#define	MSG_RET_OK			(1)
#define	MSG_RET_ERR_MA		(-1)	//�ڴ����ʧ��
#define	MSG_RET_ERR_PARAM	(-2)	//�ӿڲ�������
#define	MSG_RET_ERR_PACK	(-3)	//���ݴ��ʧ��
#define	MSG_RET_ERR_CRC		(-4)	//����У��ʧ�� 



/**************************�������ͼ��ṹ����(˽��)***************************
*
*��ע: ���ļ���,��ϣ�����ⲿʹ�õ����ݽṹ�����������ﶨ��
*
*****************************************************************************/

/*Ӧ�ò㶨��-modbus�Ĵ���*/
typedef enum
{
    MBREG_MOTOR_SLAVE_ADDR = 40001,     //�����ַ(RW)
    MBREG_MOTOR_CMD = 40050,            //�����������(W)
    MBREG_MOTOR_RESET,                  //��������λ(W)
    MBREG_MOTOR_TARGETSPEED_POS,        //������е�Ŀ��λ��(W)
    MBREG_MOTOR_FDBKPOS_ZERO,           //������õ�ǰλ��Ϊ��λ(W)
    MBREG_MOTOR_RPM,                    //����ٶ�(RW)
    MBREG_MOTOR_FDBKPOS,                //�����ǰ�߶�(R)
    MBREG_MOTOR_STATE,                  //���״̬(R)
	MBREG_MOTOR_CLR_FAULT,				//����������(W)
}MODBUS_REG_t;

/*modbus�ӿڵ����������*/
typedef enum
{
	e_mmc_none = 0,
	e_mmc_up,
	e_mmc_dn,
	e_mmc_stop,
    e_mmc_reset,
	e_mmc_clrfault,
	e_mmc_goto,
    e_mmc_poszero,
}mod_m_cmd_t;
 

/*modbus���Ͷ���*/
typedef struct
{
    uint8_t     Interface;  /*���ݽӿ�����*/
    QUEUE_t     *QSend;     /*���ݷ��Ͷ���*/
    uint8_t     State;      /*��Ϣ������״̬*/
    uint8_t     Slave_Flag;
}MSG_HANDLER_t;

/*modbus�Ĵ�����ֵ��������*/
MB_REG_KVP_t g_KVPTable[] =
{
    {MBREG_MOTOR_SLAVE_ADDR, 0},
    {MBREG_MOTOR_CMD, 0},
    {MBREG_MOTOR_RESET, 0},
    {MBREG_MOTOR_TARGETSPEED_POS, 0},
    {MBREG_MOTOR_FDBKPOS_ZERO, 0},
    {MBREG_MOTOR_RPM, 0},
    {MBREG_MOTOR_FDBKPOS, 0},
    {MBREG_MOTOR_STATE, 0},
	{MBREG_MOTOR_CLR_FAULT, 0},
    //��Ч�Ĵ���ֵ(�ݶ�0xFFFF),��Ϊ������������־,������ڽ�β.
    {0xFFFF, 0},
};



/*CAN�ӿ���Ϣ����*/
//�������״̬
typedef enum
{
	e_cms_stop = 0,
	e_cms_up = 0x01,
	e_cms_dn = 0x10,
}can_m_sta_t;

//������״̬
typedef enum
{
	e_css_normal = 1,
	e_css_fault_ovc = 21,
	e_css_fault_hall = 27,
	e_css_fault_hot = 213,
	e_css_fault_udv = 221,
	e_css_fault_ovv = 222,
	e_css_lost_pos = 255,
}can_sys_sta_t;    


/*ϵͳ����״̬����*/
typedef enum
{
    E_SYS_STATE_INIT = 0,       //�ϵ��ʼ��
    E_SYS_STATE_ZERO,     		//������λ
    E_SYS_STATE_IDLE,           //�����޶���
    E_SYS_STATE_MOTOR_RUN,      //�������
    E_SYS_STATE_MOTOR_GOTOPOS,  //����Զ����е��̶�λ��(��λ��)
    E_SYS_STATE_FAULT,          //ϵͳ����
    E_SYS_STATE_SETTINGS,       //���ü���λ��
    E_SYS_STATE_SETZEROPOS,     //���õ�ǰλ��Ϊ��λ
	E_SYS_CONFIG_INIT,			//���ó�ʼ��
    SYSTEM_STATE_NB,
}SYS_STATE_t;

/*ϵͳ״̬��������*/
typedef void (*FP_t)(void*);

/*�������ö���*/
typedef enum
{
    E_READY_TO_SET = 0,
    E_SET_MEMORY_POS_1,
    E_SET_MEMORY_POS_2,
    E_SET_MEMORY_POS_3,
    E_SET_MEMORY_POS_4,
//  E_SET_TOP_LIMIT_POS,
//  E_SET_BTM_LIMIT_POS,
//  E_CANCEL_LIMIT_POS,
}SETPOS_TYPE_t;

/*ϵͳ����״̬�ṹ��G��*/
typedef struct
{
    float           ModbusColumnPos;    //�Ƹ�Ŀ��λ��
    uint8_t     	MotorMoveCmd;       //�����������      modbusΪʲôû���������  
    /*���������ź�*/
    uint8_t     	MCMode;             //0:����  1:�ٶȱջ�   2:�ٶ�λ�ñջ�
    uint8_t     	MSRMode;            //0:PWM�źŵ���     1:AD�źŵ���
    uint8_t     	ComMode;            //0:485�ӿ�           1:MODBUS�ӿ�
    uint8_t     	MotorDirSignal;     //0:��ת          1:��ת
    /*ϵͳ״̬*/
    SYS_STATE_t 	State;              //��ǰ״̬
    uint8_t     	Step;               //��ǰ״̬�Ӳ���
    /*ĸ�ߵ�ѹ*/
    uint16_t    	BusVoltageMV;
    /*���������ر�־*/
    int32_t     	RunDir;             //�����������
//    uint8_t     	MotorMoveCmd;       //�����������
    MOTOR_SPD_t 	MotorSpeed;         //�Ƹ˱ջ�Ŀ���ٶ�
    uint16_t    	MotorDC;            //�Ƹ˿���Ŀ��ռ�ձ�
    uint16_t    	LimitFlag;          //��λ��־
    uint16_t    	MotorFlag;          //���������־
    /*ϵͳ��־*/
    uint8_t     	MemoryPosSetFlag;   //�����õļ���λ�ñ�־
    uint16_t    	FaultFlag;          /*ϵͳ�쳣��־*/
    uint8_t			SaveIntoFlash;      /*Flash��ȡ��־*/
    uint8_t     	ZeroFound;          /*���ҵ�����λ��־*/
	uint8_t			HandControlling;	/*�ֶ�������Ч��־*/
	uint8_t			ResetHandControlling;/*��λ�ֿر�־*/
	mod_m_cmd_t		ModbusCmd;			/*ModbusͨѶ����-����*/
	can_m_cmd_t		CanCmd;				/*CanͨѶ����-����*/
    /*���е�Ŀ��λ��*/
    MOTOR_POS_t 	GotoHallPos;
    /*�Ƹ�λ��(��λ: mm)*/
    float       	ColumnPosMM[MOTOR_NB];          //�Ƹ˵�ǰλ��
    float       	ColumnSpcPosMM[COLUMN_POS_NB];  //�Ƹ�ָ��λ��
    /*�ֿ�����ʾ����*/
    uint8_t     	HSSendTimer;
	SYSTEM_CONFIG_t SysCon;
	/*����������ʱ��*/
	uint16_t 		CfgCount;
	/* ��ѹ��־λ�͹�ѹ����ֵ */
	uint8_t			OVVFlag;
	uint16_t		OVVCount;
	/* ����_��flash��־ */
	uint16_t		PowerDownSaveFlash;
	/* �ϵ�yλ��־λ */
	uint8_t 		PowerOnResetFlag;
	uint8_t			MotorLockedRotor;    
}SYSTEM_t;

/*ɲ��״̬�ṹ��G��*/
float Brake_Store_Pos; //ɲ������ʱ�Ƹ��Ѵ����λ��
float Current_Pos;//ɲ������ʱ�Ƹ�ʵʱ��λ��
float Brake_Pos_1;//��е����1mm
uint8_t Brake_State;  //ɲ��״̬  // 1 ����λ�ͼ����֮�䣻2 ����λ�ͼ����֮�䣻
uint8_t Brake_Fail_Flag; //  ɲ��ʧ��
uint16_t receive_ceiling_switch = 0;//��������λ����
uint16_t receive_bottom_switch = 0;//��������λ����
uint16_t receive_ceiling_flag = 0;
uint16_t receive_bottom_flag = 0;
uint8_t First_OVC_Flag = 0;//�״ι�����λ��
/*****************************��������(˽��)**********************************
*
*��ע: ���ļ���,��ϣ�����ⲿ���õĺ���ͳһ����������
*
*****************************************************************************/
static void system_Init_Handler(void*);
static void system_FindZero_Handler(void*);
static void system_Idle_Handler(void*);
static void system_MotorRun_Handler(void*);
static void system_MotorGotoPos_Handler(void*);
static void system_Fault_Handler(void*);
static void system_Settings_Handler(void*);
static void system_SetZeroPos_Handler(void*);
static void system_ConfigInit_Handler(void*);

static void system_CB_CanOpenSdoTx(uint8_t Index);
void system_FSM_StateJump(SYSTEM_t* pSystem, SYS_STATE_t NextState);
float system_Hall_PER_MM(void);
uint16_t system_Get_Fault(uint16_t FaultMask);
/********************************��������*************************************
*
*��ע: ��Ҫ��.h�ж������,��ֹ���.c�������.h,���±���󱨴������ظ�����
*
*****************************************************************************/
uint32_t    g_Counter = 0;
uint16_t    g_SwdtCounter = 0;

/*ϵͳ״̬����״̬����ָ��(����˳�����SYS_STATE_t��״̬˳��һ��)*/
FP_t SystemStateHandler[SYSTEM_STATE_NB] =
{
    system_Init_Handler,
    system_FindZero_Handler,
    system_Idle_Handler,
    system_MotorRun_Handler,
    system_MotorGotoPos_Handler,
    system_Fault_Handler,
    system_Settings_Handler,
    system_SetZeroPos_Handler,
	system_ConfigInit_Handler,
};


SYSTEM_t 					System;
ADC_CH_CTRL_t   			hADCChannalMSR; //�����ٶȵ�����ťģ�����

ADC_CH_CTRL_t   			hADCChannalTemp;//�¶Ȳ���



MSG_HANDLER_t   			hMsgHandler[1];
MB_CTRL_t       			hModbusLink;
UART_CTRL_t     			hUartRS485;    

CAN_CTRL_t					hCANController;	//CAN������
co_node_t					sCONode;		//CANOpen�ڵ�  
extern uint32_t 			CODict_Index2000[20];  

UART_CTRL_t     			hUartReserved;    
syscon_uart_cfg_handle_t	hUartData;

//UART_CTRL_t       hUartHS;    //�ֿ�������������ʾ

/********************************��������************************************
*������:

*������������: ϵͳ����-�Ƹ�λ�ü���

*��������: ��

*��������ֵ: ��

*��ע:
*****************************************************************************/
/*ת��: �Ƹ��ٶ�mm/s->���ת��rpm*/
MOTOR_SPD_t system_ColumnSpeed_2_MotorRPM(float Speed_mmps)    //mm/s
{
    MOTOR_SPD_t TargetRPM = 0;
    float TargetRPS_f = 0;	//���ת��/s

    TargetRPS_f = Speed_mmps * (System.SysCon.Sys_GearRatio) / (System.SysCon.Sys_Lead);
    TargetRPM = (MOTOR_SPD_t)(TargetRPS_f * 60);
    return TargetRPM;
}
/*ת��: ���ת��rpm->�Ƹ��ٶ�mm/s*/
float system_MotorRPM_2_ColumnSpeed(MOTOR_SPD_t MotorRpm)
{
    float ColumnSpd_mmps = 0;

	ColumnSpd_mmps = (float)MotorRpm * (System.SysCon.Sys_Lead) / (System.SysCon.Sys_GearRatio);
	ColumnSpd_mmps = ColumnSpd_mmps / 60;
    return ColumnSpd_mmps;
}
/*�޸��Ƹ��ٶ�*/
MOTOR_SPD_t system_Modify_Column_Speed(uint8_t Column, float Speed_mmps)    //mm/s
{
    MOTOR_SPD_t TargetRPM = 0;

	TargetRPM = system_ColumnSpeed_2_MotorRPM(Speed_mmps);
    mc_app_Write_Param((1<<Column), e_map_targetspd, TargetRPM);
    return TargetRPM;
}
/*�����Ƹ��ض�λ��*/
void system_Set_Column_SpecificPos(uint8_t ColumnPosIndex, float ColumnSpcPosMM)
{
    //assert(ColumnPosIndex < COLUMN_POS_NB);

    System.ColumnSpcPosMM[ColumnPosIndex] = ColumnSpcPosMM;
}
/*��ȡ�Ƹ��ض�λ��*/
float system_Get_Column_SpecificPos(uint8_t ColumnPosIndex)
{
    //assert(ColumnPosIndex < COLUMN_POS_NB);

    return System.ColumnSpcPosMM[ColumnPosIndex];
}
/*ת��: �Ƹ��г�mm->���HALL����ֵ*/
MOTOR_POS_t system_Column_Pos_2_HallData(float ColumnSpcPosMM)
{
    MOTOR_POS_t MotorHallData = 0;
    float   HallsPerMM = system_Hall_PER_MM();

    MotorHallData = (MOTOR_POS_t)(ColumnSpcPosMM * HallsPerMM) + BTM_POS_HALLDATA;
    return MotorHallData;
}
/*ת��: ���HALL����ֵ->�Ƹ��г�mm*/
float system_Column_HallData_2_Pos(MOTOR_POS_t MotorHallData)
{
    float ColumnSpcPosMM = 0;
    float HallsPerMM = system_Hall_PER_MM();

    if(MotorHallData > BTM_POS_HALLDATA)
    {
        ColumnSpcPosMM = (float)(MotorHallData - BTM_POS_HALLDATA) / HallsPerMM;
        return ColumnSpcPosMM;
    }
    return 0;
}
/*��ȡ�Ƹ��г�λ��(1=0.1mm)*/
float system_Get_Column_CurrentPos(uint8_t Column)
{
    int32_t ReadMotorParam = 0;

    if(mc_app_Read_Param((1<<Column), e_map_fdbkpos, &ReadMotorParam))
    {
        return system_Column_HallData_2_Pos(ReadMotorParam);
    }
    return NAN;
}
/*�Ƹ�λ�ø���*/
void system_Column_Update_Pos_Signal(void)
{
	if((System.SysCon.Sys_CommunicationType == E_MODBUS)||(System.SysCon.Sys_CommunicationType == E_NOCOM))
	{
     if(System.State == E_SYS_STATE_ZERO || SYSTEM_GET_FLAG(System.FaultFlag, FAULT_POS))
		System.ColumnPosMM[0] = 0;
	else
		System.ColumnPosMM[0] = system_Get_Column_CurrentPos(0);
	}
	if(System.SysCon.Sys_CommunicationType == E_CAN)
	{
		if(!System.ZeroFound)
		{
			System.ColumnPosMM[0] = 0;
			gpio_adapter_Reset_Pin(AT_TOP_GPIO_Port, AT_TOP_Pin);
			gpio_adapter_Reset_Pin(AT_BTM_GPIO_Port, AT_BTM_Pin);
		}else
		{
			//λ�ø���
			System.ColumnPosMM[0] = system_Get_Column_CurrentPos(0);

		}
		
	}
	//�����λ�ź�(�����Ƹ���λ�����ź�)
	if(SYSTEM_GET_FLAG(System.LimitFlag, M1_TOP))
		gpio_adapter_Set_Pin(AT_TOP_GPIO_Port, AT_TOP_Pin);
	else
		gpio_adapter_Reset_Pin(AT_TOP_GPIO_Port, AT_TOP_Pin);
	
	if(SYSTEM_GET_FLAG(System.LimitFlag, M1_BTM))
		gpio_adapter_Set_Pin(AT_BTM_GPIO_Port, AT_BTM_Pin);
	else
		gpio_adapter_Reset_Pin(AT_BTM_GPIO_Port, AT_BTM_Pin);
}
/********************************��������************************************
*������:

*������������: ϵͳ����-�����ع���

*��������: ��

*��������ֵ: ��

*��ע:
*****************************************************************************/
/*���ֹͣ�ж�*/
BOOL system_Is_Motor_Stop(uint16_t Motor)
{
    BOOL Ret = TRUE;
    if(Motor & MOTOR1)
    {
        if(mc_app_Get_State(MOTOR1) != e_mas_idle)
            Ret = FALSE;
        else
            SYSTEM_CLR_FLAG(System.MotorFlag, M1_MOVE);
    }
    if(Motor & MOTOR2)
    {
        if(mc_app_Get_State(MOTOR2) != e_mas_idle)
            Ret = FALSE;
        else
            SYSTEM_CLR_FLAG(System.MotorFlag, M2_MOVE);
    }
    return Ret;
}
/*���ִ�е��������*/
void system_Motor_Move(uint16_t Motor, mc_app_cmd_t Cmd, int32_t argv)
{
    uint16_t MoveFlag = 0;
    if(Cmd != e_mac_start_closeloop && Cmd != e_mac_start_openloop)
        return;
    if(argv == DIR_UP)
        MoveFlag = M1_UP;
    else
        MoveFlag = M1_DOWN;
    if(Motor & MOTOR1)
    {
        mc_app_Set_Single_Motor_Cmd(MOTOR1, Cmd, argv);
        SYSTEM_SET_FLAG(System.MotorFlag, MoveFlag);
    }
    if(Motor & MOTOR2)
    {
        mc_app_Set_Single_Motor_Cmd(MOTOR2, Cmd, argv);
        SYSTEM_SET_FLAG(System.MotorFlag, MoveFlag << 1);
    }
}
/*���ִ�е����б��ֹͣ*/
void system_Motor_Ramp_Stop(uint16_t Motor, mc_app_cmd_t Cmd)
{
    if(Cmd != e_mac_stop_closeloop && Cmd != e_mac_stop_openloop)
        return;
    if(Motor & MOTOR1)
    {
        mc_app_Set_Single_Motor_Cmd(MOTOR1, Cmd, 0);
    }
    if(Motor & MOTOR2)
    {
        mc_app_Set_Single_Motor_Cmd(MOTOR2, Cmd, 0);
    }
}
/*���ִ�е����ֹͣ*/
void system_Motor_Stop(uint16_t Motor)
{
    if(Motor & MOTOR1)
    {
        mc_app_Set_Single_Motor_Cmd(MOTOR1, e_mac_stop, 0);
        SYSTEM_CLR_FLAG(System.MotorFlag, M1_MOVE);
    }
    if(Motor & MOTOR2)
    {
        mc_app_Set_Single_Motor_Cmd(MOTOR2, e_mac_stop, 0);
        SYSTEM_CLR_FLAG(System.MotorFlag, M2_MOVE);
    }
}
/*����������ʱ�쳣*/
void system_Clear_Motor_Fault(uint16_t Motor)
{
    if(Motor & MOTOR1)
    {
		SYSTEM_CLR_FLAG(System.FaultFlag, FAULT_M1_ALL);
		mc_app_Write_Param(MOTOR1, e_map_fault, 0);
    }
    if(Motor & MOTOR2)
    {
        SYSTEM_CLR_FLAG(System.FaultFlag, FAULT_M2_ALL);
        mc_app_Write_Param(MOTOR2, e_map_fault, 0);
    }
}
/*�������ʱ����*/
void system_Motor_Protect(void)
{
    int32_t MFault[4] = {0};

    mc_app_Read_Param(MOTOR1, e_map_fault, &MFault[0]);
    mc_app_Read_Param(MOTOR2, e_map_fault, &MFault[1]);
    if(MFault[0] & MOTOR_FAULT_OVC)
        SYSTEM_SET_FLAG(System.FaultFlag, FAULT_M1_OVC);
    if(MFault[0] & MOTOR_FAULT_HALL)
        SYSTEM_SET_FLAG(System.FaultFlag, FAULT_M1_HAB);
    if(MFault[1] & MOTOR_FAULT_OVC)
        SYSTEM_SET_FLAG(System.FaultFlag, FAULT_M2_OVC);
    if(MFault[1] & MOTOR_FAULT_HALL)
        SYSTEM_SET_FLAG(System.FaultFlag, FAULT_M2_HAB);
}
/*�������*/
void system_Motor_Task(void)
{
    /*�������*/
    mc_app_Loop_Task();

    /*�������ʱ����*/
    system_Motor_Protect();
}

/********************************��������************************************
*������:

*������������: ϵͳ���ù���

*��������: ��

*��������ֵ: ��

*��ע:
*****************************************************************************/
/* �Ƹ˲�����ʼ��*/
void system_Motor_Init(SYSTEM_t* System)
{
	SYSTEM_t* pSystem = System;
	syscon_cfg_handle_t hSystemConfig = sys_cfg_Hadnle();
    
	/*�Ƹ˲���*/
	pSystem->SysCon.Sys_GearRatio = ((float)hSystemConfig.Config_GearRatio / 1000);                 //���ּ��ٱ�
	pSystem->SysCon.Sys_Lead = (float)hSystemConfig.Config_Lead;                                    //˿�˵���           
	pSystem->SysCon.Sys_Route = (float)hSystemConfig.Config_Route;                                  //�г�
	pSystem->SysCon.Sys_SpeedMmps = ((float)hSystemConfig.Config_SpeedMmps/10);                     //�Ƹ��ٶ�
	
	pSystem->SysCon.Sys_OvcValue = hSystemConfig.Config_OvcValue;                                   //����������ֵ
	pSystem->SysCon.Sys_OverVoltage = ((float)hSystemConfig.Config_OverVoltage/10);                 //��ѹ������ֵ
	pSystem->SysCon.Sys_UnderVoltage = ((float)hSystemConfig.Config_UnderVoltage/10);               //Ƿѹ������ֵ
	
	
	/*ϵͳ������ʼ��*/	
	pSystem->SysCon.Sys_NodeSlaveAddr = hSystemConfig.Config_NodeSlaveAddr;                         //ͨѶ�ڵ�ID
	pSystem->SysCon.Sys_CommunicationType = (syscon_cfg_com_type_t)hSystemConfig.Config_CommunicationType;  //ͨѶ����

	pSystem->SysCon.Sys_DIFunction = (syscon_cfg_di_fun_t)hSystemConfig.Config_DIFunction;          //�������빦��
	pSystem->SysCon.Sys_DOFunction = (syscon_cfg_do_fun_t)hSystemConfig.Config_DOFunction;          //�����������
	pSystem->SysCon.Sys_ActiveValue = (syscon_cfg_pin_polarity_t)hSystemConfig.Config_ActiveValue;  //���ż���
	
	pSystem->SysCon.Sys_ResetRaise = (float)hSystemConfig.Config_ResetRaise;                        //��λ̧��
	pSystem->SysCon.Sys_ResetRunMode = (syscon_cfg_reset_run_mode_t)hSystemConfig.Config_ResetRunMode;      //��λ����ģʽ
	pSystem->SysCon.Sys_ResetDirection = (syscon_cfg_reset_direction_t)hSystemConfig.Config_ResetDirection; //��λ����
	pSystem->SysCon.Sys_ResetMode = (syscon_cfg_reset_judgment_mode_t)hSystemConfig.Config_ResetMode;       //��λ�жϷ�ʽ
	pSystem->SysCon.Sys_MotorRunMode = (syscon_cfg_motor_run_mode_t)hSystemConfig.Config_MotorRunMode;      //�������ģʽ
	
	pSystem->SysCon.Sys_TopDetection = (syscon_cfg_top_detect_t)hSystemConfig.Config_TopDetection;  //�������
	pSystem->SysCon.Sys_BtmDetection = (syscon_cfg_btm_detect_t)hSystemConfig.Config_BtmDetection;  //���׼��
	
}

/*��ͬͨѶ��ʽ��ʼ���������*/
void system_Communication_Motor_Init(void)
{
    uint32_t Data[10] = {0};
	uint32_t Check = 0;
	float ColumnSpdMMPS = 0;
    if((System.SysCon.Sys_CommunicationType != E_MODBUS)&&(System.SysCon.Sys_CommunicationType != E_CAN))
        return;
    if(System.SysCon.Sys_CommunicationType == E_MODBUS)
    {
        /*��ʼ���������ĵ�����Ʋ���*/
        flash_adapter_Read(WORD, FSA_SYS_PARAM, WORD, Data, 3);
        if(Data[0] == 0xAA55)
        {
            Check = Data[0] + Data[1];
            if(Check == Data[2])
            {
                hModbusLink.SlaveAddr = Data[1];
            }
        }
    }else if(System.SysCon.Sys_CommunicationType == E_CAN)
    {
        /*CanOpen�����ֵ��ʼ��*/
        CO_SET_OD(OD_SI_VERSION, SW_VERSION);
        CO_SET_OD(OD_SI_MINIVERSION, SW_MINI_VERSION);
        CO_SET_OD(OD_SI_TRG_SPD, (uint32_t)(System.SysCon.Sys_SpeedMmps * 10));
        CO_SET_OD(OD_SI_HEARTBEAT, DEFAULT_HEART_BEAT);
        CO_SET_OD(OD_SI_NODEID, System.SysCon.Sys_NodeSlaveAddr);
        CO_SET_OD(OD_SI_BAUDRATE, 2);	//0-1M, 1-500k, 2-250k, 3-125k(��δʵ���޸Ĳ����ʵĹ���)
        System.CanCmd = e_can_none;
        
        /*��ʼ���������ĵ�����Ʋ���*/
        flash_adapter_Read(WORD, FSA_SYS_PARAM, WORD, Data, 5);
        if(Data[0] == 0xAA55)
        {
            Check = Data[0] + Data[1] + Data[2] + Data[3];
            if(Check == Data[4])
            {
                ColumnSpdMMPS = (float)Data[1] / 10.0f;		//flash�Ͷ����ֵ��д�����ٶȵ�λ:1=0.1mm/s
                System.MotorSpeed = system_Modify_Column_Speed(0, ColumnSpdMMPS);
                sCONode.HeartBeatMs = Data[2];
                sCONode.NodeID = Data[3];
                CO_SET_OD(OD_SI_TRG_SPD, Data[1]);
                CO_SET_OD(OD_SI_HEARTBEAT, Data[2]);
                CO_SET_OD(OD_SI_NODEID, Data[3]);
            }
        }

        /*CAN�˲�������*/
        //	can_adapter_SetFilter_IDMask(CAN1, CanFilterSel1, 0, 0x1FFFFFFF);	//����ID������(��������)
        can_adapter_SetFilter_ID(CAN1, CanFilterSel1, 0);	//��Զ����ID == 0��֡(��������)
        can_adapter_SetFilter_IDMask(CAN1, CanFilterSel2, sCONode.NodeID, CO_ID_MASK_FUNC);
    }
}

/*����ĸ�λ�жϷ�ʽ*/
void system_Reset_Noreset(SYSTEM_t* pSys)
{
    if((System.SysCon.Sys_ResetMode != E_DETECTION_SIGNAL)&&(System.SysCon.Sys_ResetMode != E_DETECTION_ANOMALY))
    {
        return;
    }
    if(System.SysCon.Sys_ResetMode == E_DETECTION_SIGNAL)
    {
		System.ZeroFound = 0;
		SYSTEM_SET_FLAG(pSys->SaveIntoFlash, FS_MOTOR_SATE);  
        system_FSM_StateJump(pSys, E_SYS_STATE_FAULT);
    }else if(System.SysCon.Sys_ResetMode == E_DETECTION_ANOMALY)
    {
        SYSTEM_SET_FLAG(pSys->FaultFlag, FAULT_POS);
		SYSTEM_SET_FLAG(pSys->SaveIntoFlash, FS_MOTOR_SATE);
		System.PowerOnResetFlag = 1;
        system_FSM_StateJump(pSys, E_SYS_STATE_FAULT);   
    }
}

/*�����λģʽ*/
BTN_STATE_t syste_Reset_Mode(void)
{
    if((System.SysCon.Sys_ResetDirection != E_RESET_BTM)&&(System.SysCon.Sys_ResetDirection != E_RESET_TOP))
    {
        return E_BTN_STA_UNDEFINED;
    }
    if(System.SysCon.Sys_ResetDirection == E_RESET_BTM)
    {
        return btn_Get_State(KEY_MOTOR_DOWN);
    }else if(System.SysCon.Sys_ResetDirection == E_RESET_TOP)
    {
        return btn_Get_State(KEY_MOTOR_UP);
    }
	return E_BTN_STA_UNDEFINED;
}

/*�жϸ�λ����*/
bool system_Reset_Direction(SYSTEM_t* pSys)
{
	bool Ret = 0;
    if(((System.SysCon.Sys_ResetDirection != E_RESET_BTM)&&(System.SysCon.Sys_ResetDirection != E_RESET_TOP))\
        ||((System.SysCon.Sys_CommunicationType != E_MODBUS)&&(System.SysCon.Sys_CommunicationType != E_CAN)\
		&&(System.SysCon.Sys_CommunicationType != E_NOCOM)))
    {
        return 0;
    }
    if(System.SysCon.Sys_ResetDirection == E_RESET_BTM)
    {
		if(btn_Match_Event(KEY_MOTOR_DOWN, E_BTN_EVT_PRESS, 0))
		{
			system_Clear_Motor_Fault(MOTOR1);   //������ԭ��: ��λ��ͣ���������ֹͣ,���ܻ��ת,��������.
			System.ResetHandControlling = 1;
			Ret = 1;
		}
    }else  if(System.SysCon.Sys_ResetDirection == E_RESET_TOP)
    {
        if(btn_Match_Event(KEY_MOTOR_UP, E_BTN_EVT_PRESS, 0))
		{
			system_Clear_Motor_Fault(MOTOR1);   //������ԭ��: ��λ��ͣ���������ֹͣ,���ܻ��ת,��������.
			System.ResetHandControlling = 1;
			Ret = 1;
		}
    }
    if(System.SysCon.Sys_CommunicationType == E_MODBUS)
    {
        /*modbusЭ��-��λ��ʽ*/
        if( pSys->ModbusCmd == e_mmc_reset)
        {
            pSys->ModbusCmd = e_mmc_none;
            system_Clear_Motor_Fault(MOTOR1);   //������ԭ��: ��λ��ͣ���������ֹͣ,���ܻ��ת,��������.
            System.ResetHandControlling = 0;
			Ret = 1;
        }
        
    }
    else if(System.SysCon.Sys_CommunicationType == E_CAN)
    {
         /*modbusЭ��-��λ��ʽ*/
        if( pSys->CanCmd == e_can_reset)
        {
            pSys->CanCmd = e_can_none;
            system_Clear_Motor_Fault(MOTOR1);   //������ԭ��: ��λ��ͣ���������ֹͣ,���ܻ��ת,��������.
            System.ResetHandControlling = 0;
			Ret = 1;
        }
    }
    
	return Ret;
}

/*��λ����*/
bool system_Reset_Start(void)
{
	bool Ret = 0;
    if((System.SysCon.Sys_ResetDirection != E_RESET_BTM)&&(System.SysCon.Sys_ResetDirection != E_RESET_TOP))
    {
        return 0;
    }
    if(system_Is_Motor_Stop(MOTOR1))
    {
        if(System.SysCon.Sys_ResetDirection == E_RESET_BTM)
        {
            system_Motor_Move(MOTOR1, e_mac_start_closeloop, DIR_DOWN);
        }else if(System.SysCon.Sys_ResetDirection == E_RESET_TOP)
        {
            system_Motor_Move(MOTOR1, e_mac_start_closeloop, DIR_UP);
        }
        Ret = 1;
    }
	return Ret;
}

/*��λ���ķ�ʽ*/
bool system_Reset_Detection_Mode(SYSTEM_t* pSys)
{
	bool Ret = 0;
    static float g_ZeroPosHall;
    if((System.SysCon.Sys_ResetDirection != E_RESET_BTM)&&(System.SysCon.Sys_ResetDirection != E_RESET_TOP))
    {
        return 0;
    }
    if(mc_app_Get_State(MOTOR1) == e_mas_idle)
    {
        if(SYSTEM_GET_FLAG(pSys->FaultFlag, FAULT_M1_HAB | FAULT_M1_OVC))    //������hall�쳣�������ǵ��׶�ת
        {
            system_Clear_Motor_Fault(MOTOR1);   //������ԭ��: ��ֹ�ͻ���״̬��ȫ�ֹ����жϳ�ͻ,����ϵͳ����FAULT״̬.
            if(System.SysCon.Sys_ResetDirection == E_RESET_BTM)
            {
                SYSTEM_CLR_FLAG(pSys->MotorFlag, M1_DOWN);
                g_ZeroPosHall = BTM_POS_HALLDATA;
            }else if(System.SysCon.Sys_ResetDirection == E_RESET_TOP)
            {
                SYSTEM_CLR_FLAG(pSys->MotorFlag, M1_UP);
                g_ZeroPosHall = system_Column_Pos_2_HallData(System.SysCon.Sys_Route);
            }
        }
    }
    if(SYSTEM_MATCH_FLAG(pSys->MotorFlag, 0))
    {
        /*������Ʋ����ָ�*/
        for(uint8_t i = 0; i< SYS_MOTOR_NB; i++)
        {
            mc_app_Write_Param((MOTOR1<<i), e_map_drvoutput_max, (PWM_OUTPUT_FULLSCALE / 100) * MAX_PWM_PERCENT);//̧�����ռ�ձ�Ҫ�����ֵ,��ֹ����̧������
            mc_app_Write_Param((MOTOR1<<i), e_map_fdbkpos, g_ZeroPosHall);
        }
        Ret = 1;
    }
	return Ret;
}

/*��λ��ĳ�ʼλ��*/
bool system_Reset_Initial_Position(void)
{
	bool Ret = 0;
    MOTOR_POS_t TargetHallPos = 0;
    if(((System.SysCon.Sys_ResetDirection != E_RESET_BTM)&&(System.SysCon.Sys_ResetDirection != E_RESET_TOP))\
        ||(System.SysCon.Sys_ResetRaise<(-100))||(System.SysCon.Sys_ResetRaise>100))
    {
        return 0;
    }
    if((mc_app_Get_State(MOTOR1) == e_mas_idle)&&(System.SysCon.Sys_ResetRaise != 0))
    {
        if(System.SysCon.Sys_ResetDirection == E_RESET_BTM)
        {
            TargetHallPos = (MOTOR_POS_t)((system_Hall_PER_MM()) * (BTM_POS_HALLDATA + System.SysCon.Sys_ResetRaise));   
//			TargetHallPos = (BTM_POS_HALLDATA + (System.SysCon.Sys_ResetRaise));
		}else if(System.SysCon.Sys_ResetDirection == E_RESET_TOP)
        {
            TargetHallPos = (MOTOR_POS_t)((system_Hall_PER_MM()) * (System.SysCon.Sys_Route + System.SysCon.Sys_ResetRaise));   
//			TargetHallPos = (System.SysCon.Sys_Route + (System.SysCon.Sys_ResetRaise)); 
		}
        mc_app_Set_Single_Motor_Cmd(MOTOR1, e_mac_goto_targetpos, TargetHallPos);	/* ̧�߽��͵���hallֵ����Ҫ��MMת��ΪHALL */
    }else
    {
        Ret = 1;
    }
	return Ret;
}

/* ��λ������ģʽ*/
bool system_Reset_Run_Mode(BTN_STATE_t ResetState)
{
	bool Ret = 0;
    if(((System.SysCon.Sys_ResetRunMode != E_RESET_INCHING)&&(System.SysCon.Sys_ResetRunMode != E_RESET_CONTINUOUS))\
        ||((System.SysCon.Sys_ResetDirection != E_RESET_BTM)&&(System.SysCon.Sys_ResetDirection != E_RESET_TOP))\
        ||(ResetState == E_BTN_STA_UNDEFINED))
    {
        return 0;
    }
    if(System.SysCon.Sys_ResetRunMode == E_RESET_CONTINUOUS)
    {
        /*����-�ٴΰ��¸�λ��ֹͣ��λ*/
        if(System.SysCon.Sys_ResetDirection == E_RESET_BTM)
        {
            if(btn_Match_Event(KEY_MOTOR_DOWN, E_BTN_EVT_PRESS, 0))
            {
                Ret = 1;
            }
        }else if(System.SysCon.Sys_ResetDirection == E_RESET_TOP)
        {
            if(btn_Match_Event(KEY_MOTOR_UP, E_BTN_EVT_PRESS, 0))
            {
                Ret = 1;
            }
        }
    }else if(System.SysCon.Sys_ResetRunMode == E_RESET_INCHING)
    {
		if(System.ResetHandControlling == 1)
		{
			/*�㶯-�ɿ���λ��ֹͣ��λ*/
			if(ResetState == E_BTN_STA_RELEASED)
			{
				Ret = 1;
			}
		}
        
    }
	return Ret;
}

/*���������ģʽ*/
bool system_Motor_Run_Mode(BTN_STATE_t MUState, BTN_STATE_t MDState)
{
	bool Ret = 0;
    if(((System.SysCon.Sys_MotorRunMode != E_MOTOR_INCHING)&&(System.SysCon.Sys_MotorRunMode != E_MOTOR_CONTINUOUS))\
        || (MUState == E_BTN_STA_UNDEFINED)||(MDState == E_BTN_STA_UNDEFINED))
    {
        return 0;
    }
    if(System.SysCon.Sys_MotorRunMode == E_MOTOR_INCHING)
    {
		if(System.HandControlling == 1)
		{
			/*�㶯-�������ֵ��ͣ��*/
			if((MUState == E_BTN_STA_RELEASED && System.RunDir == DIR_UP) || \
				(MDState == E_BTN_STA_RELEASED && System.RunDir == DIR_DOWN))
			{
				Ret = 1;
			}
		}
    }else if(System.SysCon.Sys_MotorRunMode == E_MOTOR_CONTINUOUS)
    {
		System.HandControlling = 0;
        /*����-�����ٴΰ��µ��ͣ��*/
        if((btn_Match_Event(KEY_MOTOR_DOWN, E_BTN_EVT_PRESS, 0))||(btn_Match_Event(KEY_MOTOR_UP, E_BTN_EVT_PRESS, 0)))
        {
            Ret = 1;
        }
    }
	return Ret;
}

/*���������λ�����е�Ŀ��λ�÷�ʽ*/
void system_Idle_ComGotoPos(SYSTEM_t* pSys)
{
    if((System.SysCon.Sys_CommunicationType != E_MODBUS)&&(System.SysCon.Sys_CommunicationType != E_CAN))
    {
        return;
    }
    if(System.SysCon.Sys_CommunicationType == E_MODBUS)
    {
        /*ModbusͨѶ-�������е�Ŀ��λ��״̬*/
        if(pSys->HandControlling == 0)
        {
            if(pSys->ModbusCmd == e_mmc_goto)
            {
                pSys->ModbusCmd = e_mmc_none;
                pSys->GotoHallPos = system_Column_Pos_2_HallData(pSys->ModbusColumnPos);
                system_FSM_StateJump(pSys, E_SYS_STATE_MOTOR_GOTOPOS);
                return;
            }
        }
        if(pSys->ModbusCmd == e_mmc_poszero)
        {
            pSys->ModbusCmd = e_mmc_none;
            system_FSM_StateJump(pSys, E_SYS_STATE_SETZEROPOS);
            return;
        }
    }else if(System.SysCon.Sys_CommunicationType == E_CAN)
    {
        /*CANͨѶ-�������е�Ŀ��λ��״̬*/
        if(pSys->HandControlling == 0)
        {
            if(pSys->CanCmd == e_can_goto)
            {
                pSys->CanCmd = e_can_none;
                pSys->GotoHallPos = system_Column_Pos_2_HallData(CO_GET_OD(OD_SI_CMD_M_GOTO));
                system_FSM_StateJump(pSys, E_SYS_STATE_MOTOR_GOTOPOS);
                return;
            }
        }
    }
}

/*�����λ������*/
void system_Idle_ComRun(SYSTEM_t* pSys)
{
	float ColumnPosTop = 0;
    float ColumnPosBtm = 0;
    if((System.SysCon.Sys_CommunicationType != E_MODBUS)&&(System.SysCon.Sys_CommunicationType != E_CAN))
    {
        return;
    }
    if(System.SysCon.Sys_CommunicationType == E_MODBUS)
    {
        if(pSys->ModbusCmd == e_mmc_up)
		{
			pSys->ModbusCmd = e_mmc_none;
			pSys->RunDir = DIR_UP;
			if(System.ZeroFound == 1)
			{
				ColumnPosTop = system_Get_Column_SpecificPos(COLUMN_POS_SET_TOP);
				pSys->GotoHallPos = system_Column_Pos_2_HallData(ColumnPosTop);
				system_FSM_StateJump(pSys, E_SYS_STATE_MOTOR_GOTOPOS);
				return;
			}
			system_FSM_StateJump(pSys, E_SYS_STATE_MOTOR_RUN);
			return;
		}else if(pSys->ModbusCmd == e_mmc_dn)
		{
			pSys->ModbusCmd = e_mmc_none;
			pSys->RunDir = DIR_DOWN;
			if(System.ZeroFound == 1)
			{
				ColumnPosBtm = system_Get_Column_SpecificPos(COLUMN_POS_ZERO);
				pSys->GotoHallPos = system_Column_Pos_2_HallData(ColumnPosBtm);
				system_FSM_StateJump(pSys, E_SYS_STATE_MOTOR_GOTOPOS);
				return;
			}
			system_FSM_StateJump(pSys, E_SYS_STATE_MOTOR_RUN);
			return;
		}
    }else if(System.SysCon.Sys_CommunicationType == E_CAN)
    {
        if(pSys->CanCmd == e_can_up)
		{
			pSys->CanCmd = e_can_none;
			pSys->RunDir = DIR_UP;
			if(System.ZeroFound == 1)
			{
				ColumnPosTop = system_Get_Column_SpecificPos(COLUMN_POS_SET_TOP);
				pSys->GotoHallPos = system_Column_Pos_2_HallData(ColumnPosTop);
				system_FSM_StateJump(pSys, E_SYS_STATE_MOTOR_GOTOPOS);
				return;
			}
			system_FSM_StateJump(pSys, E_SYS_STATE_MOTOR_RUN);
			return;
		}else if(pSys->CanCmd == e_can_dn)
		{
			pSys->CanCmd = e_can_none;
			pSys->RunDir = DIR_DOWN;
			if(System.ZeroFound == 1)
			{
				ColumnPosBtm = system_Get_Column_SpecificPos(COLUMN_POS_ZERO);
				pSys->GotoHallPos = system_Column_Pos_2_HallData(ColumnPosBtm);
				system_FSM_StateJump(pSys, E_SYS_STATE_MOTOR_GOTOPOS);
				return;
			}
			system_FSM_StateJump(pSys, E_SYS_STATE_MOTOR_RUN);
			return;
		}
    }
}
/*�������������ʽ*/
bool system_Idle_Start(SYSTEM_t* pSys, float Columnpos)
{
    float ColumnPosTop = 0;
    float ColumnPosBtm = 0;

    if((System.SysCon.Sys_ResetMode != E_DETECTION_SIGNAL)&&(System.SysCon.Sys_ResetMode != E_DETECTION_ANOMALY))
    {
        return 0;
    }
    if(System.SysCon.Sys_ResetMode == E_DETECTION_SIGNAL)
    {
		if(System.ZeroFound == 1)
		{
			if(pSys->ColumnPosMM[0] + 0.5f < Columnpos)
			{
				if(pSys->RunDir == DIR_UP)
				{
					ColumnPosTop = system_Get_Column_SpecificPos(COLUMN_POS_SET_TOP);
					pSys->GotoHallPos = system_Column_Pos_2_HallData(ColumnPosTop);
					system_FSM_StateJump(pSys, E_SYS_STATE_MOTOR_GOTOPOS);
					return 1;
				}
			}
			if(pSys->ColumnPosMM[0] > Columnpos + 0.5f)
			{
				if(pSys->RunDir == DIR_DOWN)
				{
					ColumnPosBtm = system_Get_Column_SpecificPos(COLUMN_POS_ZERO);
					pSys->GotoHallPos = system_Column_Pos_2_HallData(ColumnPosBtm);
					system_FSM_StateJump(pSys, E_SYS_STATE_MOTOR_GOTOPOS);
					return 1;
				}
				
			}
		}else
		{
			system_FSM_StateJump(pSys, E_SYS_STATE_MOTOR_RUN);
			return 1;
		}
    }else if(System.SysCon.Sys_ResetMode == E_DETECTION_ANOMALY)
    {
		if(pSys->ColumnPosMM[0] + 0.5f < Columnpos)
		{
			if(pSys->RunDir == DIR_UP)
			{
				ColumnPosTop = system_Get_Column_SpecificPos(COLUMN_POS_SET_TOP);
				pSys->GotoHallPos = system_Column_Pos_2_HallData(ColumnPosTop);
				system_FSM_StateJump(pSys, E_SYS_STATE_MOTOR_GOTOPOS);
				return 1;
			}
		}
		if(pSys->ColumnPosMM[0] > Columnpos + 0.5f)
		{
			if(pSys->RunDir == DIR_DOWN)
			{
				ColumnPosBtm = system_Get_Column_SpecificPos(COLUMN_POS_ZERO);
				pSys->GotoHallPos = system_Column_Pos_2_HallData(ColumnPosBtm);
				system_FSM_StateJump(pSys, E_SYS_STATE_MOTOR_GOTOPOS);
				return 1;
			}
			
		}
		
    }
	return 0;
}

/*��λ��Ŀ��λ�ø���*/
MOTOR_POS_t system_Communication_Updategotopos(void)
{
    if((System.SysCon.Sys_CommunicationType != E_MODBUS)&&(System.SysCon.Sys_CommunicationType != E_CAN))
    {
        return 0;
    }
    if(System.SysCon.Sys_CommunicationType == E_MODBUS)
    {
        return system_Column_Pos_2_HallData(System.ModbusColumnPos);
    }else if(System.SysCon.Sys_CommunicationType == E_CAN)
    {
        return system_Column_Pos_2_HallData(CO_GET_OD(OD_SI_CMD_M_GOTO));
    }
	return 0;
}

/* ��λ������ָ��*/
void system_Communication_None(void)
{
    if((System.SysCon.Sys_CommunicationType != E_MODBUS)&&(System.SysCon.Sys_CommunicationType != E_CAN))
        return;
    if(System.SysCon.Sys_CommunicationType == E_MODBUS)
    {
        System.ModbusCmd = e_mmc_none;
    }else if(System.SysCon.Sys_CommunicationType == E_CAN)
    {
        System.CanCmd = e_can_none;
    }
}

/*��λ�����е�Ŀ��λ��*/
bool system_Communication_Goto(void)
{
	bool Ret = 0;
    if((System.SysCon.Sys_CommunicationType != E_MODBUS)&&(System.SysCon.Sys_CommunicationType != E_CAN))
        return 0;
    if(System.SysCon.Sys_CommunicationType == E_MODBUS)
    {
        if(System.ModbusCmd == e_mmc_goto)
            Ret = 1;
    }else if(System.SysCon.Sys_CommunicationType == E_CAN)
    {
        if(System.CanCmd == e_can_goto)
            Ret = 1;
    }
	return Ret;
}

/*��λ��ֹͣ����*/
bool system_Communication_Stop(void)
{
	bool Ret = 0;
    if((System.SysCon.Sys_CommunicationType != E_MODBUS)&&(System.SysCon.Sys_CommunicationType != E_CAN))
        return 0;
    if(System.SysCon.Sys_CommunicationType == E_MODBUS)
    {
        if(System.ModbusCmd == e_mmc_stop)
            Ret = 1;
    }else if(System.SysCon.Sys_CommunicationType == E_CAN)
    {
        if(System.CanCmd == e_can_stop)
            Ret = 1;
    }
	return Ret;
}

/*��λ������ָ��*/
bool system_Communication_Up(void)
{
	bool Ret = 0;
    if((System.SysCon.Sys_CommunicationType != E_MODBUS)&&(System.SysCon.Sys_CommunicationType != E_CAN))
        return 0;
    if(System.SysCon.Sys_CommunicationType == E_MODBUS)
    {
        if(System.ModbusCmd == e_mmc_up)
            Ret = 1;
    }else if(System.SysCon.Sys_CommunicationType == E_CAN)
    {
        if(System.CanCmd == e_can_up)
            Ret = 1;
    }
	return Ret;
}

/*��λ���½�ָ��*/
bool system_Communication_Down(void)
{
	bool Ret = 0;
    if((System.SysCon.Sys_CommunicationType != E_MODBUS)&&(System.SysCon.Sys_CommunicationType != E_CAN))
        return 0;
    if(System.SysCon.Sys_CommunicationType == E_MODBUS)
    {
        if(System.ModbusCmd == e_mmc_dn)
            Ret = 1;
    }else if(System.SysCon.Sys_CommunicationType == E_CAN)
    {
        if(System.CanCmd == e_can_dn)
            Ret = 1;
    }
	return Ret;
}

/* ��λ����λָ��*/
bool system_Communication_Reset(void)
{
	bool Ret = 0;
    if((System.SysCon.Sys_CommunicationType != E_MODBUS)&&(System.SysCon.Sys_CommunicationType != E_CAN))
        return 0;
    if(System.SysCon.Sys_CommunicationType == E_MODBUS)
    {
        if(System.ModbusCmd == e_mmc_reset)
		{
			System.ModbusCmd = e_mmc_none;
			System.ResetHandControlling = 0;
            Ret = 1;
		}
    }else if(System.SysCon.Sys_CommunicationType == E_CAN)
    {
		if(System.CanCmd == e_can_reset)
		{
			System.CanCmd =  e_can_none;
			System.ResetHandControlling = 0;
			Ret = 1;
		}
    }
	return Ret;
}


/*�жϰ����¼�*/
bool system_Get_Key_Event(void)
{
	bool Ret = 0;
    if((System.SysCon.Sys_ResetDirection != E_RESET_BTM)&&(System.SysCon.Sys_ResetDirection != E_RESET_TOP))
    {
        return 0;
    }
    if(System.SysCon.Sys_ResetDirection == E_RESET_BTM)
    {
		if(btn_Match_Event(KEY_MOTOR_DOWN, E_BTN_EVT_PRESS, 0))
		{
			System.ResetHandControlling = 1;
			Ret = 1;
		}
    }else if(System.SysCon.Sys_ResetDirection == E_RESET_TOP)
    {
		if(btn_Match_Event(KEY_MOTOR_UP, E_BTN_EVT_PRESS, 0))
		{
			System.ResetHandControlling = 1;
			Ret = 1;
		}
    }
	return Ret;
}

/*��������*/
bool system_Get_LongKey(void)
{
	bool Ret = 0;
    if((System.SysCon.Sys_ResetDirection != E_RESET_BTM)&&(System.SysCon.Sys_ResetDirection != E_RESET_TOP))
    {
        return 0;
    }
    if(System.SysCon.Sys_ResetDirection == E_RESET_BTM)
    {
		if(btn_Match_Event(KEY_MOTOR_DOWN, E_BTN_EVT_LONG_PRESS, 1000))
		{
			Ret = 1;
		}
    }else if(System.SysCon.Sys_ResetDirection == E_RESET_TOP)
    {
		if(btn_Match_Event(KEY_MOTOR_UP, E_BTN_EVT_LONG_PRESS, 1000))
		{
			Ret = 1;
		}
    }
	return Ret;
}

/*HALL_PER_MM*/
float system_Hall_PER_MM(void)
{
    return ((System.SysCon.Sys_GearRatio*HPR)/System.SysCon.Sys_Lead);
}


/*�Ƹ��������*/
float system_Speedmmps_Max(void)
{
    return ((System.SysCon.Sys_SpeedMmps)*(1+SPEED_INDEX));
}


/* �Ƹ���С����*/
float system_Speedmmps_Min(void)
{
    return ((System.SysCon.Sys_SpeedMmps)*(1-SPEED_INDEX));
}

/* ���ص��׼������� */
syscon_cfg_btm_detect_t system_CfgBtmDetet(void)
{
	return System.SysCon.Sys_BtmDetection;
}

/* ���ص����������� */
syscon_cfg_top_detect_t system_CfgTopDetet(void)
{
	return System.SysCon.Sys_TopDetection;
}

/* �жϵ�������Ƿ����е��źſ��ش� */
void system_SignalSwitch(void)
{
	if(system_CfgBtmDetet() == E_BTM_SIGNAL_SWITCH)
	{
		if(System.RunDir == DIR_DOWN && (System.LimitFlag & M1_BTM))
		{
			mc_app_Write_Param(MOTOR1, e_map_fdbkpos, BTM_POS_HALLDATA);//ֻ�ڵ������ʱ����λ��,��ֹ�����󴥷�(�ͻ�������24V/5Vʱ���д�����)
			System.ZeroFound = 1;
		}
	}
	if(system_CfgTopDetet() == E_TOP_SIGNAL_SWITCH)
	{
		if(System.RunDir == DIR_UP && (System.LimitFlag & M1_TOP))
		{
			mc_app_Write_Param(MOTOR1, e_map_fdbkpos, (MOTOR_POS_t)((system_Hall_PER_MM())*System.SysCon.Sys_Route));//ֻ�ڵ������ʱ����λ��,��ֹ�����󴥷�(�ͻ�������24V/5Vʱ���д�����)
			System.ZeroFound = 1;
		}
	}
}

/********************************��������************************************
*������:

*������������: ϵͳ����-ϵͳ�쳣�������

*��������: ��

*��������ֵ: ��

*��ע:
*****************************************************************************/
/*��ѹ/Ƿѹ����*/
void system_UDV_OVV_Protect(void)
{
    uint16_t ADData_Voltage = 0;    //�����ģ����
    float BusVoltage = 0;
    
    // float R1 = VM_R1;  //��ֵ��λ: k��
    // float R2 = VM_R2; //��ѹ�� = R1/(R1+R2)

    /*��ѹ����*/
    if(1 == adc_adapter_SCM_1Ch_Convert(ADCH_VM_ADCH))
    {

        ADData_Voltage = adc_adapter_Get_Channel_Result(ADCH_VM_ADCH);
        BusVoltage = (float)ADData_Voltage * ADC_REF_VOLTAGE / ADC_FULL_SCALE;

        BusVoltage = (BusVoltage*(VM_R1 + VM_R2) / VM_R1) ;

        /*�ж��Ƿ����쳣*/
        if(BusVoltage > System.SysCon.Sys_OverVoltage)
        {
			System.OVVFlag = 1;
        }else
        {
			System.OVVFlag = 0;
			System.OVVCount = 0;
            if(BusVoltage < (System.SysCon.Sys_OverVoltage - HYSTERESIS_VOLTAGE))    //��ѹ�ָ�����
                SYSTEM_CLR_FLAG(System.FaultFlag, FAULT_OVV);
        }
        if(BusVoltage < System.SysCon.Sys_UnderVoltage)
        {
            SYSTEM_SET_FLAG(System.FaultFlag, FAULT_UDV);
        }else
        {
            if(BusVoltage > System.SysCon.Sys_UnderVoltage + HYSTERESIS_VOLTAGE) //Ƿѹ�ָ�����
                SYSTEM_CLR_FLAG(System.FaultFlag, FAULT_UDV);
        }
		if(System.OVVCount >= OVV_COUNT_MAX)
		{
			SYSTEM_SET_FLAG(System.FaultFlag, FAULT_OVV);
		}
        System.BusVoltageMV = (uint16_t)(BusVoltage * 1000);
    }
}
/*���ȱ���*/
void system_OVT_Protect(void)
{
	static uint16_t ADData_Temperature[1];
	float Volatage_85C = 1.69f; //85��C�Ĳ�����ѹֵ
	float Volatage_45C = 0.64f; //45��C�Ĳ�����ѹֵ
	float CmpValue = 0;
	static uint8_t CmpEn = 0;

	/*�¶Ȳ���*/
	if(1 == adc_adapter_SCM_1Ch_Convert(ADCH_TEMP_ADCH))
	{
		ADData_Temperature[0] = adc_adapter_Get_Channel_Result(ADCH_TEMP_ADCH);
		CmpEn |= 0x01;
	}
	/*�ж��Ƿ����쳣*/
	if(CmpEn == 0x01)
	{
		CmpValue = Volatage_85C * ADC_FULL_SCALE / ADC_REF_VOLTAGE;
		if(ADData_Temperature[0] > (uint16_t)CmpValue)
		{
			SYSTEM_SET_FLAG(System.FaultFlag, FAULT_OVT);
		}
		CmpValue = Volatage_45C * ADC_FULL_SCALE / ADC_REF_VOLTAGE;
		if(ADData_Temperature[0] < (uint16_t)CmpValue)
		{
			SYSTEM_CLR_FLAG(System.FaultFlag, FAULT_OVT);
		}
		CmpEn = 0;
	}
}
/*��ȡϵͳ�쳣*/
uint16_t system_Get_Fault(uint16_t FaultMask)
{
    return (SYSTEM_GET_FLAG(System.FaultFlag, FaultMask));
}
/*ϵͳ�쳣���*/


void system_Check_Fault_Task(void)
{
    
    /*��ѹ/Ƿѹ����*/
    system_UDV_OVV_Protect();
    // mc_cur_Sample();
    /*���ȱ���*/
    // system_OVT_Protect();
}
/********************************��������************************************
*������:

*������������: ϵͳ����-ϵͳ��������

*��������: ��

*��������ֵ: ��

*��ע:
*****************************************************************************/

/*ϵͳ�ź�����-�Ƹ���λ(1ms��ʱ����)*/
void system_Read_Column_Limit(void)
{
	static uint8_t s_LimitBtmDbc[MOTOR_NB] = {0};
	static uint8_t s_LimitTopDbc[MOTOR_NB] = {0};
	GPIO_PIN_LEVEL_t ActiveLel = E_GPIO_PIN_RESET;
	/*M1-����λ*/
	if(gpio_adapter_Read_Pin(GPIO_M1_LIMIT_BTM_PORT, GPIO_M1_LIMIT_BTM_PIN) == ActiveLel)
	{
		if(s_LimitBtmDbc[0]++ > 10)
		{
			s_LimitBtmDbc[0] = 10;
			SYSTEM_SET_FLAG(System.LimitFlag, M1_BTM);
		}
	}else
	{
		if(s_LimitBtmDbc[0]-- < 1)
		{
			s_LimitBtmDbc[0] = 0;
			SYSTEM_CLR_FLAG(System.LimitFlag, M1_BTM);
		}
	}
	/*M1-����λ*/
	if(gpio_adapter_Read_Pin(GPIO_M1_LIMIT_TOP_PORT, GPIO_M1_LIMIT_TOP_PIN) == ActiveLel)
	{
		if(s_LimitTopDbc[0]++ > 10)
		{
			s_LimitTopDbc[0] = 10;
			SYSTEM_SET_FLAG(System.LimitFlag, M1_TOP);
		}
	}else
	{
		if(s_LimitTopDbc[0]-- < 1)
		{
			s_LimitTopDbc[0] = 0;
			SYSTEM_CLR_FLAG(System.LimitFlag, M1_TOP);
		}
	}
}
/*ϵͳ�ź����-�쳣״ָ̬ʾ(������ʵʱ�Ժõĵط�����)*/
void system_Set_Fault_Signal(void)
{
//    if(SYSTEM_GET_FLAG(System.FaultFlag, FAULT_ALL))
//    {
//        if((System.State != E_SYS_STATE_ZERO) || (System.State == E_SYS_STATE_ZERO && System.Step != 3)) //д�����Ľ�
//            gpio_adapter_Set_Pin(FAULT_GPIO_Port, FAULT_Pin);
//    }else
//        gpio_adapter_Reset_Pin(FAULT_GPIO_Port, FAULT_Pin);
}
/*ϵͳ�ź����-���ת��(HALL��ƽ״̬���ʱ����)*/
void system_Set_MotorSpd_Signal(void)
{
//  gpio_adapter_Toggle_Pin(GPIO_MOTOR_SPEED_PORT, GPIO_MOTOR_SPEED_PIN);
}
/*ϵͳ�ź�����-�������*/
void system_Read_MotorSpd_Regulator_Signal(void)
{
//  static float HYS_Voltage = 0.05;    //���͵�ѹ(��λ:V)
//  static int16_t  s_ADData_MSR_Temp = 0;
//  int16_t ADData_MSR = 0;
//  int16_t ADData_HYS = 0;     //�������
//
//  /*��ѹ����*/
//  if(1 == adc_adapter_SCM_1Ch_Convert(ADCH_MSPD_ADCH))
//  {
//      ADData_MSR = adc_adapter_Get_Channel_Result(ADCH_MSPD_ADCH);
//      ADData_HYS = (uint16_t)(HYS_Voltage * ADC_FULL_SCALE / ADC_REF_VOLTAGE);
//      if(ADData_MSR >= ADC_FULL_SCALE - 0.1)  //��ֹ��λ��ģ������ﲻ��3.3V,�����������ٶ�
//      {
//          System.MotorSpeed = M_TARGRT_RPM;
//          System.MotorDC = M_MAX_DC;
//      }else
//      {
//          if(abs(ADData_MSR - s_ADData_MSR_Temp) >= ADData_HYS)
//          {
//              //ע: ����/�ջ�Ŀ����Сֵ��mc_spd, mc_pos����������,��ʹ�����������0,���ʵ�ʿ���/�ջ�Ŀ��Ҳ����0������СĿ��ֵ!
//              System.MotorSpeed = M_TARGRT_RPM * ADData_MSR / ADC_FULL_SCALE;
//              System.MotorDC = (uint32_t)M_MAX_DC * ADData_MSR / ADC_FULL_SCALE;
//              s_ADData_MSR_Temp = ADData_MSR;
//          }
//      }
//  }
}
/*ϵͳ�ź����-������з���,��ͣ�ź�(1ms����)*/
void system_Read_MotorControl_Signal(void)
{
//  static uint8_t s_SignalDbc[2] = {0};
//  GPIO_PIN_LEVEL_t ActiveLel_1 = E_GPIO_PIN_RESET;
//
//  /*��������ź�*/
//  if(gpio_adapter_Read_Pin(GPIO_KEY_DIR_PORT, GPIO_KEY_DIR_PIN) == ActiveLel_1)
//  {
//      if(s_SignalDbc[0]++ > 10)
//      {
//          s_SignalDbc[0] = 10;
//          System.MotorDirSignal = 1;
//      }
//  }else
//  {
//      if(s_SignalDbc[0]-- < 1)
//      {
//          s_SignalDbc[0] = 0;
//          System.MotorDirSignal = 0;
//      }
//  }
}
/*ϵͳ�ź�����-�������ģʽ(1ms��ʱ����)*/
//SignalMode: 0-��ƽ��Ч,1-������Ч
//int8_t system_Read_MC_Mode_Signal(uint8_t SignalMode)
//{
#if 0
    static int8_t s_Debounce = 0;
    static GPIO_PIN_LEVEL_t s_ValidLvl = E_GPIO_PIN_RESET;
    static GPIO_PIN_LEVEL_t s_Lvl = E_GPIO_PIN_RESET;
    GPIO_PIN_LEVEL_t Lvl = E_GPIO_PIN_RESET;

    /*�����ж�*/
    Lvl = gpio_adapter_Read_Pin(GPIO_MC_MODE_SEL_PORT, GPIO_MC_MODE_SEL_PIN);
    if(s_Lvl == Lvl)
        s_Debounce++;
    else
        s_Debounce = 0;
    s_Lvl = Lvl;
    /*�������ģʽѡ��*/
    if(s_Debounce >= 10)
    {
        s_Debounce = 10;
        if(!SignalMode)
        {
            if(Lvl == E_GPIO_PIN_RESET)
                System.MCMode = 0;
            else
                System.MCMode = 2;
            return 1;
        }else
        {
            if(s_ValidLvl != Lvl)
            {
                s_ValidLvl = Lvl;
                if(Lvl == E_GPIO_PIN_RESET)
                    System.MCMode = 0;
                else
                    System.MCMode = 2;
                return 1;
            }
        }
    }
    return 0;
#else
//    System.MCMode = 1;    //�ٶȱջ�ģʽ
//    return 1;
#endif
//}
/*ϵͳ�ź����-LED���ƽӿ�*/
void system_Led_Ctrl(uint8_t Led, uint8_t Cmd)
{
//  GPIO_PORT_t Port;
//  GPIO_PIN_t Pin;
//  if(Led == LED1)
//  {
//      Port = GPIO_LED1_PORT;
//      Pin = GPIO_LED1_PIN;
//  }else if(Led == LED2)
//  {
//      Port = GPIO_LED2_PORT;
//      Pin = GPIO_LED2_PIN;
//  }else
//      return;
//  /**/
//  if(Cmd == 0)
//      gpio_adapter_Reset_Pin(Port, Pin);
//  else if(Cmd == 1)
//      gpio_adapter_Set_Pin(Port, Pin);
//  else
//      gpio_adapter_Toggle_Pin(Port, Pin);
}


/*����modbus�Ĵ���*/
void system_Update_MBReg(void)
{
    int32_t Value32 = 0;
    uint8_t Motor_Dir = 0;
    uint8_t Motor_Fault = 0;
    float Motor_PosMM = 0;
    float Motor_Spd_mmps = 0;
    /*�������ʵ��״̬,����modbus�Ĵ�����*/
    //MBREG_MOTOR_SLAVE_ADDR       �������ַ(RW)
    modbus_Write_1Reg(g_KVPTable, MBREG_MOTOR_SLAVE_ADDR, hModbusLink.SlaveAddr);
    
    //  MBREG_MOTOR_STATE             //���״̬(R)
    if(MC_RET_OK == mc_app_Read_Param(MOTOR1, e_map_dir, &Value32))
    {
        if(Value32 == 0)
            Motor_Dir = 0x03;
        else if(Value32 == 1)
            Motor_Dir = 0x01;
        else
            Motor_Dir = 0x02;
        modbus_Write_1Reg(g_KVPTable, MBREG_MOTOR_STATE, Motor_Dir);
    }
    if(System.State == E_SYS_STATE_ZERO || System.State == E_SYS_STATE_FAULT)
    {
		Motor_Fault = 0x04;
        if(SYSTEM_GET_FLAG(System.FaultFlag, FAULT_M1_OVC))
            Motor_Fault = 0x05;
        if(SYSTEM_GET_FLAG(System.FaultFlag, FAULT_M1_HAB))
            Motor_Fault = 0x06;
        if(SYSTEM_GET_FLAG(System.FaultFlag, FAULT_UDV))
            Motor_Fault = 0x07;
        if(SYSTEM_GET_FLAG(System.FaultFlag, FAULT_OVV))
            Motor_Fault = 0x08;
		if(SYSTEM_GET_FLAG(System.FaultFlag, FAULT_OVT))
            Motor_Fault = 0x09;
        modbus_Write_1Reg(g_KVPTable, MBREG_MOTOR_STATE, Motor_Fault);
    }
    
    //  MBREG_MOTOR_RPM,    �����ǰ�ٶ�(RW)
    if(MC_RET_OK == mc_app_Read_Param(MOTOR1, e_map_fdbkspd, &Value32))
    {
        Motor_Spd_mmps = system_MotorRPM_2_ColumnSpeed(Value32);
        modbus_Write_1Reg(g_KVPTable, MBREG_MOTOR_RPM, Motor_Spd_mmps);
    }
        
    //  MBREG_MOTOR_FDBKPOS,            //���λ��(R)
    if(MC_RET_OK == mc_app_Read_Param(MOTOR1, e_map_fdbkpos, &Value32))
    {
        Motor_PosMM = system_Column_HallData_2_Pos(Value32);
        modbus_Write_1Reg(g_KVPTable, MBREG_MOTOR_FDBKPOS, Motor_PosMM);
    }
     
    //ֻд����
    modbus_Write_1Reg(g_KVPTable, MBREG_MOTOR_CMD, 0);
	modbus_Write_1Reg(g_KVPTable, MBREG_MOTOR_CLR_FAULT, 0);
    modbus_Write_1Reg(g_KVPTable, MBREG_MOTOR_RESET, 0);
    modbus_Write_1Reg(g_KVPTable, MBREG_MOTOR_TARGETSPEED_POS, 0);
    modbus_Write_1Reg(g_KVPTable, MBREG_MOTOR_FDBKPOS_ZERO, 0);
}


/*ϵͳ�źŴ���*/
void system_Signal_LoopTask(void)
{
//    system_Set_Fault_Signal();

//    system_Read_MotorSpd_Regulator_Signal();
}
/********************************��������************************************
*������:

*������������: ϵͳ����-ϵͳ��������flash���

*��������: ��

*��������ֵ: ��

*��ע:
*****************************************************************************/
uint8_t system_Save_Flash(void* System, uint8_t SaveIndex)
{
	SYSTEM_t* pSystem = System;
	syscon_cfg_handle_t hSystemConfig = sys_cfg_Hadnle();
	MOTOR_POS_t WriteIntoFlash[20] = {0};
    int32_t ReadParam[MOTOR_NB] = {0};
	uint8_t Ret = 0;
	/*���Ƹ�λ��*/
	if(SaveIndex & FS_MOTOR_SATE)
	{
		if(SYSTEM_GET_FLAG(pSystem->SaveIntoFlash, FS_MOTOR_SATE))
		{
			SYSTEM_CLR_FLAG(pSystem->SaveIntoFlash, FS_MOTOR_SATE);
			flash_adapter_Erase(PAGE, FSA_MOTOR_STATE, 1); 
			mc_app_Read_Param(MOTOR1, e_map_fdbkpos, &ReadParam[0]);
			WriteIntoFlash[0] = 0x55AA;
			WriteIntoFlash[1] = (pSystem->FaultFlag & FAULT_M1_ALL);
			if(pSystem->FaultFlag == 1)
				WriteIntoFlash[2] = ReadParam[0];
			else 
				WriteIntoFlash[2] = 0xFFFF;
			WriteIntoFlash[3] = WriteIntoFlash[0] + WriteIntoFlash[1] + WriteIntoFlash[2];  //���У��
			flash_adapter_Write(WORD, FSA_MOTOR_STATE, WORD, WriteIntoFlash, 4);
			Ret |= FS_MOTOR_SATE;
		}
	}
	/*���Ƹ���λ*/
	if(SaveIndex & FS_MOTOR_LIMIT)
	{
		if(SYSTEM_GET_FLAG(pSystem->SaveIntoFlash, FS_MOTOR_LIMIT))
		{
			SYSTEM_CLR_FLAG(pSystem->SaveIntoFlash, FS_MOTOR_LIMIT);
			flash_adapter_Erase(PAGE, FSA_MOTOR_LIMIT, 1);
			WriteIntoFlash[0] = 0x5555;
			WriteIntoFlash[1] = system_Get_Column_SpecificPos(COLUMN_POS_SET_TOP);
			WriteIntoFlash[2] = WriteIntoFlash[0] + WriteIntoFlash[1];  //���У��
			flash_adapter_Write(WORD, FSA_MOTOR_LIMIT, WORD, WriteIntoFlash, 3);
			Ret |= FS_MOTOR_LIMIT;
		}
	}
	/*��ϵͳ����*/
	if(SaveIndex & FS_SYS_PARAM)
	{
		if(SYSTEM_GET_FLAG(pSystem->SaveIntoFlash, FS_SYS_PARAM))
		{
			SYSTEM_CLR_FLAG(pSystem->SaveIntoFlash, FS_SYS_PARAM);
			flash_adapter_Erase(PAGE, FSA_SYS_PARAM, 1);
			WriteIntoFlash[0] = 0xAA55;
			if(pSystem->SysCon.Sys_CommunicationType == E_MODBUS)
			{
				WriteIntoFlash[1] = hModbusLink.SlaveAddr;
				WriteIntoFlash[2] = WriteIntoFlash[0] + WriteIntoFlash[1];
				flash_adapter_Write(WORD, FSA_SYS_PARAM, WORD, WriteIntoFlash, 3);
			}else if(pSystem->SysCon.Sys_CommunicationType == E_CAN)
			{
				WriteIntoFlash[1] = CO_GET_OD(OD_SI_TRG_SPD);
				WriteIntoFlash[2] = CO_GET_OD(OD_SI_HEARTBEAT);
				WriteIntoFlash[3] = CO_GET_OD(OD_SI_NODEID);
				WriteIntoFlash[4] = WriteIntoFlash[0] + WriteIntoFlash[1] + WriteIntoFlash[2] + WriteIntoFlash[3];
				flash_adapter_Write(WORD, FSA_SYS_PARAM, WORD, WriteIntoFlash, 5);
			}
			Ret |= FS_SYS_PARAM;
		}	
	}
	/*��ϵͳ���ò���*/
	if(SaveIndex & FS_SYS_CONFIG)
	{
		if(SYSTEM_GET_FLAG(pSystem->SaveIntoFlash, FS_SYS_CONFIG))
		{
			SYSTEM_CLR_FLAG(pSystem->SaveIntoFlash, FS_SYS_CONFIG);
			flash_adapter_Erase(PAGE, FSA_SYS_CONFIG, 1);
			WriteIntoFlash[0] = 0x5A5A;
			WriteIntoFlash[1] = hSystemConfig.Config_GearRatio;
			WriteIntoFlash[2] = hSystemConfig.Config_Lead;
			WriteIntoFlash[3] = hSystemConfig.Config_Route;
			WriteIntoFlash[4] = hSystemConfig.Config_SpeedMmps;
			WriteIntoFlash[5] = hSystemConfig.Config_OvcValue;
			WriteIntoFlash[6] = hSystemConfig.Config_OverVoltage;
			WriteIntoFlash[7] = hSystemConfig.Config_UnderVoltage;
			WriteIntoFlash[8] = WriteIntoFlash[0] + WriteIntoFlash[1] + WriteIntoFlash[2] + WriteIntoFlash[3] + WriteIntoFlash[4] + WriteIntoFlash[5] + WriteIntoFlash[6] + WriteIntoFlash[7]; //���У��
			flash_adapter_Write(WORD, FSA_SYS_CONFIG, WORD, WriteIntoFlash, 9);
			Ret |= FS_SYS_CONFIG;
		}
	}
	/*�������ò���*/
	if(SaveIndex & FS_MOTOR_CONFIG)
	{
		if(SYSTEM_GET_FLAG(pSystem->SaveIntoFlash, FS_MOTOR_CONFIG))
		{
			SYSTEM_CLR_FLAG(pSystem->SaveIntoFlash, FS_MOTOR_CONFIG);
			flash_adapter_Erase(PAGE, FSA_MOTOR_CONFIG, 1);
			WriteIntoFlash[0] = 0x5AA5;
			WriteIntoFlash[1] = hSystemConfig.Config_NodeSlaveAddr;
			WriteIntoFlash[2] = hSystemConfig.Config_CommunicationType;
			WriteIntoFlash[3] = hSystemConfig.Config_ResetRunMode;
			WriteIntoFlash[4] = hSystemConfig.Config_ResetDirection;
			WriteIntoFlash[5] = hSystemConfig.Config_ResetMode;
			WriteIntoFlash[6] = hSystemConfig.Config_ResetRaise;
			WriteIntoFlash[7] = hSystemConfig.Config_MotorRunMode;
			WriteIntoFlash[8] = hSystemConfig.Config_TopDetection;
			WriteIntoFlash[9] = hSystemConfig.Config_BtmDetection;
			WriteIntoFlash[10] = WriteIntoFlash[0] + WriteIntoFlash[1] + WriteIntoFlash[2] + WriteIntoFlash[3] + WriteIntoFlash[4] + WriteIntoFlash[5] + WriteIntoFlash[6] + WriteIntoFlash[7] + WriteIntoFlash[8] + WriteIntoFlash[9]; //���У��
			flash_adapter_Write(WORD, FSA_MOTOR_CONFIG, WORD, WriteIntoFlash, 11);
			Ret |= FS_MOTOR_CONFIG;
		}
	}
	/*��ײ����ò���*/
	if(SaveIndex & FS_BOOT_CONFIG)
	{
		if(SYSTEM_GET_FLAG(pSystem->SaveIntoFlash, FS_BOOT_CONFIG))
		{
			SYSTEM_CLR_FLAG(pSystem->SaveIntoFlash, FS_BOOT_CONFIG);
			flash_adapter_Erase(PAGE, FSA_BOOT_CONFIG, 1);
			WriteIntoFlash[0] = 0xAA55;
			WriteIntoFlash[1] = hSystemConfig.Config_HallDirectionSel;
			WriteIntoFlash[2] = hSystemConfig.Config_PhaseDirectionSel;
			WriteIntoFlash[3] = WriteIntoFlash[0] + WriteIntoFlash[1] + WriteIntoFlash[2]; //���У��
			flash_adapter_Write(WORD, FSA_BOOT_CONFIG, WORD, WriteIntoFlash, 4);
			Ret |= FS_BOOT_CONFIG;
		}
	}
	return Ret;
}
/********************************��������************************************
*������:

*������������: ϵͳ����-ϵͳ״̬������

*��������: ��

*��������ֵ: ��

*��ע:
*****************************************************************************/
/*״̬����-��ʼ��״̬(����Ӧ�ò���ô˺�����Ӹ�return,��ֹ������2���޸�״̬���������)*/
void system_FSM_StateJump(SYSTEM_t* pSystem, SYS_STATE_t NextState)
{
    SYS_STATE_t State = pSystem->State;
    
    if(NextState == E_SYS_STATE_INIT)
    {

    }else if(NextState == E_SYS_STATE_ZERO)
    {
        if(State == E_SYS_STATE_INIT || State == E_SYS_STATE_FAULT || State == E_SYS_STATE_IDLE)
            goto JUMP;
    }else if(NextState == E_SYS_STATE_IDLE)
    {
        if(State == E_SYS_STATE_INIT || State == E_SYS_STATE_ZERO || State == E_SYS_STATE_MOTOR_RUN \
			|| State == E_SYS_STATE_FAULT || State == E_SYS_STATE_MOTOR_GOTOPOS || State == E_SYS_STATE_SETTINGS\
            || State == E_SYS_STATE_SETZEROPOS || State == E_SYS_CONFIG_INIT)
            goto JUMP;
    }else if(NextState == E_SYS_STATE_MOTOR_RUN)
    {
        if(State == E_SYS_STATE_IDLE)
            goto JUMP;
    }else if(NextState == E_SYS_STATE_FAULT)
    {
        if(State == E_SYS_STATE_INIT || State == E_SYS_STATE_MOTOR_RUN || State == E_SYS_STATE_ZERO \
			|| State == E_SYS_STATE_IDLE || State == E_SYS_STATE_MOTOR_GOTOPOS || State == E_SYS_STATE_SETZEROPOS)
            goto JUMP;
    }else if(NextState == E_SYS_STATE_SETTINGS)
    {
        if(State == E_SYS_STATE_IDLE)
            goto JUMP;
    }else if(NextState == E_SYS_STATE_MOTOR_GOTOPOS)
    {
        if(State == E_SYS_STATE_IDLE)
            goto JUMP;
    }else if(NextState == E_SYS_STATE_SETZEROPOS)
    {
         if(State == E_SYS_STATE_IDLE)
            goto JUMP;
	}else if(NextState == E_SYS_CONFIG_INIT)
    {
         if(State == E_SYS_STATE_IDLE || State == E_SYS_STATE_FAULT)
            goto JUMP;
    }else
    {   //do nothing
    }
    return;
JUMP:
    pSystem->State = NextState;
    pSystem->Step = 0;
}
/*ϵͳ���õĳ�ʼ��*/
void system_ConfigInit_Handler(void* System)
{
	SYSTEM_t* pSystem = System;
	sys_cfg_flag_handle_t* hSystemFlagCfg = sys_cfg_FlagHandle();
//	SysCfg->ControlMode = E_CONFIGMODE;
	hSystemFlagCfg->PerameterSetMode = E_CONFIGMODESET;
	SYSTEM_SET_FLAG(hSystemFlagCfg->ControlMode,SYS_CFG_CONTROL_MODE);
	if(pSystem->Step == 0)
	{
		pSystem->CfgCount = 0;
		pSystem->Step++;
	}
	else if(pSystem->Step == 1)
	{
		if(SYSTEM_GET_FLAG(hSystemFlagCfg->SaveCfgFlashFlag,SYS_CFG_SAVE_FLASH_FLAG))
		{
			SYSTEM_CLR_FLAG(hSystemFlagCfg->SaveCfgFlashFlag,SYS_CFG_SAVE_FLASH_FLAG);
			SYSTEM_SET_FLAG(pSystem->SaveIntoFlash, FS_SYS_CONFIG);
			SYSTEM_SET_FLAG(pSystem->SaveIntoFlash, FS_MOTOR_CONFIG);
			SYSTEM_SET_FLAG(pSystem->SaveIntoFlash, FS_BOOT_CONFIG);
			/*���flash����*/
			(void)system_Save_Flash(pSystem, (FS_SYS_CONFIG | FS_MOTOR_CONFIG | FS_BOOT_CONFIG));
			sys_cfg_Controller();/*�˳�����ģʽ��ʾ*/
			hSystemFlagCfg->PerameterSetMode = E_IDLEMODESET;
			/*��������Ժ���Ҫ�ѹ��ڵ��������ص�Flashȫ����*/
			flash_adapter_Erase(PAGE, FSA_MOTOR_STATE, 1);
			flash_adapter_Erase(PAGE, FSA_MOTOR_LIMIT, 1);
			flash_adapter_Erase(PAGE, FSA_SYS_PARAM, 1);
/*************************************************************ϵͳ��λ����*******************************************************************************/
			__DSB();                                                          /* Ensure all outstanding memory accesses included
																		   buffered write are completed before reset */
			SCB->AIRCR  = (uint32_t)((0x5FAUL << SCB_AIRCR_VECTKEY_Pos)    |
								   (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
									SCB_AIRCR_SYSRESETREQ_Msk    );         /* Keep priority group unchanged */
			__DSB();                                                          /* Ensure completion of memory access */

			for(;;)                                                           /* wait until reset */
			{
				__NOP();
			}
/************************************************************************************************************************************************/
		}
		
		if(pSystem->CfgCount >= 3000)
		{
			sys_cfg_Controller();/*�˳�����ģʽ��ʾ*/
			pSystem->Step++;
		}
	}
	else if(pSystem->Step == 2)
	{
		system_FSM_StateJump(pSystem, E_SYS_STATE_IDLE);
	}
}

/*״̬����-��ʼ��״̬*/
void system_Init_Handler(void* System)
{
    SYSTEM_t* pSystem = System;
	uint32_t Data[10] = {0};
    uint32_t Check = 0;
    system_Communication_None();
	/*�Ƹ����ò�����ʼ��*/
	system_Motor_Init(pSystem);
//    /*��ʼ���������ģʽ*/
//    while(system_Read_MC_Mode_Signal(0) == 0);
	/*��ʼ��������Ʋ���*/
	pSystem->MotorDC = M_MAX_DC;
	pSystem->MotorSpeed = system_Modify_Column_Speed(0, pSystem->SysCon.Sys_SpeedMmps);
    
    /*��ʼ������λ*/
    //system_Set_Column_SpecificPos(COLUMN_POS_ZERO, RESET_RAISE);
	if(pSystem->SysCon.Sys_ResetDirection == E_RESET_BTM)
	{
		system_Set_Column_SpecificPos(COLUMN_POS_ZERO, (BTM_POS_HALLDATA + pSystem->SysCon.Sys_ResetRaise));
		system_Set_Column_SpecificPos(COLUMN_POS_TOP_LIMIT, pSystem->SysCon.Sys_Route);
		system_Set_Column_SpecificPos(COLUMN_POS_SET_TOP, pSystem->SysCon.Sys_Route);
	}else if(pSystem->SysCon.Sys_ResetDirection == E_RESET_TOP)
	{
		system_Set_Column_SpecificPos(COLUMN_POS_ZERO, BTM_POS_HALLDATA);
		system_Set_Column_SpecificPos(COLUMN_POS_TOP_LIMIT, (pSystem->SysCon.Sys_Route + pSystem->SysCon.Sys_ResetRaise));
		system_Set_Column_SpecificPos(COLUMN_POS_SET_TOP, (pSystem->SysCon.Sys_Route + pSystem->SysCon.Sys_ResetRaise));
	}
    flash_adapter_Read(DOUBLE_WORD, FSA_MOTOR_LIMIT, WORD, Data, sizeof Data / sizeof Data[0]);
    if(Data[0] == 0x5555)
    {
        Check = Data[0] + Data[1];
        if(Check == Data[2])
        {
            system_Set_Column_SpecificPos(COLUMN_POS_SET_TOP, (float)Data[1]);
        }
    }
	system_Communication_Motor_Init(); //ϵͳ���ó�ʼ��
	
	/*��ʼ���Ƹ�λ��*/
    flash_adapter_Read(WORD, FSA_MOTOR_STATE, WORD, Data, 4);
    if(Data[0] != 0x55AA)
    {
		system_Reset_Noreset(pSystem); //����Ƿ���Ҫ��λ
    }else
    {
        Check = Data[0] + Data[1] + Data[2];
        if(Check != Data[3])
        {
			system_Reset_Noreset(pSystem); //����Ƿ���Ҫ��λ
        }else
        {
			SYSTEM_SET_FLAG(pSystem->FaultFlag, Data[1]);
			if(Data[2] == 0xFFFF)
			{
				 pSystem->ZeroFound = 0;
			}else
			{
				mc_app_Write_Param((MOTOR1), e_map_fdbkpos, Data[2]);
				pSystem->ColumnPosMM[0] = system_Get_Column_CurrentPos(0);
				pSystem->ZeroFound = 1;
			}
			flash_adapter_Erase(PAGE, FSA_MOTOR_STATE, 1);
			SYSTEM_SET_FLAG(pSystem->SaveIntoFlash, FS_MOTOR_SATE);	/* ����״̬��Ҳ��Ҫ���ù��Ϻ��г̱����־����֤��ε����Ժ��ϵ������֮ǰ�Ĺ��ϴ��� */
            system_FSM_StateJump(pSystem, E_SYS_STATE_IDLE);
        }
    }

}
/*״̬����-����״̬*/
void system_FindZero_Handler(void* System)
{
    SYSTEM_t* pSystem = System;
    BTN_STATE_t KeyMResetState = syste_Reset_Mode();

    //������ʼ��
    if(pSystem->Step == 0)
    {
        flash_adapter_Erase(PAGE, FSA_MOTOR_STATE, 1);
        for(uint8_t i = 0; i< SYS_MOTOR_NB; i++)
        {
            mc_app_Write_Param((MOTOR1<<i), e_map_drvoutput_max, PWM_OUTPUT_FULLSCALE / 2);
            system_Modify_Column_Speed(i, pSystem->SysCon.Sys_SpeedMmps / 2);  //��λ������
        }
        pSystem->Step = 2;
    }
    /*��λ��ͣ��,������λ�������ж�*/
    else if(pSystem->Step == 1)
    {
		for(uint8_t i = 0; i< SYS_MOTOR_NB; i++)
        {
            mc_app_Write_Param((MOTOR1<<i), e_map_drvoutput_max, PWM_OUTPUT_FULLSCALE / 2);
            system_Modify_Column_Speed(i, pSystem->SysCon.Sys_SpeedMmps / 2);  //��λ������
        }
		if(system_Reset_Direction(pSystem) == 1)
		{
			pSystem->Step++;
		}
    }
    /*���������λ����*/
    else if(pSystem->Step == 2)
    {
        if(system_Reset_Start() == 1)
        {
            pSystem->Step++;
        }
    }
    /*���е�0λ�жϼ�����쳣���(2�ּ�Ⲣ��: ��λ���� || ��ת)*/
    else if(pSystem->Step == 3)
    {
		if(system_Reset_Detection_Mode(pSystem) == 1)
		{
			pSystem->Step++;
		}
    }
    /*��λ���ʼλ��*/
    else if(pSystem->Step == 4)
    {
		if(system_Reset_Initial_Position() == 1)
		{
			pSystem->Step++;
		}
    }
    /*��λ���е���ʼλ�����*/
    else if(pSystem->Step == 5)
    {
        if(mc_app_Get_State(MOTOR1) == e_mas_idle)
        {
            if(!SYSTEM_GET_FLAG(pSystem->FaultFlag, FAULT_M1_ALL))  //�޵�����ϼ���
            {
                pSystem->Step++;
            }
        }
    }
    /*��λ�ɹ�,�˳���λ*/
    else if(pSystem->Step == 6)
    {
        pSystem->ZeroFound = 1;
		system_Communication_None();
        SYSTEM_CLR_FLAG(pSystem->FaultFlag, FAULT_POS);
        SYSTEM_SET_FLAG(pSystem->SaveIntoFlash, FS_MOTOR_SATE);
        if(KeyMResetState == E_BTN_STA_RELEASED)
        {
            for(uint8_t i = 0; i< SYS_MOTOR_NB; i++)
            {
                mc_app_Write_Param((MOTOR1<<i), e_map_drvoutput_max, (PWM_OUTPUT_FULLSCALE / 100) * MAX_PWM_PERCENT);
                system_Modify_Column_Speed(i, pSystem->SysCon.Sys_SpeedMmps);  //�ٶȻָ������ٶ�
            }
            system_FSM_StateJump(pSystem, E_SYS_STATE_IDLE);
            return;
        }
    }
    /*����ֹͣ*/
    else if(pSystem->Step == 100)
    {
        if(system_Is_Motor_Stop(MOTOR1))
        {
            pSystem->Step = 1;       /*֮ǰmodbusͨѶ��ֱ�Ӹ�Step=1*/
            if(pSystem->ZeroFound == 1)
            {
                SYSTEM_CLR_FLAG(pSystem->FaultFlag, FAULT_POS);
                SYSTEM_SET_FLAG(pSystem->SaveIntoFlash, FS_MOTOR_SATE);
            }
            for(uint8_t i = 0; i< SYS_MOTOR_NB; i++)
            {
                mc_app_Write_Param((MOTOR1<<i), e_map_drvoutput_max, (PWM_OUTPUT_FULLSCALE / 100) * MAX_PWM_PERCENT);
                system_Modify_Column_Speed(i, pSystem->SysCon.Sys_SpeedMmps);  //�ٶȻָ������ٶ�
            }
        }else
        {
            system_Motor_Ramp_Stop(MOTOR1, e_mac_stop_openloop);   //ps: ��λ��ͣ,��ִ����ֹͣ�и��ʻ����е��׶�ת,��������
        }
    }else
    {
        //do nothing
    }
    /*��������쳣���(����ʱ��������)*/
    if(system_Get_Fault(FAULT_UDV | FAULT_OVV))
    {
    
        if(pSystem->ZeroFound == 1)
            SYSTEM_SET_FLAG(pSystem->SaveIntoFlash, FS_MOTOR_SATE);
        system_FSM_StateJump(pSystem, E_SYS_STATE_FAULT);
    }else if(system_Get_Fault(FAULT_MOTOR_ALL))
    {
        if(pSystem->Step != 3 && pSystem->Step != 100)
		{
			SYSTEM_SET_FLAG(pSystem->SaveIntoFlash, FS_MOTOR_SATE);
            system_FSM_StateJump(pSystem, E_SYS_STATE_FAULT);
		}
    }else
    {
		if(system_Reset_Run_Mode(KeyMResetState) == 1)
		{
			if(pSystem->Step > 2 && pSystem->Step < 100)
                pSystem->Step = 100;
		}
		/*�а�������-��ͣ*/
		if(KeyMResetState != E_BTN_STA_RELEASED)
		{
			pSystem->ResetHandControlling = 1;	//��Ϊ�ֿ�ģʽ(�����ͬ�������)
		}
    }
}
/*״̬����-����״̬*/
void system_Idle_Handler(void* System)
{
    SYSTEM_t* pSystem = System;
	sys_cfg_flag_handle_t* hSystemFlagCfg = sys_cfg_FlagHandle();
    float ColumnPos = 0;
    BTN_STATE_t KeyMUState = btn_Get_State(KEY_MOTOR_UP);
    BTN_STATE_t KeyMDState = btn_Get_State(KEY_MOTOR_DOWN);
	pSystem->RunDir = DIR_STOP;
	
	/*��λ���������ù��� ����ӿ���״̬�������ó�ʼ��״̬*/
	if(SYSTEM_GET_FLAG(hSystemFlagCfg->EnterConfigFlag,SYS_CFG_ENTER_CFG_FLAG))
	{
		SYSTEM_CLR_FLAG(hSystemFlagCfg->EnterConfigFlag,SYS_CFG_ENTER_CFG_FLAG);
		system_FSM_StateJump(pSystem, E_SYS_CONFIG_INIT);
		return;
	}
    
	/*�쳣��⼰LED��ʾ*/
	if(system_Get_Fault(FAULT_NOT_MOTOR_ALL))
	{
		system_FSM_StateJump(pSystem, E_SYS_STATE_FAULT);
	}
    /*���flash����*/
//	(void)system_Save_Flash(pSystem, (FS_MOTOR_SATE | FS_MOTOR_LIMIT | FS_SYS_PARAM ));
    
	if(pSystem->ZeroFound == 1)
	{
		system_Idle_ComGotoPos(pSystem);
	}
	/*����������״̬(ע: IO�������ȼ�������λ��ͨѶ����)*/
	if(KeyMUState == E_BTN_STA_RELEASED && KeyMDState == E_BTN_STA_RELEASED)
    {
		pSystem->HandControlling = 0;
		system_Idle_ComRun(pSystem);
    }else
    {
		if(pSystem->SysCon.Sys_MotorRunMode == E_MOTOR_INCHING)
		{
			pSystem->HandControlling = 1;
		}else
		{
		}
		system_Communication_None();		//�ֿ�ģʽ�²���ӦͨѶ����
		if(KeyMUState != E_BTN_STA_RELEASED && KeyMDState != E_BTN_STA_RELEASED)
		{
			return;		//��ϼ������Ǽ�ͣ����,���������
		}else
		{
            if(KeyMUState == E_BTN_STA_FIRSTPRESSHOLD || KeyMUState == E_BTN_STA_LONGPRESS)
            {
                ColumnPos = system_Get_Column_SpecificPos(COLUMN_POS_SET_TOP);
				pSystem->RunDir = DIR_UP;
				if(system_Idle_Start(pSystem,ColumnPos) == 1)
					return;
            }
            if(KeyMDState == E_BTN_STA_FIRSTPRESSHOLD || KeyMDState == E_BTN_STA_LONGPRESS)
            {
                ColumnPos = system_Get_Column_SpecificPos(COLUMN_POS_ZERO);
                pSystem->RunDir = DIR_DOWN;
                if(system_Idle_Start(pSystem,ColumnPos) == 1)
					return;
            }
			
		}
    }
    
}
/*״̬����-�������״̬*/
void system_MotorRun_Handler(void* System)
{
    SYSTEM_t* pSystem = System;
    BTN_STATE_t KeyMUState = btn_Get_State(KEY_MOTOR_UP);
    BTN_STATE_t KeyMDState = btn_Get_State(KEY_MOTOR_DOWN);

    /*���������ʼ��*/
    if(pSystem->Step == 0)
    {
		if(SYSTEM_GET_FLAG(pSystem->LimitFlag, M1_TOP) && pSystem->RunDir == DIR_UP)
		{
            system_FSM_StateJump(pSystem, E_SYS_STATE_IDLE);
            return;
		}else if(SYSTEM_GET_FLAG(pSystem->LimitFlag, M1_BTM) && pSystem->RunDir == DIR_DOWN)
		{
			mc_app_Write_Param(MOTOR1, e_map_fdbkpos, BTM_POS_HALLDATA);//ֻ�ڵ������ʱ����λ��,��ֹ�����󴥷�(�ͻ�������24V/5Vʱ���д�����)
            pSystem->ZeroFound = 1;
            system_FSM_StateJump(pSystem, E_SYS_STATE_IDLE);
            return;
		}else
		{
			pSystem->Step++;
            /*�������*/
			flash_adapter_Erase(PAGE, FSA_MOTOR_STATE, 1);    //����ʱ�����Ѵ����λ��
			system_Motor_Move(MOTOR1, e_mac_start_closeloop, pSystem->RunDir);
		}
		btn_Clr_Event(KEY_MOTOR_UP);
		btn_Clr_Event(KEY_MOTOR_DOWN);
    }
	/*�������*/
    else if(pSystem->Step == 1)
    {
        /*�������ʵʱ�ٶȵ���*/
        mc_app_Write_Param(MOTOR1, e_map_targetspd, pSystem->MotorSpeed);
    }
    /*ִ����ֹͣ����*/
    else if(pSystem->Step == 100)
    {
        system_Motor_Ramp_Stop(MOTOR1, e_mac_stop_closeloop);
        pSystem->Step++;
    }
    /*ִ�м�ͣ����*/
    else if(pSystem->Step == 200)
    {
        mc_app_Set_Single_Motor_Cmd(MOTOR1, e_mac_stop, 0);
        pSystem->Step++;
    }
    /*�жϵ���ǹ���ֹͣ*/
    else
    {
        if(system_Is_Motor_Stop(MOTOR1))
        {
			system_SignalSwitch();
			if(pSystem->ZeroFound == 1)
				SYSTEM_SET_FLAG(pSystem->SaveIntoFlash, FS_MOTOR_SATE);
            system_FSM_StateJump(pSystem, E_SYS_STATE_IDLE);
            return;
        }
    }

    /*�������ʱ�쳣(����ʱ��������)-��ͣ*/
    if(system_Get_Fault(FAULT_M1_HAB))
    {
		if(system_CfgBtmDetet() == E_BTM_HALL_ABNORMAL)
		{
			if(pSystem->RunDir == DIR_DOWN)
			{
				system_Clear_Motor_Fault(MOTOR1);
				mc_app_Write_Param(MOTOR1, e_map_fdbkpos, BTM_POS_HALLDATA);//ֻ�ڵ������ʱ����λ��,��ֹ�����󴥷�(�ͻ�������24V/5Vʱ���д�����)
				pSystem->ZeroFound = 1;
				if(pSystem->Step < 200)
				{
					pSystem->Step = 200;
				}
			}else if(system_CfgTopDetet() != E_TOP_HALL_ABNORMAL)
			{
				SYSTEM_SET_FLAG(pSystem->SaveIntoFlash, FS_MOTOR_SATE);
				system_FSM_StateJump(pSystem, E_SYS_STATE_FAULT);
				return;
			}				
		}
		if(system_CfgTopDetet() == E_TOP_HALL_ABNORMAL)
		{
			
			if(pSystem->RunDir == DIR_UP)
			{
				system_Clear_Motor_Fault(MOTOR1);
				mc_app_Write_Param(MOTOR1, e_map_fdbkpos, (MOTOR_POS_t)((system_Hall_PER_MM())*pSystem->SysCon.Sys_Route));//ֻ�ڵ������ʱ����λ��,��ֹ�����󴥷�(�ͻ�������24V/5Vʱ���д�����)
				pSystem->ZeroFound = 1;
				if(pSystem->Step < 200)
				{
					pSystem->Step = 200;	
				}
			}else if(system_CfgBtmDetet() != E_BTM_HALL_ABNORMAL)
			{
				SYSTEM_SET_FLAG(pSystem->SaveIntoFlash, FS_MOTOR_SATE);
				system_FSM_StateJump(pSystem, E_SYS_STATE_FAULT);
				return;
			}
		}
		if((system_CfgBtmDetet() != E_BTM_HALL_ABNORMAL) &&(system_CfgTopDetet() != E_TOP_HALL_ABNORMAL))
		{
			SYSTEM_SET_FLAG(pSystem->SaveIntoFlash, FS_MOTOR_SATE);
			system_FSM_StateJump(pSystem, E_SYS_STATE_FAULT);
			return;
		}
    }
    /*�������ʱ�쳣(����ʱ��������)-��ͣ*/
    else if(system_Get_Fault(FAULT_OVV | FAULT_UDV | FAULT_M1_OVC))
    {
        SYSTEM_SET_FLAG(pSystem->SaveIntoFlash, FS_MOTOR_SATE);
        system_FSM_StateJump(pSystem, E_SYS_STATE_FAULT);
    }
    /*��Ӳ��λ-��ͣ*/
    else if((pSystem->RunDir == DIR_UP && (pSystem->LimitFlag & M1_TOP)) || \
			(pSystem->RunDir == DIR_DOWN && (pSystem->LimitFlag & M1_BTM)) )
    {
        if(pSystem->Step < 200)
            pSystem->Step = 200;
    }
    /*������ϼ�����-��ͣ*/
	else if(KeyMUState != E_BTN_STA_RELEASED && KeyMDState != E_BTN_STA_RELEASED)
	{
		pSystem->HandControlling = 1;
        if(pSystem->Step < 200)
            pSystem->Step = 200;	
	}
    /*ת����ֹͣ����*/
    else
    {
		/*�������ģʽ  �㶯or����*/
		if(system_Motor_Run_Mode(KeyMUState,KeyMDState) == 1)
		{
			if(pSystem->Step < 100)
				pSystem->Step = 100;
		}
		/*�а�������-��ͣ*/
		if(KeyMUState != E_BTN_STA_RELEASED || KeyMDState != E_BTN_STA_RELEASED)
		{
			pSystem->HandControlling = 1;	//��Ϊ�ֿ�ģʽ(�����ͬ�������)
		}
    }
    /*�ֿ�ģʽ�²���Ӧ��λ������*/
	if(pSystem->HandControlling == 1)
	{
		system_Communication_None();  //��λ������ָ��
	}else
	{
		/*ͨѶ������-���е�Ŀ��λ��*/
		if(system_Communication_Goto() == 1)
		{
			//pSystem->ModbusCmd = e_cmc_none;		�����ModbusCanCmd����(����),��ϵͳ����IDLE״̬�����Ӧ������
			if(pSystem->Step < 100)
				pSystem->Step = 100;
		}
		/*ͨѶ������-ֹͣ*/
		if(system_Communication_Stop() == 1)
		{
			system_Communication_None();
			if(pSystem->Step < 100)
				pSystem->Step = 100;
		}		
		/*ͨѶ������-����/�½�*/
		else if(((system_Communication_Down() ==1) && pSystem->RunDir == DIR_UP) || \
				((system_Communication_Up() == 1) && pSystem->RunDir == DIR_DOWN))
		{
			//pSystem->ModbusCmd = e_cmc_none;		�����ModbusCanCmd����(����),��ϵͳ����IDLE״̬�����Ӧ������
			if(pSystem->Step < 100)
				pSystem->Step = 100;
		}
	}

}
/*״̬����-����Զ����е�Ŀ��λ��״̬*/
void system_MotorGotoPos_Handler(void* System)
{
    SYSTEM_t* pSystem = System;
    int32_t ReadParam[MOTOR_NB] = {0};
    static MOTOR_POS_t s_TargetHallPos = 0;
    MOTOR_POS_t ColumnHallPos = 0;
	int8_t SysSign = 0;	
    float ColumnPosTop = 0;
    float ColumnPosBtm = 0;
    MOTOR_SPD_t MotorSpd = 0;
    static float s_SlowStopMM = 0;
    BTN_STATE_t KeyMUState = btn_Get_State(KEY_MOTOR_UP);
    BTN_STATE_t KeyMDState = btn_Get_State(KEY_MOTOR_DOWN);

    //�����Ƹ˵�ǰλ��
    ColumnPosTop = system_Get_Column_SpecificPos(COLUMN_POS_SET_TOP);
    ColumnPosBtm = system_Get_Column_SpecificPos(COLUMN_POS_ZERO);


    /*����״�������ʼ��*/
    if(pSystem->Step == 0)
    {
        mc_app_Read_Param(MOTOR1, e_map_fdbkpos, &ReadParam[0]);
        if(pSystem->GotoHallPos + 10 < ReadParam[0])
        {
            pSystem->RunDir = DIR_DOWN;
			flash_adapter_Erase(PAGE, FSA_MOTOR_STATE, 1);    //����ʱ�����Ѵ����λ��
            mc_app_Set_Single_Motor_Cmd(MOTOR1, e_mac_goto_targetpos, pSystem->GotoHallPos);
            pSystem->Step = 2;
        }else if(pSystem->GotoHallPos > ReadParam[0] + 10)
        {
            pSystem->RunDir = DIR_UP;		
			flash_adapter_Erase(PAGE, FSA_MOTOR_STATE, 1);    //����ʱ�����Ѵ����λ��
            mc_app_Set_Single_Motor_Cmd(MOTOR1, e_mac_goto_targetpos, pSystem->GotoHallPos);
            pSystem->Step = 2;
        }else
        {
            system_FSM_StateJump(pSystem, E_SYS_STATE_IDLE);
            return;
        }
		btn_Clr_Event(KEY_MOTOR_UP);
		btn_Clr_Event(KEY_MOTOR_DOWN);
    }else if(pSystem->Step == 1)
    {
		/*Ŀ��λ�ø��µ�����*/   
        mc_app_Read_Param(MOTOR1, e_map_fdbkpos, &ReadParam[0]);
        if(pSystem->GotoHallPos + 10 < ReadParam[0])
        {
            pSystem->RunDir = DIR_DOWN;
            mc_app_Set_Single_Motor_Cmd(MOTOR1, e_mac_goto_targetpos, pSystem->GotoHallPos);
        }else if(pSystem->GotoHallPos > ReadParam[0] + 10)
        {
            pSystem->RunDir = DIR_UP;
            mc_app_Set_Single_Motor_Cmd(MOTOR1, e_mac_goto_targetpos, pSystem->GotoHallPos);
        }
		pSystem->Step++;
    }
    /*�������*/
    else if(pSystem->Step == 2)
    {
        /*�������ʵʱ�ٶȵ���*/
        mc_app_Write_Param(MOTOR1, e_map_ol_targetdc, pSystem->MotorDC);
		mc_app_Write_Param(MOTOR1, e_map_targetspd, pSystem->MotorSpeed); //modbus�ĳ����������е�Ŀ��λ�õĳ�������û��ʵʱ�����ٶȵģ�������Ҫ����
        /*������ֹͣ����(ֵѡȡ���ٶ�,���ٶȶ��й�ϵ,Ŀǰֻ����ֹͣ�ٶ�)*/
        mc_app_Read_Param(MOTOR1, e_map_fdbkspd, &MotorSpd);
		SysSign = (MotorSpd >= 0)? 1 : (-1) ;
		s_SlowStopMM = (MotorSpd + (SysSign*250)) / 500 + (SysSign*MOTOR_SLOW_STOP) ;  //���㹫ʽCAN��modbus�Ĳ�һ����Ҫ����ȷ��
//        s_SlowStopMM = (MotorSpd + 250) / 500 + 0.1;	 //ת��<3000,ѡ���ֹͣ����
        /*���е�Ŀ��λ���ж�*/
        if(mc_app_Get_State(MOTOR1) == e_mas_idle)
        {
            SYSTEM_SET_FLAG(pSystem->SaveIntoFlash, FS_MOTOR_SATE);
            system_FSM_StateJump(pSystem, E_SYS_STATE_IDLE);
            return;
        }
    }
    /*ִ����ֹͣ����*/
    else if(pSystem->Step == 100)
    {
        mc_app_Read_Param(MOTOR1, e_map_fdbkpos, &ReadParam[0]);
        s_TargetHallPos = ReadParam[0] + (MOTOR_POS_t)(s_SlowStopMM * (system_Hall_PER_MM()));   //��ֹͣ
        if(pSystem->RunDir == DIR_UP)
        {
            ColumnHallPos = system_Column_Pos_2_HallData(ColumnPosTop);
            if(s_TargetHallPos > ColumnHallPos)
                s_TargetHallPos = ColumnHallPos;
        }else //RunDir == DIR_DOWN
        {
            ColumnHallPos = system_Column_Pos_2_HallData(ColumnPosBtm);
            if(s_TargetHallPos < ColumnHallPos)
                s_TargetHallPos = ColumnHallPos;
        }
        mc_app_Set_Single_Motor_Cmd(MOTOR1, e_mac_goto_targetpos, s_TargetHallPos);
        pSystem->Step++;
    }
    /*ִ�м�ͣ����*/
    else if(pSystem->Step == 200)
    {
        mc_app_Set_Single_Motor_Cmd(MOTOR1, e_mac_stop, 0);
        pSystem->Step++;
    }
    /*�жϵ���ǹ���ֹͣ*/
    else
    {
        if(system_Is_Motor_Stop(MOTOR1))
        {
            SYSTEM_SET_FLAG(pSystem->SaveIntoFlash, FS_MOTOR_SATE);
            system_FSM_StateJump(pSystem, E_SYS_STATE_IDLE);
            return;
        }
    }
    /*�������ʱ�쳣(����ʱ��������)-��ͣ*/
    if(system_Get_Fault(FAULT_MOTOR_ALL))
    {
		SYSTEM_SET_FLAG(pSystem->SaveIntoFlash, FS_MOTOR_SATE);
        system_FSM_StateJump(pSystem, E_SYS_STATE_FAULT);
		return;
    }
    /*�������ʱ�쳣(����ʱ��������)-��ͣ*/
    else if(system_Get_Fault(FAULT_OVV | FAULT_UDV))
    {
        SYSTEM_SET_FLAG(pSystem->SaveIntoFlash, FS_MOTOR_SATE);
        system_FSM_StateJump(pSystem, E_SYS_STATE_FAULT);
		return;
    }
	/*��Ӳ��λ-��ͣ*/
    else if((pSystem->RunDir == DIR_UP && (pSystem->LimitFlag & M1_TOP)) || \
			(pSystem->RunDir == DIR_DOWN && (pSystem->LimitFlag & M1_BTM)) )
    {
        if(pSystem->Step < 200)
            pSystem->Step = 200;
    }
	/*������ϼ�����-��ͣ*/
	else if(KeyMUState != E_BTN_STA_RELEASED && KeyMDState != E_BTN_STA_RELEASED)
	{
		pSystem->HandControlling = 1;
        if(pSystem->Step < 200)
            pSystem->Step = 200;
	}
    /*�˳��Զ�����,ת����ֹͣ*/
    else
    {
		/*�������ģʽ  �㶯or����*/
		if(system_Motor_Run_Mode(KeyMUState,KeyMDState) == 1)
		{
			if(pSystem->Step < 100)
				pSystem->Step = 100;
		}
		/*�а�������-��ͣ*/
		if(KeyMUState != E_BTN_STA_RELEASED || KeyMDState != E_BTN_STA_RELEASED)
		{
			pSystem->HandControlling = 1;	//��Ϊ�ֿ�ģʽ(�����ͬ�������)
		}	
    }
    /*�ֿز����²���ӦͨѶ����*/
	if(pSystem->HandControlling == 1)
	{
		system_Communication_None(); //��λ������ָ��		
	}else
	{
		/*ͨѶ������-Ŀ��λ�ø���*/
		if(system_Communication_Goto() == 1)
		{
			system_Communication_None(); //��λ������ָ��
			if(pSystem->Step < 100 && pSystem->Step > 0)
			{
				pSystem->Step = 1;
				pSystem->GotoHallPos = system_Communication_Updategotopos();
			}
		}
		/*ͨѶ������-ֹͣ*/
		else if(system_Communication_Stop() == 1)
		{
			system_Communication_None(); //��λ������ָ��
			if(pSystem->Step < 100)
				pSystem->Step = 100;
		}
		/*ͨѶ������-����/�½�*/
		else if(((system_Communication_Down() ==1) && pSystem->RunDir == DIR_UP) || \
				((system_Communication_Up() == 1) && pSystem->RunDir == DIR_DOWN))
		{
			//pSystem->ModbusCmd = e_cmc_none;		�����ModbusCanCmd����(����),��ϵͳ����IDLE״̬�����Ӧ������
			if(pSystem->Step < 100)
				pSystem->Step = 100;
		}
	}
}
/*״̬����-�������״̬*/
void system_Fault_Handler(void* System)
{
    SYSTEM_t* pSystem = System;
	sys_cfg_flag_handle_t* hSystemFlagCfg = sys_cfg_FlagHandle();
	BTN_STATE_t KeyMUState = btn_Get_State(KEY_MOTOR_UP);  /*�������¼�����ᴥ�������Ĺ���*/
    BTN_STATE_t KeyMDState = btn_Get_State(KEY_MOTOR_DOWN);
//	system_Communication_None();   /* ����״ֱ̬�ӽ���λ��ָ��Ϊ���л������ */
    /*��ѹ\Ƿѹ\���ȹ��ϻָ���,�˳�����״̬*/

    if(system_Get_Fault(FAULT_ALL) == 0)
    {
		if(mc_app_Get_State(MOTOR1) == e_mas_idle)
		{
			system_Communication_None();
			system_FSM_StateJump(pSystem, E_SYS_STATE_IDLE);
			return;
		}
    }else
    {
		/*�жϵ��λ�ö�ʧ��hall�쳣*/
        if(system_Get_Fault(FAULT_M1_HAB | FAULT_POS))
        {
			if(pSystem->SysCon.Sys_ResetMode == E_DETECTION_SIGNAL)
			{
				
			}else
			{
				SYSTEM_SET_FLAG(pSystem->FaultFlag, FAULT_POS);//HALL�쳣������,ǿ���ù��ϱ�־������������ź�
			}
            pSystem->ZeroFound = 0;     
        }
    }
	/*��λ���������ù��� ����ӿ���״̬�������ó�ʼ��״̬*/
	if(SYSTEM_GET_FLAG(hSystemFlagCfg->EnterConfigFlag,SYS_CFG_ENTER_CFG_FLAG))
	{
		SYSTEM_CLR_FLAG(hSystemFlagCfg->EnterConfigFlag,SYS_CFG_ENTER_CFG_FLAG);
		system_FSM_StateJump(pSystem, E_SYS_CONFIG_INIT);
		return;
	}
	/*�ȴ����ֹͣ*/
    if(pSystem->Step == 0)
    {
        /*���ֹͣ*/
        if(mc_app_Get_State(MOTOR1) != e_mas_idle)
        {
            mc_app_Set_Single_Motor_Cmd(MOTOR1, e_mac_stop, 0);
        }else
        {
			pSystem->RunDir = DIR_STOP;
            SYSTEM_CLR_FLAG(pSystem->MotorFlag, M1_MOVE | M2_MOVE);
            /*���flash����*/
			pSystem->Step++;
        }
    }else if(pSystem->Step == 1)
	{
		/*�ж��Ƿ��������ǵ���ģʽ�����Ƿ��������*/
		if(pSystem->BusVoltageMV  < POWER_DOWN_SAVE_FLASH)
		{
			system_Communication_None();
			pSystem->PowerDownSaveFlash = 1;
		}else
		{
			if(pSystem->PowerOnResetFlag == 1)  /* �����Ժ���Ҫ��λ����λ��Ҫֱ�ӽ��븴λ״̬ */
			{
				pSystem->PowerOnResetFlag = 0;
				pSystem->Step++;
			}
			if(system_Get_LongKey() || (pSystem->ModbusCmd == e_mmc_clrfault) || (pSystem->CanCmd == e_can_clrfault))
			{
				if(system_Get_Fault(FAULT_POS | FAULT_M1_HAB))
				{
					pSystem->ZeroFound = 0;
					pSystem->Step++;
				}
				system_Clear_Motor_Fault(MOTOR1);
			}
		}
	}else if(pSystem->Step == 2)
	{
		/*��Ҫ��λģʽ�·��͸�λָ����븴λ״̬*/
		if(pSystem->SysCon.Sys_ResetMode == E_DETECTION_ANOMALY)
		{
			if(((system_Get_Fault(FAULT_M1_ALL) == 0)&&(system_Communication_Reset() == 1)) || (system_Get_Key_Event() == 1)) /*��λ��Ҫ��������ϣ����յ���λ���ĸ�λָ��򰴼�ִ�и�λ*/
			{
				system_FSM_StateJump(pSystem, E_SYS_STATE_ZERO);
				return;
			}
		}
	}
	/*��ѹ����10v�ж�Ϊ����Cʼ��flash*/
	if(pSystem->PowerDownSaveFlash  == 1)
	{	
		pSystem->PowerDownSaveFlash  = 0;
		(void)system_Save_Flash(pSystem, FS_MOTOR_SATE | FS_SYS_PARAM | FS_MOTOR_LIMIT);
	}
}
/*״̬����-ϵͳ����״̬(eg: ����λ��)*/
void system_Settings_Handler(void* System)
{
//  SYSTEM_t* pSystem = System;
//  MOTOR_POS_t ColumnHallPos = 0;
//  int32_t ReadMotorParam = 0;
//  uint8_t SaveFlag = 0;
//  static SETPOS_TYPE_t s_SetType = E_READY_TO_SET;
//  MOTOR_POS_t MemoryPosHall[10] = {0};
//
//  /*�ֿ�����ʾ*/
//  HSLedDisplaySetting(s_SetType);
//
//  /*��ʼ��*/
//  if(pSystem->Step == 0)
//  {
//      s_SetType = E_READY_TO_SET;
//      timer_Set_SW_Timer_AlarmTime(SW_TIMER_0, 2000);//SW_TIMER_0:�˳�����״̬��ʱ
//      timer_ResetStart_SW_Timer(SW_TIMER_0);
//      pSystem->Step++;
//  }
//  /*����ǰλ������Ϊ����λ��*/
//  else if(pSystem->Step == 1)
//  {
//      mc_app_Read_Param(MOTOR1, e_map_fdbkpos, &ReadMotorParam);
//      if(btn_Match_Event(KEY_1, E_BTN_EVT_PRESS, 0))
//          SaveFlag = E_SET_MEMORY_POS_1;
//      else if(btn_Match_Event(KEY_2, E_BTN_EVT_PRESS, 0))
//          SaveFlag = E_SET_MEMORY_POS_2;
//      else if(btn_Match_Event(KEY_3, E_BTN_EVT_PRESS, 0))
//          SaveFlag = E_SET_MEMORY_POS_3;
//      else if(btn_Match_Event(KEY_4, E_BTN_EVT_PRESS, 0))
//          SaveFlag = E_SET_MEMORY_POS_4;
//      if(SaveFlag)
//      {
//          system_Set_Column_SpecificPos(COLUMN_POS_MEMORY1 + (SaveFlag - E_SET_MEMORY_POS_1), system_Column_HallData_2_Pos(ReadMotorParam));
//          s_SetType = SaveFlag;
//          SYSTEM_SET_FLAG(pSystem->SaveIntoFlash, FS_MOTOR_MEMORYPOS);
//          pSystem->MemoryPosSetFlag |= (1 << (SaveFlag - E_SET_MEMORY_POS_1));
//      }
//      /*����λ�ô����flash*/
//      if(system_Save_Flash(pSystem, FS_MOTOR_MEMORYPOS))
//      {
//          timer_Set_SW_Timer_AlarmTime(SW_TIMER_0, 1000);//������ɺ�,��ʱ��ʾ1s
//          timer_ResetStart_SW_Timer(SW_TIMER_0);
//      }
//  }

//  /*��ʱ�˳���ǰ״̬*/
//  if(timer_Get_SW_Timer_Event(SW_TIMER_0, SW_TIMER_EVT_ALARM))
//  {
//      timer_Abort_SW_Timer(SW_TIMER_0);
//      system_FSM_StateJump(pSystem, E_SYS_STATE_IDLE);
//      return;
//  }
}
/*״̬����-���õ�ǰλ��Ϊ��λ*/
void system_SetZeroPos_Handler(void* System)
{
    SYSTEM_t* pSystem = System;
    int32_t ReadParam[MOTOR_NB] = {0};
    MOTOR_POS_t ColumnHallPos = 0;
    float ColumnmmPos = 0;
 
    float ColumnPosZero = 0;  
    
    //�����Ƹ˵�ǰλ��
    ColumnPosZero = system_Get_Column_SpecificPos(COLUMN_POS_ZERO);
    
    if(pSystem->Step == 0)
    {
        mc_app_Read_Param(MOTOR1, e_map_fdbkpos, &ReadParam[0]);
        ColumnHallPos = system_Column_Pos_2_HallData(ColumnPosZero);
        if(ColumnHallPos != ReadParam[0])
        {
            ColumnmmPos = system_Column_HallData_2_Pos(ReadParam[0]);
            system_Set_Column_SpecificPos(COLUMN_POS_ZERO, ColumnmmPos);
            system_FSM_StateJump(pSystem, E_SYS_STATE_IDLE);
        }else
        {
            system_FSM_StateJump(pSystem, E_SYS_STATE_IDLE);
        }
    }
    /*�������ʱ�쳣(����ʱ��������)-��ͣ*/
    if(system_Get_Fault(FAULT_MOTOR_ALL))
    {
		SYSTEM_SET_FLAG(pSystem->SaveIntoFlash, FS_MOTOR_SATE);
        system_FSM_StateJump(pSystem, E_SYS_STATE_FAULT);
		return;
    }
    /*�������ʱ�쳣(����ʱ��������)-��ͣ*/
    else if(system_Get_Fault(FAULT_OVV | FAULT_UDV))
    {
        SYSTEM_SET_FLAG(pSystem->SaveIntoFlash, FS_MOTOR_SATE);
        system_FSM_StateJump(pSystem, E_SYS_STATE_FAULT);
		return;
    }
    
}

/*ϵͳ״̬�����*/
void system_FSM_Task(void)
{
    SystemStateHandler[System.State](&System);
}
/********************************��������************************************
*������:

*������������: ϵͳ��Ϣ����-���մ����ص�����

*��������: ��

*��������ֵ: ��

*��ע:
*****************************************************************************/

//#if (PROTOCOL_TYPE == MODBUS_PROTOCOL)
/*modbus�ӿ�Ӧ�ô����ص�*/
int8_t system_msgHandler_MB_AppCB03(uint8_t* RcvData, uint8_t Len)
{
    uint8_t FunCode = RcvData[MB_PDU_OFFSET_FUNC];
    uint16_t RegAddr = 0;
    uint16_t RegNb = 0;

    uint16_t RegValueStart = 0;
    int32_t Value32 = 0;
    float Motor_PosMM = 0;
    float Motor_Spd_mmps = 0;
    uint8_t Motor_Dir = 0;
    uint8_t Motor_Fault = 0;
    if((hModbusLink.SlaveAddr != RcvData[MB_ADU_OFFSET_ADDR])&&(RcvData[MB_ADU_OFFSET_ADDR] != MB_ADDR_BD))
    {
        hMsgHandler[0].Slave_Flag = 1;
        return -1;
    }else
    {
        hMsgHandler[0].Slave_Flag = 0;
    }
    if(FunCode == MB_FUNC_READ_HOLDING_REG)
    {
        RegAddr = (RcvData[MB_PDU_OFFSET_FUNC16_REGADDR] << 8) | RcvData[MB_PDU_OFFSET_FUNC16_REGADDR + 1];
        RegNb = (RcvData[MB_PDU_OFFSET_FUNC03_REGNB] << 8) | RcvData[MB_PDU_OFFSET_FUNC03_REGNB + 1];
    }
    for(uint16_t i = 0; i < RegNb; i++)
    {
        switch(RegAddr)
        {
            //�������ַ
            case MBREG_MOTOR_SLAVE_ADDR:
                modbus_Write_1Reg(g_KVPTable, MBREG_MOTOR_SLAVE_ADDR, hModbusLink.SlaveAddr);
                break;

            //����ٶ�
            case MBREG_MOTOR_RPM:
                if(MC_RET_OK == mc_app_Read_Param(MOTOR1, e_map_fdbkspd, &Value32))
                {
                    Motor_Spd_mmps = system_MotorRPM_2_ColumnSpeed(Value32);
                    modbus_Write_1Reg(g_KVPTable, MBREG_MOTOR_RPM, Motor_Spd_mmps);
                }
                break;

            //�����ǰλ��
            case MBREG_MOTOR_FDBKPOS:
                if(MC_RET_OK == mc_app_Read_Param(MOTOR1, e_map_fdbkpos, &Value32))
                {
                    Motor_PosMM = system_Column_HallData_2_Pos(Value32);
                    modbus_Write_1Reg(g_KVPTable, MBREG_MOTOR_FDBKPOS, Motor_PosMM);
                }
                break;

            //�����ǰ״̬
            case MBREG_MOTOR_STATE:
                if(MC_RET_OK == mc_app_Read_Param(MOTOR1, e_map_dir, &Value32))
                {
                    if(Value32 == 0)
                        Motor_Dir = 0x03;
                    else if(Value32 == 1)
                        Motor_Dir = 0x01;
                    else
                        Motor_Dir = 0x02;
                    modbus_Write_1Reg(g_KVPTable, MBREG_MOTOR_STATE, Motor_Dir);
                }
                if(System.State == E_SYS_STATE_ZERO || System.State == E_SYS_STATE_FAULT)
                {
                    Motor_Fault = 0x04;
                    if(SYSTEM_GET_FLAG(System.FaultFlag, FAULT_M1_OVC))
                        Motor_Fault = 0x05;
                    if(SYSTEM_GET_FLAG(System.FaultFlag, FAULT_M1_HAB))
                        Motor_Fault = 0x06;
                    if(SYSTEM_GET_FLAG(System.FaultFlag, FAULT_UDV))
                        Motor_Fault = 0x07;
                    if(SYSTEM_GET_FLAG(System.FaultFlag, FAULT_OVV))
                        Motor_Fault = 0x08;
					if(SYSTEM_GET_FLAG(System.FaultFlag, FAULT_OVT))
                        Motor_Fault = 0x09;
                    modbus_Write_1Reg(g_KVPTable, MBREG_MOTOR_STATE, Motor_Fault);
                }
                break; 

            default:
                break;
        }
        RegAddr++;
        RegValueStart += 2;
    }
    return 1;
}  
/********************************��������************************************
*������:

*������������: ϵͳ��Ϣ����-���մ����ص�����

*��������: ��

*��������ֵ: ��

*��ע:
*****************************************************************************/
/*modbus�ӿ�Ӧ�ô����ص�*/
int8_t system_msgHandler_MB_AppCB(uint8_t* RcvData, uint8_t Len)
{
    uint8_t FunCode = RcvData[MB_PDU_OFFSET_FUNC];
    uint16_t RegAddr = 0;
    uint16_t RegNb = 0;
    int16_t RegValue = 0;
    uint16_t RegValueStart = 0;
    if((hModbusLink.SlaveAddr != RcvData[MB_ADU_OFFSET_ADDR])&&(RcvData[MB_ADU_OFFSET_ADDR] != MB_ADDR_BD))
    {
        hMsgHandler[0].Slave_Flag = 1;
        return -1;
    }else
    {
        hMsgHandler[0].Slave_Flag = 0;
    }
    if(FunCode == MB_FUNC_WRITE_1_REG)
    {
        //(0x06)д�����Ĵ���
        RegValueStart = MB_PDU_OFFSET_FUNC06_REGVALUE;
        RegAddr = (RcvData[MB_PDU_OFFSET_FUNC06_REGADDR] << 8) | RcvData[MB_PDU_OFFSET_FUNC06_REGADDR + 1];
        RegNb = 1;
    }else if(FunCode == MB_FUNC_WRITE_REGS)
    {
        //(0x10)д����Ĵ���
        RegValueStart = MB_PDU_OFFSET_FUNC16_REGVALUE;
        RegAddr = (RcvData[MB_PDU_OFFSET_FUNC16_REGADDR] << 8) | RcvData[MB_PDU_OFFSET_FUNC16_REGADDR + 1];
        RegNb = (RcvData[MB_PDU_OFFSET_FUNC16_REGNB] << 8) | RcvData[MB_PDU_OFFSET_FUNC16_REGNB + 1];
    }
    for(uint16_t i = 0; i < RegNb; i++)
    {
        RegValue = (RcvData[RegValueStart] << 8) | RcvData[RegValueStart + 1];
        switch(RegAddr)
        {
            //�������ַ
            case MBREG_MOTOR_SLAVE_ADDR:
                if((RegValue >= 0) && (RegValue < 128))
                {
                    hModbusLink.LastSlaveAddr = hModbusLink.SlaveAddr;  //��¼�ӻ��ϴεĵ�ַ��Ϣ��Ϊ�˽���ӻ����л���ַʱû��Ӧ���bug
                    hModbusLink.SlaveAddr = RegValue;
                    SYSTEM_SET_FLAG(System.SaveIntoFlash, FS_SYS_PARAM);
					sys_cfg_NodeSlaveAddr(hModbusLink.SlaveAddr);
                }
                break;

            //������� 1������  2���½�  3��ֹͣ
            case MBREG_MOTOR_CMD:
                if(RegValue == 1)
                    System.ModbusCmd = e_mmc_up;
                else if(RegValue == 2)
                    System.ModbusCmd = e_mmc_dn;
                else if(RegValue == 3)
                    System.ModbusCmd = e_mmc_stop;    
                break;

            //�����λ
            case MBREG_MOTOR_RESET:
                if(RegValue == 1)
                    System.ModbusCmd = e_mmc_reset;
                break;

            //������е�Ŀ��λ��             ����Ŀ��λ�õķ�Χ
            case MBREG_MOTOR_TARGETSPEED_POS:
                System.ModbusColumnPos = RegValue;
                System.ModbusCmd = e_mmc_goto;
                break; 

            //������õ�ǰλ��Ϊ��λ
            case MBREG_MOTOR_FDBKPOS_ZERO:
                if(RegValue == 0)
                    System.ModbusCmd = e_mmc_poszero;
                break;

            //�������ٶ�
            case MBREG_MOTOR_RPM:
                if(RegValue > (system_Speedmmps_Max()))
                    RegValue =(system_Speedmmps_Max());
                else if(RegValue < (system_Speedmmps_Min()))
                    RegValue = (system_Speedmmps_Min());
                System.MotorSpeed = system_ColumnSpeed_2_MotorRPM(RegValue);
                break;
			/* ���������� */
			case MBREG_MOTOR_CLR_FAULT:
				system_Clear_Motor_Fault(MOTOR1);	//ֻ�е�����Ͽ��Ա����,ϵͳ�����޷����
				System.ModbusCmd = e_mmc_clrfault;  /* ��������Ժ���������㣬��ֹ���������֮ǰ����������� */
				break;
            default:
                break;
        }
        RegAddr++;
        RegValueStart += 2;
    }
    return 1;
}

/********************************��������************************************
*������:

*������������: modbus-����

*��������: ��

*��������ֵ: ��

*��ע:
*****************************************************************************/
/*��ȡ���*/
MSG_HANDLER_t* msgHandler_Get_Handle(uint8_t Interface)
{
    return &hMsgHandler[Interface];
}
/*װ�������*/
int8_t msgHandler_hInit(MSG_HANDLER_t *MsgHandler, uint8_t Interface, uint8_t SlaveFlag)
{
    QUEUE_t *Queue = {0};

    MsgHandler->Interface = Interface;
    MsgHandler->Slave_Flag = SlaveFlag;
    if(QOP_SUCCESS == queue_Create(&Queue))
    {
        MsgHandler->QSend = Queue;
    }else
    {
        return MSG_RET_ERR_MA;
    }
    return MSG_RET_OK;
}

/********************************��������************************************
*������:

*������������: ��Ϣ������-���ʹ���

*��������: ��

*��������ֵ: ��

*��ע:
*****************************************************************************/
/*��������(δ���)�������*/
int8_t msgHandler_Pop_SendData(uint8_t Interface, uint8_t *Data, uint8_t DataLen)
{
    MSG_HANDLER_t *MsgHandler = msgHandler_Get_Handle(Interface);

    if(!Data || !DataLen)
        return MSG_RET_ERR_PARAM;
    if(MsgHandler->Slave_Flag == 0)
    {
        if(queue_Push(MsgHandler->QSend, Data, DataLen) != QOP_SUCCESS)
        {
            return QOP_ERR_DATA;
        }
        return MSG_RET_OK;
    }else
    {
        return QOP_ERR_DATA;
    } 
}
/*�жϽӿڵ�ǰ�Ƿ�ɷ�������*/
BOOL msgHandler_Send_Ready(MSG_HANDLER_t *MsgHandler)
{
    uint8_t Ready = 0;
    if(uart_Transmit_Ready(hUartRS485.Ins.Channel) == TRUE)
        Ready++;
    if(modbus_Send_Ready(&hModbusLink) == TRUE)
        Ready++;
    return (BOOL)(Ready == 2);
}
/*��������*/
int8_t msgHandler_Send(MSG_HANDLER_t *MsgHandler)
{
    /*�ӷ��Ͷ��л�ȡδ�������*/
    uint8_t *Data = queue_Get_HeadNode_Data(MsgHandler->QSend);
    uint8_t DataLen = queue_Get_HeadNode_DataSize(MsgHandler->QSend);
    if((modbus_rtu_Pack(&hModbusLink, Data, DataLen) == MB_RET_OK))
    {
        queue_Pop(MsgHandler->QSend, 0, 0);
        //����ָ��,ָ����������
        Data = hModbusLink.MsgSend.Msg;
        DataLen = hModbusLink.MsgSend.MsgLen;
    }else
    {
        queue_Pop(MsgHandler->QSend, 0, 0);
        return MSG_RET_ERR_PACK;    //���ݴ���,������
    }
    uart_adapter_Transmit_Polling(hUartRS485.Ins.Channel, Data, DataLen);
    return MSG_RET_OK;
}
/*�жϽӿ��Ƿ������*/
BOOL msgHandler_Send_Cplt(MSG_HANDLER_t *MsgHandler)
{
    BOOL Ret = FALSE;
    if(uart_Get_State(hUartRS485.Ins.Channel, UART_TC))
    {
        uart_Clr_State(hUartRS485.Ins.Channel, UART_TC);
        Ret = TRUE;
    }
    if(Ret == TRUE)
    {
        modbus_Set_XfFlag(&hModbusLink, MB_XF_SEND_CPLT);
    }
    return Ret;
}
/*���ʹ�����*/
void msgHandler_Send_Handler(MSG_HANDLER_t *MsgHandler)
{
    /*����State�����ԭ��: ����Ӳ��ͨ���첽�жϷ�����,�ж��оͿ���λmsgHandler_Send_Ready()Ϊ���ж������Ӳ�����;�����־!
      ��������뻥�Ᵽ֤�ж�"�������"�����ж��Ƿ�"���Է���",��QSend����2����������ʱ,"���Է���"�ж���������,��������2��ճ��*/
    if(MsgHandler->State == 0)
    {
        
        //���Է���
        if(msgHandler_Send_Ready(MsgHandler) && (!queue_Empty(MsgHandler->QSend)))
        {
            gpio_adapter_Set_Pin(UART_485_DIR_PORT,UART_485_DIR_PIN);
            if(msgHandler_Send(MsgHandler) == MSG_RET_OK)
                MsgHandler->State = 1;
        }
    }else
    {
        //�������
        if(msgHandler_Send_Cplt(MsgHandler))
        {
            MsgHandler->State = 0;
        }
    }
}

/********************************��������************************************
*������:

*������������: modbusͨѶ��ؽ��պ�������

*��������: ��

*��������ֵ: ��

*��ע:
*****************************************************************************/
/*�жϽӿ��Ƿ�������*/
BOOL msgHandler_Rcv_Cplt(MSG_HANDLER_t *MsgHandler, uint8_t *RcvData, uint16_t *RcvLen)
{
    BOOL Ret = FALSE;
    if(uart_Get_Receive(hUartRS485.Ins.Channel, RcvData, RcvLen) == UART_RET_OK)
    {
        Ret = TRUE;
    }
    return Ret;
}
/*��·�����*/
int8_t msgHandler_Parse_RcvMsg(MSG_HANDLER_t *MsgHandler, uint8_t *RcvMsg, uint16_t MsgLen)
{
    int8_t Ret = 0;
    Ret = modbus_rtu_Unpack(&hModbusLink, RcvMsg, (uint8_t)MsgLen);   //ǿתuint8_t��modbus֡��󳤶�Ϊ255.
    if(Ret == MB_RET_OK)
    {
        modbus_Set_XfFlag(&hModbusLink, MB_XF_RCV_CPLT);
    }else if(Ret == MB_RET_ERR_CRC)
    {
        return MSG_RET_ERR_CRC;
    }
    return MSG_RET_OK;
}
/*Ӧ�ò㴦��*/
int8_t msgHandler_Msg_Handler(MSG_HANDLER_t *MsgHandler, uint8_t *RcvMsg, uint16_t MsgLen)
{
    modbus_Func_Handler(&hModbusLink, MsgHandler->Interface);
    return MSG_RET_OK;
}
/*���մ�����*/
void msgHandler_Receive_Handler(MSG_HANDLER_t *MsgHandler)
{
    uint8_t ReData[100] = {0};
    uint16_t ReByteNb = 0;
    gpio_adapter_Reset_Pin(UART_485_DIR_PORT,UART_485_DIR_PIN);
    /*�ж����ݽ������*/
    if(msgHandler_Rcv_Cplt(MsgHandler, ReData, &ReByteNb))
    {
        if(msgHandler_Parse_RcvMsg(MsgHandler, ReData, ReByteNb) == MSG_RET_OK)
        {
            msgHandler_Msg_Handler(MsgHandler, 0, 0);
        }
    }
}


/*modbusͨѶ*/
void System_Modbus_Task(void)
{
    /*����ģ��*/
//    queue_Test();

    /*��Ϣ������ģ��*/
//    msgHandler_Test();
    msgHandler_Send_Handler(&hMsgHandler[0]);
    msgHandler_Receive_Handler(&hMsgHandler[0]);

    /*��·�������ģ��*/
    modbus_Host_Controller(&hModbusLink);
    modbus_Slave_Controller(&hModbusLink);
    modbus_Reg_CallBack_FuncHandlerCB(&hModbusLink, system_msgHandler_MB_AppCB, MB_FUNC_WRITE_1_REG);
    modbus_Reg_CallBack_FuncHandlerCB(&hModbusLink, system_msgHandler_MB_AppCB03, MB_FUNC_READ_HOLDING_REG);
    /*�����������ģ��*/
//    uart_Test();
}

/*����ϵͳ״̬,����CanOpen�����ֵ�*/
void system_Update_OD(void)
{
	//uint32_t Temp = 0;
	
	/*���״̬-OD_SI_M_STATE*/
	if(System.RunDir == DIR_UP)
		CO_SET_OD(OD_SI_M_STATE, e_cms_up);
	else if(System.RunDir == DIR_DOWN)
		CO_SET_OD(OD_SI_M_STATE, e_cms_dn);	
	else	//DIR_STOP
		CO_SET_OD(OD_SI_M_STATE, e_cms_stop);
	
	/*ϵͳ״̬-OD_SI_SYS_STATE*/
	if(System.FaultFlag)
	{
		if(System.FaultFlag & FAULT_M1_ALL)	//���ȱ�����쳣
		{
			if(System.FaultFlag & FAULT_M1_OVC)
				CO_SET_OD(OD_SI_SYS_STATE, e_css_fault_ovc);
			else if(System.FaultFlag & FAULT_M1_HAB)
				CO_SET_OD(OD_SI_SYS_STATE, e_css_fault_hall);
		}else	//ϵͳ�쳣
		{
			if(System.FaultFlag & FAULT_UDV)
				CO_SET_OD(OD_SI_SYS_STATE, e_css_fault_udv);
			else if(System.FaultFlag & FAULT_OVV)	
				CO_SET_OD(OD_SI_SYS_STATE, e_css_fault_ovv);
			else if(System.FaultFlag & FAULT_OVT)	
				CO_SET_OD(OD_SI_SYS_STATE, e_css_fault_hot);
			else if(System.FaultFlag & FAULT_POS)
				CO_SET_OD(OD_SI_SYS_STATE, e_css_lost_pos);
		}
	}else
	{
		if(System.SysCon.Sys_ResetMode == E_DETECTION_SIGNAL)
		{
			CO_SET_OD(OD_SI_SYS_STATE, e_css_normal);
		}else
		{
			if(System.ZeroFound)
			{
				CO_SET_OD(OD_SI_SYS_STATE, e_css_normal);
			}
		}
	}
	
	/*�Ƹ�λ��-OD_SI_FDBK_POS*/
	CO_SET_OD(OD_SI_FDBK_POS, (uint32_t)(System.ColumnPosMM[0] * 10));
}    

/*ϵͳ�ص�����-���¶����ֵ��Ӧ��ϵͳ����*/
void system_CB_CanOpenSdoTx(uint8_t Index)
{
	uint32_t ColumSpdLimit = 0;	//1=0.1mm/s

	switch(Index)
	{
		case OD_SI_TRG_SPD:
			SYSTEM_SET_FLAG(System.SaveIntoFlash, FS_SYS_PARAM);
			ColumSpdLimit = (system_Speedmmps_Min()) * 10;
			if(CO_GET_OD(OD_SI_TRG_SPD) < ColumSpdLimit)
				CO_SET_OD(OD_SI_TRG_SPD, ColumSpdLimit);
			ColumSpdLimit = (system_Speedmmps_Max()) * 10;
			if(CO_GET_OD(OD_SI_TRG_SPD) > ColumSpdLimit)
				CO_SET_OD(OD_SI_TRG_SPD, ColumSpdLimit);
			System.MotorSpeed = system_ColumnSpeed_2_MotorRPM((float)CO_GET_OD(OD_SI_TRG_SPD) / 10);
			break;

		case OD_SI_CMD_M_UP:
			System.CanCmd = e_can_up;
			break;

		case OD_SI_CMD_M_DN:
			System.CanCmd = e_can_dn;
			break;
		
		case OD_SI_CMD_M_STOP:
			System.CanCmd = e_can_stop;
			break;
		
		case OD_SI_CMD_M_GOTO:
			System.CanCmd = e_can_goto;
			break;
		
		case OD_SI_CLR_FAULT:
			system_Clear_Motor_Fault(MOTOR1);	//ֻ�е�����Ͽ��Ա����,ϵͳ�����޷����
			System.CanCmd = e_can_clrfault;		/* ��������Ժ��CAN����״̬Ϊnone */
			break;
		
		case OD_SI_HEARTBEAT:
			SYSTEM_SET_FLAG(System.SaveIntoFlash, FS_SYS_PARAM);
			break;
		
		case OD_SI_NODEID:
			SYSTEM_SET_FLAG(System.SaveIntoFlash, FS_SYS_PARAM);
			can_adapter_SetFilter_IDMask(CAN1, CanFilterSel2, sCONode.NodeID, CO_ID_MASK_FUNC);
			sys_cfg_NodeSlaveAddr(sCONode.NodeID);
			break;
		case OD_SI_CMD_M_RESET:
			System.CanCmd = e_can_reset;
			break;
		default:
			break;
	}
}
/*CanOpenͨѶ*/
void system_Com_Task(void)
{
	/*���ճ�ʱ����*/
	can_adapter_BusRecover(CAN1);
	/*���¶����ֵ�*/
	system_Update_OD();
	/*������Ϣ����*/
	co_RcvHandle(&sCONode);
	/*����������Ϣ*/
	co_HeartBeat(&sCONode);
}
//#endif

//CANJ1939
void system_CJ_Task(void )
{
	/*���ճ�ʱ����*/
	can_adapter_BusRecover(CAN1);
	/*���²����ֵ�*/
	update_para();
	/*������Ϣ����*/	
	cj_RcvHandle();
	/*��Ϣ����*/
	cj_initiate_send();
}

/*����can��������*/
void system_set_CanCmd(can_m_cmd_t CanCmd)
{
	if(CanCmd == e_can_up)
	{
		SYSTEM_CLR_FLAG(System.MotorLockedRotor, M_LOCKED_FLAG);
	}
	System.CanCmd = CanCmd;
}
/*��ȡ����ɸ�λ��־*/
uint8_t system_get_ZeroFound(void)
{
	return System.ZeroFound;	
}
/*��ȡ���ϱ�־*/
uint16_t system_get_FaultFlag(void)
{
	return System.FaultFlag;
}
/*��ȡ�Ƹ˵�ǰλ��*/
float system_get_columnPosMM(void)
{
	return System.ColumnPosMM[0];
}
/*��ȡ��λ��־*/
uint16_t system_get_LimitFlag(void)
{
	return System.LimitFlag;
}
//��ȡ��ǰ��ɲ�Ƿ�����
uint8_t system_get_LockedFlag(void)
{
	return System.MotorLockedRotor;
}

/*��ȡɲ��״̬*/
uint8_t system_get_BrakeState(void)
{
	return Brake_State;
}
/********************************��������************************************
*������:

*������������: ϵͳ����-��ں���

*��������: ��

*��������ֵ: ��

*��ע:
*****************************************************************************/
/*��λ��ͨѶ���*/
void system_Commsg_Init(void)
{
	syscon_cfg_handle_t hSystemConfig = sys_cfg_Hadnle();

	UART_INS_t UartConfig = {UART_485, UART3};
	uart_hInit(&hUartRS485, &UartConfig, 0);
	/*��Ϣ�ӿڳ�ʼ��-��·��*/
#if 0   //����
	modbus_hInit(&hModbusLink, MB_NODE_HOST, 0, g_KVPTable);
#else   //�ӻ�
	modbus_hInit(&hModbusLink, MB_NODE_SLAVE, hSystemConfig.Config_NodeSlaveAddr, g_KVPTable); // MB_NODE_SLAVE_ADDR  SysConfig->Config_NodeSlaveAddr
	modbus_Reg_CallBack_Ack(&hModbusLink, msgHandler_Pop_SendData);
#endif
	msgHandler_hInit(&hMsgHandler[0], 0, 0);

	/*can��������ʼ��*/
	can_hInit(&hCANController, M4_CAN, CAN1);
	can_adapter_Clr_RB(CAN1);	//����RB�����е�����,��ֹ��Ӧ�ò�������
	
	/*CanOpen�ڵ��ʼ��*/		
	co_hInit(&sCONode, hSystemConfig.Config_NodeSlaveAddr);
	co_SetNodeState(&sCONode, e_ns_pre_operational);
	co_sdo_tx_RegWriteODCallback(&sCONode, system_CB_CanOpenSdoTx);

}
/* �ײ����ó�ʼ�� */
void system_MotorInit(void)
{
	syscon_cfg_handle_t hSystemConfig = sys_cfg_Hadnle();
	mc_app_ModifyHallDir(MOTOR1, hSystemConfig.Config_HallDirectionSel);
	mc_app_ModifyDrvDir(MOTOR1, hSystemConfig.Config_PhaseDirectionSel);
	mc_app_Write_Param(MOTOR1, e_map_ocp_thh, hSystemConfig.Config_OvcValue);
}
	

/*ϵͳ��ʼ�����*/
void system_Init(void)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
    flash_adapter_Init();
    timer_IT_Enable(16, TIMER_IT_UPDATE);
    adc_adapter_Calibration();
#endif
//  /*adc_adapterģ���ʼ��(ϵͳ)*/
//  ADCH_t  MSPD_ADCH = {ADCH_MSPD_ADC, ADCH_MSPD_ADCH, ADCH_MSPD_PORT, ADCH_MSPD_PIN};
//  adc_adapter_hInit(&hADCChannalMSR, &MSPD_ADCH);
//  adc_adapter_Set_Channal_SmpTime(ADCH_MSPD_ADCH, 10);
//  adc_adapter_Set_Channal_SmpIntervalTime(ADCH_MSPD_ADCH, 5);
//  adc_adapter_Channel_Enable(ADCH_MSPD_ADCH);

    // ADCH_t  VM_ADCH = {ADCH_VM_ADC, ADCH_VM_ADCH, ADCH_VM_PORT, ADCH_VM_PIN};
    // adc_adapter_hInit(&hADCChannalMBV, &VM_ADCH);
    // adc_adapter_Set_Channal_SmpTime(ADCH_VM_ADCH, 500);
    // adc_adapter_Set_Channal_SmpIntervalTime(ADCH_VM_ADCH, 8);
    // adc_adapter_Channel_Enable(ADCH_VM_ADCH);

	ADCH_t  HOT_ADCH = {ADCH_TEMP_ADC, ADCH_TEMP_ADCH, ADCH_TEMP_PORT, ADCH_TEMP_PIN};
	adc_adapter_hInit(&hADCChannalTemp, &HOT_ADCH);
	adc_adapter_Set_Channal_SmpTime(ADCH_TEMP_ADCH, 100);
	adc_adapter_Set_Channal_SmpIntervalTime(ADCH_TEMP_ADCH, 2);
	adc_adapter_Channel_Enable(ADCH_TEMP_ADCH);

    // ADCH_t  VM_ADIVM = {ADCH_IVM_ADC, ADCH_IVM_ADCH, ADCH_IVM_PORT, ADCH_IVM_PIN};
    // adc_adapter_hInit(&hADCChannalIVM, &VM_ADIVM);
    // adc_adapter_Set_Channal_SmpTime(ADCH_IVM_ADCH, 80);
    // adc_adapter_Set_Channal_SmpIntervalTime(ADCH_IVM_ADCH,1);
    // adc_adapter_Channel_Enable(ADCH_IVM_ADCH);

    (void)btn_Init();

    mc_app_Init();

	system_Commsg_Init();
	
	sys_cfg_UartConfigInit();
	
	system_MotorInit();
}
/*��λ��ͨѶ���*/
void system_Commsg_Task(void)
{
	if(System.SysCon.Sys_CommunicationType == E_MODBUS)
	{
		System_Modbus_Task();
		system_Update_MBReg();
	}else if(System.SysCon.Sys_CommunicationType == E_CAN)
	{
		system_Com_Task();
	}
	/* ��λ��ͨѶ�������� */
	sys_cfg_MsgUartConfigHandler(&hUartData);
}

/*���Ź�ι������*/
void system_SwdtTask(void)
{
	if(g_SwdtCounter >= 200)
	{
		g_SwdtCounter = 0;
		//gpio_adapter_Toggle_Pin(GPIO_LED_PORT, GPIO_LED_PIN); //hz test
		SWDT_RefreshCounter();		/*���Ź�ι������*/
	}
}
/*�������*/
void system_Test(void)
{
//  /*����ģ�����*/
//  debug_Test();
//
//  /*flashģ�����*/
//  flash_adapter_Test();

//  /*��ťģ�����*/
//  btn_Test();
	
	/*CAN������ģ�����*/
//	can_Test();
	
//	/*mainLoopִ��Ч�ʲ���*/
//    gpio_adapter_Toggle_Pin(GPIO_LED2_PORT, GPIO_LED2_PIN);
	
	/*���ڲ��Ժ���*/
//	sys_cfg_DeliveryData();
//	crc_Test();
}
/*ϵͳ��̨���*/
void system_Loop_Task(void)
{
	/*���Ź�ι������*/
	system_SwdtTask();
	/**/
//  btn_Loop_Task();

    /*ϵͳ������Ϣ*/
//    system_Signal_LoopTask();
	
    /*ϵͳ״̬������*/
    system_FSM_Task();
	
	/*�Ƹ�λ�ø���*/
    system_Column_Update_Pos_Signal();
    
    /*�����������*/
    system_Motor_Task();

    /*ϵͳ�쳣�������*/
    system_Check_Fault_Task();
    
	/*ͨѶ����*/
	system_Commsg_Task();

    /*��������(test only)*/
//    system_Test();


	
}
/*ϵͳ��ʱ���(250us����1��)*/
void system_Timer_Task(void)
{
    static uint16_t _1msCnt = 0;

//    gpio_adapter_Toggle_Pin(GPIO_LED_PORT, GPIO_LED_PIN);
//    gpio_adapter_Set_Pin(TEST2_GPIO_Port, TEST2_Pin);   //20us�ߵ�ƽ(128MHz)
    mc_app_Timer_250us();
    if(++_1msCnt >= 4)
    {
        _1msCnt = 0;
        g_Counter++;
		g_SwdtCounter++;
        System.HSSendTimer++;
        mc_app_Timer_1ms();
        //msgHandler_Timer_1ms();
        btn_Timer_Task_1ms();
        timer_SW_Timer_Run();
        system_Read_Column_Limit();
        //system_Read_MotorControl_Signal();
        if(System.State != E_SYS_STATE_INIT)
        {
//            system_Read_MC_Mode_Signal(1);
        }
		/*����SYSTEM_RUN_LED_TIMEms�ĵ�������ʾϵͳ��������*/
//		if(g_Counter >= SYSTEM_RUN_LED_TIME)
//		{
//			g_Counter = 0;

//		}
        modbus_Timer(&hModbusLink);
        co_HeartBeatTimer(&sCONode);
     
        System.CfgCount++;
		if(System.OVVFlag == 1)
		{
			System.OVVCount++;
		}
    }
}
/********************************��������************************************
*������:

*������������: �жϻص�����

*��������: ��

*��������ֵ: ��

*��ע:
*****************************************************************************/
#if (MCU_TYPE == MCU_TYPE_STM32)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim16)
    {
        system_Timer_Task();
    }
}
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
/*��ʱ���ж�*/
void TIM_Callback(void)
{
    /*����ж�(10Hz)*/
    if(Bt_GetIntFlag(TIM0, BtUevIrq) == TRUE)
    {
        Bt_ClearIntFlag(TIM0, BtUevIrq);
        //system_Quit_Lowpower(FALSE);
        //gpio_adapter_Toggle_Pin(GPIO_TEST1_PORT, GPIO_TEST1_PIN);
    }

    /*����ж�(4KHz)*/
    if(Bt_GetIntFlag(TIM1, BtUevIrq) == TRUE)
    {
        Bt_ClearIntFlag(TIM1, BtUevIrq);
        //i2c_adapter_Timer();
        system_Timer_Task();
    }
}
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
/*��ʱ���ж�ISR*/
void Timer01B_CallBack(void)
{
    if(Set == TIMER0_GetFlag(M4_TMR01, Tim0_ChannelB))
    {
        TIMER0_ClearFlag(M4_TMR01, Tim0_ChannelB);
        system_Timer_Task();
        // uart_Idle_Callback(UART3);
		// uart_Idle_Callback(UART1);
    }
    tickTimer_Update();
    /* ���� UDS ��ʱ�� */
    uds_ms_update();    
    adc_adapter_SampleInterval_Timer();
    isotp_ms_update();		
		//can_Timer_1ms(CAN1); 		
}
/*�����ж�ISR*/
void ExtInt_Callback(void)
{
    if(Set == EXINT_IrqFlgGet(M1_HALLA_EIRQ_CH))
    {
        EXINT_IrqFlgClr(M1_HALLA_EIRQ_CH);
        system_Set_MotorSpd_Signal();
    }
    if(Set == EXINT_IrqFlgGet(M1_HALLB_EIRQ_CH))
    {
        EXINT_IrqFlgClr(M1_HALLB_EIRQ_CH);
        system_Set_MotorSpd_Signal();
    }
    if(Set == EXINT_IrqFlgGet(M1_HALLC_EIRQ_CH))
    {
        EXINT_IrqFlgClr(M1_HALLC_EIRQ_CH);
        system_Set_MotorSpd_Signal();
    }
    mc_app_Trigger_Task_HallEdge();
}
/*UART�����ж�ISR*/
void Uart_Callback_Rx(void)
{
    uart_Rx_Callback(UART3);
}
void Uart_MCU_Callback_Rx(void)
{
    uart_Rx_Callback(UART1);
}

/*UART�����ж�ISR*/
void Uart_Callback_Err(void)
{
    uart_Err_Callback(UART3);
}
void Uart_MCU_Callback_Err(void)
{
    uart_Err_Callback(UART1);
}

/*UART�����ж�ISR*/
void Uart_Callback_Tx(void)
{
    uart_TXE_Callback(UART3);
}
void Uart_MCU_Callback_Tx(void)
{
    uart_TXE_Callback(UART1);
}

/*UART��������ж�ISR*/
void Uart_Callback_TC(void)
{
    uart_TC_Callback(UART3);
}
void Uart_MCU_Callback_TC(void)
{
    uart_TC_Callback(UART1);
}

/*UART�����ж�ISR*/
void Uart_Callback_Idle(void)
{
    TIMER0_Cmd(M4_TMR02, Tim0_ChannelA, Disable);
    uart_Idle_Callback(UART3);
}
void Uart_MCU_Callback_Idle(void)
{
    TIMER0_Cmd(M4_TMR01, Tim0_ChannelA, Disable);
    uart_Idle_Callback(UART1);
}



/* UDS�첽���ջ�����: CAN ISR������+�ñ�־����ѭ������ */
uint8_t g_uds_rx_buffer[ISOTP_BUFFER_SIZE];
uint16_t g_uds_rx_len = 0;
uint32_t g_uds_rx_can_id = 0;
volatile uint8_t g_uds_rx_pending = 0;
void CAN_RxIrqCallBack(void)
{
    CAN_RCV_FRAME_t rx_frame;
    uint32_t ext_id;
    static uint8_t isotp_buffer[ISOTP_BUFFER_SIZE];
    uint16_t isotp_len;
    int8_t isotp_ret;
    
    if(can_Err_Callback(CAN1) == CAN_RET_OK)
    {
        return;
    }
    
    if (can_Rx_Callback(CAN1) == CAN_RET_OK)
    {
        while (can_Get_Receive(CAN1, &rx_frame, 1) == CAN_RET_OK)
        {
            ext_id = rx_frame.ExtID & EXT_ID_MASK;
            
            isotp_ret = isotp_receive_frame(CAN1, ext_id, rx_frame.Data, 
                                            rx_frame.Cst.Control_f.DLC, 
                                            isotp_buffer, &isotp_len);
            
            if (isotp_ret == ISOTP_OK)
            {
                // OTA_I("ISOTP_OK from interrupt: out_len=%d", isotp_len);
                /* ������ȫ�ֻ��������ñ�־λ */
                memcpy(g_uds_rx_buffer, isotp_buffer, isotp_len);
                g_uds_rx_len = isotp_len;
                g_uds_rx_can_id = ext_id;
                g_uds_rx_pending = 1;
            }
        }
    }
}

/*ϵͳ��̨���*/
void hz_loop(void)
{
	/*���Ź�ι������*/
	// system_SwdtTask();
    // /*ϵͳ״̬������*/
    // system_FSM_Task();
	// /*�Ƹ�λ�ø���*/
    // system_Column_Update_Pos_Signal();
    // /*�����������*/
    // system_Motor_Task();
    // delay_ms(1000);

    /* ��Դ����->�Ƹ�λ��->���״̬*/
    



    /*ϵͳ�쳣�������*/
    // system_Check_Fault_Task();
}



#endif
