#include "hz_nopwm.h"
#include "hc32f46x_gpio.h"

void Motor_noPWM_Init(void)
{
    stc_port_init_t     stcPortInit;
    
    // 初始化结构体清零
    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enPinMode = Pin_Mode_Out;       // 输出模式
    stcPortInit.enPinDrv = Pin_Drv_H;           // 高驱动能力
    stcPortInit.enPullUp = Disable;             // 禁用上拉
    stcPortInit.enExInt = Disable;              // 禁用外部中断
    stcPortInit.enPinOType = Pin_OType_Cmos;    // CMOS输出类型
 
    PORT_Init(NOPWM_POSITIVE_PORT, NOPWM_POSITIVE_PIN, &stcPortInit);

    PORT_OE(NOPWM_POSITIVE_PORT, NOPWM_POSITIVE_PIN, Enable);

    PORT_ResetBits(NOPWM_POSITIVE_PORT, NOPWM_POSITIVE_PIN);


    PORT_Init(NOPWM_NEGATIVE_PORT, NOPWM_NEGATIVE_PIN, &stcPortInit);

    PORT_OE(NOPWM_NEGATIVE_PORT, NOPWM_NEGATIVE_PIN, Enable);

    PORT_ResetBits(NOPWM_NEGATIVE_PORT, NOPWM_NEGATIVE_PIN);

}


void MotorNoPWM_Negative_Up_Run(void)
{
    PORT_SetBits(NOPWM_NEGATIVE_PORT,NOPWM_NEGATIVE_PIN);
}

void MotorNoPWM_Positive_Up_Run(void)
{
    PORT_SetBits(NOPWM_POSITIVE_PORT,NOPWM_POSITIVE_PIN);    
}

void MotorNoPWM_Negative_Up_Stop(void)
{
    PORT_ResetBits(NOPWM_NEGATIVE_PORT,NOPWM_NEGATIVE_PIN);
}

void MotorNoPWM_Positive_Up_Stop(void)
{
    PORT_ResetBits(NOPWM_POSITIVE_PORT,NOPWM_POSITIVE_PIN);    
}
