#include "motor_cmd.h"
#include "hardware.h"
#include "gpio_adapter.h"
#include "hc32f46x_gpio.h"
#include "sys_def.h"
#include "main.h"
#include "adapter_pwm.h"
#include "hz_nopwm.h"
#include "hz_timer.h"
#include "motor_adc.h"
#include "dev_pwm.h"
extern SystemContext g_sys_ctx;
// 模块内部静态变量
static MotorCmdState s_motor_cmd = MOTOR_STOP;
static bool s_motor_executed = false;
// 跟踪当前PWM状态，确保每次启动都缓启动
static MotorCmdState s_last_motor_cmd = MOTOR_STOP;  // 记录上一次指令状态
static float s_current_duty = 0.0f;                  // 记录当前占空比

static NonBlockingDelay_t Motor_slowstart_Delay;
static NonBlockingDelay_t Motor_Positive_Delay;
static NonBlockingDelay_t Motor_Negative_Delay;
void MotorCmd_Init(void) {
    s_motor_cmd = MOTOR_STOP;
    s_last_motor_cmd = MOTOR_STOP;
    s_motor_executed = false;
    s_current_duty = 0.0f;
    // MotorPWM_Init();
    nbDelay_Init(&Motor_slowstart_Delay, MOTOR_SLOWSTART_TIME*1000);
    nbDelay_Init(&Motor_Positive_Delay, MOTOR_POSITIVE_TIME);
    nbDelay_Init(&Motor_Negative_Delay, MOTOR_NEGATIVE_TIME);    
}

// 执行电机控制
SystemError MotorCmd_Execute(PowerPolarity power_input, RodPosition pos_input) {
    SystemError err = ERROR_NONE;
    s_motor_executed = false;


    // 位置错误时立即停止电机
    if (pos_input == ROD_POS_ERROR) {
        Stop();  
        s_motor_cmd = MOTOR_STOP;
        s_last_motor_cmd = MOTOR_STOP;
        s_current_duty = 0.0f;
        return ERROR_HALL_SAMPLE_FAIL;
    }
    
    // 根据指令和位置控制电机
    switch (power_input) {
        case POWER_POSITIVE:
            // 不是上限位时启动/维持伸展
            if (pos_input != ROD_POS_UPPER) {
                // 关键修改：从停止状态启动 或 首次进入该状态时，都触发缓启动
                if (s_last_motor_cmd != MOTOR_RUN_EXTEND) {//上状态不是MOTOR_RUN_EXTEND的时候会启动
                    nbDelay_Start(&Motor_Positive_Delay);
                }
                if(nbDelay_IsComplete(&Motor_Positive_Delay))
                {
                    Positive_Run();
                }
                All_DevicePwm_Update();    
                s_motor_cmd = MOTOR_RUN_EXTEND;
            } else {
                // 到达上限位，停止电机   
                Stop();                       
                s_motor_cmd = MOTOR_STOP;
                s_current_duty = 0.0f;
            }
            break;
            
        case POWER_NEGATIVE:
            // 不是下限位时启动/维持收缩
            if (pos_input != ROD_POS_LOWER) {
                // 关键修改：从停止状态启动 或 首次进入该状态时，都触发缓启动
                if (s_last_motor_cmd != MOTOR_RUN_RETRACT) {
                    nbDelay_Start(&Motor_Negative_Delay);
                }
                if(nbDelay_IsComplete(&Motor_Negative_Delay))
                {
                    Negative_Run();
                }
                All_DevicePwm_Update();   
                s_motor_cmd = MOTOR_RUN_RETRACT;
            } else {
                // 到达下限位，停止电机            
                Stop();         
                s_motor_cmd = MOTOR_STOP;
                s_current_duty = 0.0f;
            }
            break;
        case POWER_DOWN:
        case POWER_POSITIVE_TO_NEGATIVE:
        case POWER_NEGATIVE_TO_POSITIVE:

            Stop();  
            s_motor_cmd = MOTOR_STOP;
            s_current_duty = 0.0f;                        
        default:
            // 无指令或无效指令时停止电机
            Stop();                        
            s_motor_cmd = MOTOR_STOP;
            s_current_duty = 0.0f;
            err = (power_input >= POWER_MAX) ? ERROR_CMD_INVALID : ERROR_NONE;
            break;
    }
    
    // 更新状态记录
    s_last_motor_cmd = s_motor_cmd;
    // 更新当前占空比（模拟缓启动过程）
    if (s_motor_cmd != MOTOR_STOP) {
        s_current_duty = (s_current_duty < MOTOR_MAX_DUTY) ? 
                         s_current_duty + 0.5f : MOTOR_MAX_DUTY;
    }
    
    s_motor_executed = (err == ERROR_NONE);
    return err;
}

MotorCmdState MotorCmd_GetState(bool *executed) {
    if (executed) *executed = s_motor_executed;
    return s_motor_cmd;
}

// 立即停止电机
// void MotorCmd_StopImmediately(void) {
//     // MotorPWM_Stop(motor1);
//     // MotorPWM_Stop(motor2);                
//     // MotorNoPWM_Positive_Up_Stop();        
//     // MotorNoPWM_Negative_Up_Stop();  

//     MotorCmd_Stop();    
//     s_motor_cmd = MOTOR_STOP;
//     s_last_motor_cmd = MOTOR_STOP;
//     s_current_duty = 0.0f;
// }


// void MotorCmd_Positive_Init(void)
// {
//     // PHU PLU为IO；PHV PLV为PWM,98%->2%
//     // IO初始化
//     Motor_PHU_IO_Init();
//     // PWM初始化
//     Motor_PHV_PWM_Init();
// }

// void MotorCmd_Positive_SlowStart(void)
// {
//     MotorPWM_SlowStart(motor1);
//     MotorPWM_SlowStart(motor3);
//     MotorPWM_SlowStart(motor4);
//     // PWMCompare_Enable(PWM_OUT_POSITIVE_TIMERA, PWM_OUT_POSITIVE_CHANNEL);
// }

// void MotorCmd_Negative_Init(void)
// {
//     // PHV PLV为IO；PHU PLU为PWM,98%->2%
//     // IO初始化
//     Motor_PHV_IO_Init();
//     // PWM初始化
//     Motor_PHU_PWM_Init();
// }

// void MotorCmd_Negative_SlowStart(void)
// {
//     MotorPWM_SlowStart(motor2);
//     // PWMCompare_Enable(PWM_OUT_NEGATIVE_TIMERA, PWM_OUT_NEGATIVE_CHANNEL);
// }

// void MotorCmd_Stop(void)
// {
//     // PWMCompare_Disable(PWM_OUT_NEGATIVE_TIMERA, PWM_OUT_NEGATIVE_CHANNEL);
//     // PWMCompare_Disable(PWM_OUT_POSITIVE_TIMERA, PWM_OUT_POSITIVE_CHANNEL);
//     // PHU PLU;PHV PLV 都为IO，都MCU输出高电平（给驱动芯片输入高电平），则H桥上桥臂都为高，下桥臂都为低，不导通。
//     // Motor_PHU_IO_Init();
//     // Motor_PHV_IO_Init();

//     MotorPWM_setDutyInstant(motor1,98);
//     MotorPWM_setDutyInstant(motor2,98);
// }
