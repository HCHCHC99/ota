#include "sys_main_state.h"
#include "power.h"
#include "hall_position.h"
#include "motor_cmd.h"
#include "system.h"
#include "main.h"
#include "hall_position.h"
#include "hz_timer.h"
#include "hardware.h"
#include "adapter_pwm.h"
#include "main.h"
#include "adc_adapter.h"
#include "sys_def.h"
#include "usart_usb.h"
#include "motor_adc.h"
#include "dev_pwm.h"
#include "device_manager.h"
// 全局系统上下文定义
static int test =0;
SystemContext g_sys_ctx;
NonBlockingDelay_t Primary_Protect_Delay;
NonBlockingDelay_t Secondary_Protect_Delay;
#if hz_usart1_debug
NonBlockingDelay_t Usart1_Usb_Send_Delay;
        static uint8_t firstSend = 1;
#endif
int First_fault_sig = 0;
int First_fault_protect_times = 0;
int Second_fault_protect_running = 0;

int count_p_to_n = 0;
int count_n_to_p = 0;

// 辅助函数：处理断电状态
static void handle_power_down_state(void) {
    switch (g_sys_ctx.power_sample_state) {
        case SAMPLE_POSITIVE:
            g_sys_ctx.power_state = POWER_POSITIVE;
            g_sys_ctx.power_valid = 1;
            g_sys_ctx.main_state = SYS_STATE_HALL_SAMPLE;
            break;
        case SAMPLE_NEGATIVE:
            g_sys_ctx.power_state = POWER_NEGATIVE;
            g_sys_ctx.power_valid = 1;
            g_sys_ctx.main_state = SYS_STATE_HALL_SAMPLE;
            break;
        case SAMPLE_POWER_DOWN:
            g_sys_ctx.main_state = SYS_STATE_HALL_SAMPLE;
            break;
        case SAMPLE_ERROR:
            g_sys_ctx.power_state = POWER_ERROR;
            g_sys_ctx.main_state = SYS_STATE_HALL_SAMPLE;
            break;
        default:
            // 保持当前状态
            break;
    }
}

// 辅助函数：处理未确定状态
static void handle_undetermined_state(void) {
    switch (g_sys_ctx.power_sample_state) {
        case SAMPLE_POSITIVE:
            g_sys_ctx.power_state = POWER_POSITIVE;
            g_sys_ctx.power_valid = 1;
            g_sys_ctx.main_state = SYS_STATE_HALL_SAMPLE;
            break;
        case SAMPLE_NEGATIVE:
            g_sys_ctx.power_state = POWER_NEGATIVE;
            g_sys_ctx.power_valid = 1;
            g_sys_ctx.main_state = SYS_STATE_HALL_SAMPLE;
            break;
        case SAMPLE_UNDETERMINED:
            // 保持未确定状态
            break;
        case SAMPLE_ERROR:
            g_sys_ctx.power_state = POWER_ERROR;
            g_sys_ctx.main_state = SYS_STATE_HALL_SAMPLE;
            break;
        default:
            // 其他状态处理
            break;
    }
}

// 辅助函数：处理稳定电源状态（正/负）
static void handle_stable_power_state(void) {
    if (g_sys_ctx.power_sample_state == SAMPLE_ERROR) {
        g_sys_ctx.power_state = POWER_ERROR;
        g_sys_ctx.main_state = SYS_STATE_HALL_SAMPLE;
        return;
    }

    if (g_sys_ctx.power_sample_state == SAMPLE_POWER_DOWN) {
        g_sys_ctx.power_state = POWER_DOWN;
        g_sys_ctx.main_state = SYS_STATE_HALL_SAMPLE;
        return;
    }

    // 检查极性是否一致
    if ((g_sys_ctx.power_state == POWER_POSITIVE && g_sys_ctx.power_sample_state == SAMPLE_POSITIVE) ||
        (g_sys_ctx.power_state == POWER_NEGATIVE && g_sys_ctx.power_sample_state == SAMPLE_NEGATIVE)) {
        g_sys_ctx.power_valid = 1;
        g_sys_ctx.main_state = SYS_STATE_HALL_SAMPLE;
        return;
    }

    // 极性反转检测
    if (g_sys_ctx.power_state == POWER_POSITIVE && g_sys_ctx.power_sample_state == SAMPLE_NEGATIVE) {
        g_sys_ctx.power_state = POWER_POSITIVE_TO_NEGATIVE;
        count_p_to_n = 0;
        return;
    }

    if (g_sys_ctx.power_state == POWER_NEGATIVE && g_sys_ctx.power_sample_state == SAMPLE_POSITIVE) {
        g_sys_ctx.power_state = POWER_NEGATIVE_TO_POSITIVE;
        count_n_to_p = 0;
        return;
    }
}

// 辅助函数：处理过渡状态
static void handle_transition_state(void) {
    if (g_sys_ctx.power_state == POWER_POSITIVE_TO_NEGATIVE) {
        if (g_sys_ctx.power_sample_state == SAMPLE_NEGATIVE) {
            if (++count_p_to_n >= 2000) {
                g_sys_ctx.power_state = POWER_NEGATIVE;
                g_sys_ctx.power_valid = 1;
            }
            g_sys_ctx.main_state = SYS_STATE_HALL_SAMPLE;
        } else if (g_sys_ctx.power_sample_state == SAMPLE_POSITIVE) {
            // 反转回原状态
            count_p_to_n = 0;
            count_n_to_p = 0;
            g_sys_ctx.power_state = POWER_NEGATIVE_TO_POSITIVE;
        }
    } else if (g_sys_ctx.power_state == POWER_NEGATIVE_TO_POSITIVE) {
        if (g_sys_ctx.power_sample_state == SAMPLE_POSITIVE) {
            if (++count_n_to_p >= 2000) {
                g_sys_ctx.power_state = POWER_POSITIVE;
                g_sys_ctx.power_valid = 1;
            }
            g_sys_ctx.main_state = SYS_STATE_HALL_SAMPLE;
        } else if (g_sys_ctx.power_sample_state == SAMPLE_NEGATIVE) {
            // 反转回原状态
            count_p_to_n = 0;
            count_n_to_p = 0;
            g_sys_ctx.power_state = POWER_POSITIVE_TO_NEGATIVE;
        }
    }
}


void SysMainState_Init(void) {
    memset(&g_sys_ctx, 0, sizeof(SystemContext));
    g_sys_ctx.main_state = SYS_STATE_INIT;
    g_sys_ctx.Motor_Current = 0.0f;
    g_sys_ctx.Motor_Voltage = 0.0f;
    g_sys_ctx.Voltage_Limit_Upper = THRESHOLD_VOLTAGE_OVER;
    g_sys_ctx.Voltage_Limit_Lower = THRESHOLD_VOLTAGE_UNDER;
    g_sys_ctx.Current_Limit_Upper = THRESHOLD_CURRENT_OVER;
    g_sys_ctx.Current_Limit_Recovery = THRESHOLD_CURRENT_RECOVERY;

    nbDelay_Init(&Primary_Protect_Delay, PRIMARY_PROTECT_TIME*1000);
    nbDelay_Init(&Secondary_Protect_Delay, SECONDARY_PROTECT_TIME*1000);
#if hz_usart1_debug
    nbDelay_Init(&Usart1_Usb_Send_Delay, USART1_USB_SEND_TIME*1000);
#endif
}

// 主状态机处理（核心流程控制）
void SysMainState_Process(void) {
    switch (g_sys_ctx.main_state) {
        case SYS_STATE_INIT:
            MotorCmd_Init();
            Power_Init();
            // 初始化完成后进入电源采样
            g_sys_ctx.main_state = SYS_STATE_POWER_SAMPLE;
            break;
        case SYS_STATE_POWER_SAMPLE: {
            SystemError err = Power_IOSample_NonBlocking();
            if (err == ERROR_IO_SAMPLE_FAIL || err == ERROR_POWER_CONFLICT) {
                g_sys_ctx.main_state = SYS_STATE_FAULT;
                test = 1;
                Power_ResetSampleState();
                break;
            }

            if (!Power_HasValidData()) {
                break; // 等待有效数据,SAMPLE_UNDETERMINED为无效
            }

            bool sampling_power_valid;
            PowerSampleState sampling_power_state = Power_GetState(&sampling_power_valid);
            if (sampling_power_state == POWER_UNDETERMINED || !sampling_power_valid) {
                break; // 数据未就绪，继续采样
            }

            g_sys_ctx.power_sample_state = sampling_power_state;
            g_sys_ctx.power_sample_valid = sampling_power_valid;
            g_sys_ctx.power_valid = 0;

            // 处理确定的电源状态转换
            switch (g_sys_ctx.power_state) {
                case POWER_DOWN:
                    handle_power_down_state();
                    break;
                case POWER_UNDETERMINED:
                    handle_undetermined_state();
                    break;
                case POWER_POSITIVE:
                case POWER_NEGATIVE:
                    handle_stable_power_state();
                    break;
                case POWER_POSITIVE_TO_NEGATIVE:
                case POWER_NEGATIVE_TO_POSITIVE:
                    handle_transition_state();
                    break;
                default:
                    // 未知状态处理，可考虑复位或错误处理
                    break;
            }
            break;
        }
        case SYS_STATE_HALL_SAMPLE: {
            SystemError err = HallPosition_Sample();
            // 采样失败时进入故障
            if (err != ERROR_NONE) {
                g_sys_ctx.main_state = SYS_STATE_FAULT;
                test = 2;
                break;
            }

            g_sys_ctx.rod_pos = HallPosition_GetState(&g_sys_ctx.rod_pos_valid);

            // 采样有效时进入指令生成
            if (g_sys_ctx.rod_pos != ROD_POS_UNKNOWN && g_sys_ctx.rod_pos_valid) {

                // 位置错误时进入故障
                if (g_sys_ctx.rod_pos == ROD_POS_ERROR) {
                    g_sys_ctx.main_state = SYS_STATE_FAULT;
                    test = 3;
                    break;
                }
                g_sys_ctx.main_state = SYS_STATE_MOTOR_CMD;
            }
            break;
        }
        
        case SYS_STATE_MOTOR_CMD: {
            // 直接根据“电源状态改变的推杆指令”和霍尔位置控制电机  //新增过流检测位
            SystemError err = MotorCmd_Execute(g_sys_ctx.power_state, g_sys_ctx.rod_pos);
            if (err != ERROR_NONE) {
                g_sys_ctx.main_state = SYS_STATE_FAULT;
                test = 4;
                break;
            }
            g_sys_ctx.motor_cmd = MotorCmd_GetState(&g_sys_ctx.motor_cmd_executed);
            //电机控制完成后直接回到保护采样
            g_sys_ctx.main_state = SYS_STATE_MOTOR_PROTECT;
            // 无电流电压保护
            // g_sys_ctx.main_state = SYS_STATE_POWER_SAMPLE;
            break;
        }
        
        case SYS_STATE_MOTOR_PROTECT:
        {
            SystemError err = MotorProtect_Generate(g_sys_ctx.motor_cmd);
            MotorProtect_GetMeasure(&g_sys_ctx.Motor_Current, &g_sys_ctx.Motor_Voltage, &g_sys_ctx.Protect_valid,&g_sys_ctx.current_limit_valid);
            // 1. 优化限位状态设置（互斥判断）
            if (g_sys_ctx.current_limit_valid)
            {
                if (g_sys_ctx.motor_cmd == MOTOR_RUN_EXTEND)                        // 当前正转且限位
                {
                    g_sys_ctx.current_limit_state = CURRENT_LIMITING_POSITIVE;      // 进入正转限位
                }
                else if (g_sys_ctx.motor_cmd == MOTOR_RUN_RETRACT)                  // 当前反转且限位
                {
                    g_sys_ctx.current_limit_state = CURRENT_LIMITING_NEGATIVE;      // 进入反转限位
                }
            }
            // 2. 合并取消限位逻辑
            else if (!g_sys_ctx.current_limit_valid)
            {
                if ((g_sys_ctx.current_limit_state == CURRENT_LIMITING_POSITIVE && g_sys_ctx.motor_cmd == MOTOR_RUN_RETRACT) ||     //当前处在正转限位，且电机即将反转
                    (g_sys_ctx.current_limit_state == CURRENT_LIMITING_NEGATIVE && g_sys_ctx.motor_cmd == MOTOR_RUN_EXTEND))        // 当前处在反转限位，且电机即将正转
                {
                    g_sys_ctx.current_limit_state = CURRENT_LIMIT_NONE;// 取消限位
                }
            }
            // // 过流（不分方向）
            // if (g_sys_ctx.current_limit_valid)
            // {
            //     if (g_sys_ctx.motor_cmd == MOTOR_RUN_EXTEND)                        
            //     {
            //         g_sys_ctx.current_limit_state = CURRENT_LIMIT_ANY;      
            //     }
            //     else if (g_sys_ctx.motor_cmd == MOTOR_RUN_RETRACT)                  
            //     {
            //         g_sys_ctx.current_limit_state = CURRENT_LIMIT_ANY;      
            //     }
            // }else if(0==g_sys_ctx.current_limit_valid)
            // {
            //     g_sys_ctx.current_limit_state = CURRENT_LIMIT_NONE;
            // }


#if hz_usart1_debug
            uint8_t Volatge_string[] = "Volatge:";
            uint8_t Current_string[] = "Current:";

            if(firstSend==1)
            {
                nbDelay_Start(&Usart1_Usb_Send_Delay);
            }
            firstSend++;
            if(nbDelay_IsComplete(&Usart1_Usb_Send_Delay))
            {
                firstSend = 0;
                USART1_Send_Data_IT(Space,sizeof(Space)-1);
                USART1_Send_Data_IT(Volatge_string,sizeof(Volatge_string)-1);
                USART1_Send_Float_IT(g_sys_ctx.Motor_Voltage,3);
                USART1_Send_Data_IT(Space,sizeof(Space)-1);
                USART1_Send_Data_IT(Current_string,sizeof(Current_string)-1);
                USART1_Send_Float_IT(g_sys_ctx.Motor_Current,3);
                USART1_Send_Data_IT(Enter,sizeof(Enter)-1);
            }
#endif   
            if (g_sys_ctx.Protect_valid)
            {}
            else
            {
                // 测量无效时的处理，可根据实际需求调整
                // g_sys_ctx.Motor_Current = 0.0f;
                // g_sys_ctx.Motor_Voltage = 0.0f;
                if(err == ERROR_OVER_CURRENT)
                {
                    g_sys_ctx.Time_Current_Over++;
                }
            }
            if(err != ERROR_NONE)
            {
                g_sys_ctx.main_state = SYS_STATE_FAULT;
                test = 5;
                break;
            }

            g_sys_ctx.main_state = SYS_STATE_POWER_SAMPLE;
            break;
        }
        
        case SYS_STATE_FAULT:
            // 故障处理逻辑
            // MotorCmd_StopImmediately();
            First_fault_sig++;

            // 检查是否达到二级保护条件（10次一级保护）
            if(First_fault_protect_times >= 10)
            {
                // 二级保护：掉电并重新初始化
                // 这里可以添加实际的掉电操作代码（如关闭外设电源等）
                // 例如：Power_Down();  // 假设存在这样的掉电函数
                
                // 启动二级保护延时，确保掉电操作完成
                nbDelay_Start(&Secondary_Protect_Delay);

                // 等待二级保护延时完成
                if(nbDelay_IsComplete(&Secondary_Protect_Delay))
                {
                    // 重置所有故障计数
                    First_fault_protect_times = 0;
                    First_fault_sig = 0;
                    // 回到初始状态
                    g_sys_ctx.main_state = SYS_STATE_POWER_SAMPLE;
                    break;
                }
            }
            // 一级保护:要根据不同的错误码进行不同的处理，如过流->降低速度
            else if(First_fault_sig == 1)
            {
                nbDelay_Start(&Primary_Protect_Delay);
            }
            if(nbDelay_IsComplete(&Primary_Protect_Delay) && First_fault_protect_times < 10)
            {
                g_sys_ctx.main_state = SYS_STATE_POWER_SAMPLE;
                First_fault_sig = 0;
                First_fault_protect_times++;
            }
            break;
        
        default:
            break;
    }
}



