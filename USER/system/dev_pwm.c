#include "dev_pwm.h"
#include "hz_timer.h"
#include <math.h>
#include <stdlib.h>
#include "string.h"

// 通道管理
static PWM_Channel_t s_pwmChannels[MAX_PWM_CHANNELS] ;
static uint8_t       s_numChannels = 0;

PWM_Handle_t phu = NULL;
PWM_Handle_t plu = NULL;
PWM_Handle_t phv = NULL;
PWM_Handle_t plv = NULL;

static en_result_t PWM_Duty_3s(PWM_Channel_t* channel, float startDuty, float targetDuty, 
                              uint16_t totalSteps, uint16_t stepDelayMs) 
{
    float currentDuty = startDuty;
    const float step = (targetDuty > startDuty) ? 1.0f : -1.0f;
    
    for(uint16_t i = 0; i < totalSteps; i++) {
        currentDuty += step;
        
        /* 边界检查 */
        if((step > 0 && currentDuty > targetDuty) || 
           (step < 0 && currentDuty < targetDuty)) {
            currentDuty = targetDuty;
        }

        /* 更新PWM：通过函数指针调用驱动层函数 */
        channel->ops.set_duty(channel->Timer, channel->timerChannel, currentDuty);
        
        /* 动态延时保证总时间3秒 */
        tickTimer_DelayMs(stepDelayMs);
    }
    
    /* 最终确认 */
    return channel->ops.set_duty(channel->Timer, channel->timerChannel, targetDuty);
}

PWM_Handle_t MotorPWM_Create(en_port_t port, en_pin_t pin, 
                                 M4_TMRA_TypeDef* Timer, 
                                 en_timera_channel_t timerChannel,
                                 bool polarity)
{
    if(s_numChannels >= MAX_PWM_CHANNELS) {
        return NULL;
    }
    
    PWM_Channel_t* channel = &s_pwmChannels[s_numChannels];
    
    // 初始化硬件配置
    channel->port = port;
    channel->pin = pin;
    channel->Timer = Timer;
    channel->timerChannel = timerChannel;
    
    // 初始化操作集：赋值驱动层的操作集实例
    channel->ops = hz_pwm_ops;
    
    // 初始化状态
    channel->state = PWM_IDLE;
    channel->startDuty = 0.0f;
    channel->targetDuty = 0.0f;
    channel->currentDuty = 0.0f;
    channel->frequency = 0;
    channel->totalSteps = 0;
    channel->currentStep = 0;
    channel->lastUpdateTime = 0;
    channel->stepDelayMs = 0;
    channel->polarity = polarity;
    s_numChannels++;
    return (PWM_Handle_t)channel;
}

void PWM_Destroy(PWM_Handle_t handle)
{
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    if(channel) {
        PWM_Stop(channel);
        // 重置通道状态
        memset(channel, 0, sizeof(PWM_Channel_t));
    }
}

en_result_t PWM_setDuty_noblock(PWM_Handle_t handle, float targetDuty, float time) {
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    if(!channel) {
        return ErrorInvalidParameter;
    }
    
    /* 参数合法性检查 */
    if (targetDuty < 0.0f || targetDuty > 100.0f) {
        return ErrorInvalidParameter;
    }

    if (channel->state != PWM_IDLE) {
        channel->state = PWM_IDLE;
    }

    /* 获取当前占空比：通过函数指针调用驱动层函数 */
    float currentDuty = channel->ops.get_duty(channel->Timer, channel->timerChannel);

    /* 计算需要变化的步数和步进延时 */
    float dutyDiff = fabsf(targetDuty - currentDuty);
    uint16_t totalSteps = (uint16_t)(dutyDiff / 1.0f); // 按1%步进
    uint16_t stepDelayMs = time * 1000 / totalSteps;

    /* 初始化缓启动控制结构 */
    channel->startDuty = currentDuty;
    channel->targetDuty = targetDuty;
    channel->currentDuty = currentDuty;
    channel->totalSteps = totalSteps;
    channel->currentStep = 0;
    channel->stepDelayMs = stepDelayMs;
    channel->lastUpdateTime = tickTimer_GetCount();
    channel->state = MOTOR_PWM_RAMPING;

    /* 立即设置初始占空比：通过函数指针调用驱动层函数 */
    return channel->ops.set_duty(channel->Timer, channel->timerChannel, currentDuty);
}

en_result_t PWM_setDuty_block(PWM_Handle_t handle, float targetDuty) {
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    if(!channel) {
        return ErrorInvalidParameter;
    }
    
    /* 参数合法性检查 */
    if (targetDuty < 0.0f || targetDuty > 100.0f) {
        return ErrorInvalidParameter;
    }

    /* 获取当前占空比：通过函数指针调用驱动层函数 */
    float currentDuty = channel->ops.get_duty(channel->Timer, channel->timerChannel);

    /* 计算需要变化的步数 */
    float dutyDiff = fabsf(targetDuty - currentDuty);
    uint16_t totalSteps = (uint16_t)(dutyDiff / 1.0f); // 按1%步进
    uint16_t stepDelayMs = MOTORPWM_BLOCK_TIME * 1000 / totalSteps;         // 总时间3秒
    
    /* 调用非阻塞精确时间控制函数 */
    return PWM_Duty_3s(channel, currentDuty, targetDuty, totalSteps, stepDelayMs);
}

// // 新增的非阻塞更新函数
// void PWM_DutyUpdate(PWM_Handle_t handle) {
//     PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    
//     channel->frequency=channel->ops.get_frequency(channel->Timer);

//     if(!channel || channel->state != MOTOR_PWM_RAMPING) {
//         return;
//     }

//     uint32_t currentTime = tickTimer_GetCount();
//     uint32_t elapsedTime = currentTime - channel->lastUpdateTime;

//     if (elapsedTime >= channel->stepDelayMs) {
//         /* 执行一步占空比更新 */
//         channel->currentStep++;
        
//         if (channel->currentStep >= channel->totalSteps) {
//             /* 缓启动完成 */
//             channel->currentDuty = channel->targetDuty;
//             channel->state = MOTOR_PWM_COMPLETE;
//         } else {
//             /* 计算下一步占空比 */
//             const float step = (channel->targetDuty > channel->startDuty) ? 1.0f : -1.0f;
//             channel->currentDuty += step;
            
//             /* 边界检查 */
//             if ((step > 0 && channel->currentDuty > channel->targetDuty) || 
//                 (step < 0 && channel->currentDuty < channel->targetDuty)) {
//                 channel->currentDuty = channel->targetDuty;
//             }
//         }

//         /* 更新PWM占空比：通过函数指针调用驱动层函数 */
//         channel->ops.set_duty(channel->Timer, channel->timerChannel, channel->currentDuty);

//         /* 更新时间戳 */
//         channel->lastUpdateTime = currentTime;
//     }
// }
void PWM_DutyUpdate(PWM_Handle_t handle) {
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    
    if(!channel || channel->state != MOTOR_PWM_RAMPING) {
        return;
    }

    uint32_t currentTime = tickTimer_GetCount();
    uint32_t elapsedTime = currentTime - channel->lastUpdateTime;

    if (elapsedTime >= channel->stepDelayMs) {
        channel->currentStep++;
        
        if (channel->currentStep >= channel->totalSteps) {
            // 到达终点
            channel->currentDuty = channel->targetDuty;
            channel->state = MOTOR_PWM_COMPLETE;
        } else {
            // 线性插值计算新占空比
            float dutyRange = channel->targetDuty - channel->startDuty;
            float newDuty = channel->startDuty + dutyRange * channel->currentStep / channel->totalSteps;
            
            // 边界检查
            if (dutyRange > 0) {
                // 上升：确保不超过目标值
                channel->currentDuty = (newDuty < channel->targetDuty) ? newDuty : channel->targetDuty;
            } else {
                // 下降：确保不低于目标值
                channel->currentDuty = (newDuty > channel->targetDuty) ? newDuty : channel->targetDuty;
            }
        }

        // 更新硬件PWM
        channel->ops.set_duty(channel->Timer, channel->timerChannel, channel->currentDuty);
        channel->lastUpdateTime = currentTime;
    }
}
// 瞬间修改PWM占空比（无渐变过程）
en_result_t PWM_setDutyInstant(PWM_Handle_t handle, float duty) {
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    if(!channel) {
        return ErrorInvalidParameter;
    }
    
    /* 参数合法性检查 */
    if (duty < 0.0f || duty > 100.0f) {
        return ErrorInvalidParameter;
    }

    /* 如果正在进行渐变，先停止渐变过程 */
    if (channel->state == MOTOR_PWM_RAMPING) {
        PWM_StopRamp(handle);
    }

    /* 立即设置占空比：通过函数指针调用驱动层函数 */
    en_result_t result = channel->ops.set_duty(channel->Timer, channel->timerChannel, duty);
    
    /* 如果设置成功，更新当前占空比记录 */
    if (result == Ok) {
        channel->currentDuty = duty;

        channel->startDuty = 0;
        channel->targetDuty = 0;
    }
    
    return result;
}

// 以下辅助函数（获取状态、启停控制等）
PWM_State_t PWM_GetState(PWM_Handle_t handle) {
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    return channel ? channel->state : PWM_IDLE;
}

bool PWM_IsRampingComplete(PWM_Handle_t handle) {
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    return channel && channel->state == MOTOR_PWM_COMPLETE;
}

void PWM_StopRamp(PWM_Handle_t handle) {
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    if(channel) {
        channel->state = PWM_IDLE;
    }
}

void PWM_SlowStart(PWM_Handle_t handle,float slowstart_start_duty,float slowstart_target_duty, float time) {
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    if(channel) {
        // channel->ops.compare_disable(channel->Timer, channel->timerChannel);
        channel->ops.set_duty(channel->Timer, channel->timerChannel, slowstart_start_duty);
        channel->ops.compare_enable(channel->Timer, channel->timerChannel);
        // channel->ops.compare_disable(channel->Timer, channel->timerChannel);
        // channel->ops.init(channel->port, channel->pin, channel->Timer, channel->timerChannel, slowstart_start_duty, channel->polarity);
        // channel->ops.compare_enable(channel->Timer, channel->timerChannel);        
        PWM_setDuty_noblock(handle, slowstart_target_duty, time);
    }
}

void PWM_Stop(PWM_Handle_t handle) {
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    if(channel) {
        channel->ops.compare_disable(channel->Timer, channel->timerChannel);
        channel->ops.init(channel->port, channel->pin, channel->Timer, channel->timerChannel, 90.0f, channel->polarity); // 使用默认占空比
    }
}

void PWM_InitAllChannels(void)
{
    memset(s_pwmChannels, 0, sizeof(s_pwmChannels));
    s_numChannels = 0;
}

en_result_t PWM_setFreq(PWM_Handle_t handle, uint16_t freqHz) {
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    if(!channel) {
        return ErrorInvalidParameter;
    }
    
    // 如果正在进行渐变，先停止渐变过程
    if (channel->state == MOTOR_PWM_RAMPING) {
        PWM_StopRamp(handle);
    }
    
    // 调用函数指针的频率设置函数
    en_result_t result = channel->ops.set_frequency(channel->Timer, freqHz);
    
    // 如果频率设置成功，更新当前占空比记录（因为频率改变可能会影响占空比计算）
    if (result == Ok) {
        channel->currentDuty = channel->ops.get_duty(channel->Timer, channel->timerChannel);
    }
    
    return result;
}

// ============================================================================
// 设备管理层接口函数实现
// ============================================================================

en_result_t PWM_DeviceRead(void* handle, void* data, uint32_t size)
{
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    PwmDeviceData_t* pwm_data = (PwmDeviceData_t*)data;
    
    if (!channel || !pwm_data || size < sizeof(PwmDeviceData_t)) {
        return ErrorInvalidParameter;
    }
    
    // 读取PWM设备数据
    pwm_data->duty_cycle = channel->currentDuty;
    pwm_data->frequency = (uint16_t)channel->frequency;
    pwm_data->state = (uint8_t)channel->state;
    
    return Ok;
}

en_result_t PWM_DeviceWrite(void* handle, const void* data, uint32_t size)
{
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    const PwmDeviceData_t* pwm_data = (const PwmDeviceData_t*)data;
    
    if (!channel || !pwm_data || size < sizeof(PwmDeviceData_t)) {
        return ErrorInvalidParameter;
    }
    
    // 写入占空比
    if (pwm_data->duty_cycle >= 0.0f && pwm_data->duty_cycle <= 100.0f) {
        return PWM_setDuty_noblock((PWM_Handle_t)channel, pwm_data->duty_cycle, 3);
    }
    
    return ErrorInvalidParameter;
}

en_result_t PWM_DeviceInit(void* handle)
{
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    if (!channel) {
        return ErrorInvalidParameter;
    }
    
    // 初始化PWM硬件
    channel->ops.init(channel->port, channel->pin, channel->Timer, channel->timerChannel, 50.0f, channel->polarity); // 使用默认占空比
    channel->ops.compare_enable(channel->Timer, channel->timerChannel);
    
    return Ok;
}

en_result_t PWM_DeviceControl(void* handle, DeviceCommandData_t* cmd)
{
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    if (!channel || !cmd) {
        return ErrorInvalidParameter;
    }

    // 直接使用命令码进行判断，无需转换
    switch (cmd->cmd) {
        case CMD_PWM_SET_DUTY_INSTANT: {
            if(cmd->params && cmd->param_size == sizeof(PwmDutyParams_t)) {
                PwmDutyParams_t* params = (PwmDutyParams_t*)cmd->params;
                return PWM_setDutyInstant((PWM_Handle_t)channel, params->duty_cycle);
            }
            break;
        }

        case CMD_PWM_SET_DUTY_SLOW: {
            if (cmd->params && cmd->param_size == sizeof(PwmDutyParams_t)) {
                PwmDutyParams_t* params = (PwmDutyParams_t*)cmd->params;
                return PWM_setDuty_noblock((PWM_Handle_t)channel, params->duty_cycle, params->change_duty_time);
            }
            break;
        }
        
        case CMD_PWM_SET_FREQUENCY: {
            if (cmd->params && cmd->param_size == sizeof(PwmFreqParams_t)) {
                PwmFreqParams_t* params = (PwmFreqParams_t*)cmd->params;
                channel->ops.set_frequency(channel->Timer, params->frequency);
                channel->frequency = params->frequency;
                return Ok;
            }
            break;
        }
        
        case CMD_PWM_SLOWSTART: {
            if(cmd->params && cmd->param_size == sizeof(PwmDutyParams_t)) {
                PwmDutyParams_t* params = (PwmDutyParams_t*)cmd->params;
                PWM_SlowStart((PWM_Handle_t)channel, params->slowstart_start_duty, 
                             params->slowstart_target_duty, params->change_duty_time);
                return Ok;
            }
            break;
        }
            
        case CMD_PWM_STOP:
            PWM_Stop((PWM_Handle_t)channel);
            return Ok;
            
        default:
            // 处理通用命令（如果需要）
            switch (cmd->cmd) {
                case CMD_DEVICE_ENABLE:
                    // 启用设备
                    return Ok;
                case CMD_DEVICE_DISABLE:
                    // 禁用设备
                    return Ok;
                default:
                    break;
            }
            break;
    }
    
    return ErrorInvalidParameter;
}


en_result_t PWM_DeviceUpdate(void* handle)
{
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    if (!channel) {
        return ErrorInvalidParameter;
    }
    
    // 更新PWM状态（非阻塞操作）
    PWM_DutyUpdate((PWM_Handle_t)channel);
    return Ok;
}

void SetSlow_PWMDuty(PWM_Handle_t device_name, PWM_Duty_mode Opened)
{
    DeviceCommandData_t cmd;
    PwmDutyParams_t duty_params;
    memset(&cmd, 0, sizeof(DeviceCommandData_t));
    memset(&duty_params, 0, sizeof(PwmDutyParams_t));    
    
    if(device_name==plv) {
        cmd.device_id = 0;
    } else if(device_name==phv) {
        cmd.device_id = 1;
    } else if(device_name==plu) {
        cmd.device_id = 2;
    } else if(device_name==phu) {
        cmd.device_id = 3;
    }
    
    if(SlowUp==Opened) {
        duty_params.change_duty_time = PWM_SLOWSTART_TIME;
        duty_params.slowstart_start_duty = 2;
        duty_params.slowstart_target_duty = 98;
    } else if (SlowDowm==Opened) {
        duty_params.change_duty_time = PWM_SLOWSTART_TIME;
        duty_params.slowstart_start_duty = 98;
        duty_params.slowstart_target_duty = 2;
    }

    cmd.cmd = CMD_PWM_SLOWSTART;  // 直接使用PWM命令
    cmd.params = &duty_params;
    cmd.param_size = sizeof(PwmDutyParams_t);
    Device_Control(cmd.device_id, &cmd);    
}

void SetDutyInstant_PWMDuty(PWM_Handle_t device_name, float target_duty)
{
    DeviceCommandData_t cmd;
    PwmDutyParams_t duty_params;
    memset(&cmd, 0, sizeof(DeviceCommandData_t));
    memset(&duty_params, 0, sizeof(PwmDutyParams_t));   
    if(device_name==plv)
    {
        cmd.device_id = 0;
    }
    else if(device_name==phv)
    {
        cmd.device_id = 1;
    }
    else if(device_name==plu)
    {
        cmd.device_id = 2;
    }
    else if(device_name==phu)
    {
        cmd.device_id = 3;
    }

    duty_params.duty_cycle = target_duty;

    cmd.cmd = CMD_PWM_SET_DUTY_INSTANT;
    cmd.params = &duty_params;
    cmd.param_size = sizeof(PwmDutyParams_t);
    Device_Control(cmd.device_id, &cmd);       
}
void Positive_Run(void)
{
    // LOU 0
    // HOU 1
    SetDutyInstant_PWMDuty(phu,98);
    SetDutyInstant_PWMDuty(plu,98);

    SetDutyInstant_PWMDuty(phv,2);
    SetSlow_PWMDuty(plv,SlowDowm);
    // SetDutyInstant_PWMDuty(plv, 95);

}

void Negative_Run(void)
{
    //HOV LOU 1
    //HOU LOV 0
    SetDutyInstant_PWMDuty(phv,98);
    SetDutyInstant_PWMDuty(plv,98);  

    SetDutyInstant_PWMDuty(phu,2);
    SetSlow_PWMDuty(plu,SlowDowm);
    // SetDutyInstant_PWMDuty(plu, 95);
  

}

void Stop(void)
{
    //HOV LOU 0
    //HOU LOV 0
    SetDutyInstant_PWMDuty(phu,0);
    SetDutyInstant_PWMDuty(plu,100);   
    
    SetDutyInstant_PWMDuty(phv,0);
    SetDutyInstant_PWMDuty(plv,100);   
}
void All_DevicePwm_Update(void)
{
    // 更新所有设备状态
    for (uint8_t i = 0; i < MAX_DEVICES; i++) {
        if (s_device_registry[i].used) {
            Device_Update(i);
        }
    }
}

//example
// 应用层使用示例
void Application_Run(void)
{
    DeviceCommandData_t cmd;
    PwmDutyParams_t duty_params;
    
    // 设置PWM占空比 - 使用PWM专用命令
    duty_params.duty_cycle = 75.0f;
    cmd.cmd = CMD_PWM_SET_DUTY_SLOW;  // 直接使用，无需转换
    cmd.device_id = 0;
    cmd.params = &duty_params;
    cmd.param_size = sizeof(PwmDutyParams_t);
    
    Device_Control(0, &cmd);
}


void Application_Run1(void)
{
    // 使用统一接口控制PWM设备
    DeviceCommandData_t  cmd;
    PwmDutyParams_t duty_params;
    // PwmFreqParams_t freq_params;
    
    // 设置PWM1占空比为75%
    duty_params.duty_cycle = 15.0f;
    duty_params.change_duty_time = 5;
    cmd.cmd = CMD_PWM_SET_DUTY_SLOW;
    cmd.device_id = 1;
    cmd.params = &duty_params;
    cmd.param_size = sizeof(PwmDutyParams_t);
    Device_Control(1, &cmd);
}
void Application_Run2(void)
{
    // 使用统一接口控制PWM设备
    DeviceCommandData_t  cmd;
    PwmDutyParams_t duty_params;
    // PwmFreqParams_t freq_params;
    
    // 设置PWM1占空比为75%
    duty_params.duty_cycle = 50.0f;
    cmd.cmd = CMD_PWM_SET_DUTY_INSTANT;
    cmd.device_id = 0;
    cmd.params = &duty_params;
    cmd.param_size = sizeof(PwmDutyParams_t);
    Device_Control(0, &cmd);
}

void Application_Run3(void)
{
    // 使用统一接口控制PWM设备
    DeviceCommandData_t  cmd;
    PwmDutyParams_t duty_params;
    // PwmFreqParams_t freq_params;
    
    // 设置PWM1占空比为75%
    duty_params.change_duty_time = 7;
    duty_params.slowstart_start_duty = 10;
    duty_params.slowstart_target_duty= 95;
    cmd.cmd = CMD_PWM_SLOWSTART;
    cmd.device_id = 0;
    cmd.params = &duty_params;
    cmd.param_size = sizeof(PwmDutyParams_t);
    Device_Control(0, &cmd);
}

void Application_Run4(void)
{
    // 使用统一接口控制PWM设备
    DeviceCommandData_t  cmd;
    PwmDutyParams_t duty_params;
    // PwmFreqParams_t freq_params;
    
    memset(&cmd, 0, sizeof(PwmCommandData_t ));
    memset(&duty_params, 0, sizeof(PwmDutyParams_t));
    // 设置PWM1占空比为75%
    duty_params.change_duty_time = PWM_SLOWSTART_TIME;
    duty_params.slowstart_start_duty = PWM_SLOWSTART_START_DUTY;
    duty_params.slowstart_target_duty = PWM_SLOWSTART_TARGET_DUTY;
    cmd.cmd = CMD_PWM_SLOWSTART;
    cmd.params = &duty_params;
    cmd.param_size = sizeof(PwmDutyParams_t);
    Device_Control(0, &cmd);
}
void Application_Run5(void)
{
    DeviceCommandData_t  cmd;
    PwmFreqParams_t freq_params;

    freq_params.frequency = 15000;
    cmd.cmd = CMD_PWM_SET_FREQUENCY;

    cmd.params = &freq_params;
    cmd.param_size = sizeof(PwmFreqParams_t);
    Device_Control(0,&cmd);
}
void Application_Run6(void)
{
    DeviceCommandData_t  cmd;
    PwmFreqParams_t freq_params;

    freq_params.frequency = 12000;
    cmd.cmd = CMD_PWM_SET_FREQUENCY;

    cmd.params = &freq_params;
    cmd.param_size = sizeof(PwmFreqParams_t);
    Device_Control(0,&cmd);
}
void Application_Run7(void)
{
    DeviceCommandData_t  cmd;
    cmd.cmd = CMD_PWM_STOP;
    cmd.device_id = 0;
    Device_Control(0,&cmd);
}



