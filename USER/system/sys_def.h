#ifndef SYS_DEF_H
#define SYS_DEF_H

#include "hc32f46x.h"
#include <stdbool.h>
#include <string.h>



// ====================== 核心状态定义 ======================
// 电源极性状态机
typedef enum {
    POWER_UNDETERMINED,             // 未确定（初始状态/检测冲突）
    POWER_POSITIVE,                 // 正极性
    POWER_POSITIVE_TO_NEGATIVE,     // 正to负
    POWER_NEGATIVE,                 // 负极性
    POWER_NEGATIVE_TO_POSITIVE,     // 负to正
    POWER_DOWN,                     // 掉电
    POWER_ERROR,                    // 电源错误
    POWER_MAX
} PowerPolarity;

typedef enum {
    a,
    b
} Powertest;

typedef enum {
    SAMPLE_UNDETERMINED,  // 未确定（初始状态/检测冲突）
    SAMPLE_POSITIVE,      // 正极性
    SAMPLE_NEGATIVE,      // 负极性
    SAMPLE_POWER_DOWN,    // 掉电
    SAMPLE_ERROR,         // 电源错误
    SAMPLE_MAX
} PowerSampleState;


// 电机指令状态机（含缓启动/缓停止状态）
typedef enum {
    MOTOR_STOP,                  // 停止
    MOTOR_SOFT_START_EXTEND,     // 正向缓启动（伸出方向）
    MOTOR_RUN_EXTEND,            // 正向持续运行（伸出）
    MOTOR_SOFT_STOP_EXTEND,      // 正向缓停止（伸出方向）
    MOTOR_SOFT_START_RETRACT,    // 负向缓启动（缩回方向）
    MOTOR_RUN_RETRACT,           // 负向持续运行（缩回）
    MOTOR_SOFT_STOP_RETRACT,     // 负向缓停止（缩回方向）
    MOTOR_STATE_MAX
} MotorCmdState;

// 推杆位置状态
typedef enum {
    ROD_POS_UNKNOWN,     // 未知位置
    ROD_POS_UPPER,       // 上限位
    ROD_POS_LOWER,       // 下限位
    ROD_POS_OTHER,       // 其它位置
    ROD_POS_ERROR,       // 位置错误
    ROD_POS_MAX
} RodPosition;

// 系统主状态机
typedef enum {
    SYS_STATE_INIT,      // 初始化
    SYS_STATE_POWER_SAMPLE,  // 电源IO采样（窗口滤波）
    SYS_STATE_HALL_SAMPLE,   // 霍尔位置采样（窗口滤波）
    SYS_STATE_MOTOR_CMD, // 生成电机指令（含缓动控制）
    SYS_STATE_MOTOR_PROTECT,//ADC
    SYS_STATE_COMPLETE,  // 流程完成
    SYS_STATE_FAULT,     // 故障
    SYS_STATE_MAX
} SysMainState;

// 错误码（扩展）
typedef enum {
    ERROR_NONE = 0,
    ERROR_IO_SAMPLE_FAIL,   // IO采样失败
    ERROR_CMD_INVALID,      // 指令无效
    ERROR_HALL_SAMPLE_FAIL, // 霍尔采样失败
    ERROR_POWER_CONFLICT,    // 电源冲突错误
    ERROR_UNDER_VOLTAGE,    // 欠压错误
    ERROR_OVER_CURRENT,     // 过流错误
    ERROR_OVER_VOLTAGE,      // 过压错误
    ERROR_MAX
} SystemError;


// 电流限位状态机 - 新增
typedef enum {
    CURRENT_LIMIT_NONE,       // 未限位（正常状态）
    CURRENT_LIMITING_POSITIVE,// 正向检测到过流限位
    CURRENT_LIMIT_POSITIVE,   // 正向过流限位（正转禁止）
    CURRENT_LIMITING_NEGATIVE, // 反向检测到过流限位
    CURRENT_LIMIT_NEGATIVE,   // 反向过流限位（反转禁止）
    CURRENT_LIMIT_ANY,        // 过流（不分方向）
    CURRENT_LIMIT_BOTH,       // 双向过流限位（特殊故障）
    CURRENT_LIMIT_MAX
} CurrentLimitState;


// 系统上下文定义（全局状态）
typedef struct {
    // 电源状态机相关
    PowerPolarity power_state;
    bool power_valid;       // 电源检测结果有效性
    
    Powertest powertest;

    PowerSampleState power_sample_state; //采样
    bool power_sample_valid;

    // 霍尔位置相关
    RodPosition rod_pos;
    bool rod_pos_valid;     // 霍尔位置检测有效性
    

    // 电机指令状态机相关
    MotorCmdState motor_cmd;
    bool motor_cmd_executed; // 电机指令执行状态
    uint8_t motor_pwm;      // 当前PWM占空比（0-100）

    float Motor_Current;
    float Motor_Voltage;
    bool Protect_valid;

    float Voltage_Limit_Upper;
    float Voltage_Limit_Lower;
    float Current_Limit_Upper;
    float Current_Limit_Recovery;          // 过流恢复阈值（新增）

    uint16_t Time_Current_Over; //过流累计次数
    CurrentLimitState current_limit_state; // 当前电流限位状态
    bool current_limit_valid;              // 限位状态是否有效（避免无效采样导致误判）

    SystemError error;
    
    SysMainState main_state;
} SystemContext;
#endif

 // SYS_DEF_H
