#ifndef MOTOR_H
#define MOTOR_H
#include "adc_adapter.h"
#include "system.h"
#include "sys_def.h"

/******************************应用层业务宏定义*******************************/
#define THRESHOLD_CURRENT_OVER    12.0f   // 过流阈值，单位A
#define THRESHOLD_VOLTAGE_UNDER   9.0f   // 欠压阈值，单位V
#define THRESHOLD_VOLTAGE_OVER    13.0f  // 过压阈值，单位V
#define THRESHOLD_CURRENT_RECOVERY 4.0f  // 过流恢复阈值

#define hz_adc_debug        1  // 调试开关
#define hz_cur_watch        1
#define TIME_SAMPLE_CURRENT_OVER   5     // 过流判断连续采样次数
#define VM_R1               2           // 电压分压电阻1
#define VM_R2               10          // 电压分压电阻2
#define hz_IVM_OFFSET       -0.1        // 电流偏移补偿

// 采样参数（应用层专属）
#define SampleTime_vm               50
#define SampleIntervalTime_vm       2
#define SampleTime_cur              20
#define SampleIntervalTime_cur      1

// 电流采样比较结果（应用层业务枚举）
typedef enum {
    ADC_COMPARE_ALL_ABOVE = 1,         // 所有采样点超过阈值
    ADC_COMPARE_NOT_ALL_ABOVE = 2,     // 不是所有采样点超过阈值
    ADC_COMPARE_NOT_ENOUGH_SAMPLES = 0,// 未达到采样个数
    ADC_COMPARE_NOT_READY = -1,        // 未到采样间隔
    ADC_COMPARE_DISABLED = -2,         // 功能禁用
    ADC_COMPARE_INITIALIZING = 3       // 初始化状态
} ADC_Compare_Result;

/******************************应用层变量声明*******************************/
// 电机电压、电流采样句柄（原adc_adapter.c中的全局变量移到应用层）
extern ADC_CH_CTRL_t    hADCChannalMBV; // 母线电压采样句柄
extern ADC_CH_CTRL_t    hADCChannalIVM; // 电流采样句柄

// 采样缓冲区（应用层专属）
#define SAMPLE_BUFFER_SIZE_vm                   32
extern uint16_t sample_buffer_vm[SAMPLE_BUFFER_SIZE_vm];
#define SAMPLE_BUFFER_SIZE_cur                  32
extern uint16_t sample_buffer_cur[SAMPLE_BUFFER_SIZE_cur];

/******************************应用层接口声明*******************************/
// 电机ADC初始化（包含电流、电压通道）
void ADC_Init_hz(void);

// 电机保护测量结果获取
void MotorProtect_GetMeasure(float *current, float *voltage, bool *valid, bool *current_over_valid);

// 电机保护逻辑处理
SystemError MotorProtect_Generate(MotorCmdState motor_cmd);

// 电流采样初始化
void ADC_CUR_Init(void);

// 电流采样数组比较（过流判断）
ADC_Compare_Result ADC_CUR_ARRAY_COMPARE(float* CUR_Voltage_ptr);
ADC_Compare_Result ADC_CUR_ARRAY_COMPARE_OPAMP(float* CUR_Voltage_ptr);
// 电压采样初始化
void ADC_VM_Init(void);

// 电压采样处理
void ADC_VM(float* VM_Voltage_ptr);

#endif
