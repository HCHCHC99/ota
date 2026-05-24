#include "power.h"
#include "hardware.h"
#include "gpio_adapter.h"
#include "hz_timer.h"
#include "sys_main_state.h"
// 模块内部静态变量


// 模块内部静态变量
static PowerSampleState s_power_state = SAMPLE_UNDETERMINED;
static bool s_power_valid = false;
static uint8_t s_power_pos_sample_window[WINDOWS_NUM];
static uint8_t s_power_neg_sample_window[WINDOWS_NUM];

// 新增：非阻塞采样相关变量
static PowerSamplingState s_sample_state = POWER_SAMPLE_IDLE;
static uint8_t s_samples_collected = 0;        // 已收集的采样点数量
static NonBlockingDelay_t s_sample_delay;
static NonBlockingDelay_t Test_PositiveToNegative_Time;
static NonBlockingDelay_t Test_NegativeToPositive_Time;
static NonBlockingDelay_t Test_Stop_Time1;
static NonBlockingDelay_t Test_Stop_Time2;


static bool s_first_sample = true;             // 是否是第一次采样




void Power_Init(void) {
    s_power_state = SAMPLE_UNDETERMINED;
    s_power_valid = false;
    memset(s_power_pos_sample_window, 0, sizeof(s_power_pos_sample_window));
    memset(s_power_neg_sample_window, 0, sizeof(s_power_neg_sample_window));
    
    // 初始化非阻塞采样状态
    s_sample_state = POWER_SAMPLE_IDLE;
    s_samples_collected = 0;
    s_first_sample = true;
    nbDelay_Init(&s_sample_delay, 1);

    nbDelay_Init(&Test_PositiveToNegative_Time, 20000);
    nbDelay_Init(&Test_NegativeToPositive_Time, 20000);
    nbDelay_Init(&Test_Stop_Time1, 1000);
    nbDelay_Init(&Test_Stop_Time2, 1000); 
}

void Power_ResetSampleState(void) {
    s_sample_state = POWER_SAMPLE_IDLE;
    s_samples_collected = 0;
    s_first_sample = true;
    nbDelay_Stop(&s_sample_delay);
}


/**
 * @brief 添加新采样点并滑动窗口
 */
static void Power_AddSampleToWindow(uint8_t level1, uint8_t level2) {
    if (s_samples_collected < WINDOWS_NUM) {
        // 初始填充阶段：直接添加到末尾
        s_power_pos_sample_window[s_samples_collected] = level1;
        s_power_neg_sample_window[s_samples_collected] = level2;
        s_samples_collected++;
    } else {
        // 窗口已满：滑动窗口（移除最早的点，添加新点）
        for (uint8_t i = 0; i < WINDOWS_NUM - 1; i++) {
            s_power_pos_sample_window[i] = s_power_pos_sample_window[i + 1];
            s_power_neg_sample_window[i] = s_power_neg_sample_window[i + 1];
        }
        s_power_pos_sample_window[WINDOWS_NUM - 1] = level1;
        s_power_neg_sample_window[WINDOWS_NUM - 1] = level2;
    }
}



/**
 * @brief 辅助函数：检查缓冲区中所有样本是否都等于目标值
 * @param buf 采样缓冲区
 * @param target 目标电平值
 * @param count 要检查的样本数量
 * @return 所有样本都等于目标值返回true，否则返回false
 */
static bool checkAllSamples(const uint8_t* buf, uint8_t target, uint8_t count) {
    for (uint8_t i = 0; i < count; i++) {
        if (buf[i] != target) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 电源窗口滤波与状态检测
 * @param sample_count 采样点数
 * @return 检测有效返回true，否则返回false
 */
static bool Power_CheckWindow(uint8_t sample_count) {
    // 检查各采样窗口的电平一致性
    // const bool pos_window_all_high = checkAllSamples(s_power_pos_sample_window, POWER_HIGH_LEVEL, sample_count);
    // const bool pos_window_all_low  = checkAllSamples(s_power_pos_sample_window, POWER_LOW_LEVEL, sample_count);
    // const bool neg_window_all_high = checkAllSamples(s_power_neg_sample_window, POWER_HIGH_LEVEL, sample_count);
    // const bool neg_window_all_low  = checkAllSamples(s_power_neg_sample_window, POWER_LOW_LEVEL, sample_count);

    // // 正向电源：正极窗口全高且负极窗口全低
    // if (pos_window_all_high && neg_window_all_low) {
    //     s_power_state = SAMPLE_POSITIVE;
    //     return true;
    // }

    // // 负向电源：正极窗口全低且负极窗口全高
    // if (pos_window_all_low && neg_window_all_high) {
    //     s_power_state = SAMPLE_NEGATIVE;
    //     return true;
    // }

    // // 掉电状态：正负极窗口均全低
    // if (pos_window_all_low && neg_window_all_low) {
    //     s_power_state = SAMPLE_POWER_DOWN;
    //     return true;
    // }

    // // 电源冲突：正负极窗口均全高
    // if (pos_window_all_high && neg_window_all_high) {
    //     s_power_state = SAMPLE_ERROR;
    //     return true;
    // }

    // // 所有条件不匹配：状态未确定
    // s_power_state = SAMPLE_UNDETERMINED;
    // return false;





    
    static int state = -1; // 0:停, 1:正转, 2:停, 3:反转
    
    // 初始化开始时间
    if (state == -1) {
        nbDelay_Start(&Test_Stop_Time1);
        state = 0;
    }
    
    // 状态切换逻辑
    if (state == 0 && nbDelay_IsComplete(&Test_Stop_Time1)) {
        // 停3秒结束，开始正转8秒
        state = 1;
        nbDelay_Start(&Test_PositiveToNegative_Time);
        s_power_state = SAMPLE_POSITIVE;
    }
    else if (state == 1 &&(nbDelay_IsComplete(&Test_PositiveToNegative_Time))) {
        // 正转8秒结束，开始停3秒
        state = 2;
        nbDelay_Start(&Test_Stop_Time2);
        s_power_state = SAMPLE_POWER_DOWN;
    }
    else if (state == 2 && nbDelay_IsComplete(&Test_Stop_Time2)) {
        // 停3秒结束，开始反转8秒
        state = 3;
        nbDelay_Start(&Test_NegativeToPositive_Time);
        s_power_state = SAMPLE_NEGATIVE;
    }
    else if (state == 3 && (nbDelay_IsComplete(&Test_NegativeToPositive_Time))) {
        // 反转8秒结束，回到初始停3秒
        state = 0;
        nbDelay_Start(&Test_Stop_Time1);
        s_power_state = SAMPLE_POWER_DOWN;
    }




    // 根据当前状态返回对应的检测结果
    switch(s_power_state) {
        case SAMPLE_POSITIVE:
            return true;  // 模拟正转条件
        case SAMPLE_NEGATIVE:
            return true;  // 模拟反转条件
        case SAMPLE_POWER_DOWN:
            return true;  // 模拟停止条件
        default:
            return false;
    }
}



// /**
//  * @brief 电源IO采样函数
//  * @param 采样窗口大小=采样间隔*采样点数
//  * @return SystemError 采样结果状态码
//  * typedef enum {
//     ERROR_NONE = 0,
//     ERROR_IO_SAMPLE_FAIL,   // IO采样失败
//     ERROR_CMD_INVALID,      // 指令无效
//     ERROR_HALL_SAMPLE_FAIL, // 霍尔采样失败
//     ERROR_POWER_CONFLICT    // 电源冲突错误
// } SystemError;
//  * 
//  * @note 采样间隔1ms，在power.h用WINDOWS_NUM定义了窗口点数
//  */
// SystemError Power_IOSample(void) {
//     SystemError err = ERROR_NONE;
//     uint8_t pos_level, neg_level;
//     uint8_t sample_idx;
    

//     // 按指定点数采集样本（间隔1ms）
//     for (sample_idx = 0; sample_idx < WINDOWS_NUM; sample_idx++) {
//         // 读取电源正负极IO电平（通过硬件驱动）
//         pos_level = PORT_GetBit(POWER_POSITIVE_IO_PORT, POWER_POSITIVE_IO_PIN) ? 
//                         POWER_HIGH_LEVEL : POWER_LOW_LEVEL;
//         neg_level = PORT_GetBit(POWER_NEGATIVE_IO_PORT, POWER_NEGATIVE_IO_PIN) ? 
//                         POWER_HIGH_LEVEL : POWER_LOW_LEVEL;
        
//         // 存储采样值到缓冲区
//         s_power_pos_sample_window[sample_idx] = pos_level;
//         s_power_neg_sample_window[sample_idx] = neg_level;
        
//         // tickTimer_DelayMs(1); // 采样间隔
//     }

//     // 确认完成指定次数采样后进行有效性判断
//     if (sample_idx == WINDOWS_NUM) {
//         s_power_valid = Power_CheckWindow(WINDOWS_NUM); // 传入窗口大小用于判断
//         if (!s_power_valid) {
//             err = ERROR_IO_SAMPLE_FAIL;       // 采样无效
//         } else if (s_power_state == POWER_ERROR) {
//             err = ERROR_POWER_CONFLICT;       // 电源冲突
//         }
//     } else {
//         // 异常情况：未完成指定次数采样
//         s_power_valid = false;
//     }
    
//     return err;
// }


SystemError Power_IOSample_NonBlocking(void) {
    SystemError err = ERROR_NONE;
    uint8_t pos_level, neg_level;
    
    switch (s_sample_state) {
        case POWER_SAMPLE_IDLE:
            // 第一次采样或重置后：立即开始采样
            if (s_first_sample) {
                s_sample_state = POWER_SAMPLE_READING;
                s_first_sample = false;
            } else {
                // 非第一次采样：等待采样间隔
                nbDelay_Start(&s_sample_delay);
                s_sample_state = POWER_SAMPLE_WAITING;
            }
            break;
            
        case POWER_SAMPLE_WAITING:
            // 检查是否到达采样间隔
            if (nbDelay_IsComplete(&s_sample_delay)) {
                s_sample_state = POWER_SAMPLE_READING;
            }
            break;
            
        case POWER_SAMPLE_READING:
            // 读取当前IO电平
            pos_level = PORT_GetBit(POWER_POSITIVE_IO_PORT, POWER_POSITIVE_IO_PIN) ? 
                            POWER_HIGH_LEVEL : POWER_LOW_LEVEL;
            neg_level = PORT_GetBit(POWER_NEGATIVE_IO_PORT, POWER_NEGATIVE_IO_PIN) ? 
                            POWER_HIGH_LEVEL : POWER_LOW_LEVEL;
            
            // 添加到滑动窗口
            Power_AddSampleToWindow(pos_level, neg_level);
            
            // 如果有足够的数据点（至少WINDOWS_NUM个），立即处理
            if (s_samples_collected >= WINDOWS_NUM) {
                s_sample_state = POWER_SAMPLE_PROCESSING;
            } else {
                // 数据不足，等待下一个采样周期
                nbDelay_Start(&s_sample_delay);
                s_sample_state = POWER_SAMPLE_WAITING;
            }
            break;
            
        case POWER_SAMPLE_PROCESSING:
            // 处理采样结果
            s_power_valid = Power_CheckWindow(WINDOWS_NUM); 
            
            if (!s_power_valid) {
                // err = ERROR_IO_SAMPLE_FAIL;  //不能说无效就ERROR
            } else if (s_power_state == POWER_ERROR) {
                // err = ERROR_POWER_CONFLICT;
            }
            
            // 处理完成后，立即开始下一个采样周期
            nbDelay_Start(&s_sample_delay);
            s_sample_state = POWER_SAMPLE_WAITING;
            break;
            
        default:
            s_sample_state = POWER_SAMPLE_IDLE;
            break;
    }
    
    return err;
}


PowerSampleState Power_GetState(bool *valid) {
    // 只要有足够的数据点就返回当前状态（即使还在持续采样）
    if (s_samples_collected < WINDOWS_NUM) {
        if (valid) *valid = false;
        return SAMPLE_UNDETERMINED;
    }
    
    if (valid) *valid = s_power_valid;
    return s_power_state;
}

// 新增：检查是否有有效的采样数据
bool Power_HasValidData(void) {
    return (s_samples_collected >= WINDOWS_NUM) && s_power_valid;
}

