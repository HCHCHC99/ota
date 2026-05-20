#include "hall_position.h"
#include "hardware.h"

#include "gpio_adapter.h"
#include "hall_position.h"
#include "hardware.h"

// 模块内部静态变量
static RodPosition s_rod_pos = ROD_POS_UNKNOWN;
static bool s_rod_pos_valid = false;
// 上限位霍尔采样缓存及索引
static uint8_t s_hall_upper_window[10];  // upper的10点采样缓存
// 下限位霍尔采样缓存及索引
static uint8_t s_hall_lower_window[10];  // lower的10点采样缓存


// 初始化函数：分别初始化两个霍尔传感器的缓存和状态
void HallPosition_Init(void) {
    s_rod_pos = ROD_POS_UNKNOWN;
    s_rod_pos_valid = false;
    
    // 初始化上限位霍尔缓冲区
    memset(s_hall_upper_window, 0, sizeof(s_hall_upper_window));
    
    // 初始化下限位霍尔缓冲区
    memset(s_hall_lower_window, 0, sizeof(s_hall_lower_window));
    
}

// 重置函数：复用初始化逻辑
void HallPosition_Reset(void) {
    HallPosition_Init();
}

// 辅助函数：检查缓冲区中所有10个样本是否都等于目标值
static bool checkHallStable(const uint8_t* buf, uint8_t target) {
    for (uint8_t i = 0; i < 10; i++) {
        if (buf[i] != target) {
            return false;
        }
    }
    return true;
}

// 霍尔窗口滤波（分别处理upper和lower的10点采样）
static bool Hall_CheckWindow(void) {
    // 检查上限位的稳定状态
    bool upper_high = checkHallStable(s_hall_upper_window, HALL_HIGH_LEVEL);
    bool upper_low  = checkHallStable(s_hall_upper_window, HALL_LOW_LEVEL);
    
    // 检查下限位的稳定状态
    bool lower_high = checkHallStable(s_hall_lower_window, HALL_HIGH_LEVEL);
    bool lower_low  = checkHallStable(s_hall_lower_window, HALL_LOW_LEVEL);
    
    // 只有当两个霍尔都处于稳定状态时，才进行位置判断
    if ((upper_high || upper_low) && (lower_high || lower_low)) {
        if (upper_low && lower_high) {
            s_rod_pos = ROD_POS_UPPER;  // 上限位：upper稳定低，lower稳定高
            return true;
        } else if (lower_low && upper_high) {
            s_rod_pos = ROD_POS_LOWER;  // 下限位：lower稳定低，upper稳定高
            return true;
        } else if (upper_high && lower_high) {
            s_rod_pos = ROD_POS_OTHER;  // 其它位置：两者都稳定高
            return true;
        } else if (upper_low && lower_low) {
            s_rod_pos = ROD_POS_ERROR;  // 错误：两者都稳定低
            return true;
        }
    }

    // // 无霍尔限位
    // if ((upper_high || upper_low) && (lower_high || lower_low)) {
    //     if (upper_low && lower_high) {
    //         s_rod_pos = ROD_POS_OTHER;  // 上限位：upper稳定低，lower稳定高
    //         return true;
    //     } else if (lower_low && upper_high) {
    //         s_rod_pos = ROD_POS_OTHER;  // 下限位：lower稳定低，upper稳定高
    //         return true;
    //     } else if (upper_high && lower_high) {
    //         s_rod_pos = ROD_POS_OTHER;  // 其它位置：两者都稳定高
    //         return true;
    //     } else if (upper_low && lower_low) {
    //         s_rod_pos = ROD_POS_OTHER;  // 错误：两者都稳定低
    //         return true;
    //     }
    // }    
    
    // 任一霍尔状态不稳定则返回未知
    s_rod_pos = ROD_POS_UNKNOWN;
    return false;
}


// IO采样：分别对upper和lower进行独立采样，各自累计到10点后进行判断
SystemError HallPosition_Sample(void) {
    SystemError err = ERROR_NONE;
    uint8_t upper_level, lower_level;
    static bool hall_window_ready = false;  // 窗口是否就绪
    uint8_t s_hall_sample_idx; // 采样计数

    for(s_hall_sample_idx = 0;s_hall_sample_idx<=9;s_hall_sample_idx++)
    {
        upper_level = PORT_GetBit(HALL_UPPER_PORT, HALL_UPPER_PIN) ? HALL_HIGH_LEVEL : HALL_LOW_LEVEL;
        lower_level = PORT_GetBit(HALL_LOWER_PORT, HALL_LOWER_PIN) ? HALL_HIGH_LEVEL : HALL_LOW_LEVEL; 

        s_hall_upper_window[s_hall_sample_idx]=upper_level;
        s_hall_lower_window[s_hall_sample_idx]=lower_level;
    }

    // 首次填满10个点后标记窗口就绪
    if (!hall_window_ready && s_hall_sample_idx == 10) {
        hall_window_ready = true;
    }
    
    // 当两个霍尔窗口都就绪后，进行窗口判断
    if (hall_window_ready) {
        s_rod_pos_valid = Hall_CheckWindow();
        if (!s_rod_pos_valid) {
            err = ERROR_HALL_SAMPLE_FAIL;
        }
    } else {
        s_rod_pos_valid = false;  // 任一窗口未就绪，状态无效
    }
    
    return err;
}

// 获取当前霍尔位置状态
RodPosition HallPosition_GetState(bool *valid) {
    if (valid) *valid = s_rod_pos_valid;
    return s_rod_pos;
}
