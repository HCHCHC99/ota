#ifndef DEV_FLASH_H_
#define DEV_FLASH_H_

#include "main.h"
#include "hc32f46x_flash.h"
#include "device_manager.h"

// Flash 设备专用命令定义
typedef enum {
    CMD_FLASH_NONE = CMD_BASE_FLASH,
    CMD_FLASH_ERASE_SECTOR,
    CMD_FLASH_WRITE_WORD_NOCHECK,
    CMD_FLASH_WRITE_WORD_CHECK,
    CMD_FLASH_READ_WORD,
    CMD_FLASH_TEST,
    CMD_FLASH_GET_STATISTICS,
    CMD_FLASH_GET_OPERATION_HISTORY,
    CMD_FLASH_RESET_STATISTICS,
    CMD_FLASH_GET_LIFETIME_INFO,
    CMD_FLASH_MAX
} FlashCommand_t;

// 命令参数结构
typedef struct {
    uint32_t address;
    uint32_t data;
    uint32_t result;
} FlashWriteParams_t;

typedef struct {
    uint32_t address;
    uint32_t data;
} FlashReadParams_t;

typedef struct {
    uint32_t address;
} FlashEraseParams_t;

// Flash 统计信息结构
typedef struct {
    uint32_t erase_count;
    uint32_t write_count;
    uint32_t read_count;
    uint32_t error_count;
    uint32_t total_operations;
    uint32_t consecutive_errors;    // 连续错误次数
} FlashStatistics_t;

// 操作记录结构
#define FLASH_OPERATION_HISTORY_SIZE 10
#define MAX_FLASH_SECTORS 32        // 最大支持的扇区数量

typedef struct {
    FlashCommand_t operation_type;  // 使用枚举类型
    uint32_t address;
    uint32_t data;
    HC32FLASH_STATUS status;
    uint32_t sequence_number;       // 操作序列号代替时间戳
} FlashOperationRecord_t;

// 寿命信息结构
typedef struct {
    uint32_t max_erase_cycles;
    uint32_t warning_threshold;
    uint32_t current_max_erase_count;
    uint32_t current_min_erase_count;
    uint8_t wear_leveling_supported;
    uint8_t health_status;          // 健康状态：0-100
} FlashLifetimeInfo_t;

// Flash 设备配置结构
typedef struct {
    uint32_t base_address;
    uint32_t sector_size;
    uint32_t total_size;
    char name[16];
} FlashDeviceConfig_t;

// Flash 设备数据结构
typedef struct {
    FlashCommand_t last_operation;  // 使用枚举类型
    uint32_t last_address;
    uint32_t last_data;
    HC32FLASH_STATUS last_status;
    FlashDeviceConfig_t config;
    FlashStatistics_t statistics;
} FlashDeviceData_t;

// Flash 测试结果结构
typedef struct {
    uint32_t test_step;
    uint32_t test_result;
    uint32_t read_back_data;
    HC32FLASH_STATUS operation_status;
} FlashTestResult_t;

// Flash 句柄定义
typedef void* FlashHandle_t;

// 外部声明默认 Flash 句柄
extern FlashHandle_t flash_default;
extern FlashHandle_t flash_config;
extern FlashHandle_t flash_data;

// 通道管理相关宏定义
#define MAX_FLASH_DEVICES 4

// Flash 操作集
typedef struct {
    HC32FLASH_STATUS (*erase_sector)(uint32_t address);
    HC32FLASH_STATUS (*write_word_no_check)(uint32_t address, uint32_t data);
    HC32FLASH_STATUS (*write_word_check)(uint32_t address, uint32_t data);
    uint32_t (*read_word)(uint32_t address);
} flash_ops_t;

// Flash 通道控制结构
typedef struct {
    // 硬件配置
    uint32_t base_address;
    uint32_t sector_size;
    uint32_t total_size;
    
    // 操作集
    flash_ops_t ops;
    
    // 状态控制
    FlashCommand_t last_operation;  // 使用枚举类型
    uint32_t last_address;
    uint32_t last_data;
    HC32FLASH_STATUS last_status;
    uint8_t initialized;
    
    // 统计信息
    FlashStatistics_t statistics;
    
    // 操作历史 - 修改为指针方案
    FlashOperationRecord_t operation_history[FLASH_OPERATION_HISTORY_SIZE];
    FlashOperationRecord_t* latest_operation;  // 指向最新操作的指针
    uint8_t history_count;          // 有效记录数量
    uint32_t operation_total;    // 操作序列计数器
    uint32_t operation_new_index;   // 当前操作
    
    // 寿命管理
    struct {
        uint32_t max_erase_cycles;
        uint32_t warning_threshold;
        uint8_t wear_leveling_enabled;
        uint32_t sector_erase_counts[MAX_FLASH_SECTORS];  // 固定数组代替动态分配
        uint16_t sector_count;
    } lifetime;
} FlashChannel_t;

// 句柄创建/销毁函数
FlashHandle_t Flash_CreateDevice(uint32_t base_address, uint32_t sector_size, 
                                 uint32_t total_size);
void Flash_DestroyDevice(FlashHandle_t handle);

// 设备管理层接口函数
en_result_t Flash_DeviceRead(void* handle, void* data, uint32_t size);
en_result_t Flash_DeviceWrite(void* handle, const void* data, uint32_t size);
en_result_t Flash_DeviceInit(void* handle);
en_result_t Flash_DeviceControl(void* handle, DeviceCommandData_t* cmd);
en_result_t Flash_DeviceUpdate(void* handle);

// 应用层控制函数
en_result_t Flash_EraseSector(FlashHandle_t handle, uint32_t address);
en_result_t Flash_WriteWordNoCheck(FlashHandle_t handle, uint32_t address, uint32_t data);
en_result_t Flash_WriteWordCheck(FlashHandle_t handle, uint32_t address, uint32_t data);
uint32_t Flash_ReadWord(FlashHandle_t handle, uint32_t address);
en_result_t Flash_RunTest(FlashHandle_t handle, uint32_t address);

// 新增应用层函数
en_result_t Flash_GetStatistics(FlashHandle_t handle, FlashStatistics_t* stats);
en_result_t Flash_GetOperationHistory(FlashHandle_t handle, FlashOperationRecord_t* history, uint8_t* count);
en_result_t Flash_ResetStatistics(FlashHandle_t handle);
en_result_t Flash_GetLifetimeInfo(FlashHandle_t handle, FlashLifetimeInfo_t* info);

// 新增调试辅助函数
uint8_t Flash_GetValidOperationCount(FlashHandle_t handle);
en_result_t Flash_GetLatestOperation(FlashHandle_t handle, FlashOperationRecord_t* record);
en_result_t Flash_GetOperationByIndex(FlashHandle_t handle, uint8_t index, FlashOperationRecord_t* record);

// 辅助函数
en_result_t Flash_GetDeviceStatus(FlashHandle_t handle, volatile FlashDeviceData_t* status);
en_result_t Flash_GetDeviceConfig(FlashHandle_t handle, FlashDeviceConfig_t* config);
en_result_t Flash_SetDeviceConfig(FlashHandle_t handle, const FlashDeviceConfig_t* config);

// 全局初始化/反初始化函数
en_result_t Flash_InitAllDevices(void);
en_result_t Flash_Device_Deinit(void);

// 示例函数
void Flash_Device_Registration(void);
void Application_Flash_TestSequence(void);
void Application_Flash_DataAreaTest(void);

// 内部辅助函数声明
static void Flash_RecordOperation(FlashChannel_t* channel, FlashCommand_t operation,
                                 uint32_t address, uint32_t data, HC32FLASH_STATUS status);
static void Flash_UpdateStatistics(FlashChannel_t* channel, FlashCommand_t operation, HC32FLASH_STATUS status);
static uint16_t Flash_GetSectorIndex(FlashChannel_t* channel, uint32_t address);

// Flash 设备注册宏
#define REGISTER_FLASH_DEVICE(dev_id, dev_name, flash_handle) \
    DeviceManager_RegisterDevice(dev_id, dev_name, DEVICE_TYPE_FLASH, flash_handle, \
    (DeviceOps_t){ \
        .read = Flash_DeviceRead, \
        .write = Flash_DeviceWrite, \
        .init = Flash_DeviceInit, \
        .control = Flash_DeviceControl, \
        .update = Flash_DeviceUpdate \
    })

#endif
