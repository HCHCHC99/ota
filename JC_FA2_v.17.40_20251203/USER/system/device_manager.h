// device_manager.h
#ifndef DEVICE_MANAGER_H_
#define DEVICE_MANAGER_H_

#include "main.h"

#define MAX_DEVICES 16

typedef enum {
    DEVICE_STATE_UNINIT = 0,
    DEVICE_STATE_READY,
    DEVICE_STATE_BUSY,
    DEVICE_STATE_ERROR
} DeviceState_t;

typedef enum {
    DEVICE_TYPE_NONE = 0,
    DEVICE_TYPE_PWM,
    DEVICE_TYPE_POWER,
    DEVICE_TYPE_CURRENT_SENSOR,
    DEVICE_TYPE_VOLTAGE_SENSOR,
    DEVICE_TYPE_FLASH,  // 新增Flash设备类型 
    DEVICE_TYPE_MAX
} DeviceType_t;

// 命令码分段定义
#define CMD_BASE_PWM          0x1000
#define CMD_BASE_POWER        0x2000
#define CMD_BASE_SENSOR       0x3000
#define CMD_BASE_FLASH        0x4000


// 通用命令（所有设备都支持）
typedef enum {
    CMD_NONE = 0,
    CMD_DEVICE_ENABLE,
    CMD_DEVICE_DISABLE,
    CMD_DEVICE_RESET,
    CMD_DEVICE_GET_STATUS,
    CMD_COMMON_MAX = 0x0FFF  // 通用命令范围
} CommonCommand_t;

// 统一的命令数据结构
typedef struct {
    uint32_t cmd;           // 命令码
    uint8_t device_id;
    void* params;           // 指向具体参数
    uint32_t param_size;    // 参数大小
} DeviceCommandData_t;

// 简化的设备操作接口
typedef struct {
    en_result_t (*init)(void* handle);
    en_result_t (*read)(void* handle, void* data, uint32_t size);
    en_result_t (*write)(void* handle, const void* data, uint32_t size);
    en_result_t (*control)(void* handle, DeviceCommandData_t* cmd);
    en_result_t (*update)(void* handle);
} DeviceOps_t;

// 设备节点
typedef struct {
    uint8_t id;
    char name[16];
    DeviceType_t type;
    DeviceState_t state;
    void* private_data;
    DeviceOps_t ops;
    uint8_t used;
} DeviceNode_t;

extern DeviceNode_t s_device_registry[MAX_DEVICES];

// 设备管理器接口
en_result_t DeviceManager_Init(void);
en_result_t DeviceManager_RegisterDevice(uint8_t id, const char* name, DeviceType_t type, 
                                       void* private_data, DeviceOps_t ops);
en_result_t DeviceManager_UnregisterDevice(uint8_t id);
DeviceNode_t* DeviceManager_GetDevice(uint8_t id);

// 统一的应用层接口
en_result_t Device_Init(uint8_t dev_id);
en_result_t Device_Read(uint8_t dev_id, void* data, uint32_t size);
en_result_t Device_Write(uint8_t dev_id, const void* data, uint32_t size);
en_result_t Device_Control(uint8_t dev_id, DeviceCommandData_t* cmd);
en_result_t Device_Update(uint8_t dev_id);

DeviceState_t Device_GetState(uint8_t dev_id);

// 调试接口
DeviceNode_t* DeviceManager_GetRegistry(void);
uint32_t DeviceManager_GetRegistrySize(void);

#endif
