// device_manager.c
#include "device_manager.h"
#include "dev_pwm.h"  // 现在可以安全包含了
#include <string.h>

DeviceNode_t s_device_registry[MAX_DEVICES];

en_result_t DeviceManager_Init(void)
{
    memset(s_device_registry, 0, sizeof(s_device_registry));
    
    return Ok;
}

en_result_t DeviceManager_RegisterDevice(uint8_t id, const char* name, DeviceType_t type, 
                                       void* private_data, DeviceOps_t ops)
{
    if (!private_data || !name || id >= MAX_DEVICES) {
        return ErrorInvalidParameter;
    }
    
    if (s_device_registry[id].used) {
        return ErrorInvalidParameter;
    }
    
    s_device_registry[id].id = id;
    strncpy(s_device_registry[id].name, name, sizeof(s_device_registry[id].name) - 1);
    s_device_registry[id].name[sizeof(s_device_registry[id].name) - 1] = '\0';
    s_device_registry[id].type = type;
    s_device_registry[id].state = DEVICE_STATE_UNINIT;
    s_device_registry[id].private_data = private_data;
    s_device_registry[id].ops = ops;
    s_device_registry[id].used = 1;
    
    return Ok;
}

DeviceNode_t* DeviceManager_GetDevice(uint8_t id)
{
    if (id >= MAX_DEVICES || !s_device_registry[id].used) {
        return NULL;
    }
    return &s_device_registry[id];
}

en_result_t Device_Read(uint8_t dev_id, void* data, uint32_t size)
{
    DeviceNode_t* device = DeviceManager_GetDevice(dev_id);
    if (!device || !device->ops.read || !data) {
        return ErrorInvalidParameter;
    }
    
    return device->ops.read(device->private_data, data, size);
}

en_result_t Device_Write(uint8_t dev_id, const void* data, uint32_t size)
{
    DeviceNode_t* device = DeviceManager_GetDevice(dev_id);
    if (!device || !device->ops.write || !data) {
        return ErrorInvalidParameter;
    }
    
    return device->ops.write(device->private_data, data, size);
}

en_result_t Device_Control(uint8_t dev_id, DeviceCommandData_t* cmd)
{
    DeviceNode_t* device = DeviceManager_GetDevice(dev_id);
    if (!device || !device->ops.control || !cmd) {
        return ErrorInvalidParameter;
    }
    
    return device->ops.control(device->private_data, cmd);
}

en_result_t Device_Update(uint8_t dev_id)
{
    DeviceNode_t* device = DeviceManager_GetDevice(dev_id);
    if (!device || !device->ops.update) {
        return ErrorInvalidParameter;
    }
    
    return device->ops.update(device->private_data);
}

en_result_t Device_Init(uint8_t dev_id)
{
    DeviceNode_t* device = DeviceManager_GetDevice(dev_id);
    if (!device || !device->ops.init) {
        return ErrorInvalidParameter;
    }
    
    en_result_t result = device->ops.init(device->private_data);
    if (result == Ok) {
        device->state = DEVICE_STATE_READY;
    } else {
        device->state = DEVICE_STATE_ERROR;
    }

    return result;
}

DeviceState_t Device_GetState(uint8_t dev_id)
{
    DeviceNode_t* device = DeviceManager_GetDevice(dev_id);
    return device ? device->state : DEVICE_STATE_ERROR;
}

en_result_t DeviceManager_UnregisterDevice(uint8_t id)
{
    if (id >= MAX_DEVICES || !s_device_registry[id].used) {
        return ErrorInvalidParameter;
    }
    
    memset(&s_device_registry[id], 0, sizeof(DeviceNode_t));
    return Ok;
}

DeviceNode_t* DeviceManager_GetRegistry(void)
{
    return s_device_registry;
}

uint32_t DeviceManager_GetRegistrySize(void)
{
    return MAX_DEVICES;
}
