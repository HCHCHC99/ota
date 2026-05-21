# UDS固件下载 — 修改总结

## 修改概述

为HC32F460JETA MCU实现UDS (ISO 14229) 固件下载功能，通过CAN总线与TBOX通信。主要涉及ISO-TP传输层修复、UDS诊断服务响应格式优化、CAN接收路径连接、以及调试日志增强。

---

## 文件修改清单

### 1. USER/UDS/isotp_transport.h

| 修改项 | 修改前 | 修改后 | 说明 |
|--------|--------|--------|------|
| ISOTP_AUTO_FC | #define ISOTP_AUTO_FC | // #define ISOTP_AUTO_FC | 禁用自动流控，走正常ISO-TP路径 |
| ISOTP_DEFAULT_RESPONSE_ID | — | 0x18DAF103 | 新增默认响应CAN ID |
| ISOTP_ENABLE_FILTER_RECORD | — | 1 | 启用CAN ID过滤记录 |
| 过滤CAN ID列表 | — | {0x18DA03F1, 0x18DAF103, 0x18FF8118} | 只监控这3个CAN ID |
| OTA_DEBUG | — | #define OTA_DEBUG | 启用OTA调试日志 |

### 2. USER/UDS/isotp_transport.c

CAN ID不匹配时重置连接（行679-681）：
- 修改前：仅返回 ISOTP_ERROR，连接状态保持ACTIVE，导致后续帧被阻塞
- 修改后：调用 isotp_reset_connection() 并恢复 rx_dst_id，让状态机回到IDLE

OTA调试日志增加CAN ID过滤（行590-594）：
- 只在CAN ID为 0x18DA03F1、0x18DAF103、0x18FF8118 时打印状态信息
- seq计数只对这三个ID递增

FC-TRACE调试日志（4处）：
- isotp_send_flow_control() 函数入口：记录每次FC发送的参数
- OVERFLOW路径：消息过大时的FC
- FF_RESPONSE路径：收到首帧后的正常FC回复
- BLOCK_COMPLETE路径：块边界FC（本地BS>0时触发）
- 用于定位神秘的 30 00 05 流控帧来源

### 3. USER/UDS/uds_diagnostic.h

| 修改项 | 修改前 | 修改后 |
|--------|--------|--------|
| UDS_PHYSICAL_RESPONSE_ID | 0x12345678（占位值）| 0x18DAF103 |

### 4. USER/UDS/uds_did_rid.h

| 修改项 | 修改前 | 修改后 | 说明 |
|--------|--------|--------|------|
| RID_CALCULATE_CRC | 0xFF01 | 0xFEF0 | 匹配TBOX约定的RID |

### 5. USER/UDS/uds_diagnostic.c

新增未知RID校验（uds_handle_routine_control 函数）：
- 在 switch(sub_func) 之前检查RID是否在支持列表中
- 不支持时发送NRC 0x31（requestOutOfRange），3字节响应放入SF

RoutineControl响应格式统一为6字节（3处）：

| 处理分支 | 修改前 | 修改后 | UDS总响应大小 |
|----------|--------|--------|---------------|
| START (0x01) | *resp_len = 7 | *resp_len = 6 | 1 + 6 = 7字节 -> SF |
| STOP (0x02) | *resp_len = 7 | *resp_len = 6 | 1 + 6 = 7字节 -> SF |
| REQUEST_RESULTS (0x03) | *resp_len = 7 | *resp_len = 6 | 1 + 6 = 7字节 -> SF |

result编码：去掉 >> 24，使用 >> 16, >> 8, & 0xFF（3字节表示32位result）

修改原因：7字节数据 + 1字节SID = 8字节UDS响应 -> 超出SF容量(7字节) -> 必须发FF首帧 -> TBOX不发FC -> TX状态卡在SENDING_FF -> 所有后续UDS响应被阻塞

### 6. USER/UDS/flash_download.h

#define FW_FLASH_WRITE_ENABLED 0   /* 0=仅日志不写Flash, 1=实际写入Flash */

### 7. USER/main.c

初始化阶段：
- 调用 FlashDownload_Init()
- 调用 isotp_init(CAN1)
- 调用 uds_dl_init_fw() 注册下载接口

主循环CAN接收路径：
- CAN ID过滤：0x18DA03F1、0x18DAF103、0x18DBFFF0、0x18FF8118
- 调用 can_Get_Receive(CAN1, &RcvMsg, 1)
- isotp_receive_frame() -> 返回ISOTP_OK时 -> uds_receive_handler() 处理UDS请求
- 主循环中还调用：uds_process(), FlashDownload_Task(), isotp_tx_process()

---

## 数据流（修复后）

CAN中断 -> can_Get_Receive() -> isotp_receive_frame()
  |- SF (单帧): 直接返回 ISOTP_OK
  |- FF (首帧): 解析总长度 -> 发送FC (BS=0, 响应CAN ID) -> 返回 ISOTP_BUSY
  |- CF (连续帧): 校验序列号 -> 拷贝数据 -> 组装完成 -> 返回 ISOTP_OK

ISOTP_OK -> uds_receive_handler()
  |- 0x10 (会话控制) -> 肯定响应
  |- 0x27 (安全访问) -> 种子/密钥验证
  |- 0x31 (例程控制) -> 6字节数据响应，1+6=7字节SF
  |- 0x34 (请求下载) -> FlashDownload_OnRequestDownload()
  |- 0x36 (传输数据) -> FlashDownload_OnTransferData()
  |- 0x37 (传输退出) -> FlashDownload_OnTransferExit()

---

## 待解决问题

1. 神秘的FC @ seq=2：30 00 05 流控帧在收到 01 00 (enable命令, 0x18FF8118) 后发送。SF路径不应触发FC。已添加FC-TRACE日志用于下次测试定位来源。

2. 需重新编译烧录测试：验证RoutineControl流程（0x31）是否正确完成，TBOX是否继续发送0x34请求下载。

### 8. USER/UDS/uds_diagnostic.c — 0x34 RequestDownload 解析修复

0x34 请求数据格式修复（uds_handle_request_download 函数）：
- 修改前：data[1..4] 直接当作地址，data[5..8] 直接当作长度
- 修改后：按 UDS 标准格式解析 dataFormatIdentifier + addressAndLengthFormatIdentifier
  - data[1] = dataFormatIdentifier
  - data[2] = alfi（高4位=地址字节数，低4位=长度字节数）
  - 从 data[3] 开始按实际字节数提取地址和大小
- TBOX 数据示例：34 00 44 08 00 40 00 00 00 2C 90 -> addr=0x08004000, size=11408

### 9. USER/UDS/flash_download.c — Flash模拟模式增强

fw_is_address_valid() 函数：
- FW_FLASH_WRITE_ENABLED=1 时：执行真实地址范围校验
- FW_FLASH_WRITE_ENABLED=0 时：直接返回 true，跳过地址校验

Flash 擦除/写入/验证已在 FW_UPDATE_VERIFYING 状态中通过 #if FW_FLASH_WRITE_ENABLED 保护。

### 10. USER/UDS/uds_did_rid.h — RID 匹配修复

| 修改项 | 修改前 | 修改后 | 说明 |
|--------|--------|--------|------|
| RID_CALCULATE_CRC | 0xFEF0 | 0xFE00 | 匹配 TBOX 约定的 RID |

### 11. USER/UDS/uds_diagnostic.c — PROGRAMMING 会话安全状态复位

Session 切换到 PROGRAMMING_MODE 时新增安全状态复位：
- g_uds_ctrl.security_state = UDS_SECURITY_LOCKED
- g_uds_ctrl.security_attempts = 0
- g_uds_ctrl.security_seed = 0
- g_uds_ctrl.security_delay_ms = 0

修复原因：PROGRAMMING 会话后 TBOX 重新发起 27 01，但安全状态仍为 UNLOCKED，导致 NRC 0x24 (requestSequenceError)。

### 12. USER/UDS/isotp_transport.c — 调试日志清理

- 删除 [DBG] 状态调试打印
- 删除全部 4 处 [FC-TRACE] 日志
- 新增 isotp_ota_annotate() 函数：对 RX 帧（<--）增加人类可读的注释
- isotp_print_ota_frame() 中 RX 帧打印格式：
  OTA_I("seq=%-4d, %s 0x%08X, %02X ... %s", seq, direction, can_id, data..., annotation)



---

## 编码注意事项

| 文件 | 编码 |
|------|------|
| isotp_transport.c | UTF-16LE with BOM |
| isotp_transport.h | UTF-8 (no BOM) |
| uds_diagnostic.c | GBK |
| uds_diagnostic.h | GBK |
| uds_did_rid.h | GBK |
| main.c | UTF-16LE with BOM |
