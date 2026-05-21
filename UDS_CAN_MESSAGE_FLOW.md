# UDS CAN报文收发对照表

## 1. CAN ID 分配

| 角色 | CAN ID | 宏定义 |
|------|--------|--------|
| 物理寻址请求 (Tester -> ECU) | `0x18DA03F1` | `UDS_PHYSICAL_REQUEST_ID` |
| 物理寻址响应 (ECU -> Tester) | `0x18DAF103` | `UDS_PHYSICAL_RESPONSE_ID` |
| 功能寻址请求 (广播) | `0x18DBFFF0` | `UDS_FUNCTIONAL_REQUEST_ID` |
| 额外监控ID | `0x18FF8118` | (硬编码) |

---

## 2. ISO-TP 层：收到什么 -> 回复什么

### 2.1 帧类型定义

| 帧类型 | PCI值 | 含义 |
|--------|-------|------|
| SF (单帧) | `0x0N` (N=数据长度1-7) | 单帧，承载最多7字节数据 |
| FF (首帧) | `0x1N NN` (N+N=12位总长度) | 多帧首帧，承载前6字节 + 总长度 |
| CF (连续帧) | `0x2N` (N=序列号1..15) | 多帧后续帧，承载最多7字节 |
| FC (流控帧) | `0x3N` (N=流控状态) | 流控帧，ECU发送以控制接收节奏 |

### 2.2 isotp_receive_frame() 状态机

**配置参数：**
- `local_bs = 0` (块大小=0，允许发送方一次发完所有帧)
- `local_st_min = 5ms` (最小间隔时间)
- `rx_buffer = 8192` 字节
- `rx_dst_id = 0x18DAF103` (FC回复目标CAN ID)

#### 状态：ISOTP_RX_IDLE (空闲)

| 收到的帧 | 条件 | ECU动作 | 发出的CAN帧 | 返回值 |
|----------|------|---------|------------|--------|
| **SF** `[0x0N, data...]` | N=1..7 | 提取数据到out_data | 无 | `ISOTP_OK` |
| **FF** `[0x1N, len_lo, data...]` | 总长度 > 8192 | 发送溢出FC | `30 00 05 AA AA AA AA AA` → `0x18DAF103` | `ISOTP_ERROR` |
| **FF** `[0x1N, len_lo, data...]` | 总长度 <= 6 (数据全在FF中) | 提取数据到out_data | 无 | `ISOTP_OK` |
| **FF** `[0x1N, len_lo, data...]` | 总长度 > 6 (需要CF) | 拷贝前6字节，发送FC | `30 00 05 AA AA AA AA AA` → `0x18DAF103` | `ISOTP_BUSY` |
| **CF** `[0x2N, data...]` | — | 记录错误 | 无 | `ISOTP_ERROR` |
| **FC** `[0x3N, BS, ST]` | — | 调用流控处理 | 无 | `ISOTP_BUSY` |

#### 状态：ISOTP_RX_ACTIVE (接收中)

| 收到的帧 | 条件 | ECU动作 | 发出的CAN帧 | 返回值 |
|----------|------|---------|------------|--------|
| **任意** | CAN ID != rx_src_id | **重置连接!** | 无 | `ISOTP_ERROR` |
| **非CF** | frame_type != 0x20 | 重置连接 | 无 | `ISOTP_ERROR` |
| **CF** `[0x2N, data...]` | 序列号正确，数据未收完 | 拷贝数据 | 无 | `ISOTP_BUSY` |
| **CF** `[0x2N, data...]` | 序列号正确，数据收完 | 拷贝数据，组装完成 | 无 | `ISOTP_OK` |
| **CF** `[0x2N, data...]` | 序列号错误 | 重置连接 | 无 | `ISOTP_ERROR` |
| **CF** `[0x2N, data...]` | BS>0 且 达到块边界 | 拷贝数据，发送FC | `30 BS ST AA AA AA AA AA` → `0x18DAF103` | `ISOTP_BUSY` |

> 注：当前 `local_bs=0`，所以"块边界FC"条件永远不会触发。

#### 流控帧接收处理 (isotp_handle_flow_control_internal)

| 收到的FC | TX状态 | ECU动作 |
|----------|--------|---------|
| `30 00 05` (CTS) | TX_SENDING_FF | 记录BS/STmin，开始发CF |
| `31 xx xx` (WAIT) | TX_SENDING_FF | 忽略(仅日志) |
| `32 xx xx` (OVERFLOW) | TX_SENDING_FF | 重置TX |

---

### 2.3 ECU发送帧格式

#### 单帧 (SF) - 数据 <= 7字节

```
CAN ID: 0x18DAF103
8字节: [0x0N, data[0..N-1], 0xAA...填充]
```
N = 数据长度 (1..7)

#### 首帧 (FF) - 数据 > 7字节

```
CAN ID: 0x18DAF103
8字节: [0x1N, len_lo, data[0..5], 0xAA...填充]
```
N = 总长度高4位, len_lo = 总长度低8位

#### 连续帧 (CF) - 多帧发送的后续帧

```
CAN ID: 0x18DAF103
8字节: [0x2N, data[i..i+6], 0xAA...填充]
```
N = 序列号 (1..15，跳过0，循环)

#### 流控帧 (FC) - ECU作为接收方时

```
CAN ID: 0x18DAF103 (rx_dst_id)
8字节: [0x3S, BS, STmin, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA]
```
S = 流控状态 (0=CTS, 1=WAIT, 2=OVERFLOW)

---

## 3. UDS 层：收到什么SID -> 回复什么

### 3.1 通用格式

**肯定响应：** `[SID+0x40, 数据...]`，CAN ID = `0x18DAF103`

**否定响应：** `[0x7F, SID, NRC]`，CAN ID = `0x18DAF103`

**UDS层CAN ID过滤：** 只接受 `0x18DA03F1` 和 `0x18DBFFF0`。`0x18FF8118` 在此被拒绝。

### 3.2 SID 0x10 — 诊断会话控制 (DiagnosticSessionControl)

| 请求 | 条件 | 响应 |
|------|------|------|
| `10 01` | 默认会话 | `50 01` |
| `10 02` | 编程会话 | `50 02` |
| `10 03` | 扩展会话 (需当前为默认会话) | `50 03` |
| `10 03` | 扩展会话 (当前非默认) | `7F 10 22` |
| `10 XX` | 不支持子功能 | `7F 10 12` |
| `10` | 长度不足 | `7F 10 13` |

**副作用：**
- 0x01: session=DEFAULT, security=LOCKED, attempts=0
- 0x02: session=PROGRAMMING
- 0x03: session=EXTENDED, security=LOCKED

### 3.3 SID 0x27 — 安全访问 (SecurityAccess)

| 请求 | 条件 | 响应 |
|------|------|------|
| `27 01` | 请求种子，非扩展/编程会话 | `7F 27 22` |
| `27 01` | 请求种子，delay计时中 | `7F 27 37` |
| `27 01` | 请求种子，state!=LOCKED | `7F 27 24` |
| `27 01` | 请求种子，正常 | `67 01 12 34 56 78` (固定种子0x12345678) |
| `27 02` | 发送密钥，state!=SEED_SENT | `7F 27 24` |
| `27 02` | 发送密钥，长度不足(需4字节key) | `7F 27 13` |
| `27 02` | 密钥正确 (seed^0xFFFFFFFF) | `67 02` |
| `27 02` | 密钥错误，attempts<3 | `7F 27 35` |
| `27 02` | 密钥错误，attempts>=3 | `7F 27 36` (触发delay) |
| `27 XX` | 长度不足 | `7F 27 13` |

**固定种子模式 (UDS_SEED_MODE_FIXED=1):** seed=0x12345678, 正确key=0xEDCBA987

### 3.4 SID 0x31 — 例程控制 (RoutineControl)

**请求:** `31 [子功能] [RID高8] [RID低8] [可选数据...]`

**支持的RID:**

| RID | 名称 | 需要解锁 | 功能 |
|-----|------|---------|------|
| 0xFF00 | 擦除固件 | 是 | 延迟擦除(实际在0x37后执行) |
| 0xFEF0 | 计算CRC | 是 | 读取flash计算CRC32 |
| 0xFF02 | 跳转Bootloader | 否 | result=1 |
| 0xFF03 | 跳转应用程序 | 否 | result=1 |

| 请求 | 条件 | 响应 |
|------|------|------|
| `31 01 FF 00` | 擦除-正常 | `71 01 FF 00 00 00 01` |
| `31 01 FE F0` | CRC-正常 | `71 01 FE F0 xx xx xx` (3字节CRC值) |
| `31 01 FF 02` | 跳转BL-正常 | `71 01 FF 02 00 00 01` |
| `31 01 FF 03` | 跳转APP-正常 | `71 01 FF 03 00 00 01` |
| `31 02 xx xx` | 停止-正常 | `71 02 xx xx xx xx xx` (6字节数据) |
| `31 03 xx xx` | 请求结果-正常 | `71 03 xx xx xx xx xx` (6字节数据) |
| `31 xx xx xx` | RID不支持 | `7F 31 31` |
| `31 xx xx xx` | 子功能不支持 | `7F 31 12` |
| `31 xx xx xx` | 非扩展/编程会话 | `7F 31 22` |
| `31 xx FF 00` | 擦除-未解锁 | `7F 31 33` |
| `31 xx FE F0` | CRC-未解锁 | `7F 31 33` |
| `31` | 长度不足 | `7F 31 13` |

### 3.5 SID 0x34 — 请求下载 (RequestDownload)

**请求:** `34 [地址4字节] [大小4字节]` (共9字节)

| 请求 | 条件 | 响应 |
|------|------|------|
| `34 00 03 40 00 xx xx xx xx` | 正常 (地址0x34000) | `74 40 00` |
| 任意 | 下载接口未注册 | `7F 34 22` |
| 任意 | 长度<9 | `7F 34 13` |
| 任意 | 状态不是IDLE | `7F 34 24` |
| 任意 | 地址不在0x34000~0x53FFF | `7F 34 31` |
| 任意 | 大小=0 或 >256KB | `7F 34 31` |
| 任意 | 大小 > 60KB (RAM缓冲) | `7F 34 31` |

**blockSize固定返回 0x4000 (16384字节)**

### 3.6 SID 0x36 — 传输数据 (TransferData)

**请求:** `36 [块序号] [数据...]`

| 请求 | 条件 | 响应 |
|------|------|------|
| `36 01 [数据...]` | 正常 (每块) | **无响应** (response_len=0) |
| `36 01 [数据...]` | 有pending任务 | `7F 36 78` |
| 任意 | 下载接口未注册 | `7F 36 22` |
| 任意 | 长度<2 | `7F 36 13` |
| 任意 | 状态未READY | `7F 36 22` |
| 任意 | 块序号不连续 | `7F 36 24` |
| 任意 | 总数据超限 | `7F 36 31` |
| 任意 | RAM缓冲溢出 | `7F 36 31` |
| 任意 | 其他错误 | `7F 36 72` |

### 3.7 SID 0x37 — 请求传输退出 (RequestTransferExit)

**请求:** `37`

| 请求 | 条件 | 响应 |
|------|------|------|
| `37` | 正常 | **无响应** → pending → FlashDownload_Task处理 |
| `37` | 有pending任务 | `7F 37 78` |
| 任意 | 下载接口未注册 | `7F 37 22` |
| 任意 | 状态不对 | `7F 37 24` |
| 任意 | received_size != total_size | `7F 37 31` |

> 0x37后进入FW_UPDATE_VERIFYING状态，FlashDownload_Task()执行CRC校验。若FW_FLASH_WRITE_ENABLED=0则跳过flash写入。

### 3.8 SID 0x3E — 测试设备在线 (TesterPresent)

| 请求 | 条件 | 响应 |
|------|------|------|
| `3E 00` | 子功能0x00 | **无响应** (抑制) |
| `3E 80` | 子功能0x80 | `7E 80` |
| `3E XX` | 不支持子功能 | `7F 3E 12` |

### 3.9 其他SID

| SID | 功能 | 响应 |
|-----|------|------|
| 0x11 | ECU复位 (仅支持0x03软件复位) | `51 03` |
| 0x14 | 清除诊断信息 | **无响应** |
| 0x19 | 读取DTC (始终返回无DTC) | `59 [子功能] 00 00 00` |
| 0x22 | 按ID读取 (DID 0xF000/0xF001/0xF002) | `62 [DID2字节] [值2字节]` |
| 0x2E | 按ID写入 | **永远返回NRC** (所有DID只读) |
| 未定义SID | — | `7F [SID] 11` |

---

## 4. 特殊场景

### 4.1 收到 0x18FF8118

```
main.c CAN过滤: 通过
isotp_receive_frame(): 正常处理 (SF→ISOTP_OK, FF→可能发FC)
uds_receive_handler(): 被UDS层CAN ID过滤拒绝 (return -1)
                      不会产生任何UDS响应
```

### 4.2 功能寻址 0x18DBFFF0

```
main.c CAN过滤: 通过
isotp_receive_frame(): 正常处理
uds_receive_handler(): 接受处理
响应CAN ID: 仍然发到 0x18DAF103 (物理响应ID)
           不发回功能寻址ID
```

### 4.3 ISO-TP TX忙碌时

```
isotp_send_message() 返回 ISOTP_BUSY
UDS层忽略返回值 (uds_send_response 始终返回0)
响应被静默丢弃，无重试机制
```

### 4.4 会话超时

```
会话定时器 = 65535ms (默认)
超时后: session→DEFAULT, security→LOCKED, attempts=0, seed=0
每次收到UDS请求刷新定时器
```

---

## 5. 所有NRC汇总

| NRC | 含义 | 触发场景 |
|-----|------|---------|
| 0x11 | serviceNotSupported | 不支持的SID |
| 0x12 | subFunctionNotSupported | 不支持的子功能 |
| 0x13 | incorrectMessageLength | 请求长度不足 |
| 0x21 | busy | 0x36传输时状态忙 |
| 0x22 | conditionsNotCorrect | 会话不匹配/条件不满足 |
| 0x24 | requestSequenceError | 0x27状态不对/0x36块序号不连续 |
| 0x31 | requestOutOfRange | DID不支持/RID不支持/地址越界 |
| 0x33 | securityAccessDenied | 0x2E/0x31(擦除/CRC)需先解锁 |
| 0x35 | invalidKey | 0x27密钥错误 (未达最大次数) |
| 0x36 | exceededNumberOfAttempts | 0x27密钥错误达到最大次数(3) |
| 0x37 | requiredTimeDelayNotExpired | 0x27 delay计时中 |
| 0x72 | generalProgrammingFailure | 0x34/0x36/0x37编程失败 |
| 0x78 | responsePending | 0x36/0x37有pending任务 |

---

## 6. 完整下载流程示例

```
TBOX发送                     ECU回复
=======                     =======
[SF] 10 02              →   [SF] 50 02
[SF] 27 01              →   [SF] 67 01 12 34 56 78
[SF] 27 02 xx xx xx xx  →   [SF] 67 02 (密钥正确)
[SF] 31 01 FF 00        →   [SF] 71 01 FF 00 00 00 01 (擦除)
[SF] 34 00 03 40 00 ... →   [SF] 74 40 00 (请求下载)
[FF] 36 01 [258字节数据] →   [FC] 30 00 05 AA AA ... (ECU发FC)
[CF] 2x [数据]          →   无回复 (继续接收CF)
... 共36帧CF ...
                          →   无回复 (0x36无肯定响应)
... 重复45批 ...
[SF] 37                 →   无回复 (触发CRC校验)
```

---

## 7. CAN帧字节布局参考

```
单帧请求:  [0x0N][数据...][填充0xAA...]  8字节, CAN ID 0x18DA03F1
单帧响应:  [0x0N][数据...][填充0xAA...]  8字节, CAN ID 0x18DAF103
首帧请求:  [0x1N][长度低8][数据0..5][填充]  8字节, CAN ID 0x18DA03F1
首帧响应:  [0x1N][长度低8][数据0..5][填充]  8字节, CAN ID 0x18DAF103
连续帧:    [0x2N][数据0..6][填充]  8字节, N=1..15
流控帧:    [0x30][BS][STmin][0xAA×5]  8字节, CAN ID 0x18DAF103
否定响应:  [0x7F][SID][NRC]  通常作为SF的数据部分
```
