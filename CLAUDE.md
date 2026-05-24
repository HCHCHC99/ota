# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Self-locking electric actuator (自锁推杆) firmware for the **HDSC HC32F460JETA** (ARM Cortex-M4, 128MHz MPLL). The firmware controls a motorized linear actuator with CAN bus vehicle integration, UDS firmware download, Modbus diagnostics, and sensorless FOC motor control.

The actual firmware source tree is in the `JC_FA2_v.17.33_20251203/` subdirectory — all paths below are relative to that.

## Build System

- **IDE**: Keil MDK v5 (`Selflocking_FA2.uvprojx`)
- **Toolchain**: ARMCC v5 at `D:\Keil_v5\ARM\ARMCC\Bin`
- **CLI build**: Run `Selflocking_FA2.BAT` from the firmware root — compiles all sources via `ArmAsm`/`ArmCC`/`ArmLink` and outputs `.hex` via `fromelf.exe`
- **Build output**: `Objects/` (intermediate `.o`, `.axf`, `.hex`), `Listings/` (linker map)

## File Encoding

All source files use **GBK/GB2312 encoding** with Chinese comments. When editing, the Edit tool may fail on Chinese character sequences — use `sed` or Bash-based replacement as a fallback. Never convert files to UTF-8 unless explicitly asked.

## Architecture

```
driver/           ← HDSC HC32F46x DDL (peripheral drivers: CAN, UART, Timer, ADC, GPIO...)
  inc/            ← Driver headers
  src/            ← Driver source
common/           ← DDL config, startup .s, system init, MCU headers
RTT/              ← SEGGER RTT debug logging (J-Link)
USER/
  adapter/        ← HAL adaptation layer (wraps DDL APIs: adc, can, flash, gpio, pwm, timer, uart, delay, ring_buffer)
  system/         ← System state machine, device registry, flash, hardware config, CAN J1939, power management
  MotorControl/   ← Sensorless FOC: current loop, speed loop, position loop, PID, hall sensor
  CanOpen/        ← CANopen protocol (SDO, NMT)
  msg/            ← Message handler + Modbus RTU slave over UART
  UDS/            ← ISO 14229 + ISO 15765-2 diagnostic stack (firmware download)
  btn/            ← Button input driver
  math/           ← CRC library
  debug/          ← Debug macros
  main.c / main.h ← Entry point
```

### Key Code Paths

- **Motor control loop**: `USER/MotorControl/mc_app.c` orchestrates FOC via `mc_cur.c` / `mc_spd.c` / `mc_pos.c` / `mc_hall.c`
- **System state machine**: `USER/system/sys_main_state.c` — `SystemContext` struct (`g_sys_ctx`) cycles through `SYS_STATE_INIT → POWER_SAMPLE → HALL_SAMPLE → MOTOR_CMD → MOTOR_PROTECT → POWER_SAMPLE`; faults trigger two-tier protection (primary delay then secondary reset after 10 consecutive faults)
- **Device registry**: `USER/system/device_manager.c` — register-based abstraction for PWM, Flash, Timer devices
- **CAN vehicle messages**: `USER/system/canJ1939.c` — receives CAN motor commands (up/down/stop/goto/reset/clrfault); CAN adapter layer in `USER/adapter/can_adapter.c/h` wraps DDL CAN APIs with ring-buffer RX/TX
- **Modbus slave**: `USER/msg/msgHandler.c` + `USER/msg/modbus.c` — UART Modbus RTU register access; `USER/msg/queue.c` provides FIFO queue used by message handling
- **CANopen**: `USER/CanOpen/co.c` + `co_sdo.c` — CANopen protocol stack (NMT, SDO)
- **UDS firmware download**: See UDS stack section below; initialization calls `FlashDownload_Init()` → `isotp_init(CAN1)` → `uds_dl_init_fw()` in sequence before the main loop (`main.c:1788-1797`)

### Key Hardware Configuration (mc_config.h)

- **Motor type**: `MOTOR_TYPE_DC` (brushed DC), 2-phase driver (`DRIVER_PHASE_NB = 2`), dual hall sensors (`HALL_NB = 2`)
- **Driver type**: `DRV_TYPE_HB_WITH_PD` (H-bridge + pre-drive), output mode `DO_H_PWM_L_IO` (high-side PWM, low-side GPIO via TIMA4)
- **PWM**: TIMA4 unit, `PWM_OUTPUT_FULLSCALE = 2000`, 16kHz, PortB pins 6-9
- **Current sense**: ADC1 channel 5 (PA05), 10mΩ shunt, 20× op-amp gain, 12A overcurrent threshold
- **Motor speed params**: Start RPM 500, target RPM 3000, max RPM 6000, acceleration 2000 rpm/s

### Peripheral Assignments

| Peripheral | Function |
|---|---|
| TIMA4 (M4_TMRA4) | Motor PWM output (CH2=V, CH4=U) |
| Timer0 (M4_TMR02) | 1ms system tick ISR |
| USART1 | Debug/USB output (`hz_usart1_debug`) |
| USART3 | Modbus RTU RS-485 (even parity, 485 DIR pin) |
| CAN1 | Vehicle CAN + UDS download |
| ADC1 | Motor current sense (PA05) |
| PA08-PA10 | Hall sensor inputs (external interrupt, both edges) |

### Clock Tree

- **Source**: 8MHz HRC (internal) → MPLL (×32/2/2) → **128MHz** HCLK
- **Bus dividers**: EXCLK=64MHz, PCLK0=128MHz, PCLK1=64MHz, PCLK2=32MHz, PCLK3=32MHz, PCLK4=64MHz
- **Flash latency**: 4 wait states

### CAN IDs

```
TBOX request (ECU receives):  0x18DA03F1
ECU response (ECU transmits): 0x18DAF103
Functional/broadcast:         0x18DBFFF0
ISOTP filter list:            {0x18DA03F1, 0x18DAF103, 0x18FF8118, 0x18DBFFF0}
```

## UDS Diagnostic Stack

The UDS stack implements ISO 14229 (diagnostics) over ISO 15765-2 (transport) for firmware-over-CAN. Architecture follows a bridge pattern:

```
CAN ISR (system.c)
  → isotp_receive_frame()                    [ISO 15765-2 frame reassembly]
  → g_uds_rx_buffer + g_uds_rx_pending=1     [global async buffer]
Main Loop (main.c)
  → uds_receive_handler()                    [SID dispatch]
    → uds_dl_if (function pointer table)     [abstract download interface]
      → uds_dl_bridge                        [type conversion layer]
        → FlashDownload_On*()                [actual flash engine]
Response path: uds_send_response() → isotp_send_message() → CAN TX
```

### UDS Files (USER/UDS/)

| File | Role |
|---|---|
| `uds_diagnostic.c/h` | Core UDS: SID dispatch (0x10/0x11/0x22/0x27/0x2E/0x31/0x34/0x36/0x37/0x3E), session/security state machine |
| `isotp_transport.c/h` | ISO 15765-2: SF/FF/CF/FC frame handling, 8KB RX buffer, CAN ID filter recording, OTA frame logging |
| `flash_download.c/h` | Flash engine: 60KB RAM staging buffer, deferred erase+write at 0x37 TransferExit, CRC32 verify |
| `uds_dl_bridge.c` | Bridges UDS `uds_dl_if_t` interface to `FlashDownload_*()` implementation |
| `uds_dl_if.h` | Abstract download interface (function pointer table) — decouples UDS from flash implementation |
| `security_access.c/h` | Security Access (0x27): CRC8 chain seed-to-key algorithm for Level 1 unlock |
| `uds_did_rid.h` | DID/RID constants: 0xF000=FirmwareVer, 0xFF00=EraseFirmware, 0xFE00=CalcCRC |

### UDS Download Flow

1. **0x34 RequestDownload**: Validates address/size, sets `target_address` and `total_size`, state → `FW_UPDATE_READY`
2. **0x36 TransferData**: Appends data blocks to RAM buffer (`g_fw_ram_buffer[60KB]`), checks sequence number, computes running CRC32
3. **0x37 TransferExit**: Validates total received == expected, state → `FW_UPDATE_VERIFYING`, sets `pending_response = true`
4. **FlashDownload_Task()** (called from main loop): Erases target sectors → writes RAM buffer to Flash → verifies each word → state → `FW_UPDATE_COMPLETE`
5. After completion, `pending_response = false` — UDS layer sends positive response to 0x37

**Critical: `FW_FLASH_WRITE_ENABLED` is set to `0` in `flash_download.h`** — the erase/write/verify steps are compiled out. The stack logs everything but does not physically write Flash. Set to `1` to enable actual programming.

### Async Receive Pattern (CAN ISR → Main Loop)

Declared in `system.c:3771-3774` / `system.h:300-303`:
```c
volatile uint8_t g_uds_rx_pending;  // ISR sets to 1, main loop clears to 0
uint8_t g_uds_rx_buffer[8192];
uint16_t g_uds_rx_len;
uint32_t g_uds_rx_can_id;
```

`CAN_RxIrqCallBack()` (registered at `main.c:1015`) calls `isotp_receive_frame()` in ISR context. When a complete ISOTP message is reassembled, it copies to `g_uds_rx_buffer` and sets `g_uds_rx_pending = 1`. The main loop polls this flag and calls `uds_receive_handler()`.

### ISOTP_AUTO_FC → Disabled

The `ISOTP_AUTO_FC` block is now **disabled** (`isotp_transport.h:53` — commented out with `// #define ISOTP_AUTO_FC`). All frames go through the normal ISO-TP reassembly path.

### 1ms Timer ISR (Critical for UDS Timing)

`Timer01B_CallBack()` in `system.c:3680` runs every 1ms from Timer0 channel B. It drives:
- `system_Timer_Task()` — system state machine tick
- `tickTimer_Update()` — global millisecond counter
- `uds_ms_update()` — session timeout countdown (default 65535ms); when expired, resets to DEFAULT session and clears security. Also counts down `security_delay_ms` after max failed unlock attempts.
- `isotp_ms_update()` — RX timeout detection (resets partial reassembly on timeout), STmin delay counting for TX consecutive frames, TX timeout detection

### NRC 0x78 Polling Pattern (0x37 TransferExit)

The 0x37 handler doesn't send the final positive response immediately. Instead:
1. `FlashDownload_OnTransferExit()` sets `pending_response = true` → UDS handler calls `uds_send_response_pending()` → sends NRC 0x78 ("Response Pending")
2. `FlashDownload_Task()` (main loop) performs deferred erase/write/verify → sets `pending_response = false`
3. Tester polls by resending 0x37; this time `is_pending()` returns false → positive response `0x77` is sent

This is standard ISO 14229 behavior for long-running operations.

### PRINTF_BIN Magic Trigger

During 0x36 TransferData, `flash_download.c` watches for magic bytes `{0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78}` in the data stream. When detected, the entire RAM buffer content is dumped as hex via SEGGER RTT. This is the mechanism `extract_fw.py` uses to extract firmware from the ECU log (`ota data4.txt`).

### Download Interface Registration

`uds_dl_init_fw()` in `uds_dl_bridge.c` is the explicit registration call that connects the UDS layer to the flash download implementation. It calls `uds_dl_register(&g_firmware_download_iface)`, setting the global `g_dl_iface` pointer. UDS handlers access the flash engine exclusively through this function pointer table (`uds_dl_if.h`), never calling `FlashDownload_*()` functions directly.

## Python Tools (in firmware root)

**Run**: `py <script>.py` from `JC_FA2_v.17.33_20251203/`. All scripts read paths from variables at the top of each file — edit the `# ========== 路径配置 ==========` section to change input/output files. `app1.bin` and `app2.bin` are read from the same directory by default.

| Tool | Purpose |
|---|---|
| `extract_fw.py` | Extract firmware binary from OTA log (`ota data4.txt`): parses PRINTF_BIN hex dumps or ISO-TP TransferData frames, outputs `extracted_firmware.txt` |
| `compare_rec.py` | Bit error rate analysis: compares `extracted_firmware.txt` against `app1.bin` (or `app2.bin` via `REF_BIN`), reports bit/byte error rates, prints diff locations in `XX[YY]` format, plus per-bit error stats and byte-level mapping analysis |
| `compare_tbox.py` | Parse PCAN-View `.trc` trace, reassemble ISO-TP frames from TBOX CAN data, compare against `app1.bin`/`app2.bin` (via `REF_BIN`). Same error analysis as `compare_rec.py` |
| `hexdump.py` | Convert `app1.bin` and `app2.bin` to aligned hex dump `.txt` files (16 bytes/row, 8+8 grouping) |
| `bin_search.py` | Interactive binary viewer: hex dump with ASCII preview, string/hex pattern search with context display |
| `security.py` | UDS Security Access (0x27) seed-to-key calculator: CRC8-based algorithm, 4-byte seed → 4-byte key for Level 1 unlock |

### Key Data Files

| File | Role |
|---|---|
| `app1.bin` / `app2.bin` | Reference firmware binaries (gitignored — `*.bin` in `.gitignore`) |
| `true_data.trc` | PCAN-View CAN trace of TBOX firmware download (input to `compare_tbox.py`) |
| `ota data4.txt` | ECU serial log with PRINTF_BIN hex dumps and OTA frames (input to `extract_fw.py`) |
| `extracted_firmware.txt` | Firmware hex dump extracted from ECU log (output of `extract_fw.py`, input to `compare_rec.py`) |
| `tbox_extracted_firmware.txt` | Firmware hex dump extracted from CAN trace (output of `compare_tbox.py`) |
| `app1_bin.txt` / `app2_bin.txt` | Reference firmware hex dumps (output of `hexdump.py`) |
| `Selflocking_FA2.bin` | Full firmware build output (gitignored) |

### Typical Analysis Workflow

```
ota data4.txt ──[extract_fw.py]──> extracted_firmware.txt ──[compare_rec.py]──> 误码率 + Bit统计 + 字节映射
true_data.trc ──[compare_tbox.py]──> 误码率 + Bit统计 + 字节映射
app1.bin ────────[hexdump.py]──────> app1_bin.txt
```

## Root-Level File

`isotp_lf3.c` in the repo root (not in the firmware subdirectory) is an independent ISO 15765-2 implementation variant. It is **not** part of the main firmware build.

## Important Code Patterns

- **Non-blocking state machines** everywhere — no blocking delays, all modules use `_Task()` / `_LoopTask()` / `_ms_update()` patterns called from main loop or 1ms timer ISR
- **15-bit timers**: `USER/system/hz_timer.c` provides software timers on top of hardware Timer0; `NonBlockingDelay_t` pattern via `nbDelay_Init/Start/IsComplete`
- **System config struct**: `SYSTEM_CONFIG_t` in `USER/system/system.h` — gear ratio, lead, stroke, speeds, limits
- **Fault flags**: bitmask in `system.h` (M1/M2 overcurrent, hall, undervoltage, overvoltage, overtemperature, position)
- **Compile-time protocol selection**: Source conditionally compiles Modbus vs CAN paths via `#if (PROTOCOL_TYPE == MODBUS_PROTOCOL)` / `#elif (PROTOCOL_TYPE == CAN_PROTOCOL)`
- **Interface decoupling**: `uds_dl_if.h` defines a function pointer table; `uds_dl_bridge.c` registers the flash download implementation. UDS layer never directly calls FlashDownload functions — enables swapping implementations without touching protocol code.

### Debugging (J-Link + SEGGER RTT)

- **Hardware debugger**: J-Link via SWD interface
- **SEGGER RTT**: Real-time debug output over J-Link (no UART needed). Configuration in `RTT/SEGGER_RTT_Conf.h`
  - RTT Channel 0: main log output
  - Log macros in `RTT/rtt_log.h`: `LOG_CH(channel, level, color, module, fmt, ...)` 
  - Multi-module channels: LOG_CH_MAIN(0), LOG_CH_USB(1), LOG_CH_SENSOR(2), LOG_CH_MOTOR(3), LOG_CH_COMM(4), LOG_CH_UI(5)
  - Log levels: DEBUG(0), INFO(1), WARN(2), ERROR(3), FATAL(4) with ANSI color codes
  - View with J-Link RTT Viewer or J-Link RTT Client
- **OTA frame logs** in firmware root: `ota data.txt`, `ota data3.txt`, `ota data4.txt` (formatted as `seq=N, [RX/TX] CAN_ID, hex_bytes` with UDS annotation)
- **USART1 debug output**: voltage/current telemetry every 2s (when `hz_usart1_debug` enabled)
- **VS Code IntelliSense**: `.vscode/c_cpp_properties.json` configures include paths for HC32F46x + CMSIS for code browsing outside Keil. Compiler set to `armcc.exe` at `C:/Keil_v5/ARM/ARMCC/bin/`

### Pin Definitions

All GPIO/ADC/UART pin assignments are in `USER/main.h` (lines 28-118): LED ports, button inputs, limit switches, PWM output pins, pre-drive pins, ADC channels (voltage, current, temperature, potentiometer), RS-485 DIR pin, USART3 pins. Motor-specific pin configs are in `USER/MotorControl/mc_config.h`.
