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
- **Modbus slave**: `USER/msg/msgHandler.c` + `USER/msg/modbus.c` — UART Modbus RTU register access
- **UDS firmware download**: See UDS stack section below

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

## Python Tools (in firmware root)

| Tool | Purpose |
|---|---|
| `receive_bin.py` | Extract firmware binary from OTA log (`ota data4.txt`): parses PRINTF_BIN hex dumps or ISO-TP TransferData frames, outputs `extracted_firmware.txt` |
| `compare_bin.py` | Bit error rate analysis: compares `extracted_firmware.txt` against a reference `.bin`, reports bit/byte error rates, prints diff locations |
| `readbin.py` | Interactive binary viewer: hex dump with ASCII preview, string/hex pattern search with context display |
| `security.py` | UDS Security Access (0x27) seed-to-key calculator: CRC8-based algorithm, 4-byte seed → 4-byte key for Level 1 unlock |

## Root-Level File

`isotp_lf3.c` in the repo root (not in the firmware subdirectory) is an independent ISO 15765-2 implementation variant. It is **not** part of the main firmware build.

## Important Code Patterns

- **Non-blocking state machines** everywhere — no blocking delays, all modules use `_Task()` / `_LoopTask()` / `_ms_update()` patterns called from main loop or 1ms timer ISR
- **15-bit timers**: `USER/system/hz_timer.c` provides software timers on top of hardware Timer0; `NonBlockingDelay_t` pattern via `nbDelay_Init/Start/IsComplete`
- **System config struct**: `SYSTEM_CONFIG_t` in `USER/system/system.h` — gear ratio, lead, stroke, speeds, limits
- **Fault flags**: bitmask in `system.h` (M1/M2 overcurrent, hall, undervoltage, overvoltage, overtemperature, position)
- **Compile-time protocol selection**: Source conditionally compiles Modbus vs CAN paths via `#if (PROTOCOL_TYPE == MODBUS_PROTOCOL)` / `#elif (PROTOCOL_TYPE == CAN_PROTOCOL)`
- **Interface decoupling**: `uds_dl_if.h` defines a function pointer table; `uds_dl_bridge.c` registers the flash download implementation. UDS layer never directly calls FlashDownload functions — enables swapping implementations without touching protocol code.

### Logging

- Debug output via SEGGER RTT (`RTT/rtt_log.h` — macros `LOG_CH`, `LOG_LEVEL_DEBUG`, etc.) over J-Link
- OTA frame logs in firmware root: `ota data.txt`, `ota data3.txt`, `ota data4.txt` (formatted as `seq=N, [RX/TX] CAN_ID, hex_bytes`)
- USART1 debug output: voltage/current telemetry every 2s (when `hz_usart1_debug` enabled)
