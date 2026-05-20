# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Self-locking electric actuator (自锁推杆) firmware for the **HDSC HC32F460JETA** (ARM Cortex-M4). The firmware controls a motorized linear actuator with CAN bus vehicle integration, UDS firmware download, Modbus diagnostics, and sensorless FOC motor control.

## Build System

- **IDE**: Keil MDK v5 (`Selflocking_FA2.uvprojx`)
- **Toolchain path**: `D:\Keil_v5\ARM\ARMCC\Bin` (ARMCC v5)
- **CLI build**: Run `Selflocking_FA2.BAT` from the project root — requires Keil MDK v5 installed at `D:\Keil_v5`

## File Encoding

All source files use **GBK/GB2312 encoding** with Chinese comments. When editing, match the exact bytes; the Edit tool may fail on Chinese character sequences. Use `sed` or Bash-based replacement as a fallback.

## Architecture

```
driver/           ← HDSC HC32F46x DDL (peripheral drivers: CAN, UART, Timer, ADC, GPIO...)
common/           ← DDL config, startup .s, system init, MCU headers
RTT/              ← SEGGER RTT debug logging (J-Link)
USER/
  adapter/        ← HAL adaptation layer (wraps DDL APIs)
  system/         ← System state machine, device registry, flash, hardware config
  MotorControl/   ← Sensorless FOC: current loop, speed loop, position loop, PID
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
- **System state machine**: `USER/system/sys_main_state.c` (`SystemContext` → `g_sys_ctx.main_state`)
- **Device registry**: `USER/system/device_manager.c` — register-based abstraction for PWM, Flash, Timer devices
- **CAN vehicle messages**: `USER/system/canJ1939.c` — receives CAN motor commands (up/down/stop/goto/reset)
- **Modbus slave**: `USER/msg/msgHandler.c` + `USER/msg/modbus.c` — UART Modbus RTU register access
- **UDS firmware download**: `USER/UDS/uds_diagnostic.c` (ISO 14229 services) → `USER/UDS/isotp_transport.c` (ISO 15765-2 transport over CAN) → `USER/UDS/uds_dl_bridge.c` (bridge) → `USER/UDS/flash_download.c` (flash erase/write/verify)

### UDS Download Flow (known issue)

The `ISOTP_AUTO_FC` block in `isotp_transport.c` intercepts 0x36 TransferData first frames and sends both FC + ACK (positive response) **immediately upon receiving the first frame**, before any data is received or validated. This bypasses the normal ISOTP reassembly path and the CRC32 verification in `flash_download.c`. The UDS response CAN ID `UDS_PHYSICAL_RESPONSE_ID` was also set to a placeholder `0x12345678` instead of the correct `0x18DAF103`.

- **TBOX request CAN ID**: `0x18DA03F1`
- **ECU response CAN ID**: `0x18DAF103`
- **Functional/broadcast ID**: `0x18DBFFF0`

### Logging

- Debug output via SEGGER RTT (`RTT/rtt_log.h` — macros `LOG_CH`, `LOG_LEVEL_DEBUG`, etc.)
- OTA frame logs in project root: `ota data.txt`, `ota data_1.txt` (formatted as `seq=N, [RX/TX] CAN_ID, hex_bytes`)
- Python helper: `readbin.py` (reads .bin firmware files)

## Important Code Patterns

- **Non-blocking state machines** everywhere — no blocking delays, all modules use `_Task()` / `_LoopTask()` / `_ms_update()` patterns called from main loop or 1ms timer ISR
- **15-bit timers**: `USER/system/hz_timer.c` provides software timers on top of hardware Timer0
- **System config struct**: `SYSTEM_CONFIG_t` in `USER/system/system.h` — gear ratio, lead, stroke, speeds, limits
- **Fault flags**: bitmask in `system.h` (M1/M2 overcurrent, hall, undervoltage, overvoltage, overtemperature, position)
