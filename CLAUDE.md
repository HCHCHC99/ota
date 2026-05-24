# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Self-locking electric actuator (自锁推杆) firmware for the **HDSC HC32F460JETA** (ARM Cortex-M4, 128MHz MPLL). The firmware controls a motorized linear actuator with CAN bus vehicle integration, UDS firmware download, Modbus diagnostics, and sensorless FOC motor control.

The actual firmware source tree is in the `JC_FA2_v.17.32_20251203/` subdirectory — all paths below are relative to that.

## Build System

- **IDE**: Keil MDK v5 (`Selflocking_FA2.uvprojx`)
- **Toolchain**: ARMCC v5 at `D:\Keil_v5\ARM\ARMCC\Bin`
- **CLI build**: Run `Selflocking_FA2.BAT` from the firmware root — compiles all sources via `ArmAsm`/`ArmCC`/`ArmLink` and outputs `.hex` via `fromelf.exe`
- **Build output**: `Objects/` (intermediate `.o`, `.axf`, `.hex`), `Listings/` (linker map)

## File Encoding

All source files use **GBK/GB2312 encoding** with Chinese comments. When editing, match the exact bytes; the Edit tool may fail on Chinese character sequences. Use `sed` or Bash-based replacement as a fallback.

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
- **CAN vehicle messages**: `USER/system/canJ1939.c` — receives CAN motor commands (up/down/stop/goto/reset/clrfault)
- **Modbus slave**: `USER/msg/msgHandler.c` + `USER/msg/modbus.c` — UART Modbus RTU register access
- **UDS firmware download**: `USER/UDS/uds_diagnostic.c` (ISO 14229 services) → `USER/UDS/isotp_transport.c` (ISO 15765-2 transport over CAN) → `USER/UDS/uds_dl_bridge.c` (bridge) → `USER/UDS/flash_download.c` (flash erase/write/verify)

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
ISOTP filter list:            {0x18DA03F1, 0x18DAF103, 0x18FF8118}
```

### UDS Download Flow (known issue)

The `ISOTP_AUTO_FC` block in `isotp_transport.c` intercepts 0x36 TransferData first frames and sends both FC + ACK (positive response) **immediately upon receiving the first frame**, before any data is received or validated. This bypasses the normal ISOTP reassembly path and the CRC32 verification in `flash_download.c`. The UDS response CAN ID `UDS_PHYSICAL_RESPONSE_ID` was also set to a placeholder `0x12345678` instead of the correct `0x18DAF103`.

### Logging

- Debug output via SEGGER RTT (`RTT/rtt_log.h` — macros `LOG_CH`, `LOG_LEVEL_DEBUG`, etc.) over J-Link
- OTA frame logs in firmware root: `ota data.txt`, `ota data3.txt` (formatted as `seq=N, [RX/TX] CAN_ID, hex_bytes`)
- USART1 debug output: voltage/current telemetry every 2s (when `hz_usart1_debug` enabled)

### Python Tools (in firmware root)

- `readbin.py` — Interactive binary file viewer: hex dump with ASCII preview, string/hex pattern search with context display
- `security.py` — UDS Security Access (0x27) seed-to-key calculator: CRC8-based algorithm, takes 4-byte seed → outputs 4-byte key for Level 1 unlock

## Important Code Patterns

- **Non-blocking state machines** everywhere — no blocking delays, all modules use `_Task()` / `_LoopTask()` / `_ms_update()` patterns called from main loop or 1ms timer ISR
- **15-bit timers**: `USER/system/hz_timer.c` provides software timers on top of hardware Timer0; `NonBlockingDelay_t` pattern via `nbDelay_Init/Start/IsComplete`
- **System config struct**: `SYSTEM_CONFIG_t` in `USER/system/system.h` — gear ratio, lead, stroke, speeds, limits
- **Fault flags**: bitmask in `system.h` (M1/M2 overcurrent, hall, undervoltage, overvoltage, overtemperature, position)
- **Compile-time protocol selection**: Source conditionally compiles Modbus vs CAN paths via `#if (PROTOCOL_TYPE == MODBUS_PROTOCOL)` / `#elif (PROTOCOL_TYPE == CAN_PROTOCOL)`
