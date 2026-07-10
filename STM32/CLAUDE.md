# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

DJI RoboMaster A-board (STM32F427) bare-metal robot controller. Mecanum-wheel chassis (4× M3508 + C620 on CAN1), 4× servos (TIM2/TIM5 PWM), 2× steppers (GPIO bit-bang). Single-key multi-click UI (PB2). No RTOS, pure super-loop.

## Build

```bash
cmake --preset Debug                          # configure
cmake --build build/Debug                     # build → build/Debug/Uniforest_A_0628.elf
```

Target: `STM32F427IIHx`, 180 MHz (HSE 12 MHz → PLL), GCC ARM toolchain. CubeMX generated base; manual peripheral init for timers/GPIO (see servo.c pattern).

## Pin Map

| Function | Pins |
|----------|------|
| CAN1 | PD0(RX), PD1(TX) — M3508 motors |
| CAN2 | PB12(RX), PB13(TX) — unused |
| DC24V Power | PH2–PH5 (output PP, set high to enable C620) |
| Servos (50Hz PWM) | PA0 (TIM5_CH1), PA1 (TIM2_CH2), PA2 (TIM2_CH3), PA3 (TIM2_CH4) |
| Steppers | PI0(PUL1), PH12(DIR1), PH11(ENA1); PH10(PUL2), PD15(DIR2), PD14(ENA2) |
| Stepper LEDs | PD13, PD12 (on external driver board, not A-board) |
| A-board LEDs | **PE11 (red), PF14 (green)** — active-low: `RESET`=ON, `SET`=OFF |
| Key | PB2, pull-down, pressed=HIGH |
| SWD | PA13, PA14 |
| IMU (planned) | PA15(CS), PB3(SCK), PB4(MISO), PB5(MOSI) — SPI1, Mode 3 |

## Source Layout

```
Core/Inc/    motor3508.h, servo.h, stepper.h, actions.h, imu.h*
Core/Src/    main.c, motor3508.c, servo.c, stepper.c, actions.c, imu.c*
Drivers/     STM32F4xx_HAL_Driver (CMSIS + HAL)
cmake/       gcc-arm-none-eabi.cmake, stm32cubemx/CMakeLists.txt
*.ioc        CubeMX project config
```

\* IMU files are on disk but not compiled — SPI not enabled in `stm32f4xx_hal_conf.h`. IMU-dependent functions in `motor3508.c` wrapped in `#if 0`.

## Key Architecture

### PID Framework (`motor3508.h/c`)
- Generic `PID_Init()` / `PID_Compute()` with anti-windup
- Per-motor `M3508_Motor_t` has 3 cascaded PID: pos → speed → current
- Speed PID used in demo (Kp=7, Ki=0.03); position PID available but unused

### Chassis Demo (key-press 1)
- `Motor3508_ChassisDemo()`: trapezoidal velocity profile (accel→cruise→decel→hold) via `_trapezoid_run()`
- Mecanum forward: TR(idx0)=- , TL(idx1)=+ , BL(idx2)=+ , BR(idx3)=-
- Uses speed PID only, no position closure, no IMU

### Chassis Constants
- Encoder: 8192 counts/rev, gear 19:1, wheel D=152mm
- `COUNTS_PER_CM` ≈ 3260 (derived from above)

### CAN Protocol (CAN1 @ 1 Mbps)
- TX ID `0x200`: 8 bytes, big-endian int16 pairs [m1, m2, m3, m4] torque (-16000~16000)
- RX IDs `0x201-0x204`: feedback {angle u16, speed i16, torque i16, temp u8}

### Main Loop (bare-metal, ~100 Hz)
1. `Motor3508_RxPoll()` — drain CAN FIFO
2. Key detection (rising edge + debounce + multi-click window 1500ms)
3. On window expiry → LED confirm → dispatch action
4. `HAL_Delay(10)`

## LED Behavior (Current)

| Event | Red (PE11) | Green (PF14) |
|-------|-----------|-------------|
| Power-on | Both blink 3× together (150ms) → off | ← same |
| Ready | off | Flash once (200ms) |
| Key press | off | Flash once (80ms) |
| Action confirm | off | Flash N times (N=click_count, 150ms) |

## IMU Integration Status

- `imu.h` / `imu.c` written (MPU6500 SPI + IST8310 I2C-master + Mahony AHRS) — **not yet working**
- WHO_AM_I read fails → LED diagnostic shows error code 1 (SPI communication issue)
- SPI HAL files copied from STM32CubeF4 V1.28.3 to `Drivers/`
- To re-enable: `#define HAL_SPI_MODULE_ENABLED` + add `imu.c` to CMakeLists + `#include "imu.h"` in main.c
- Yaw-corrected 1m demo (`Motor3508_ChassisDemo1M`) wrapped in `#if 0` in motor3508.c
