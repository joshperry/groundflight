# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

GroundFlight repurposes helicopter/drone flight controller hardware (RadioMaster Nexus) for RC car gyro stabilization. It's a bare-metal STM32F722 firmware that applies yaw rate stabilization to steering control using IMU feedback.

**Hardware:** RadioMaster Nexus (STM32F722RET6, 216MHz Cortex-M7, 256KB RAM, 512KB Flash)

**Current Status:** Passthrough with ARM safety interlock + SRXL2 telemetry working. Stabilizer implementation pending.

## Build & Development Commands

**Prerequisites:** Nix with flakes enabled. Run `nix develop` to enter the development shell with all tools.

### Build Commands
```bash
gf-build           # Configure and build (Debug)
gf-build Release   # Build with optimizations
gf-clean           # Clean build directory
```

### Flashing Commands
```bash
gf-upload          # One-shot: send 'dfu' command, wait, then flash
gf-dfu             # Manual flash via USB DFU (shows instructions)
```

**DFU mode entry:**
- From running firmware: Type `dfu` in CLI
- Hardware method: Hold BOOT/BIND button, connect USB, release after 1s

### Serial/Debug Commands
```bash
gf-monitor         # Serial monitor via USB CDC (/dev/ttyACM0)
gf-debug           # Start OpenOCD GDB server on :3333
gf-reset           # Reset target via ST-Link
```

### CLI Commands (in firmware)
```
help        - List all commands
status      - Show system status (IMU, CRSF, ESC, ARM state)
esc         - Show SRXL2 telemetry (RPM, voltage, current, temp)
pass        - Monitor passthrough and arm state
gyro        - Show live gyro data
crsf        - Show live CRSF channel data
dfu         - Reboot to DFU bootloader
```

## Architecture Overview

### Core Data Flow
```
CRSF RX (UART4) → ARM Check (CH5) → Mixer → PWM Outputs
                         ↓
                    (when armed)
                         ↓
    CH1 → Steering servo (S1)
    CH3 → Motor ESC (ESC header, SRXL2 bidirectional)
    CH4 → E-brake servo (S3)

IMU (SPI1) → Stabilizer (future) → Correction added to steering
ESC Telemetry (SRXL2) → Speed estimate → Gain scheduling (future)
```

### Module Organization
```
src/
├── main.c                      # Main loop, CLI, ARM safety logic
├── drivers/
│   ├── icm42688.c/h           # IMU driver (ICM-42688-P via SPI)
│   ├── crsf.c/h               # CRSF protocol parser (ELRS receiver input)
│   ├── uart.c/h               # Interrupt-driven UART abstraction
│   ├── pwm.c/h                # Servo PWM output (TIM2/3/4)
│   ├── spi.c/h                # SPI bus driver
│   ├── esc.c/h                # ESC abstraction layer
│   └── srxl2.c/h              # SRXL2 protocol for Spektrum Smart ESCs
├── flight/
│   ├── gyro.c/h               # Gyro filtering and calibration
│   ├── stabilizer.c/h         # Yaw rate stabilization (stub)
│   ├── mixer.c/h              # Channel mixing (stub)
│   └── speed.c/h              # Speed estimation from ESC RPM
├── config/
│   ├── config.c/h             # Runtime configuration
│   ├── cli.c/h                # Serial CLI for tuning
│   └── eeprom.c/h             # Config persistence
├── usb/
│   └── usb_cdc.c + support    # USB CDC virtual COM port
└── target/nexus/
    ├── target.c               # Clock init, DFU, LED, pin mappings
    └── target.h
```

## Key Implementation Details

### ARM Safety Interlock
**Critical safety feature:** All outputs locked to 1500µs (neutral) until explicitly armed.

**To ARM:**
- Throttle must be at neutral (1400-1600µs)
- Flip CH5 switch high (>1700µs)

**DISARM (any of):**
- Flip CH5 low (<1300µs)
- Signal loss (CRSF failsafe)

**When disarmed:** All outputs = 1500µs. This IS the failsafe state.

### Car ESC Critical Detail
**Car ESCs use 1500µs as NEUTRAL (stopped), NOT 1000µs.**
- 1000µs = full reverse (or full brake)
- 1500µs = stopped
- 2000µs = full forward

All failsafe/disarmed states MUST output 1500µs. See TESTING.md for the bug history.

### SRXL2 Protocol (ESC Telemetry)
Bidirectional protocol for Spektrum Smart ESCs providing RPM, voltage, current, temperature.

**Physical layer:**
- Half-duplex UART on single wire (PB6, shared with PWM output)
- 115200 baud (400k negotiable but ESC may not support)
- GPIO must be **push-pull**, NOT open-drain (weak pull-up causes framing errors)

**Byte order (critical):**
- SRXL2 channel data: little-endian
- CRC-16: big-endian
- Telemetry payload (X-Bus): big-endian

**Channel values:**
- 16-bit unsigned, lower 2 bits = 0
- 0 = full reverse, 32768 = neutral, 65532 = full forward
- Surface ESCs use Channel 0 for throttle

**Common pitfalls:**
- Open-drain GPIO causes slow rise times → framing errors
- Echo on half-duplex: we receive our own packets
- Car ESC reverse requires double-tap (brake → neutral → reverse)

### DFU Bootloader Entry
Uses RTC backup register to signal bootloader:
```c
RCC->APB1ENR |= RCC_APB1ENR_PWREN;
PWR->CR1 |= PWR_CR1_DBP;           // Enable backup domain access
RTC->BKP0R = 0xDEADBEEF;           // Magic value
NVIC_SystemReset();
```

### Pin Mappings (RadioMaster Nexus)

**Clocks:**
- SYSCLK = 216 MHz (HSE 8MHz → PLL)
- APB1 = 54 MHz (timers = 108 MHz)
- APB2 = 108 MHz

**LEDs:** PC14, PC15 (active low)

**IMU (SPI1):** ICM-42688-P
- PA4 = CS, PA5 = SCK, PA6 = MISO, PA7 = MOSI
- PA15 = EXTI (data ready interrupt)
- Mode 3 (CPOL=1, CPHA=1), 13.5 MHz

**UART4 (CRSF Receiver):**
- PA0 = RX (from receiver TX)
- PA1 = TX (for telemetry back to receiver)
- **Note:** TX/RX swap enabled in hardware - Nexus PCB routes PA0 to RX pad

**USART1 (SRXL2 ESC):**
- PB6 = TX/RX (half-duplex, single-wire bidirectional)
- Shared with ESC header PWM motor output

**PWM Outputs (50Hz):**
- PB4 = TIM3_CH1 - S1 (Steering servo)
- PB0 = TIM3_CH3 - S3 (E-brake servo)
- PB3 = TIM2_CH2 - S4 (Aux)
- PB6 = TIM4_CH1 - ESC Header (Motor in PWM mode)

### Timer Configuration
APB1 timer clock = 108MHz. For 50Hz servo PWM with 1µs resolution:
- Prescaler = 107 (108MHz / 108 = 1MHz)
- Period = 19999 (20ms = 50Hz)
- CCR = pulse width in microseconds

### Channel Mapping
| CRSF Channel | Function | Output |
|--------------|----------|--------|
| CH1 (Aileron) | Steering | S1 (PB4) |
| CH3 (Throttle) | Motor/ESC | ESC Header (PB6) |
| CH4 (Rudder) | E-brake | S3 (PB0) |
| CH5 (Aux1) | **ARM switch** | — |

## Stabilization Algorithm (Future Implementation)

Core insight: **Car yaw stabilization is tail rotor logic**, just slower.

```c
// Basic algorithm (to be implemented)
steer_cmd = crsf.channels[0]              // Driver input
gyro_yaw = imu.gyro_z                     // Measured yaw rate (deg/s)
speed_mph = esc_telem.speed               // From SRXL2 RPM

// Expected yaw rate based on steering and speed
expected_yaw_rate = steer_cmd * yaw_rate_scale * steering_sensitivity(speed_mph);

// PID control on error
yaw_error = expected_yaw_rate - gyro_yaw;
correction = Kp * yaw_error + Ki * integral + Kd * derivative;
correction *= gain_knob * speed_authority(speed_mph);

// Apply correction
servo_out = constrain(steer_cmd + correction, -1.0, 1.0);
```

**Speed-dependent tuning:**
- `steering_sensitivity(speed)`: Decreases with speed (understeer model)
- `speed_authority(speed)`: Increases with speed (stability matters more)

**Braking dynamics:**
- Increase correction gain during motor braking (rear gets light)
- Reduce gain during trail braking (intentional rotation)
- Phase 2: E-brake oversteer release on excessive yaw rate

## Development Workflow

1. **Make changes** in `src/`
2. **Build:** `gf-build` (or `gf-build Release`)
3. **Flash:** `gf-upload` (handles DFU entry automatically)
4. **Monitor:** `gf-monitor` to see CLI output
5. **Test:** Use CLI commands (`status`, `esc`, `pass`, etc.)

## Testing Philosophy

**Don't test assumptions. Test actual behavior.**

Unit tests can encode the same wrong assumptions as the code. The 1000µs vs 1500µs neutral bug would have passed unit tests if we had written them with the same assumption.

See TESTING.md for full test strategy (unit tests, hardware-in-loop, ESC-in-loop, full system).

## Important Development Notes

### When Adding New Features
- Always consider ARM safety state
- Test both armed and disarmed behavior
- Disarmed outputs MUST be 1500µs (neutral)
- Use `status` CLI command to verify state

### When Modifying Drivers
- UART is interrupt-driven, careful with buffers
- PWM outputs use hardware timers (TIM2/3/4)
- IMU runs at 1kHz via data-ready interrupt
- SRXL2 is half-duplex, handle echo correctly

### HAL Dependencies
Build requires these HAL modules (see CMakeLists.txt):
- stm32f7xx_hal_uart.c
- stm32f7xx_hal_tim.c
- stm32f7xx_hal_spi.c
- stm32f7xx_hal_pcd.c (USB)

Also needs `APBPrescTable` defined in target.c.

## Code Style & Conventions

- Pure C (C99), no C++
- Bare-metal, no RTOS
- HAL for peripheral init, direct register access where performance matters
- Snake_case for functions and variables
- UPPER_CASE for defines and constants
- Keep interrupt handlers minimal, defer work to main loop

## Git Conventions

- Always add Josh as co-author on commits (Ada is the committer):
  `Co-Authored-By: Joshua Perry <josh@6bit.com>`

## Future Milestones

| # | Milestone | Status |
|---|-----------|--------|
| 1-8 | Blink → ARM interlock | ✅ |
| 9 | **Stabilizer** | 🔲 P controller on yaw rate |
| 10 | Brake-aware gain | 🔲 Increase gain during braking |
| 11 | Tuning system | 🔲 Gain knob via aux channel |
| 12 | Config save | 🔲 Persist to flash |
| 13 | ESC telemetry | ✅ SRXL2 working |
| 14 | MSP protocol | 🔲 DVR arm signal, OSD |
| 15 | HIL testing | 🔲 Automated safety tests |

## References

- Hardware specs: README.md
- Architecture details: architecture.md
- Test strategy: TESTING.md
- SRXL2 spec: https://www.spektrumrc.com/ProdInfo/Files/SPM_SRXL2_Protocol.pdf
- CRSF protocol: https://github.com/crsf-wg/crsf/wiki
- Rotorflight (Nexus target): https://github.com/rotorflight/rotorflight-firmware
