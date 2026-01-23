# GroundFlight

RC car yaw stabilization firmware for the RadioMaster Nexus flight controller.

**Status:** Passthrough working — Blink ✅ | USB CLI ✅ | IMU ✅ | CRSF ✅ | PWM ✅ | Stabilizer 🔲

## What is this?

GroundFlight repurposes helicopter/drone flight controller hardware for RC car gyro stabilization. Think of it as a "gyro" unit (like Yokomo/Yeah Racing units) but running on more capable hardware with full configurability.

The core idea: use the same control algorithms that keep helicopter tail rotors locked in place to counteract yaw (oversteer/understeer) in RC cars.

## Hardware

**Target:** [RadioMaster Nexus](https://www.radiomasterrc.com/products/nexus-flybarless-controller) FBL Controller

| Component | Spec |
|-----------|------|
| MCU | STM32F722RET6 (216MHz Cortex-M7, 256KB RAM, 512KB Flash) |
| IMU | ICM-42688-P (6-axis, ±2000°/s @ 1kHz) |
| Flash | W25N01G (128MB, for blackbox logging) |
| Receiver | CRSF/ELRS via UART4 (420kbaud) |
| ESC Telemetry | SRXL2 via UART3 (for RPM-based speed estimation) |
| Outputs | 5x PWM (steering, throttle, e-brake, aux, motor) |
| CLI | USB CDC (Virtual COM port) |

### Pin Mapping

From Rotorflight target configuration:

```
Clocks:
  SYSCLK = 216 MHz (HSE 8MHz → PLL)
  APB1 = 54 MHz (timers = 108 MHz)
  APB2 = 108 MHz
  USB = 48 MHz (PLL_Q = 9)

LEDs:
  PC14, PC15 (active low)

SPI1 (IMU - ICM-42688-P):
  PA4  = CS
  PA5  = SCK
  PA6  = MISO
  PA7  = MOSI
  PA15 = EXTI (data ready interrupt)
  Mode 3 (CPOL=1, CPHA=1), 13.5 MHz
  WHO_AM_I = 0x47

SPI2 (Flash - W25N01G):
  PB12 = CS
  PB13 = SCK
  PB14 = MISO
  PB15 = MOSI

UART3 (ESC Telemetry):
  PB10 = RX
  PB11 = TX

UART4 (CRSF Receiver):
  PA0 = RX    (directly from receiver TX)
  PA1 = TX    (for telemetry back to receiver)
  Note: TX/RX swap enabled in hardware - Nexus PCB routes
        PA0 to the RX pad, PA1 to TX pad

UART6 (Spare):
  PC6 = RX
  PC7 = TX

PWM Outputs (50Hz, 1µs resolution):
  PB4 = TIM3_CH1 - Servo 1 (Steering)
  PB5 = TIM3_CH2 - Servo 2 (Throttle)
  PB0 = TIM3_CH3 - Servo 3 (E-brake)
  PB3 = TIM2_CH2 - Servo 4 (Aux)
  PB6 = TIM4_CH1 - Motor 1 (ESC)
```

### Nexus Port Pinout

**Port A (UART4 - CRSF):** Connect ELRS receiver here
- Pin 1: GND
- Pin 2: 5V
- Pin 3: RX (to receiver TX)
- Pin 4: TX (to receiver RX, for telemetry)

## Quick Start

### Prerequisites

- [Nix](https://nixos.org/download.html) with flakes enabled
- USB access to the Nexus (add udev rules for `plugdev` group)

### Enter Development Shell

```bash
nix develop
```

This provides: ARM toolchain, CMake, dfu-util, picocom, and STM32CubeF7 HAL.

### Build

```bash
gf-build          # Configure and build (Debug)
gf-build Release  # Build with optimizations
```

### Flash

**First time (or recovery) — Hardware DFU mode:**
1. Disconnect USB
2. Hold BOOT/BIND button on Nexus
3. Connect USB while holding button
4. Release after 1 second
5. Run `gf-upload`

**Subsequent flashes — From running firmware:**
```bash
gf-upload   # Sends 'dfu' command automatically, then flashes
```

**Or manually:**
```bash
> dfu                    # In CLI, reboot to bootloader
$ gf-dfu                 # Flash the binary
```

### Connect to CLI

```bash
gf-monitor   # USB serial monitor (Ctrl-C to exit)
```

## CLI Commands

```
> help
Commands:
  help      - This help
  status    - Show system status
  version   - Show firmware version
  gyro      - Show live gyro data (any key to stop)
  gyroraw   - Show raw gyro values
  cal       - Calibrate gyro (keep device still!)
  crsf      - Show live CRSF channel data
  servo N P - Set servo N (0-4) to pulse P (1000-2000)
  pass      - Monitor passthrough (always active)
  dfu       - Reboot to DFU bootloader
  reboot    - Reboot system
```

### Example Session

```
> status
GroundFlight Status:
  IMU:      ICM-42688-P (WHO_AM_I=0x47)
  CRSF:     Connected (1234 frames, 0 errors)
  Link:     RSSI=-65dBm LQ=100%
  ESC:      Not initialized
  Clock:    216 MHz
  Uptime:   12345 ms

> crsf
Live CRSF data (any key to stop):
CH:  1     2     3     4     5     6     7     8
     992  992  1500   992   172  1811   992   992  LQ=100%

> cal
Calibrating gyro - keep device still...
Calibration complete.
Bias: X=-0.12 Y=0.08 Z=-0.03 dps
```

## Operating Modes

### Passthrough Mode (Current Default)

CRSF channels map directly to PWM outputs:
- CH1 (Aileron) → Steering servo
- CH2 (Elevator) → Aux servo
- CH3 (Throttle) → Throttle/ESC
- CH4 (Rudder) → E-brake servo

Failsafe: All servos center on signal loss.

### Stabilized Mode (Coming Soon)

Gyro feedback applied to steering:
```
steering_out = steering_in + (gyro_yaw * gain)
```

## Development Milestones

| # | Milestone | Status | Notes |
|---|-----------|--------|-------|
| 1 | Blink | ✅ | PC14 LED, active low |
| 2 | USB CDC CLI | ✅ | Virtual COM port, no UART needed |
| 3 | IMU | ✅ | ICM-42688-P, ±2000°/s @ 1kHz |
| 4 | CRSF | ✅ | 420kbaud, interrupt-driven, 0 CRC errors |
| 5 | PWM | ✅ | TIM2/3/4, 50Hz, 1µs resolution |
| 6 | E-brake | ✅ | Passthrough on CH4 |
| 7 | ESC throttle | ✅ | Passthrough on CH3 |
| 8 | **Stabilizer** | 🔲 | P controller on yaw rate |
| 9 | Brake-aware | 🔲 | Reduce gain during braking |
| 10 | Tuning | 🔲 | Gain knob via aux channel, CLI params |
| 11 | Config save | 🔲 | Persist to flash |
| 12 | ESC telemetry | 🔲 | SRXL2 for speed-from-RPM |

## Architecture

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│  CRSF RX    │────▶│   Mixer     │────▶│  PWM Out    │
│  (steering, │     │             │     │  (servos)   │
│   throttle) │     └──────┬──────┘     └─────────────┘
└─────────────┘            │
                           │ correction
                    ┌──────┴──────┐
                    │ Stabilizer  │◀──── Gyro Z (yaw rate)
                    │  (PI/PID)   │
                    └──────┬──────┘
                           │
                    ┌──────┴──────┐
                    │ Speed Est.  │◀──── ESC telemetry (RPM)
                    │ (gain sched)│
                    └─────────────┘
```

The stabilizer applies yaw correction to steering based on gyro rate. At higher speeds (from ESC telemetry), correction gain is reduced to prevent overcorrection.

## Tested Hardware

- **Flight Controller:** RadioMaster Nexus
- **Receiver:** RadioMaster RP3-H (ELRS 2.4GHz)
- **Transmitter:** Jumper T-20 (ELRS), RadioMaster TX16S
- **Also compatible:** Any ELRS receiver with CRSF output

## Project Structure

```
groundflight/
├── flake.nix              # Nix dev environment & STM32CubeF7
├── CMakeLists.txt         # Build configuration
├── cmake/
│   └── arm-none-eabi.cmake
├── linker/
│   └── STM32F722RETx_FLASH.ld
├── startup/
│   └── startup_stm32f722xx.s
├── lib/
│   └── stubs/             # Minimal stubs for non-Nix builds
└── src/
    ├── main.c             # Main loop, CLI commands
    ├── stm32f7xx_hal_conf.h
    ├── drivers/
    │   ├── icm42688.c/h   # IMU driver
    │   ├── crsf.c/h       # CRSF protocol parser
    │   ├── uart.c/h       # Interrupt-driven UART
    │   ├── pwm.c/h        # Servo PWM output
    │   ├── spi.c/h        # SPI bus driver
    │   └── esc_*.c/h      # ESC telemetry (stub)
    ├── flight/
    │   ├── stabilizer.c/h # Yaw stabilization (stub)
    │   ├── mixer.c/h      # Channel mixing (stub)
    │   └── gyro.c/h       # Gyro processing (stub)
    ├── config/
    │   └── *.c/h          # Configuration (stub)
    ├── usb/
    │   └── *.c/h          # USB CDC implementation
    └── target/nexus/
        ├── target.c       # Clock init, DFU, LED
        └── target.h
```

## Technical Notes

### DFU Bootloader Entry

Uses RTC backup register to signal bootloader:
```c
RCC->APB1ENR |= RCC_APB1ENR_PWREN;
PWR->CR1 |= PWR_CR1_DBP;  // Enable backup domain access
RTC->BKP0R = 0xDEADBEEF;  // Magic value
NVIC_SystemReset();       // System bootloader checks BKP0R
```

### UART4 Pin Swap

The Nexus PCB routes UART4 pins opposite to STM32 defaults. Enable hardware swap:
```c
huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
huart4.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
```

### Timer Configuration

APB1 timer clock = 108MHz. For 50Hz servo PWM with 1µs resolution:
- Prescaler = 107 (108MHz / 108 = 1MHz)
- Period = 19999 (20ms = 50Hz)
- CCR = pulse width in microseconds

### HAL Dependencies

The build requires these HAL modules (added to CMakeLists.txt):
- stm32f7xx_hal_uart.c (CRSF)
- stm32f7xx_hal_tim.c (PWM)
- stm32f7xx_hal_spi.c (IMU)
- stm32f7xx_hal_pcd.c (USB)

Also needs `APBPrescTable` defined (in target.c):
```c
const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};
```

## References

- [ICM-42688-P Datasheet](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/)
- [CRSF Protocol Wiki](https://github.com/crsf-wg/crsf/wiki)
- [SRXL2 Protocol Spec](https://www.spektrumrc.com/ProdInfo/Files/SPM_SRXL2_Protocol.pdf)
- [Rotorflight Source](https://github.com/rotorflight/rotorflight-firmware) (Nexus target reference)
- [Betaflight Source](https://github.com/betaflight/betaflight)

## License

MIT
