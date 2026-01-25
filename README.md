# GroundFlight

RC car yaw stabilization firmware for the RadioMaster Nexus flight controller.

**Status:** Passthrough with ARM — Blink ✅ | USB CLI ✅ | IMU ✅ | CRSF ✅ | PWM ✅ | ARM ✅ | Stabilizer 🔲

## What is this?

GroundFlight repurposes helicopter/drone flight controller hardware for RC car gyro stabilization. Think of it as a "gyro" unit (like Yokomo/Yeah Racing units) but running on more capable hardware with full configurability.

The core idea: use the same control algorithms that keep helicopter tail rotors locked in place to counteract yaw (oversteer/understeer) in RC cars.

## Hardware

**Target:** [RadioMaster Nexus (discontinued)](https://www.rotorflight.org/docs/controllers/rm-nexus) FBL Controller

| Component | Spec |
|-----------|------|
| MCU | STM32F722RET6 (216MHz Cortex-M7, 256KB RAM, 512KB Flash) |
| IMU | ICM-42688-P (6-axis, ±2000°/s @ 1kHz) |
| Flash | W25N01G (128MB, for blackbox logging) |
| Receiver | CRSF/ELRS via UART4 (420kbaud) |
| ESC Telemetry | SRXL2 via UART3 (for RPM-based speed estimation) |
| Outputs | 5x PWM (steering, e-brake, aux on servo headers; motor on ESC header) |
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

UART3 (ESC Telemetry - SRXL2):
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
  PB4 = TIM3_CH1 - S1 (Steering)
  PB5 = TIM3_CH2 - S2 (unused)
  PB0 = TIM3_CH3 - S3 (E-brake)
  PB3 = TIM2_CH2 - S4 (Aux)
  PB6 = TIM4_CH1 - ESC Header (Motor/Throttle)
```

### Nexus Connector Pinout

**Port A (UART4 - CRSF):** Connect ELRS receiver here
- Pin 1: GND
- Pin 2: 5V
- Pin 3: RX (to receiver TX)
- Pin 4: TX (to receiver RX, for telemetry)

**ESC Header:** Connect ESC signal wire here (throttle output)

**S1/S2/S3 Servo Headers:** Connect steering servo to S1, e-brake to S3

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

## Channel Mapping

| CRSF Channel | Function | Output |
|--------------|----------|--------|
| CH1 (Aileron) | Steering | S1 (PB4) |
| CH3 (Throttle) | Motor/ESC | ESC Header (PB6) |
| CH4 (Rudder) | E-brake servo | S3 (PB0) |
| CH5 (Aux1) | **ARM switch** | — |

**ARM is required for outputs to respond.** See Operating Modes below.

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
  pass      - Monitor passthrough and arm state
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
  Armed:    NO (flip CH5 with throttle neutral to arm)
  Clock:    216 MHz
  Uptime:   12345 ms

> pass
Passthrough monitor (any key to exit):
  CH5=Arm  CH1->Steering  CH3->Motor  CH4->Ebrake

[DISARMED] Steer:1500 Motor:1500 Brake:1500 LQ=100%
[ARMED]    Steer:1523 Motor:1500 Brake:1500 LQ=100%
```

## Operating Modes

### ARM Safety Interlock

The car will not respond to controls until armed. This prevents:
- Runaway on power-up
- Unexpected movement if transmitter is bumped
- Movement during signal loss

**To ARM:**
1. Throttle must be at neutral (1400-1600µs)
2. Flip CH5 switch high (>1700µs)

**To DISARM (any of these):**
- Flip CH5 switch low (<1300µs)
- Signal loss (CRSF failsafe)

**When disarmed:** All outputs locked to neutral (1500µs). This IS the failsafe state — there's no separate failsafe behavior.

### Passthrough Mode (Current)

When armed, CRSF channels map directly to PWM outputs:
- CH1 → Steering servo (S1)
- CH3 → Motor ESC (ESC header)
- CH4 → E-brake servo (S3)

### Stabilized Mode (Coming Soon)

Gyro feedback applied to steering:
```
steering_out = steering_in + (gyro_yaw * gain)
```

## Development Milestones

| # | Milestone | Status | Notes |
|---|-----------|--------|-------|
| 1 | Blink | ✅ | PC14 LED, active low |
| 2 | USB CDC CLI | ✅ | Virtual COM port |
| 3 | IMU | ✅ | ICM-42688-P, ±2000°/s @ 1kHz |
| 4 | CRSF | ✅ | 420kbaud, interrupt-driven |
| 5 | PWM | ✅ | TIM2/3/4, 50Hz, 1µs resolution |
| 6 | E-brake passthrough | ✅ | CH4 → S3 |
| 7 | ESC throttle | ✅ | CH3 → ESC header |
| 8 | ARM interlock | ✅ | CH5, throttle-neutral-to-arm |
| 9 | **Stabilizer** | 🔲 | P controller on yaw rate |
| 10 | Brake-aware | 🔲 | Reduce gain during braking |
| 11 | Tuning | 🔲 | Gain knob via aux channel, CLI params |
| 12 | Config save | 🔲 | Persist to flash |
| 13 | ESC telemetry | 🔲 | SRXL2 for speed-from-RPM |
| 14 | MSP protocol | 🔲 | DVR arm signal, OSD |
| 15 | HIL testing | 🔲 | Automated safety regression |

## Architecture

```
                              ┌─────────────┐
                              │  ARM Check  │◀──── CH5 switch
                              └──────┬──────┘
                                     │ armed?
        ┌─────────────┐              ▼
        │  CRSF RX    │────▶ ┌─────────────┐     ┌─────────────┐
        │  (channels) │      │   Mixer     │────▶│  PWM Out    │
        └─────────────┘      │             │     │  (servos)   │
                             └──────┬──────┘     └─────────────┘
                                    │
                                    │ correction (when stabilizer enabled)
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

The stabilizer will apply yaw correction to steering based on gyro rate. At higher speeds (from ESC telemetry), correction gain is reduced to prevent overcorrection.

## Safety

See [TESTING.md](TESTING.md) for the full testing strategy.

**Key safety properties:**
- Outputs are neutral (1500µs) until explicitly armed
- Disarm is immediate and unconditional
- Signal loss = automatic disarm
- ARM requires throttle at neutral (prevents arming while throttle is up)

**Learned the hard way:** Car ESCs use 1500µs as neutral, not 1000µs. An early bug set failsafe to 1000µs (full reverse). This is why we need hardware-in-loop testing — you can't unit test your assumptions.

## Tested Hardware

- **Flight Controller:** RadioMaster Nexus
- **Receiver:** RadioMaster RP3-H (ELRS 2.4GHz)
- **Transmitter:** Jumper T-20 (ELRS), RadioMaster TX16S
- **ESC:** Spektrum Firma 150A Smart ESC
- **Vehicle:** Arrma Infraction 6S
- **Also compatible:** Any ELRS receiver with CRSF output

## Project Structure

```
groundflight/
├── flake.nix              # Nix dev environment & STM32CubeF7
├── CMakeLists.txt         # Build configuration
├── TESTING.md             # Test strategy documentation
├── cmake/
│   └── arm-none-eabi.cmake
├── linker/
│   └── STM32F722RETx_FLASH.ld
├── startup/
│   └── startup_stm32f722xx.s
├── lib/
│   └── stubs/             # Minimal stubs for non-Nix builds
└── src/
    ├── main.c             # Main loop, CLI, ARM logic
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

### Car ESC Neutral

**Important:** Car ESCs use 1500µs as neutral (stopped), not 1000µs.
- 1000µs = full reverse (or full brake, depending on ESC mode)
- 1500µs = stopped
- 2000µs = full forward

All safe/failsafe/disarmed states must output 1500µs to the ESC.

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
