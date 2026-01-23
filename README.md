# GroundFlight

RC Car Gyro Stabilizer firmware for the RadioMaster Nexus (STM32F722).

Repurposes helicopter FBL hardware for yaw stabilization on RC surface vehicles.

## Hardware

- **MCU**: STM32F722RET6 (216 MHz Cortex-M7)
- **IMU**: ICM-42688-P (6-axis, SPI)
- **Receiver**: ELRS via CRSF (UART4, 420kbaud)
- **ESC Telemetry**: SRXL2 (UART3, half-duplex)
- **CLI**: UART6 @ 115200

## Quick Start

### Enter Development Shell

```bash
nix develop
```

### Build

```bash
gf-build          # Configure and build (Debug)
gf-build Release  # Build release version
```

Or manually:

```bash
cmake -B build
cmake --build build
```

### Flash

**USB DFU (no programmer needed):**

1. Hold the BOOT/BIND button on the Nexus
2. Connect USB while holding the button
3. Release after 1 second
4. Run `gf-dfu`

```bash
gf-dfu                      # Flash build/groundflight.bin
gf-dfu path/to/file.bin     # Flash specific file
gf-dfu-list                 # Check if device is in DFU mode
```

**ST-Link (if you have one):**

Connect an ST-Link programmer to the Nexus SWD pads.

```bash
gf-flash                    # Flash build/groundflight.elf
gf-flash path/to/file.elf   # Flash specific file
```

### Debug

```bash
gf-debug  # Start OpenOCD GDB server on :3333
```

In another terminal:

```bash
arm-none-eabi-gdb -ex 'target remote :3333' build/groundflight.elf
```

### Serial CLI

```bash
gf-cli              # Connect to /dev/ttyUSB0
gf-cli /dev/ttyACM0 # Specify port
```

## Project Structure

```
groundflight/
├── flake.nix          # Nix development environment
├── CMakeLists.txt     # Build configuration
├── cmake/
│   └── arm-none-eabi.cmake  # Cross-compilation toolchain
├── linker/
│   └── STM32F722RETx_FLASH.ld
├── startup/
│   └── startup_stm32f722xx.s
├── lib/
│   ├── stubs/         # Minimal HAL stubs for initial build
│   └── STM32CubeF7/   # (git clone separately)
└── src/
    ├── main.c
    ├── drivers/       # Hardware abstraction
    ├── flight/        # Control algorithms
    ├── config/        # Configuration & CLI
    └── target/nexus/  # Board-specific code
```

## Setting Up STM32Cube HAL

The HAL is provided automatically via the Nix flake input. Just use `nix develop` and you're set.

If building outside of Nix, clone manually:

```bash
git clone --depth 1 https://github.com/STMicroelectronics/STM32CubeF7.git lib/STM32CubeF7
```

## Development Milestones

1. ✅ **Blink** - LED blinks, basic build working
2. ⬜ **Serial** - CLI over UART6
3. ⬜ **IMU** - Read ICM-42688-P gyro
4. ⬜ **CRSF** - Parse ELRS receiver
5. ⬜ **PWM** - Steering servo output
6. ⬜ **E-brake** - E-brake servo passthrough
7. ⬜ **ESC basic** - Throttle to ESC (PWM)
8. ⬜ **Stab basic** - Close the loop, P controller
9. ⬜ **Brake aware** - Gain scheduling during braking
10. ⬜ **Tune** - Gain knob, CLI tuning
11. ⬜ **Save** - Persist config to flash
12. ⬜ **ESC telem** - SRXL2 backend, speed from RPM

## License

MIT
