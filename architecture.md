# GroundFlight - Minimal RC Car Gyro Stabilizer

## Target Hardware (RadioMaster Nexus)

```
MCU:        STM32F722RET6 (216 MHz Cortex-M7, 256KB RAM, 512KB Flash)
IMU:        ICM-42688-P (SPI)
Flash:      W25N01GVZEIG 128MB (for blackbox, optional)
Baro:       SPL06-001 (unused for car)
UARTs:      6 total
  - UART1:  DSM (unused)
  - UART2:  S.BUS input (unused)
  - UART4:  CRSF input (ELRS receiver) ← control link
  - UART3:  PORT-C → ESC telemetry (protocol selectable)
  - UART6:  PORT-B (debug/CLI)
Outputs:    4 servo/PWM
  - S1:     Steering servo
  - S2:     ESC throttle (PWM fallback, or unused if serial protocol)
  - S3:     E-brake servo
  - TAIL:   Aux (lights, etc)
Voltage:    5-12.6V input
```

## ESC Telemetry Abstraction

Keep the interface protocol-fluid - the stabilizer just wants speed/RPM, doesn't care how it arrives.

```c
// Protocol-agnostic telemetry interface
typedef struct {
    uint32_t rpm;              // Motor electrical RPM
    float    voltage;          // Pack voltage
    float    current;          // Motor current (amps)
    float    temperature;      // ESC temp (°C)
    uint32_t mah_consumed;     // Capacity used
    uint32_t last_update_ms;
    bool     valid;
} esc_telemetry_t;

// Protocol backends
typedef enum {
    ESC_PROTO_NONE,            // No telemetry, estimate from throttle
    ESC_PROTO_SRXL2,           // Spektrum Smart (Firma, Avian)
    ESC_PROTO_KISS,            // KISS/BLHeli32 telemetry
    ESC_PROTO_HOBBYWING,       // Hobbywing V5 telemetry
    ESC_PROTO_CASTLE,          // Castle Link live
} esc_protocol_t;

// The stabilizer calls this, doesn't care about protocol
float esc_get_speed_mph(void);
bool  esc_telemetry_valid(void);
```

### Supported Protocols (Phase 1)

| Protocol | ESC Examples | Wire | Notes |
|----------|--------------|------|-------|
| SRXL2 | Spektrum Firma/Avian | Half-duplex UART | MIT reference code available |
| PWM only | Any | Servo signal | Fallback, estimate speed from throttle |

### Future Protocol Candidates

| Protocol | ESC Examples | Notes |
|----------|--------------|-------|
| KISS/BLHeli32 | Most FPV ESCs | Single-wire telemetry |
| Hobbywing V5 | HW XeRun, QuicRun | Proprietary but documented |
| Castle Link | Castle Creations | Needs level shifter |

## Core Data Flow

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           Main Loop (~1kHz)                             │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
     ┌──────────────┬───────────────┼───────────────┬──────────────┐
     ▼              ▼               ▼               ▼              ▼
┌─────────┐  ┌───────────┐  ┌─────────────┐  ┌───────────┐  ┌───────────┐
│  CRSF   │  │    IMU    │  │ ESC Telem   │  │ Stabilizer│  │  Outputs  │
│ (UART4) │  │   (SPI)   │  │  (UART3)    │  │           │  │           │
├─────────┤  ├───────────┤  ├─────────────┤  ├───────────┤  ├───────────┤
│ steer   │  │ gyro_yaw  │  │ rpm         │  │ steer_in  │  │ S1: steer │
│ throt   │  │ gyro_pitch│  │ voltage     │  │ + correct │  │ S2: throt │
│ gain    │  │ accel_x   │  │ current     │  │ = output  │  │ S3: ebrake│
│ mode    │  │ accel_y   │  │ temp        │  │           │  │ S4: aux   │
│ ebrake  │  │           │  │             │  │           │  │           │
└────┬────┘  └─────┬─────┘  └──────┬──────┘  └─────┬─────┘  └─────┬─────┘
     │             │               │               │              │
     └─────────────┴───────────────┴───────────────┴──────────────┘
                                   │
                          ┌────────┴────────┐
                          │  Speed Estimate │
                          │  rpm → mph      │
                          └─────────────────┘
```

## Stabilization Algorithm

The core insight: **car yaw stabilization is tail rotor logic**, just slower.

```c
// Inputs (every loop iteration)
steer_cmd     = crsf.channels[0]     // -1.0 to 1.0, driver input
throttle_cmd  = crsf.channels[1]     // -1.0 to 1.0, sent to ESC via SRXL2
gain_knob     = crsf.channels[2]     //  0.0 to 1.0, adjustable
mode_switch   = crsf.channels[3]     //  0=off, 1=normal, 2=drift?

gyro_yaw      = imu.gyro_z           // deg/s, filtered
speed_mph     = esc_telem.speed      // from SRXL2 RPM → computed speed

// The magic
if (mode == MODE_OFF) {
    correction = 0;
} else {
    // Setpoint: expected yaw rate based on steering input and speed
    // At higher speeds, same steering angle produces less yaw rate
    expected_yaw_rate = steer_cmd * yaw_rate_scale * steering_sensitivity(speed_mph);
    
    // Error: difference between expected and actual
    yaw_error = expected_yaw_rate - gyro_yaw;
    
    // PID (though P-only might suffice for cars)
    correction = Kp * yaw_error + Ki * integral(yaw_error) + Kd * derivative(yaw_error);
    
    // Scale by user gain AND speed-based authority
    correction *= gain_knob * speed_authority(speed_mph);
}

// Output
servo_out = steer_cmd + correction;
servo_out = constrain(servo_out, -1.0, 1.0);
```

### Speed-dependent tuning functions

```c
// Steering sensitivity decreases with speed (understeer model)
float steering_sensitivity(float speed_mph) {
    // At 0 mph: 1.0 (full sensitivity)
    // At 60 mph: 0.3 (reduced due to tire slip angle physics)
    return lerp(1.0, 0.3, clamp(speed_mph / 60.0, 0.0, 1.0));
}

// Correction authority increases with speed
float speed_authority(float speed_mph) {
    // At 0 mph: 0.2 (minimal intervention)
    // At 60 mph: 1.0 (full authority, stability matters)
    return lerp(0.2, 1.0, clamp(speed_mph / 60.0, 0.0, 1.0));
}
```

### Key differences from helicopter tail:

| Aspect | Helicopter Tail | Car Steering |
|--------|-----------------|--------------|
| Actuator | Direct thrust (tail rotor) | Geometry (Ackermann) |
| Response | ~50ms | ~100-200ms (servo + tire slip) |
| Setpoint | Heading hold or rate | Rate follows stick |
| Speed coupling | Collective affects tail authority | Speed affects tire grip |
| Failure mode | Spin | Oversteer/understeer |

## Braking Dynamics

The Infraction has two braking systems:
1. **Motor braking** - ESC regenerative/dynamic braking
2. **E-brake servo** - Mechanical rear brake

This gives us interesting dynamics estimation and control opportunities.

### Observable signals during braking

```c
typedef struct {
    // From ESC telemetry
    float motor_rpm;          // Decaying during brake
    float motor_current;      // Negative during regen
    
    // From IMU
    float accel_x;            // Longitudinal decel (negative = braking)
    float gyro_pitch;         // Weight transfer (nose dives)
    float gyro_yaw;           // Stability (rear stepping out)
    
    // Commanded
    float throttle_cmd;       // Negative = brake request
    float ebrake_cmd;         // E-brake servo position
} braking_state_t;
```

### What we can estimate

```c
// Expected deceleration from brake command
float expected_decel = brake_cmd * max_decel_capability;

// Actual deceleration from accelerometer (filtered)
float actual_decel = -accel_x_filtered;

// Grip estimate: ratio of actual to expected
// < 1.0 means wheels are slipping (or about to)
float grip_estimate = actual_decel / expected_decel;

// RPM-based wheel slip detection
// If motor RPM drops faster than vehicle is decelerating, wheels locked
float expected_rpm_decay = (actual_decel / tire_circumference) * gear_ratio;
float actual_rpm_decay = (prev_rpm - current_rpm) / dt;
bool wheels_locking = (actual_rpm_decay > expected_rpm_decay * 1.5);
```

### Brake stability observations

```c
// During braking, rear gets light due to weight transfer
// Pitch rate indicates weight transfer magnitude
float weight_transfer_indicator = gyro_pitch;  // Positive = nose diving

// Yaw during braking without steering input = instability
bool unexpected_yaw = is_braking && 
                      (fabsf(steer_cmd) < 0.1f) && 
                      (fabsf(gyro_yaw) > yaw_threshold);

if (unexpected_yaw) {
    // Something is causing rotation we didn't ask for
    // Increase stability correction
}
```

### E-brake servo control

```c
// E-brake channel from TX (momentary button mapped to channel)
float ebrake_request = crsf.channels[CH_EBRAKE];

// Phase 1: Direct passthrough
// The slow servo travel already provides feathering capability
float ebrake_output = ebrake_request;

servo_set(SERVO_EBRAKE, ebrake_output);

// Note: The physical brake is a shoed disk on the center driveline
// Servo is intentionally slower than max speed for controllability
```

### E-brake Configuration

```c
// E-brake is passthrough in Phase 1
// Phase 2 adds oversteer-release logic

// Passthrough with optional rate limiting
float ebrake_output = ebrake_request;

// Phase 2: Release on excessive yaw rate
if (ebrake_oversteer_release_enabled) {
    if (fabsf(gyro_yaw) > ebrake_release_threshold) {
        // Car is spinning, release the brake to regain control
        ebrake_output = 0;
    }
}

servo_set(SERVO_EBRAKE, ebrake_output);
```

### Phase 1 vs Phase 2

**Phase 1 (MVP):**
- E-brake servo passthrough
- Motor braking with stability-aware gain adjustment
- Log braking telemetry for analysis

**Phase 2:**
- E-brake oversteer release
- Traction control under acceleration
- Weight transfer compensation

## Motor Braking Control (Phase 1)

Motor braking happens when throttle goes negative - the ESC does the actual work. What we control is the stability response during braking.

### The physics

```
Braking → weight transfers forward → rear gets light → less rear grip → oversteer tendency

At the same time:
- Front tires have MORE grip (weight on them)
- Steering authority increases
- But correction needs to be faster (less rear grip = quicker rotation)
```

### Stability gain adjustment during braking

```c
// Detect braking state
bool is_braking = (throttle_cmd < -0.1f) && (speed_mph > 5.0f);

// During braking, the rear is light and will step out easier
// Increase correction authority and responsiveness
float brake_gain_boost = 1.0f;
if (is_braking) {
    // Scale boost by brake intensity
    float brake_intensity = fabsf(throttle_cmd);  // 0.0 to 1.0
    brake_gain_boost = lerp(1.0f, 1.5f, brake_intensity);
    
    // Could also reduce D-term delay since we need faster response
}

correction *= brake_gain_boost;
```

### Brake-induced yaw detection

```c
// If we're braking and yaw rate is climbing, something's wrong
// Either:
//   1. Surface is slippery (one side has less grip)
//   2. Rear is locking up
//   3. Driver is turning while braking (trail braking - intentional)

float yaw_rate = gyro_yaw;
float yaw_acceleration = (yaw_rate - prev_yaw_rate) / dt;

// High yaw acceleration during braking = instability
if (is_braking && fabsf(yaw_acceleration) > yaw_accel_threshold) {
    // Increase correction aggressiveness
    // In Phase 2: consider modulating brake
}
```

### Current-based brake effort estimation

```c
// ESC telemetry gives us current during braking
// Negative current = regenerative braking = deceleration force

float brake_current = esc_telem.current;  // Negative when braking
float expected_decel = brake_current * current_to_decel_scale;

// Compare to actual decel from accelerometer
float actual_decel = -accel_x_filtered;

// If actual < expected, wheels might be slipping
// (motor is trying to brake harder than tires can handle)
float grip_ratio = actual_decel / expected_decel;
if (grip_ratio < 0.7f && is_braking) {
    // Low grip condition - be more aggressive with stability
    // In Phase 2: could reduce throttle brake command
}
```

### Trail braking awareness

```c
// Trail braking = braking while turning, intentional weight transfer
// We should NOT fight this, just stabilize it

bool trail_braking = is_braking && (fabsf(steer_cmd) > 0.2f);

if (trail_braking) {
    // Driver is intentionally rotating the car
    // Reduce counter-steer authority slightly to allow rotation
    // But still catch any snap oversteer
    correction *= 0.7f;
}
```

### Configuration

```c
typedef struct {
    // Motor braking behavior
    float    brake_gain_boost;        // Extra stability gain while braking (default: 1.5)
    float    brake_yaw_threshold;     // deg/s² to trigger enhanced stability
    float    trail_brake_reduction;   // Correction reduction during trail brake (default: 0.7)
    float    current_to_decel_scale;  // Calibration: amps → expected g
} brake_config_t;
```

## Module Breakdown

```
src/
├── main.c                 // Entry point, main loop
├── drivers/
│   ├── icm42688.c/.h      // IMU driver (from Betaflight, ~500 lines)
│   ├── crsf.c/.h          // CRSF parser (from Betaflight, ~400 lines)
│   ├── esc_telem.c/.h     // ESC telemetry abstraction (~200 lines)
│   ├── esc_srxl2.c/.h     // SRXL2 backend (from SpektrumRC, ~800 lines)
│   ├── esc_none.c/.h      // No-telemetry fallback (~50 lines)
│   ├── pwm.c/.h           // Servo output (~150 lines)
│   └── uart.c/.h          // UART abstraction (~200 lines)
├── flight/
│   ├── gyro.c/.h          // Filtering, calibration (~300 lines)
│   ├── speed.c/.h         // Speed from ESC telemetry (~100 lines)
│   ├── mixer.c/.h         // Input → output mapping (~150 lines)
│   ├── stabilizer.c/.h    // Steering control law (~200 lines)
│   └── braking.c/.h       // Brake state detection, gain adjustment (~200 lines)
├── config/
│   ├── config.c/.h        // Runtime config struct (~200 lines)
│   ├── cli.c/.h           // Serial CLI for tuning (~400 lines)
│   └── eeprom.c/.h        // Save/load config (~100 lines)
└── target/
    └── nexus/
        ├── target.h       // Pin definitions, clock config
        └── target.c       // Board-specific init
```

**Estimated total: ~3300 lines of C** (excluding HAL/CMSIS)

## Configuration Parameters

```c
typedef struct {
    // Gyro
    uint8_t  gyro_lpf_hz;          // Low-pass filter cutoff (default: 100)
    uint8_t  gyro_sample_rate_div; // Sample rate divider
    
    // Stabilizer
    float    kp;                   // Proportional gain (default: 0.5)
    float    ki;                   // Integral gain (default: 0.0)
    float    kd;                   // Derivative gain (default: 0.05)
    float    yaw_rate_scale;       // Expected deg/s per unit stick (default: 200)
    float    max_correction;       // Limit correction authority (default: 0.5)
    
    // Speed-based gain scheduling  
    float    low_speed_gain;       // Multiplier at low speed (default: 0.3)
    float    high_speed_gain;      // Multiplier at high speed (default: 1.0)
    float    speed_gain_max_mph;   // Speed at which high_speed_gain applies (default: 60)
    
    // Steering servo
    uint16_t steer_center_us;      // Center pulse width (default: 1500)
    uint16_t steer_range_us;       // +/- range (default: 500)
    int8_t   steer_direction;      // 1 or -1
    
    // E-brake servo
    uint16_t ebrake_min_us;        // Released position (default: 1000)
    uint16_t ebrake_max_us;        // Full engagement (default: 2000)
    int8_t   ebrake_direction;     // 1 or -1
    
    // ESC / Throttle
    uint8_t  esc_protocol;         // ESC_PROTO_* enum
    uint16_t esc_min_us;           // ESC min pulse (default: 1000)
    uint16_t esc_max_us;           // ESC max pulse (default: 2000)
    uint16_t esc_center_us;        // ESC neutral (default: 1500)
    
    // Vehicle geometry (for speed calculation)
    uint16_t motor_kv;             // Motor KV rating
    float    gear_ratio;           // Overall gear reduction
    uint16_t tire_diameter_mm;     // Tire OD in mm
    uint8_t  motor_poles;          // For eRPM → RPM conversion
    
    // Channel mapping
    uint8_t  ch_steering;          // CRSF channel for steering (default: 0)
    uint8_t  ch_throttle;          // CRSF channel for throttle (default: 1)
    uint8_t  ch_gain;              // CRSF channel for gain knob (default: 4)
    uint8_t  ch_mode;              // CRSF channel for mode switch (default: 5)
    uint8_t  ch_ebrake;            // CRSF channel for e-brake (default: 6)
    
    // Mode behavior
    uint8_t  mode_count;           // Number of modes (2 or 3)
    // mode 0: always off
    // mode 1: normal stability
    // mode 2: drift assist (reduced counter-steer)
    
    // Braking stability (Phase 1)
    float    brake_gain_boost;         // Extra stability gain while braking (default: 1.5)
    float    brake_yaw_threshold;      // deg/s² to trigger enhanced stability
    float    trail_brake_reduction;    // Correction reduction during trail brake (default: 0.7)
    
    // E-brake behavior (Phase 2)
    bool     ebrake_oversteer_release; // Release e-brake on high yaw rate
    float    ebrake_release_threshold; // deg/s yaw rate to trigger release
} config_t;
```

## CLI Interface

```
# groundflight cli

> status
Gyro:     OK (ICM-42688)
CRSF:     OK (ELRS 500Hz)
ESC:      OK (SRXL2, 42.3mph, 24.1V, 38°C)
Steer:    1520us
E-brake:  1000us (released)
Mode:     NORMAL
Yaw rate: -12.3 deg/s
Gain:     0.75

> get kp
kp = 0.500

> set kp 0.7
kp = 0.700

> set esc_protocol srxl2
esc_protocol = SRXL2

> save
Configuration saved.

> dump
# GroundFlight config
set gyro_lpf_hz = 100
set kp = 0.700
set ki = 0.000
set esc_protocol = srxl2
set gear_ratio = 14.0
set tire_diameter_mm = 139
...

> defaults
Reset to defaults.

> help
Available commands:
  status    - Show system status
  get       - Get parameter value  
  set       - Set parameter value
  save      - Save config to EEPROM
  dump      - Dump all settings
  defaults  - Reset to defaults
  cal       - Calibrate gyro
  help      - Show this help
```

## Build System

Keep it simple - straight Makefile, no cmake bloat.

```makefile
TARGET = groundflight
MCU = STM32F722xx

SRCS = src/main.c \
       src/drivers/icm42688.c \
       src/drivers/crsf.c \
       ...

include stm32f7.mk
```

## Phase 1 Milestones

1. **Blink** - Board brings up, LED blinks, clock configured
2. **Serial** - CLI over UART6, can type commands
3. **IMU** - Read gyro, print to serial
4. **CRSF** - Parse receiver, print channels
5. **PWM** - Steering servo follows stick input (no stabilization)
6. **E-brake** - E-brake servo passthrough
7. **ESC basic** - Throttle to ESC (PWM fallback mode)
8. **Stab basic** - Close the loop, basic P controller on steering
9. **Brake aware** - Increase stability gain during motor braking
10. **Tune** - Add gain knob, CLI tuning
11. **Save** - Persist config to flash
12. **ESC telem** - SRXL2 backend, speed from RPM

## Future Extensions (Phase 2+)

- **E-brake oversteer release** - Auto-release on excessive yaw rate
- **Trail braking awareness** - Reduce counter-steer during intentional rotation
- **Traction control** - Detect wheel spin via current/RPM mismatch under acceleration
- **Grip estimation** - Compare expected vs actual decel to estimate surface grip
- **Blackbox logging** - 128MB flash is there, might as well use it
- **Wheelie/stoppie detection** - Pitch rate limiting
- **ELRS Lua script** - Real-time tuning from TX
- **Multiple profiles** - Different tunes for different surfaces (pavement/dirt/wet)
- **Additional ESC protocols** - KISS, Hobbywing, Castle as needed

## Stolen Code Strategy

From Betaflight (BSD-licensed drivers):
- `src/main/drivers/accgyro/accgyro_spi_icm426xx.c` → adapt for our simpler needs
- `src/main/rx/crsf.c` → mostly usable as-is (or use your wheel bridge code)

From SpektrumRC/SRXL2 (MIT license):
- `spm_srxl.c/.h` - Complete protocol implementation
- `spm_srxl_config.h` - Configuration template for STM32
- Has hardware CRC support for STM32 F3/F7

From STM32CubeF7:
- HAL drivers for GPIO, SPI, UART, TIM
- System clock configuration

Write fresh:
- The actual stabilization logic (it's trivial)
- Speed estimation from RPM
- CLI (simple line parser)
- Config management

## ESC Telemetry Backend: SRXL2 (Spektrum Firma)

Example implementation for the Firma 150A Smart ESC.

### Available Telemetry

| Field | Use |
|-------|-----|
| **RPM** | Speed estimation (motor eRPM) |
| Voltage | Battery monitoring, low-voltage warning |
| Current | Load sensing, traction detection |
| Temperature | Thermal protection |
| Capacity (mAh) | Remaining runtime |

### Speed Calculation

```c
// Configurable vehicle geometry
// Infraction 6S example values:
//   motor_kv = 2050
//   gear_ratio = (54/11) * 2.85 = 14.0
//   tire_diameter_mm = 139
//   motor_poles = 4 (so eRPM / 2 = mechanical RPM)

float mechanical_rpm = esc_telem.rpm / (motor_poles / 2);
float wheel_rpm = mechanical_rpm / gear_ratio;
float wheel_circumference_m = tire_diameter_mm * M_PI / 1000.0f;
float speed_mps = wheel_rpm * wheel_circumference_m / 60.0f;
float speed_mph = speed_mps * 2.237f;
```

### SRXL2 Bus Wiring

```
┌─────────────────┐       CRSF (UART4)       ┌─────────────────┐
│   ELRS RX       │◄────────────────────────►│                 │
│   (2.4GHz)      │                          │                 │
└─────────────────┘                          │   GroundFlight  │
                                             │   (Nexus HW)    │
┌─────────────────┐    SRXL2 (UART3)         │                 │
│   Firma ESC     │◄────────────────────────►│                 │
│   (150A Smart)  │  throttle + telemetry    │                 │
└─────────────────┘                          └───────┬─────────┘
                                                     │
                              ┌──────────────────────┼──────────────────────┐
                              │ PWM                  │ PWM                  │ PWM
                              ▼                      ▼                      ▼
                     ┌─────────────────┐    ┌─────────────────┐    ┌─────────────┐
                     │  Steering Servo │    │  E-brake Servo  │    │     Aux     │
                     │      (S1)       │    │      (S3)       │    │    (TAIL)   │
                     └─────────────────┘    └─────────────────┘    └─────────────┘
```

### SRXL2 Implementation Notes

- Protocol: Half-duplex UART, 115.2kbaud default, can negotiate to 400k
- Spektrum provides MIT-licensed reference code: https://github.com/SpektrumRC/SRXL2
- GroundFlight acts as bus master, polls ESC for telemetry
- Handshake at startup to discover ESC capabilities
- Telemetry packets arrive ~every 11ms when polled
- STM32 F7 hardware CRC acceleration supported

## Open Questions

1. **Gyro filtering** - How much do we need? Cars have much lower frequency dynamics than helis. A simple 2nd order LPF at 50Hz might be plenty. Watch for drivetrain harmonics.

2. **Drift mode** - What should this actually do? Reduce counter-steer authority? Switch to heading-hold briefly? TBD based on testing.

3. **Failsafe** - On CRSF loss: center steering + zero throttle? Or just freeze last good values briefly then safe?

4. **E-brake feel** - The slow servo travel is intentional. Do we need to add any rate limiting in software or is the hardware sufficient?

5. **ESC protocol fallback** - If SRXL2 handshake fails, fall back to PWM-only mode gracefully?

6. **Brake current calibration** - The `current_to_decel_scale` factor needs to be calibrated per-vehicle. Log data and derive empirically, or compute from motor constants?

7. **Trail brake threshold** - What steering angle constitutes "intentional rotation" vs "minor correction while braking"? Probably needs tuning.

## Name Candidates

- GroundFlight (obvious, plays on the heritage)
- DirtFlight
- SurfaceFlight  
- TarmacFC
- GravelGyro
- StabilitySled

---

*This is a living document. Update as the project evolves.*
