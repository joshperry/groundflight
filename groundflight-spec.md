# GroundFlight - RC Car Gyro Stabilizer

Open-source yaw stabilization firmware for RC surface vehicles. Uses RadioMaster Nexus FBL hardware repurposed for ground use.

## Project Goals

1. ELRS-compatible gyro stabilization for RC cars (replaces Spektrum SR6110AT)
2. Protocol-agnostic ESC telemetry for speed estimation
3. Simple, hackable codebase (~3000 lines)
4. Phase 1: Basic stability with brake-aware gain scheduling
5. Phase 2: Advanced dynamics control (traction, grip estimation)

---

## Hardware Platform

### RadioMaster Nexus (Helicopter FBL Controller)

```
MCU:        STM32F722RET6
            - 216 MHz ARM Cortex-M7
            - 256KB RAM, 512KB Flash
            - Hardware FPU, DSP instructions

IMU:        ICM-42688-P (SPI)
            - 6-axis (3 gyro, 3 accel)
            - Up to 32kHz sample rate
            - Low noise: 0.7 mdps/√Hz

Flash:      W25N01GVZEIG 128MB NAND (blackbox, Phase 2)

Baro:       SPL06-001 (unused for car application)

Voltage:    5V - 12.6V input (2S-3S LiPo direct, or BEC)
```

### UART Assignments

| UART | Nexus Label | Function | Baud | Notes |
|------|-------------|----------|------|-------|
| UART4 | CRSF | ELRS receiver input | 420000 | Control link |
| UART3 | PORT-C | ESC telemetry | 115200/400000 | SRXL2 half-duplex |
| UART6 | PORT-B | Debug CLI | 115200 | Tuning interface |
| UART1 | DSM | Unused | - | |
| UART2 | SBUS | Unused | - | |

### PWM/Servo Outputs

| Output | Nexus Label | Function | Signal |
|--------|-------------|----------|--------|
| S1 | S1 | Steering servo | 50-333Hz PWM |
| S2 | S2 | ESC throttle (PWM fallback) | 50-333Hz PWM |
| S3 | S3 | E-brake servo | 50-333Hz PWM |
| S4 | TAIL | Aux (lights, etc) | 50-333Hz PWM |

### SPI Bus

| Device | Function | Speed |
|--------|----------|-------|
| ICM-42688-P | IMU | Up to 24MHz |
| W25N01G | Blackbox flash | Up to 104MHz |

---

## Target Vehicle: Arrma Infraction 6S

1/7 scale street basher, 80+ mph capable.

### Drivetrain Geometry

```
Motor:              Firma 2050Kv brushless
Motor poles:        4 (eRPM / 2 = mechanical RPM)
Pinion:             11T (stock)
Spur:               54T (stock)
Diff ratio:         2.85 (internal)
Overall gear ratio: (54/11) × 2.85 = 14.0:1
Tire diameter:      139mm (stock)
Tire circumference: 437mm
```

### Speed Calculation

```c
float mechanical_rpm = esc_telem.rpm / (motor_poles / 2);
float wheel_rpm = mechanical_rpm / gear_ratio;
float speed_mps = wheel_rpm * (tire_diameter_mm * M_PI / 1000.0f) / 60.0f;
float speed_mph = speed_mps * 2.237f;
```

### Braking Systems

1. **Motor braking** (ESC-controlled)
   - Regenerative/dynamic braking
   - Configured via ESC parameters (see below)
   - Primary deceleration method

2. **E-brake** (servo-actuated)
   - Shoe brake on disc behind center diff
   - Brakes entire driveline (all 4 wheels via diffs)
   - More stable than rear-only handbrake
   - Stock: momentary button, servo rate-limited for feathering

```
                    ┌─────────────┐
    Front Diff ◄────┤ Center Diff ├────► Rear Diff
                    └──────┬──────┘
                           │
                      ┌────┴────┐
                      │ E-brake │  ← Brakes entire driveline
                      │  disc   │
                      └─────────┘
```

---

## ESC: Spektrum Firma 150A Smart

### Specifications

```
Continuous current:  150A
Burst current:       950A
LiPo cells:          3-6S
BEC output:          6V/7.4V selectable @ 5A
Waterproof:          IP67 rated
Motor type:          Sensorless brushless
Telemetry:           SRXL2 (Spektrum Smart Technology)
```

### Programmable Parameters (ESC handles internally)

| Parameter | Options | Default |
|-----------|---------|---------|
| Running Mode | Forward w/ Brake, Forward/Reverse w/ Brake | Fwd w/ Brake |
| Drag Brake Force | 0%, 2%, 4%, 6%, 8%, 10%, 12%, 14%, 16% | 0% |
| Low-voltage Cutoff | 3.2V, 3.3V, 3.4V, 3.5V, 3.6V, 3.7V | 3.2V |
| Start Mode/Punch | Level 1-9 | Level 4 |
| Max Brake Force | 25%, 37.5%, 50%, 62.5%, 75%, 87.5%, 100%, disabled | 100% |
| Max Reverse Force | 25%, 50%, 75%, 100% | 100% |
| Neutral Range | 6% (Narrow), 9% (Normal), 12% (Wide) | 9% |
| Timing | 0° to 26.25° in 3.75° steps | 15° |
| Motor Rotation | CCW, CW | CCW |
| BEC Voltage | 6.0V, 7.4V | 6.0V |

**Key insight:** The ESC handles brake force limiting internally. GroundFlight sends throttle commands; the ESC applies its own brake curves based on Max Brake Force setting. Phase 1 does not modulate throttle for stability - the ESC's tuning is the primary brake feel adjustment.

### SRXL2 Telemetry Available

| Field | Type | Use |
|-------|------|-----|
| RPM | uint32 | Speed estimation (motor eRPM) |
| Voltage | float | Battery monitoring |
| Current | float | Load sensing (Phase 2: grip estimation) |
| Temperature | float | Thermal protection |
| Capacity | uint32 | mAh consumed |

---

## System Architecture

### Data Flow (1kHz main loop)

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
```

### Wiring Diagram

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

---

## Stabilization Algorithm

Core insight: **Car yaw stabilization = helicopter tail rotor logic**, just slower response.

### Control Law

```c
// Inputs (every loop iteration)
float steer_cmd   = crsf.channels[CH_STEER];    // -1.0 to 1.0
float throttle    = crsf.channels[CH_THROTTLE]; // -1.0 to 1.0
float gain_knob   = crsf.channels[CH_GAIN];     //  0.0 to 1.0
uint8_t mode      = decode_mode(crsf.channels[CH_MODE]);

float gyro_yaw    = imu_get_gyro_z();           // deg/s, filtered
float speed_mph   = esc_get_speed_mph();        // from telemetry or estimate

// Stabilization
float correction = 0.0f;

if (mode != MODE_OFF) {
    // Expected yaw rate from steering input (decreases with speed)
    float expected_yaw = steer_cmd * yaw_rate_scale * steering_sensitivity(speed_mph);
    
    // Error
    float yaw_error = expected_yaw - gyro_yaw;
    
    // PID correction
    correction = pid_update(&yaw_pid, yaw_error);
    
    // Scale by gain knob and speed authority
    correction *= gain_knob * speed_authority(speed_mph);
    
    // Brake-aware gain adjustment
    correction *= get_brake_multiplier(throttle, steer_cmd);
}

// Output
float servo_out = constrain(steer_cmd + correction, -1.0f, 1.0f);
servo_set(SERVO_STEER, servo_out);

// Passthrough
servo_set(SERVO_THROTTLE, throttle);  // Or via SRXL2
servo_set(SERVO_EBRAKE, crsf.channels[CH_EBRAKE]);
```

### Speed-Dependent Functions

```c
// Steering sensitivity decreases with speed (understeer model)
float steering_sensitivity(float speed_mph) {
    float t = clamp(speed_mph / speed_gain_max_mph, 0.0f, 1.0f);
    return lerp(1.0f, 0.3f, t);
}

// Correction authority increases with speed
float speed_authority(float speed_mph) {
    float t = clamp(speed_mph / speed_gain_max_mph, 0.0f, 1.0f);
    return lerp(low_speed_gain, high_speed_gain, t);
}
```

### Brake-Aware Gain (Phase 1)

The ESC handles brake force internally. We only adjust *stability gain* based on detected braking state.

```c
float get_brake_multiplier(float throttle, float steer_cmd) {
    bool is_braking = (throttle < -0.1f);
    bool is_trail_braking = is_braking && (fabsf(steer_cmd) > trail_brake_steer_threshold);
    
    if (is_trail_braking) {
        // Driver intentionally rotating - reduce counter-steer
        return trail_brake_reduction;  // e.g., 0.7
    } else if (is_braking) {
        // Straight braking - rear is light, boost stability
        float intensity = fabsf(throttle);
        return lerp(1.0f, brake_gain_max, intensity);  // e.g., up to 1.5
    }
    return 1.0f;
}
```

### Helicopter vs Car Comparison

| Aspect | Helicopter Tail | Car Steering |
|--------|-----------------|--------------|
| Actuator | Direct thrust (tail rotor) | Ackermann geometry |
| Response time | ~50ms | ~100-200ms (servo + tire slip) |
| Setpoint | Heading hold or rate | Rate follows stick |
| Speed coupling | Collective affects authority | Speed affects tire grip |
| Failure mode | Spin | Oversteer/understeer |

---

## ESC Telemetry Abstraction

Protocol-agnostic interface - stabilizer doesn't care how telemetry arrives.

```c
typedef struct {
    uint32_t rpm;              // Motor electrical RPM
    float    voltage;          // Pack voltage
    float    current;          // Motor current (amps)
    float    temperature;      // ESC temp (°C)
    uint32_t mah_consumed;     // Capacity used
    uint32_t last_update_ms;
    bool     valid;
} esc_telemetry_t;

typedef enum {
    ESC_PROTO_NONE,            // No telemetry, estimate from throttle
    ESC_PROTO_SRXL2,           // Spektrum Smart (Firma, Avian)
    ESC_PROTO_KISS,            // Future: KISS/BLHeli32
    ESC_PROTO_HOBBYWING,       // Future: Hobbywing V5
    ESC_PROTO_CASTLE,          // Future: Castle Link
} esc_protocol_t;

// API
void  esc_init(esc_protocol_t proto);
void  esc_update(void);  // Called from main loop
float esc_get_speed_mph(void);
bool  esc_telemetry_valid(void);
const esc_telemetry_t* esc_get_telemetry(void);
```

### SRXL2 Implementation Notes

- Half-duplex UART on single wire (TX/RX tied, tristate control)
- 115.2kbaud default, can negotiate to 400kbaud
- GroundFlight acts as bus master, polls ESC
- Telemetry packets ~every 11ms when polled
- MIT-licensed reference: https://github.com/SpektrumRC/SRXL2
- STM32 F7 has hardware CRC acceleration

---

## Module Structure

```
src/
├── main.c                     // Entry point, main loop
├── drivers/
│   ├── icm42688.c/.h          // IMU driver (~500 lines, from Betaflight)
│   ├── crsf.c/.h              // CRSF parser (~400 lines, from Betaflight)
│   ├── esc_telem.c/.h         // ESC telemetry abstraction (~200 lines)
│   ├── esc_srxl2.c/.h         // SRXL2 backend (~800 lines, from SpektrumRC)
│   ├── esc_none.c/.h          // No-telemetry fallback (~50 lines)
│   ├── pwm.c/.h               // Servo output (~150 lines)
│   └── uart.c/.h              // UART abstraction (~200 lines)
├── flight/
│   ├── gyro.c/.h              // Filtering, calibration (~300 lines)
│   ├── speed.c/.h             // Speed from ESC telemetry (~100 lines)
│   ├── mixer.c/.h             // Input → output mapping (~150 lines)
│   └── stabilizer.c/.h        // Control law (~200 lines)
├── config/
│   ├── config.c/.h            // Runtime config (~200 lines)
│   ├── cli.c/.h               // Serial CLI (~400 lines)
│   └── eeprom.c/.h            // Flash save/load (~100 lines)
└── target/
    └── nexus/
        ├── target.h           // Pin definitions, peripheral config
        └── target.c           // Board-specific init
```

**Estimated total: ~3300 lines of C** (excluding HAL/CMSIS)

---

## Configuration

```c
typedef struct {
    // Gyro
    uint8_t  gyro_lpf_hz;              // Low-pass filter cutoff (default: 100)
    
    // Stabilizer PID
    float    kp;                       // Proportional gain (default: 0.5)
    float    ki;                       // Integral gain (default: 0.0)
    float    kd;                       // Derivative gain (default: 0.05)
    float    yaw_rate_scale;           // deg/s per unit stick (default: 200)
    float    max_correction;           // Authority limit (default: 0.5)
    
    // Speed-based gain scheduling
    float    low_speed_gain;           // Authority at 0 mph (default: 0.3)
    float    high_speed_gain;          // Authority at max speed (default: 1.0)
    float    speed_gain_max_mph;       // Speed for full authority (default: 60)
    
    // Brake-aware gain (Phase 1)
    float    brake_gain_max;           // Stability boost during braking (default: 1.5)
    float    trail_brake_steer_threshold; // Steering to detect trail brake (default: 0.2)
    float    trail_brake_reduction;    // Correction reduction (default: 0.7)
    
    // Steering servo
    uint16_t steer_center_us;          // Center pulse (default: 1500)
    uint16_t steer_range_us;           // +/- range (default: 500)
    int8_t   steer_direction;          // 1 or -1
    
    // E-brake servo
    uint16_t ebrake_min_us;            // Released (default: 1000)
    uint16_t ebrake_max_us;            // Engaged (default: 2000)
    
    // ESC
    uint8_t  esc_protocol;             // ESC_PROTO_* enum
    uint16_t esc_min_us;               // Min pulse (default: 1000)
    uint16_t esc_max_us;               // Max pulse (default: 2000)
    uint16_t esc_center_us;            // Neutral (default: 1500)
    
    // Vehicle geometry (for speed calculation)
    float    gear_ratio;               // Overall reduction (default: 14.0)
    uint16_t tire_diameter_mm;         // Tire OD (default: 139)
    uint8_t  motor_poles;              // For eRPM conversion (default: 4)
    
    // Channel mapping
    uint8_t  ch_steering;              // CRSF channel (default: 0)
    uint8_t  ch_throttle;              // CRSF channel (default: 1)
    uint8_t  ch_gain;                  // CRSF channel (default: 4)
    uint8_t  ch_mode;                  // CRSF channel (default: 5)
    uint8_t  ch_ebrake;                // CRSF channel (default: 6)
    
    // Modes
    uint8_t  mode_count;               // 2 or 3 positions
} config_t;
```

---

## CLI Interface

```
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

> cal
Calibrating gyro... keep still... done.

> help
Commands: status, get, set, save, dump, defaults, cal, help
```

---

## Phase 1 Milestones

1. **Blink** - Board brings up, LED blinks, clock configured
2. **Serial** - CLI over UART6
3. **IMU** - Read gyro, print to serial
4. **CRSF** - Parse receiver, print channels
5. **PWM** - Steering servo follows stick (no stabilization)
6. **E-brake** - E-brake servo passthrough
7. **ESC basic** - Throttle to ESC (PWM mode)
8. **Stab basic** - Close the loop, P controller
9. **Brake aware** - Gain scheduling during braking
10. **Tune** - Gain knob, CLI tuning
11. **Save** - Persist config to flash
12. **ESC telem** - SRXL2 backend, speed from RPM

---

## Phase 2+ (Future)

### Advanced Braking Control

The Firma ESC handles brake force internally, but we can potentially do better with closed-loop control:

- **Throttle modulation during yaw** - If yaw rate spikes during braking, reduce brake command to let rear regain grip
- **Grip estimation** - Compare motor current (expected decel) to accelerometer (actual decel) to estimate surface grip
- **E-brake stability interlock** - Auto-release if yaw rate exceeds threshold

```c
// Phase 2: Grip estimation
float expected_decel = fabsf(motor_current) * current_to_decel_scale;
float actual_decel = fabsf(accel_x_filtered);
float grip_ratio = actual_decel / expected_decel;

if (grip_ratio < 0.7f && is_braking) {
    // Wheels slipping - could reduce brake command
    throttle_out = throttle_cmd * 0.7f;
}
```

### Other Phase 2 Features

- **Traction control** - Detect wheel spin via current/RPM mismatch under acceleration
- **Blackbox logging** - 128MB flash available
- **Wheelie/stoppie detection** - Pitch rate limiting
- **ELRS Lua script** - Real-time tuning from TX
- **Multiple profiles** - Different surfaces (pavement/dirt/wet)
- **Additional ESC protocols** - KISS, Hobbywing, Castle

---

## Code Sources

### From Betaflight (BSD license)

- `src/main/drivers/accgyro/accgyro_spi_icm426xx.c` - IMU driver
- `src/main/rx/crsf.c` - CRSF parser

### From SpektrumRC/SRXL2 (MIT license)

- `spm_srxl.c/.h` - Protocol implementation
- `spm_srxl_config.h` - STM32 configuration template
- Hardware CRC support for F3/F7

### From STM32CubeF7

- HAL drivers (GPIO, SPI, UART, TIM)
- System clock configuration

### Write Fresh

- Stabilization logic (~100 lines core)
- Speed estimation
- CLI parser
- Config management

---

## Open Questions

1. **Gyro filtering** - Simple 2nd order LPF at 50-100Hz? Watch for drivetrain harmonics.

2. **Drift mode** - Reduce counter-steer? Heading-hold briefly? TBD from testing.

3. **Failsafe** - Center steering + zero throttle on CRSF loss?

4. **ESC fallback** - If SRXL2 handshake fails, graceful fallback to PWM?

5. **Speed without telemetry** - Estimate from throttle position? Or require telemetry for speed-based features?

---

## References

- RadioMaster Nexus: https://www.radiomasterrc.com/products/nexus-flybarless-controller
- Spektrum SRXL2: https://github.com/SpektrumRC/SRXL2
- Betaflight: https://github.com/betaflight/betaflight
- ICM-42688-P datasheet: TDK InvenSense
- STM32F722 reference manual: RM0431

---

*GroundFlight - Because your RC car deserves flight controller technology.*
