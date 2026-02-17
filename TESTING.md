# GroundFlight Testing Strategy

## Philosophy

**Don't test your assumptions. Test the actual behavior.**

A test like this encodes the same bug as the code:

```c
// BAD: Tests our assumption about what "stopped" means
#define MOTOR_SAFE_VALUE PWM_PULSE_MIN  // 1000µs - WRONG for car ESCs!
assert(pwm_get_pulse(PWM_MOTOR) == MOTOR_SAFE_VALUE);
```

The test passes. The car drives backwards. We had a bug where failsafe set motor to 1000µs (full reverse on car ESCs) instead of 1500µs (neutral). If we had written unit tests, we would have written them with the same wrong assumption.

The only way to test "motor is stopped" is to measure whether the motor is actually stopped:

```python
# GOOD: Tests actual physical behavior
assert tachometer.rpm == 0, f"Motor still spinning at {tach.rpm} RPM"
```

This principle drives our tiered testing approach.

---

## Test Tiers

### Tier 1: Unit Tests (No Hardware)

**Purpose:** Fast feedback on pure logic  
**Runs:** On every commit, in CI  
**Hardware:** None (host machine)

What we test (91 assertions, 9 suites):
- CRSF channel conversions (`crsf_to_float`, `crsf_to_us`) and boundary values
- PD stabilizer math, gain scheduling interpolation, mode transitions, gain knob clamping
- Mixer correction application and output clamping
- Speed estimation (RPM → mph conversion chain)
- Gyro Butterworth LPF convergence and calibration
- Config defaults and reset behavior
- MSP v1 frame encoding and CRC
- Full signal chain (CRSF → stabilizer → mixer → PWM)
- Failsafe safety invariants (steering/motor neutral, e-brake applied)

What we DON'T test:
- Anything involving real hardware timing
- Interrupt handling
- Actual PWM waveforms (needs Tier 2)

```bash
gf-test    # Runs all host-side tests
```

### Tier 2: Hardware-in-Loop (Nexus + Logic Analyzer)

**Purpose:** Verify real firmware behavior without motor  
**Runs:** On PR merge, release candidates  
**Hardware:** Nexus, ELRS TX/RX pair, logic analyzer or PWM capture

```
┌─────────────┐     RF      ┌─────────────┐
│  ELRS TX    │ ~~~~~~~~~~> │  ELRS RX    │
│  (USB to    │             │  (on Nexus) │
│   test PC)  │             └──────┬──────┘
└─────────────┘                    │ CRSF
                                   ▼
                            ┌─────────────┐
                            │   Nexus     │
                            │ GroundFlight│
                            └──────┬──────┘
                                   │ PWM
                                   ▼
                            ┌─────────────┐
                            │ PWM Capture │────▶ Test PC
                            │ (LA / MCU)  │      (assertions)
                            └─────────────┘
```

What we test:
- Actual PWM pulse widths at output pins
- Failsafe timing (latency from signal loss to safe output)
- Channel mapping (CH1 input → steering output)
- ARM/DISARM state transitions
- PWM frequency and jitter

Example test cases:

| Test | Procedure | Assertion |
|------|-----------|-----------|
| Boot outputs | Power on Nexus, no TX | All PWM = 1500µs ± 5µs |
| Failsafe latency | TX on → TX off | PWM reaches 1500µs within 600ms |
| Channel passthrough | Sweep CH1 172→1811 | Steering PWM = 1000→2000µs linear |
| Disarmed lockout | Disarmed, move throttle | Motor PWM stays 1500µs |

**PWM capture options:**
- Logic analyzer (Saleae, sigrok-compatible)
- Dedicated MCU (Pico/ESP32) with input capture
- Oscilloscope with serial output

### Tier 3: ESC-in-Loop (Nexus + ESC + Motor Mock)

**Purpose:** Verify ESC interprets commands correctly  
**Runs:** On safety-critical changes, new ESC support  
**Hardware:** Nexus, ESC, motor mock (or no motor), telemetry connection

```
┌─────────────┐
│   Nexus     │
│ GroundFlight│
└──────┬──────┘
       │ PWM (throttle)
       ▼
┌─────────────┐
│    ESC      │◀─── SRXL2 telemetry ───▶ Nexus
│ (Firma 150A)│
└──────┬──────┘
       │ 3-phase
       ▼
┌─────────────┐
│ Motor Mock  │  (or disconnected)
│ (resistors) │
└─────────────┘
```

**Motor mocking strategies:**

1. **No motor** — Many ESCs report commanded throttle % via telemetry even without motor attached. SRXL2 telemetry becomes our test oracle.

2. **Resistive load** — Three power resistors (wye config) on phase wires. ESC sees load impedance, reports current draw. No spinning mass.

3. **BEMF simulator** — Active circuit generating back-EMF waveforms. ESC thinks motor runs at controlled RPM. Most complex but fully deterministic.

What we test:
- ESC receives and interprets neutral correctly
- ESC telemetry reports expected state
- ESC-specific arming requirements
- Brake vs reverse behavior

Example test cases:

| Test | Procedure | Assertion (via SRXL2) |
|------|-----------|----------------------|
| ESC neutral | Send 1500µs | Throttle = 0%, RPM = 0 |
| ESC failsafe | TX off | Throttle = 0% within 600ms |
| ESC arm check | Arm with throttle up | ESC rejects (if supported) |

### Tier 4: Full System (Real Motor on Stand)

**Purpose:** Ground truth — is the motor actually behaving correctly?  
**Runs:** Release validation, new hardware qualification  
**Hardware:** Complete system with motor mounted on test stand

```
┌─────────────┐
│   Nexus     │
└──────┬──────┘
       │ PWM
       ▼
┌─────────────┐
│    ESC      │
└──────┬──────┘
       │ 3-phase
       ▼
┌─────────────┐     ┌─────────────┐
│   Motor     │────▶│ Tachometer  │───▶ Test PC
│ (on stand)  │     │ (optical)   │
└─────────────┘     └─────────────┘
```

**Tachometer options:**
- IR reflective sensor + tape stripe on motor bell
- Hall sensor reading motor magnets
- ESC telemetry RPM (if accurate)
- Audio analysis (distinctive frequency)

What we test:
- Motor actually stops on failsafe
- Motor actually stops when disarmed
- No unexpected movement on boot
- Brake force application (via deceleration rate)

Example test cases:

```python
def test_failsafe_stops_motor():
    """Motor must stop within 1 second of TX power loss."""
    arm()
    tx.set_throttle(50)
    wait_until(lambda: tach.rpm > 5000, timeout=2.0)
    
    tx.power_off()
    time.sleep(1.0)
    
    assert tach.rpm == 0, f"Motor still spinning at {tach.rpm} RPM"

def test_disarm_stops_motor():
    """Motor must stop immediately when disarmed."""
    arm()
    tx.set_throttle(30)
    wait_until(lambda: tach.rpm > 3000, timeout=2.0)
    
    disarm()
    time.sleep(0.5)
    
    assert tach.rpm == 0, f"Motor still spinning at {tach.rpm} RPM"

def test_boot_motor_stopped():
    """Motor must not move on power-up."""
    power_cycle_nexus()
    time.sleep(2.0)  # Let everything initialize
    
    # Sample RPM over 1 second
    samples = [tach.rpm for _ in range(10)]
    
    assert all(rpm == 0 for rpm in samples), f"Motor moved on boot: {samples}"
```

---

## Safety-Critical Test Cases

These MUST pass before any release:

| ID | Test | Tier | Pass Criteria |
|----|------|------|---------------|
| S1 | Boot with no TX | T2+ | All outputs = neutral |
| S2 | Boot with TX off | T2+ | All outputs = neutral |
| S3 | TX power loss (stationary) | T2+ | Outputs → neutral < 600ms |
| S4 | TX power loss (moving) | T4 | Motor stops < 1s |
| S5 | Disarm while throttle applied | T3+ | Motor stops immediately |
| S6 | Arm rejected if throttle high | T2+ | Stays disarmed |
| S7 | Bad CRSF frames | T2 | Ignored, no output change |
| S8 | CRSF timeout | T2+ | Failsafe triggers |

---

## Hardware Test Rig

### Minimum Viable Rig (Tier 2)

Parts needed:
- Nexus flight controller
- ELRS TX module (USB-connected to test PC)
- ELRS RX (bound to TX, connected to Nexus)
- Raspberry Pi Pico or similar (PWM capture)
- USB hub
- 5V power supply

Pico firmware captures PWM pulse widths on multiple channels, reports via USB serial.

### Full Rig (Tier 3-4)

Additional parts:
- Spektrum Firma 150A ESC
- Brushless motor (mounted to test stand)
- IR tachometer sensor
- Power supply for ESC (or LiPo with safety enclosure)
- Emergency stop button (cuts ESC power)

---

## Future Work

- [x] Implement Tier 1 unit test framework — Unity v2.6.0, 91 assertions across 9 suites (`gf-test`)
- [ ] Build PWM capture firmware for Pico
- [ ] Create test harness Python library
- [ ] Design motor test stand with safety enclosure
- [ ] Add SRXL2 telemetry for ESC-in-loop testing
- [ ] CI integration for Tier 1 tests
- [ ] Automated Tier 2 tests on dedicated hardware

---

## Lessons Learned

### 2024-01-XX: Failsafe Drove Car Backwards

**Bug:** Failsafe set motor PWM to 1000µs (PWM_PULSE_MIN), thinking this meant "motor off" like aircraft throttle.

**Reality:** Car ESCs use 1500µs as neutral. 1000µs = full reverse.

**Root cause:** Developer assumption about ESC behavior encoded in both code AND hypothetical tests.

**Fix:** Set motor to PWM_PULSE_CENTER (1500µs) on failsafe.

**Prevention:** Tier 3+ testing with real ESC would catch this. Telemetry showing "throttle = -100%" or motor spinning would fail the "motor stopped" assertion regardless of what pulse width we thought was correct.

**Takeaway:** Safety-critical defaults must be validated against actual hardware behavior, not developer assumptions.
