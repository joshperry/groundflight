/**
 * GroundFlight - RC Car Gyro Stabilizer
 * 
 * Main entry point and control loop
 * Target: RadioMaster Nexus (STM32F722)
 */

#include "target.h"
#include "usb_cdc.h"
#include "icm42688.h"
#include "crsf.h"
#include "pwm.h"
#include "esc.h"
#include "config.h"
#include "gyro.h"
#include "speed.h"
#include "stabilizer.h"
#include "mixer.h"
#include "osd.h"
#include <string.h>

/* Forward declarations */
void drivers_init(void);
void flight_init(void);

/* Simple CLI state */
#define CLI_BUF_SIZE 128
static char cli_buf[CLI_BUF_SIZE];
static uint8_t cli_pos = 0;

static void cli_process_line(const char *line);
static void cli_poll(void);

/* Non-blocking CLI monitor state machine */
typedef enum {
    MONITOR_NONE = 0,
    MONITOR_GYRO,
    MONITOR_CRSF,
    MONITOR_PASS,
    MONITOR_STAB,
} monitor_mode_t;

static monitor_mode_t monitor_mode = MONITOR_NONE;
static uint32_t monitor_last_ms = 0;
#define MONITOR_INTERVAL_MS 50

static void cli_monitor_tick(uint32_t now);
static void cli_monitor_stop(void);

/* IMU state */
static bool imu_ok = false;

/* CRSF state */
static bool crsf_ok = false;

/* PWM state */
static bool pwm_ok = false;

/* ESC state */
static bool esc_ok = false;

/* Arm state */
static bool armed = false;

static bool motor_test_mode = false;
static uint32_t motor_test_start_ms = 0;
#define MOTOR_TEST_TIMEOUT_MS (5 * 60 * 1000)  /* 5 minutes */

/* Arm channel configuration */
#define ARM_CHANNEL         4       /* CH5 (0-indexed) */
#define ARM_THRESHOLD_HIGH  1700    /* Above this = arm requested */
#define ARM_THRESHOLD_LOW   1300    /* Below this = disarm requested */
#define THROTTLE_CHANNEL    2       /* CH3 (0-indexed) */
#define THROTTLE_NEUTRAL_MIN 1400   /* Throttle must be in this range to arm */
#define THROTTLE_NEUTRAL_MAX 1600

/* Simple printf helper for integers */
static void print_int(int32_t val)
{
    char buf[16];
    int i = 0;
    bool neg = false;
    
    if (val < 0) {
        neg = true;
        val = -val;
    }
    
    if (val == 0) {
        buf[i++] = '0';
    } else {
        while (val > 0) {
            buf[i++] = '0' + (val % 10);
            val /= 10;
        }
    }
    
    if (neg) {
        usb_cdc_print("-");
    }
    
    /* Reverse and print */
    while (i > 0) {
        uint8_t c = buf[--i];
        usb_cdc_send(&c, 1);
    }
}

/* Print float with 2 decimal places */
static void print_float(float val)
{
    if (val < 0) {
        usb_cdc_print("-");
        val = -val;
    }

    int32_t int_part = (int32_t)val;
    int32_t frac_part = (int32_t)((val - (float)int_part) * 100.0f + 0.5f);

    /* Handle rounding overflow (e.g., 9.999 -> frac rounds to 100) */
    if (frac_part >= 100) {
        int_part++;
        frac_part = 0;
    }

    print_int(int_part);
    usb_cdc_print(".");
    if (frac_part < 10) usb_cdc_print("0");
    print_int(frac_part);
}

/* ============================================================================
 * Main Entry Point
 * ============================================================================ */

int main(void)
{
    /* Initialize target hardware (clocks, GPIO, systick) */
    target_init();
    
    /* Initialize USB CDC for CLI */
    usb_cdc_init();
    
    /* Initialize subsystems
     * Config must load before flight_init reads gains */
    config_init();
    drivers_init();
    flight_init();

    /* Enable stabilizer if gyro was calibrated on boot */
    if (imu_ok) {
        stabilizer_set_mode(STAB_MODE_NORMAL);
    }

    /* Initialize MSP OSD on UART6 (Port B) */
    osd_init();

    /* Start watchdog last - everything must be initialized first */
    target_watchdog_init();
    
    /* Main loop */
    uint32_t last_blink = 0;
    bool led_state = false;
    bool sent_banner = false;
    
    while (1) {
        target_watchdog_feed();
        uint32_t now = target_millis();
        
        /* Send banner once USB is connected */
        if (!sent_banner && usb_cdc_is_connected()) {
            target_delay_ms(100);  /* Let host enumerate */
            usb_cdc_print("\r\n\r\n");
            usb_cdc_print("=================================\r\n");
            usb_cdc_print("  GroundFlight v0.1.0\r\n");
            usb_cdc_print("  RC Car Gyro Stabilizer\r\n");
            usb_cdc_print("=================================\r\n");
            usb_cdc_print("\r\nIMU: ");
            if (imu_ok) {
                usb_cdc_print("ICM-42688-P detected\r\n");
            } else {
                usb_cdc_print("NOT DETECTED\r\n");
            }
            usb_cdc_print("CRSF: ");
            if (crsf_ok) {
                usb_cdc_print("UART4 initialized\r\n");
            } else {
                usb_cdc_print("NOT INITIALIZED\r\n");
            }
            usb_cdc_print("PWM:  ");
            if (pwm_ok) {
                usb_cdc_print("5 channels ready\r\n");
            } else {
                usb_cdc_print("NOT INITIALIZED\r\n");
            }
            if (imu_ok) {
                float bx, by, bz;
                icm42688_get_gyro_bias(&bx, &by, &bz);
                usb_cdc_print("Gyro: Auto-calibrated (bias Z=");
                print_float(bz);
                usb_cdc_print(" dps)\r\n");
                usb_cdc_print("Mode: Stabilization active ('stab off' to disable)\r\n");
            } else {
                usb_cdc_print("Mode: Passthrough (no IMU)\r\n");
            }
            usb_cdc_print("\r\n> ");
            sent_banner = true;
        }
        
        /* Process CLI input */
        cli_poll();

        /* Non-blocking CLI monitor display */
        cli_monitor_tick(now);

        /* Process CRSF frames */
        if (crsf_ok) {
            crsf_process();
        }

        /* Read and filter IMU data */
        if (imu_ok) {
            icm42688_data_t imu_data;
            if (icm42688_read(&imu_data)) {
                /* Update gyro filter with raw data */
                gyro_update(imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);
            }
        }

        /* Update speed estimate from ESC telemetry */
        if (esc_ok && esc_telemetry_valid()) {
            const esc_telemetry_t *telem = esc_get_telemetry();
            speed_update(telem->rpm);
        }

        /* Blink LED - faster when USB connected, fastest if IMU ok */
        uint32_t blink_interval = imu_ok ? 100 : (usb_cdc_is_connected() ? 250 : 500);
        if ((now - last_blink) >= blink_interval) {
            target_led_toggle();
            led_state = !led_state;
            last_blink = now;
        }
        
        /* Auto-disable motor test mode after timeout */
        if (motor_test_mode && (now - motor_test_start_ms) >= MOTOR_TEST_TIMEOUT_MS) {
            motor_test_mode = false;
            esc_set_throttle(ESC_THROTTLE_CENTER);
            usb_cdc_print("\r\n[motor_test auto-disabled after 5 min]\r\n> ");
        }

        /* Arm/Disarm logic */
        if (crsf_ok && pwm_ok) {
            const crsf_state_t *crsf = crsf_get_state();
            
            /* Disarm conditions (checked first, always apply) */
            if (crsf->failsafe) {
                armed = false;  /* Signal loss = disarm */
            } else {
                uint16_t arm_ch = crsf_to_us(crsf->channels[ARM_CHANNEL]);
                uint16_t throttle_ch = crsf_to_us(crsf->channels[THROTTLE_CHANNEL]);
                
                if (arm_ch < ARM_THRESHOLD_LOW) {
                    armed = false;  /* Switch low = disarm */
                } else if (!armed && arm_ch > ARM_THRESHOLD_HIGH) {
                    /* Arm only if throttle neutral */
                    if (throttle_ch >= THROTTLE_NEUTRAL_MIN && 
                        throttle_ch <= THROTTLE_NEUTRAL_MAX) {
                        armed = true;
                    }
                }
            }
            
            /* Output logic - stabilizer pipeline */
            if (armed) {
                /* Get filtered gyro data */
                const gyro_filtered_t *gyro = gyro_get_filtered();

                /* Convert CRSF channels to normalized float */
                float steer_cmd = crsf_to_float(crsf->channels[0]);    /* CH1 */
                float throttle_cmd = crsf_to_float(crsf->channels[2]); /* CH3 */
                float ebrake_cmd = crsf_to_float(crsf->channels[3]);   /* CH4 */

                /* E-brake is 0.0 to 1.0, not bipolar */
                ebrake_cmd = (ebrake_cmd + 1.0f) / 2.0f;

                /* Get current speed */
                float speed_mph = speed_get_mph();

                /* Run stabilizer (gain_knob defaults to 1.0 for MVP) */
                float correction = stabilizer_update(steer_cmd, gyro->yaw_rate,
                                                     speed_mph, throttle_cmd, 1.0f);

                /* Run mixer */
                mixer_update(steer_cmd, throttle_cmd, ebrake_cmd, 0.0f, correction);

                /* Get mixed outputs */
                const mixer_output_t *mixed = mixer_get_output();

                /* Convert back to CRSF range and output */
                uint16_t steer_crsf = (uint16_t)(CRSF_CHANNEL_MID +
                                      mixed->steer * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MID));
                uint16_t ebrake_crsf = (uint16_t)(CRSF_CHANNEL_MIN +
                                       mixed->ebrake * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN));

                pwm_set_crsf(PWM_STEERING, steer_crsf);
                pwm_set_crsf(PWM_EBRAKE, ebrake_crsf);
                esc_set_throttle_crsf(crsf->channels[2]);  /* Throttle passthrough */
            } else {
                /* Disarmed = failsafe = outputs neutral */
                pwm_set_pulse(PWM_STEERING, PWM_PULSE_CENTER);
                pwm_set_pulse(PWM_EBRAKE,   PWM_PULSE_CENTER);
                if(!motor_test_mode) {
                    esc_set_throttle(ESC_THROTTLE_CENTER);
                }
            }
        }
        
        /* Process ESC telemetry (for SRXL2 mode) */
        if (esc_ok) {
            esc_process();
        }

        /* Update OSD */
        osd_set_armed(armed);
        osd_update(now);
    }
    
    return 0;
}

/* ============================================================================
 * CLI Processing
 * ============================================================================ */

static void cli_poll(void)
{
    /* During monitor mode, input is consumed by cli_monitor_tick() */
    if (monitor_mode != MONITOR_NONE) return;

    int16_t c;

    while ((c = usb_cdc_read_byte()) >= 0) {
        if (c == '\r' || c == '\n') {
            if (cli_pos > 0) {
                cli_buf[cli_pos] = '\0';
                usb_cdc_print("\r\n");
                cli_process_line(cli_buf);
                cli_pos = 0;
                usb_cdc_print("> ");
            }
        } else if (c == '\b' || c == 0x7F) {
            /* Backspace */
            if (cli_pos > 0) {
                cli_pos--;
                usb_cdc_print("\b \b");
            }
        } else if (c >= 0x20 && c < 0x7F) {
            /* Printable character */
            if (cli_pos < CLI_BUF_SIZE - 1) {
                cli_buf[cli_pos++] = (char)c;
                /* Echo */
                uint8_t echo = (uint8_t)c;
                usb_cdc_send(&echo, 1);
            }
        }
    }
}

static void cli_monitor_stop(void)
{
    monitor_mode = MONITOR_NONE;
    usb_cdc_print("\r\n> ");
}

static void cli_monitor_tick(uint32_t now)
{
    if (monitor_mode == MONITOR_NONE) return;

    /* Check for any key to stop */
    if (usb_cdc_available() > 0) {
        usb_cdc_read_byte();
        cli_monitor_stop();
        return;
    }

    /* Rate-limit display to 20Hz */
    if ((now - monitor_last_ms) < MONITOR_INTERVAL_MS) return;
    monitor_last_ms = now;

    switch (monitor_mode) {
    case MONITOR_GYRO: {
        icm42688_data_t data;
        if (icm42688_read(&data)) {
            usb_cdc_print("\r  ");
            print_float(data.gyro_x);
            usb_cdc_print("      ");
            print_float(data.gyro_y);
            usb_cdc_print("      ");
            print_float(data.gyro_z);
            usb_cdc_print("      ");
            print_float(data.temp_c);
            usb_cdc_print(" C   ");
        }
        break;
    }

    case MONITOR_CRSF: {
        const crsf_state_t *crsf = crsf_get_state();
        usb_cdc_print("\r    ");
        for (int i = 0; i < 8; i++) {
            uint16_t val = crsf->channels[i];
            if (val < 1000) usb_cdc_print(" ");
            if (val < 100)  usb_cdc_print(" ");
            print_int(val);
            usb_cdc_print(" ");
        }
        if (crsf->failsafe) {
            usb_cdc_print(" [FAILSAFE]");
        } else {
            usb_cdc_print(" LQ=");
            print_int(crsf->link.uplink_link_quality);
            usb_cdc_print("%");
        }
        usb_cdc_print("   ");
        break;
    }

    case MONITOR_PASS: {
        /* Display uses the same arm/output state driven by main loop */
        usb_cdc_print("\r");
        if (armed) {
            usb_cdc_print("[ARMED]   ");
        } else {
            usb_cdc_print("[DISARMED]");
        }
        usb_cdc_print(" Steer:");
        print_int(pwm_get_pulse(PWM_STEERING));
        usb_cdc_print(" Motor:");
        print_int(pwm_get_pulse(PWM_MOTOR));
        usb_cdc_print(" Brake:");
        print_int(pwm_get_pulse(PWM_EBRAKE));
        const crsf_state_t *crsf_p = crsf_get_state();
        usb_cdc_print(" LQ=");
        print_int(crsf_p->link.uplink_link_quality);
        usb_cdc_print("%    ");
        break;
    }

    case MONITOR_STAB: {
        const gyro_filtered_t *gyro = gyro_get_filtered();
        const crsf_state_t *crsf_s = crsf_get_state();
        float steer_cmd = crsf_to_float(crsf_s->channels[0]);
        float speed_mph = speed_get_mph();

        usb_cdc_print("\r  ");
        print_float(gyro->yaw_rate);
        usb_cdc_print("     ");
        print_float(speed_mph);
        usb_cdc_print("   ");
        print_float(steer_cmd);
        usb_cdc_print("      ");
        /* Show last correction from stabilizer (already computed by main loop) */
        float expected = steer_cmd * config_get()->yaw_rate_scale;
        float error = expected - gyro->yaw_rate;
        float correction = config_get()->kp * error;
        print_float(correction);
        usb_cdc_print("       ");
        print_float(error);
        usb_cdc_print("    ");
        break;
    }

    default:
        break;
    }
}

static void cli_process_line(const char *line)
{
    /* Skip leading spaces */
    while (*line == ' ') line++;
    
    if (strlen(line) == 0) {
        return;
    }
    
    if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
        usb_cdc_print("Commands:\r\n");
        usb_cdc_print("  help      - This help\r\n");
        usb_cdc_print("  status    - Show system status\r\n");
        usb_cdc_print("  version   - Show firmware version\r\n");
        usb_cdc_print("  gyro      - Show live gyro data (any key to stop)\r\n");
        usb_cdc_print("  gyroraw   - Show raw gyro values\r\n");
        usb_cdc_print("  cal       - Calibrate gyro (keep device still!)\r\n");
        usb_cdc_print("  crsf      - Show live CRSF channel data\r\n");
        usb_cdc_print("  servo N P - Set servo N (0-4) to pulse P (1000-2000)\r\n");
        usb_cdc_print("  pass      - Monitor passthrough and arm state\r\n");
        usb_cdc_print("  stab      - Monitor stabilizer (gyro, speed, correction)\r\n");
        usb_cdc_print("  stab on   - Enable stabilizer\r\n");
        usb_cdc_print("  stab off  - Disable stabilizer\r\n");
        usb_cdc_print("  esc       - Show ESC status and telemetry\r\n");
        usb_cdc_print("  esc pwm   - Switch to PWM mode\r\n");
        usb_cdc_print("  esc srxl2 - Switch to SRXL2 mode\r\n");
        usb_cdc_print("  dfu       - Reboot to DFU bootloader\r\n");
        usb_cdc_print("  reboot    - Reboot system\r\n");
    }
    else if (strcmp(line, "status") == 0) {
        usb_cdc_print("GroundFlight Status:\r\n");
        
        usb_cdc_print("  IMU:      ");
        if (imu_ok) {
            usb_cdc_print("ICM-42688-P (WHO_AM_I=0x");
            uint8_t who = icm42688_who_am_i();
            if (who >= 0x10) {
                uint8_t h = (who >> 4);
                usb_cdc_send((uint8_t*)(h < 10 ? "0123456789" + h : "ABCDEF" + h - 10), 1);
            }
            uint8_t l = (who & 0x0F);
            usb_cdc_send((uint8_t*)(l < 10 ? "0123456789" + l : "ABCDEF" + l - 10), 1);
            usb_cdc_print(")\r\n");
        } else {
            usb_cdc_print("NOT DETECTED\r\n");
        }
        
        usb_cdc_print("  CRSF:     ");
        if (crsf_ok) {
            const crsf_state_t *crsf = crsf_get_state();
            if (crsf_is_connected()) {
                usb_cdc_print("Connected (");
                print_int(crsf->frame_count);
                usb_cdc_print(" frames, ");
                print_int(crsf->error_count);
                usb_cdc_print(" errors)\r\n");
                usb_cdc_print("  Link:     RSSI=");
                print_int(-(int32_t)crsf->link.uplink_rssi_1);
                usb_cdc_print("dBm LQ=");
                print_int(crsf->link.uplink_link_quality);
                usb_cdc_print("%\r\n");
            } else if (crsf->failsafe) {
                usb_cdc_print("No signal (failsafe)\r\n");
            } else {
                usb_cdc_print("Waiting for data...\r\n");
            }
        } else {
            usb_cdc_print("NOT INITIALIZED\r\n");
        }
        
        usb_cdc_print("  ESC:      ");
        if (esc_ok) {
            if (esc_get_mode() == ESC_MODE_PWM) {
                usb_cdc_print("PWM mode\r\n");
            } else {
                usb_cdc_print("SRXL2 mode");
                if (esc_telemetry_valid()) {
                    const esc_telemetry_t *telem = esc_get_telemetry();
                    usb_cdc_print(" (");
                    print_int(telem->rpm);
                    usb_cdc_print(" RPM)\r\n");
                } else {
                    usb_cdc_print(" (no telemetry)\r\n");
                }
            }
        } else {
            usb_cdc_print("NOT INITIALIZED\r\n");
        }
        
        usb_cdc_print("  Armed:    ");
        if (armed) {
            usb_cdc_print("YES\r\n");
        } else {
            usb_cdc_print("NO (flip CH5 with throttle neutral to arm)\r\n");
        }
        
        usb_cdc_print("  Clock:    ");
        extern uint32_t SystemCoreClock;
        print_int(SystemCoreClock / 1000000);
        usb_cdc_print(" MHz\r\n");
        
        usb_cdc_print("  Uptime:   ");
        print_int(target_millis());
        usb_cdc_print(" ms\r\n");
        
        /* Show gyro bias if calibrated */
        float bx, by, bz;
        icm42688_get_gyro_bias(&bx, &by, &bz);
        if (bx != 0 || by != 0 || bz != 0) {
            usb_cdc_print("  Gyro bias: X=");
            print_float(bx);
            usb_cdc_print(" Y=");
            print_float(by);
            usb_cdc_print(" Z=");
            print_float(bz);
            usb_cdc_print(" dps\r\n");
        }
    }
    else if (strcmp(line, "version") == 0) {
        usb_cdc_print("GroundFlight v0.1.0\r\n");
        usb_cdc_print("Target: RadioMaster Nexus (STM32F722)\r\n");
    }
    else if (strcmp(line, "gyro") == 0) {
        if (!imu_ok) {
            usb_cdc_print("Error: IMU not detected\r\n");
            return;
        }
        usb_cdc_print("Live gyro data (any key to stop):\r\n");
        usb_cdc_print("    X (roll)    Y (pitch)   Z (yaw)     Temp\r\n");
        monitor_mode = MONITOR_GYRO;
        monitor_last_ms = target_millis();
    }
    else if (strcmp(line, "gyroraw") == 0) {
        if (!imu_ok) {
            usb_cdc_print("Error: IMU not detected\r\n");
            return;
        }
        icm42688_raw_data_t raw;
        if (icm42688_read_raw(&raw)) {
            usb_cdc_print("Gyro X: "); print_int(raw.gyro_x); usb_cdc_print("\r\n");
            usb_cdc_print("Gyro Y: "); print_int(raw.gyro_y); usb_cdc_print("\r\n");
            usb_cdc_print("Gyro Z: "); print_int(raw.gyro_z); usb_cdc_print("\r\n");
            usb_cdc_print("Accel X: "); print_int(raw.accel_x); usb_cdc_print("\r\n");
            usb_cdc_print("Accel Y: "); print_int(raw.accel_y); usb_cdc_print("\r\n");
            usb_cdc_print("Accel Z: "); print_int(raw.accel_z); usb_cdc_print("\r\n");
            usb_cdc_print("Temp: "); print_int(raw.temp); usb_cdc_print("\r\n");
        }
    }
    else if (strcmp(line, "cal") == 0) {
        if (!imu_ok) {
            usb_cdc_print("Error: IMU not detected\r\n");
            return;
        }
        usb_cdc_print("Calibrating gyro - keep device still...\r\n");
        icm42688_calibrate_gyro(1000);
        
        float bx, by, bz;
        icm42688_get_gyro_bias(&bx, &by, &bz);
        usb_cdc_print("Calibration complete.\r\n");
        usb_cdc_print("Bias: X=");
        print_float(bx);
        usb_cdc_print(" Y=");
        print_float(by);
        usb_cdc_print(" Z=");
        print_float(bz);
        usb_cdc_print(" dps\r\n");

        /* Auto-enable stabilizer after calibration */
        stabilizer_set_mode(STAB_MODE_NORMAL);
        usb_cdc_print("Stabilizer auto-enabled (use 'stab off' to disable)\r\n");
    }
    else if (strcmp(line, "crsf") == 0) {
        if (!crsf_ok) {
            usb_cdc_print("Error: CRSF not initialized\r\n");
            return;
        }
        usb_cdc_print("Live CRSF data (any key to stop):\r\n");
        usb_cdc_print("CH:  1     2     3     4     5     6     7     8\r\n");
        monitor_mode = MONITOR_CRSF;
        monitor_last_ms = target_millis();
    }
    else if (strncmp(line, "servo ", 6) == 0) {
        if (!pwm_ok) {
            usb_cdc_print("Error: PWM not initialized\r\n");
            return;
        }
        /* Parse "servo N P" */
        int channel = -1;
        int pulse = -1;
        const char *p = line + 6;
        
        /* Parse channel number */
        while (*p == ' ') p++;
        if (*p >= '0' && *p <= '4') {
            channel = *p - '0';
            p++;
        }
        
        /* Parse pulse width */
        while (*p == ' ') p++;
        if (*p >= '0' && *p <= '9') {
            pulse = 0;
            while (*p >= '0' && *p <= '9') {
                pulse = pulse * 10 + (*p - '0');
                p++;
            }
        }
        
        if (channel < 0 || channel > 4 || pulse < 500 || pulse > 2500) {
            usb_cdc_print("Usage: servo <0-4> <1000-2000>\r\n");
            usb_cdc_print("  0=steering, 1=throttle, 2=ebrake, 3=aux, 4=motor\r\n");
            return;
        }
        
        pwm_set_pulse((pwm_channel_t)channel, (uint16_t)pulse);
        usb_cdc_print("Set servo ");
        print_int(channel);
        usb_cdc_print(" to ");
        print_int(pulse);
        usb_cdc_print("us\r\n");
    }
    else if (strcmp(line, "pass") == 0) {
        if (!crsf_ok || !pwm_ok) {
            usb_cdc_print("Error: CRSF and PWM must be initialized\r\n");
            return;
        }
        usb_cdc_print("Passthrough monitor (any key to exit):\r\n");
        usb_cdc_print("  CH5=Arm  CH1->Steering  CH3->Motor  CH4->Ebrake\r\n\r\n");
        monitor_mode = MONITOR_PASS;
        monitor_last_ms = target_millis();
    }
    else if (strcmp(line, "stab on") == 0) {
        stabilizer_set_mode(STAB_MODE_NORMAL);
        usb_cdc_print("Stabilizer ENABLED (normal mode)\r\n");
    }
    else if (strcmp(line, "stab off") == 0) {
        stabilizer_set_mode(STAB_MODE_OFF);
        usb_cdc_print("Stabilizer DISABLED\r\n");
    }
    else if (strcmp(line, "stab") == 0) {
        if (!imu_ok || !crsf_ok) {
            usb_cdc_print("Error: IMU and CRSF must be initialized\r\n");
            return;
        }
        usb_cdc_print("Stabilizer monitor (any key to exit):\r\n");
        usb_cdc_print("  Gyro[yaw]  Speed   Steer   Correction  Error\r\n\r\n");
        monitor_mode = MONITOR_STAB;
        monitor_last_ms = target_millis();
    }
    else if (strcmp(line, "esc") == 0 || strncmp(line, "esc ", 4) == 0) {
        const char *arg = (strlen(line) > 4) ? line + 4 : NULL;
        
        if (arg == NULL) {
            /* Show ESC status and telemetry */
            usb_cdc_print("ESC Status:\r\n");
            usb_cdc_print("  Mode:     ");
            if (esc_get_mode() == ESC_MODE_PWM) {
                usb_cdc_print("PWM (no telemetry)\r\n");
            } else {
                usb_cdc_print("SRXL2");
                if (esc_is_connected()) {
                    usb_cdc_print(" [CONNECTED]\r\n");
                } else {
                    usb_cdc_print(" [HANDSHAKING]\r\n");
                }
            }
            
            if (esc_get_mode() == ESC_MODE_SRXL2) {
                /* Show ESC device ID and baud info */
                uint8_t esc_id, esc_baud_cap;
                uint32_t baud, rehs;
                esc_get_srxl2_debug(&esc_id, &esc_baud_cap, &baud, &rehs);
                usb_cdc_print("  ESC ID:   0x");
                const char hex[] = "0123456789ABCDEF";
                char h[3] = { hex[(esc_id >> 4) & 0xF], hex[esc_id & 0xF], 0 };
                usb_cdc_print(h);
                usb_cdc_print(" (baud cap: 0x");
                h[0] = hex[(esc_baud_cap >> 4) & 0xF];
                h[1] = hex[esc_baud_cap & 0xF];
                usb_cdc_print(h);
                usb_cdc_print(")\r\n");
                usb_cdc_print("  Baud:     ");
                print_int(baud);
                usb_cdc_print("\r\n");
                usb_cdc_print("  Re-HS:    ");
                print_int(rehs);
                usb_cdc_print("\r\n");
                
                /* Show stats */
                uint32_t tx, rx, crc_err, handshakes;
                esc_get_srxl2_stats(&tx, &rx, &crc_err, &handshakes);
                usb_cdc_print("  TX pkts:  ");
                print_int(tx);
                usb_cdc_print("\r\n");
                usb_cdc_print("  RX pkts:  ");
                print_int(rx);
                usb_cdc_print("\r\n");
                usb_cdc_print("  CRC errs: ");
                print_int(crc_err);
                usb_cdc_print("\r\n");
                
                /* Packet type breakdown */
                uint32_t pkt_hs, pkt_telem, pkt_ctrl, pkt_other;
                uint8_t last_type;
                esc_get_srxl2_pkt_stats(&pkt_hs, &pkt_telem, &pkt_ctrl, &pkt_other, &last_type);
                usb_cdc_print("  RX breakdown:\r\n");
                usb_cdc_print("    Handshake: ");
                print_int(pkt_hs);
                usb_cdc_print("\r\n");
                usb_cdc_print("    Telemetry: ");
                print_int(pkt_telem);
                usb_cdc_print("\r\n");
                usb_cdc_print("    Control:   ");
                print_int(pkt_ctrl);
                usb_cdc_print("\r\n");
                usb_cdc_print("    Other:     ");
                print_int(pkt_other);
                usb_cdc_print("\r\n");
                usb_cdc_print("    Last type: 0x");
                /* Print hex - reuse hex/h from above */
                h[0] = hex[(last_type >> 4) & 0xF];
                h[1] = hex[last_type & 0xF];
                usb_cdc_print(h);
                usb_cdc_print("\r\n");
                
                /* Show telemetry if valid */
                const esc_telemetry_t *telem = esc_get_telemetry();
                usb_cdc_print("  Telemetry: ");
                if (esc_telemetry_valid()) {
                    usb_cdc_print("Valid\r\n");
                    usb_cdc_print("  RPM:      ");
                    print_int(telem->rpm);
                    usb_cdc_print("\r\n");
                    usb_cdc_print("  Voltage:  ");
                    print_float(telem->voltage);
                    usb_cdc_print(" V\r\n");
                    usb_cdc_print("  Current:  ");
                    print_float(telem->current);
                    usb_cdc_print(" A\r\n");
                    usb_cdc_print("  Temp:     ");
                    print_float(telem->temperature);
                    usb_cdc_print(" C\r\n");
                } else {
                    usb_cdc_print("No data\r\n");
                }
            }
        } else if (strcmp(arg, "pwm") == 0) {
            usb_cdc_print("Switching to PWM mode...\r\n");
            esc_ok = esc_init(ESC_MODE_PWM);
            if (esc_ok) {
                usb_cdc_print("ESC now in PWM mode\r\n");
            } else {
                usb_cdc_print("Failed to initialize PWM mode\r\n");
            }
        } else if (strcmp(arg, "srxl2") == 0) {
            usb_cdc_print("Switching to SRXL2 mode...\r\n");
            esc_ok = esc_init(ESC_MODE_SRXL2);
            if (esc_ok) {
                usb_cdc_print("ESC now in SRXL2 mode\r\n");
                usb_cdc_print("Same ESC header - half-duplex on PB6\r\n");
            } else {
                usb_cdc_print("Failed to initialize SRXL2 mode\r\n");
            }
        } else {
            usb_cdc_print("Usage: esc [pwm|srxl2]\r\n");
        }
    }
    else if (strcmp(line, "motor_test") == 0) {
        motor_test_mode = !motor_test_mode;
        if (motor_test_mode) {
            motor_test_start_ms = target_millis();
            usb_cdc_print("Motor test ENABLED - throttle unlocked\r\n");
            usb_cdc_print("WARNING: Motors can spin! Use 'motor_test' again to disable\r\n");
        } else {
            usb_cdc_print("Motor test DISABLED\r\n");
            esc_set_throttle(ESC_THROTTLE_CENTER);
        }
    }
    else if (strncmp(line, "throttle ", 9) == 0) {
        int val = 0;
        const char *arg = line + 9;
        while (*arg >= '0' && *arg <= '9') {
            val = val * 10 + (*arg - '0');
            arg++;
        }
        if (val >= 1000 && val <= 2000) {
            esc_set_throttle((uint16_t)val);
            usb_cdc_print("Throttle set to ");
            print_int(val);
            usb_cdc_print("us\r\n");
        } else {
            usb_cdc_print("Usage: throttle <1000-2000>\r\n");
        }
    }
    else if (strcmp(line, "dfu") == 0) {
        usb_cdc_print("Rebooting to DFU bootloader...\r\n");
        target_delay_ms(100);  /* Let USB send */
        target_reboot_to_bootloader();
    }
    else if (strcmp(line, "reboot") == 0) {
        usb_cdc_print("Rebooting...\r\n");
        target_delay_ms(100);
        NVIC_SystemReset();
    }
    else {
        usb_cdc_print("Unknown command: ");
        usb_cdc_print(line);
        usb_cdc_print("\r\nType 'help' for available commands.\r\n");
    }
}

/* ============================================================================
 * Initialization
 * ============================================================================ */

void drivers_init(void)
{
    /* Initialize IMU (SPI, no blocking) */
    imu_ok = icm42688_init();

    /* Initialize CRSF receiver */
    crsf_ok = crsf_init();

    /* Initialize PWM outputs */
    pwm_ok = pwm_init();

    /* Initialize ESC FIRST - SRXL2 handshake must catch the ESC's
     * 200ms auto-announce window after power-on. */
    esc_ok = esc_init(ESC_MODE_SRXL2);

    /* Auto-calibrate gyro while keeping SRXL2 alive.
     * The cal needs ~1000 samples at 1kHz = ~1 second.
     * We interleave esc_process() so the SRXL2 handshake
     * completes during calibration instead of being starved. */
    if (imu_ok) {
        target_led_on();

        float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
        float gyro_scale = icm42688_get_gyro_scale();
        uint16_t count = 0;
        const uint16_t target_samples = 1000;

        while (count < target_samples) {
            /* Service SRXL2 handshake between gyro reads */
            if (esc_ok) esc_process();

            if (icm42688_data_ready()) {
                icm42688_raw_data_t raw;
                if (icm42688_read_raw(&raw)) {
                    sum_x += (float)raw.gyro_x / gyro_scale;
                    sum_y += (float)raw.gyro_y / gyro_scale;
                    sum_z += (float)raw.gyro_z / gyro_scale;
                    count++;
                }
            }
        }

        icm42688_set_gyro_bias(sum_x / (float)count,
                               sum_y / (float)count,
                               sum_z / (float)count);
        target_led_off();
    }
}

void flight_init(void)
{
    /* Get configuration */
    config_t *cfg = config_get();

    /* Initialize gyro filtering */
    gyro_init(cfg->gyro_lpf_hz);

    /* Initialize speed estimator */
    speed_init(cfg->gear_ratio, cfg->tire_diameter_mm, cfg->motor_poles);

    /* Initialize stabilizer with PID gains */
    stab_gains_t gains = {
        .kp = cfg->kp,
        .ki = cfg->ki,
        .kd = cfg->kd,
        .yaw_rate_scale = cfg->yaw_rate_scale,
        .max_correction = cfg->max_correction,
        .low_speed_gain = cfg->low_speed_gain,
        .high_speed_gain = cfg->high_speed_gain,
        .speed_gain_max_mph = cfg->speed_gain_max_mph,
    };
    stabilizer_init(&gains);

    /* Start disabled - gyro must be calibrated before stabilization.
     * User enables via CLI or we could auto-enable after calibration. */
    stabilizer_set_mode(STAB_MODE_OFF);

    /* Initialize mixer */
    mixer_init();
}
