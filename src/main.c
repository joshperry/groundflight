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
#include <string.h>

/* Forward declarations for driver stubs */
void drivers_init(void);
void flight_init(void);
void config_init(void);

/* Simple CLI state */
#define CLI_BUF_SIZE 128
static char cli_buf[CLI_BUF_SIZE];
static uint8_t cli_pos = 0;

static void cli_process_line(const char *line);
static void cli_poll(void);

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
    int32_t frac_part = (int32_t)((val - int_part) * 100);
    
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
    
    /* Initialize subsystems */
    drivers_init();
    flight_init();
    config_init();
    
    /* Main loop */
    uint32_t last_blink = 0;
    bool led_state = false;
    bool sent_banner = false;
    
    while (1) {
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
            usb_cdc_print("Mode: Passthrough (CH5=Arm, CH1->Steer, CH3->Motor, CH4->Brake)\r\n");
            usb_cdc_print("\r\n> ");
            sent_banner = true;
        }
        
        /* Process CLI input */
        cli_poll();
        
        /* Process CRSF frames */
        if (crsf_ok) {
            crsf_process();
        }
        
        /* Blink LED - faster when USB connected, fastest if IMU ok */
        uint32_t blink_interval = imu_ok ? 100 : (usb_cdc_is_connected() ? 250 : 500);
        if ((now - last_blink) >= blink_interval) {
            target_led_toggle();
            led_state = !led_state;
            last_blink = now;
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
            
            /* Output logic - single code path */
            if (armed) {
                pwm_set_crsf(PWM_STEERING, crsf->channels[0]);
                pwm_set_crsf(PWM_EBRAKE,   crsf->channels[3]);
                esc_set_throttle_crsf(crsf->channels[2]);  /* ESC driver handles mode */
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
    }
    
    return 0;
}

/* ============================================================================
 * CLI Processing
 * ============================================================================ */

static void cli_poll(void)
{
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
        
        while (usb_cdc_available() == 0) {
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
            target_delay_ms(50);
        }
        /* Consume the key that stopped us */
        usb_cdc_read_byte();
        usb_cdc_print("\r\n");
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
    }
    else if (strcmp(line, "crsf") == 0) {
        if (!crsf_ok) {
            usb_cdc_print("Error: CRSF not initialized\r\n");
            return;
        }
        usb_cdc_print("Live CRSF data (any key to stop):\r\n");
        usb_cdc_print("CH:  1     2     3     4     5     6     7     8\r\n");
        
        while (usb_cdc_available() == 0) {
            crsf_process();  /* Keep processing while displaying */
            
            const crsf_state_t *crsf = crsf_get_state();
            
            usb_cdc_print("\r    ");
            for (int i = 0; i < 8; i++) {
                /* Print channel value (pad to 5 chars) */
                uint16_t val = crsf->channels[i];
                if (val < 1000) usb_cdc_print(" ");
                if (val < 100)  usb_cdc_print(" ");
                print_int(val);
                usb_cdc_print(" ");
            }
            
            /* Show connection status */
            if (crsf->failsafe) {
                usb_cdc_print(" [FAILSAFE]");
            } else {
                usb_cdc_print(" LQ=");
                print_int(crsf->link.uplink_link_quality);
                usb_cdc_print("%");
            }
            usb_cdc_print("   ");
            
            target_delay_ms(50);
        }
        /* Consume the key that stopped us */
        usb_cdc_read_byte();
        usb_cdc_print("\r\n");
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
        
        while (usb_cdc_available() == 0) {
            crsf_process();
            esc_process();  /* Process ESC telemetry */
            
            const crsf_state_t *crsf = crsf_get_state();
            
            /* Arm/disarm logic (same as main loop) */
            if (crsf->failsafe) {
                armed = false;
            } else {
                uint16_t arm_ch = crsf_to_us(crsf->channels[ARM_CHANNEL]);
                uint16_t throttle_ch = crsf_to_us(crsf->channels[THROTTLE_CHANNEL]);
                
                if (arm_ch < ARM_THRESHOLD_LOW) {
                    armed = false;
                } else if (!armed && arm_ch > ARM_THRESHOLD_HIGH) {
                    if (throttle_ch >= THROTTLE_NEUTRAL_MIN && 
                        throttle_ch <= THROTTLE_NEUTRAL_MAX) {
                        armed = true;
                    }
                }
            }
            
            /* Output logic - use ESC driver for motor */
            if (armed) {
                pwm_set_crsf(PWM_STEERING, crsf->channels[0]);
                pwm_set_crsf(PWM_EBRAKE,   crsf->channels[3]);
                esc_set_throttle_crsf(crsf->channels[2]);
            } else {
                pwm_set_pulse(PWM_STEERING, PWM_PULSE_CENTER);
                pwm_set_pulse(PWM_EBRAKE,   PWM_PULSE_CENTER);
                esc_set_throttle(ESC_THROTTLE_CENTER);
            }
            
            /* Display current state */
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
            usb_cdc_print(" LQ=");
            print_int(crsf->link.uplink_link_quality);
            usb_cdc_print("%    ");
            
            target_delay_ms(50);
        }
        
        usb_cdc_read_byte();
        usb_cdc_print("\r\n");
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
    /* Initialize IMU */
    imu_ok = icm42688_init();
    
    /* Initialize CRSF receiver */
    crsf_ok = crsf_init();
    
    /* Initialize PWM outputs */
    pwm_ok = pwm_init();
    
    /* Initialize ESC driver (defaults to PWM mode) */
    esc_ok = esc_init(ESC_MODE_SRXL2);
}

void flight_init(void)
{
    /* TODO: Initialize flight control
     * - Gyro filtering
     * - Stabilizer PID
     * - Mixer
     */
}

void config_init(void)
{
    /* TODO: Load configuration from EEPROM/flash */
}

