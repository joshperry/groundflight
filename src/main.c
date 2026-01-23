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
        
        /* TODO: Add main loop processing
         * 
         * Target loop structure (1kHz):
         *   1. Read CRSF channels
         *   2. Read IMU gyro data
         *   3. Update ESC telemetry
         *   4. Run stabilizer algorithm
         *   5. Output to servos
         */
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
        usb_cdc_print("  pass      - Passthrough mode: CRSF -> servos (any key to stop)\r\n");
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
        
        usb_cdc_print("  ESC:      Not initialized\r\n");
        
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
        usb_cdc_print("Passthrough mode: CRSF -> Servos (any key to stop)\r\n");
        usb_cdc_print("  CH1->Steering  CH2->Aux  CH3->Throttle  CH4->Ebrake\r\n\r\n");
        
        while (usb_cdc_available() == 0) {
            crsf_process();
            
            const crsf_state_t *crsf = crsf_get_state();
            
            if (!crsf->failsafe) {
                /* Map CRSF channels to PWM outputs */
                /* AETR: CH0=Ail(steering), CH1=Ele, CH2=Thr, CH3=Rud */
                pwm_set_crsf(PWM_STEERING, crsf->channels[0]);  /* CH1 -> Steering */
                pwm_set_crsf(PWM_AUX,      crsf->channels[1]);  /* CH2 -> Aux */
                pwm_set_crsf(PWM_THROTTLE, crsf->channels[2]);  /* CH3 -> Throttle */
                pwm_set_crsf(PWM_EBRAKE,   crsf->channels[3]);  /* CH4 -> E-brake */
            }
            
            /* Display current state */
            usb_cdc_print("\rSteer:");
            print_int(pwm_get_pulse(PWM_STEERING));
            usb_cdc_print(" Thr:");
            print_int(pwm_get_pulse(PWM_THROTTLE));
            usb_cdc_print(" Brk:");
            print_int(pwm_get_pulse(PWM_EBRAKE));
            usb_cdc_print(" Aux:");
            print_int(pwm_get_pulse(PWM_AUX));
            
            if (crsf->failsafe) {
                usb_cdc_print(" [FAILSAFE]");
            }
            usb_cdc_print("     ");
            
            target_delay_ms(20);  /* ~50Hz update */
        }
        
        /* Return servos to center on exit */
        pwm_set_pulse(PWM_STEERING, PWM_PULSE_CENTER);
        pwm_set_pulse(PWM_THROTTLE, PWM_PULSE_CENTER);
        pwm_set_pulse(PWM_EBRAKE, PWM_PULSE_CENTER);
        pwm_set_pulse(PWM_AUX, PWM_PULSE_CENTER);
        
        usb_cdc_read_byte();
        usb_cdc_print("\r\nPassthrough stopped, servos centered.\r\n");
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

