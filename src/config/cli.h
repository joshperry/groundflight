/**
 * Serial CLI Interface
 */

#ifndef CLI_H
#define CLI_H

#include <stdint.h>
#include <stdbool.h>

void cli_init(void);
void cli_process(void);  /* Call from main loop */
void cli_print(const char *str);
void cli_printf(const char *fmt, ...);

/* Built-in commands:
 *   help      - List commands
 *   status    - Show system status
 *   get <var> - Get config variable
 *   set <var> - Set config variable  
 *   save      - Save config to flash
 *   defaults  - Reset to defaults
 *   cal       - Calibrate gyro
 *   dump      - Dump all config
 *   dfu       - Reboot to DFU bootloader
 *   reboot    - Reboot system
 */

#endif /* CLI_H */
