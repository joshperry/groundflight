/**
 * EEPROM/Flash Storage
 */

#ifndef EEPROM_H
#define EEPROM_H

#include <stdint.h>
#include <stdbool.h>

bool eeprom_init(void);
bool eeprom_read(uint32_t address, void *data, uint32_t size);
bool eeprom_write(uint32_t address, const void *data, uint32_t size);
void eeprom_erase(void);

#endif /* EEPROM_H */
