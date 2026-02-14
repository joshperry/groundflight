/**
 * Mock EEPROM for host-side testing
 *
 * Backs read/write with an in-memory array.
 */

#ifndef MOCK_EEPROM_H
#define MOCK_EEPROM_H

#include <stdint.h>
#include <stdbool.h>

#define MOCK_EEPROM_SIZE 4096

extern uint8_t mock_eeprom_data[MOCK_EEPROM_SIZE];

void mock_eeprom_reset(void);

/* Standard eeprom.h API */
bool eeprom_init(void);
bool eeprom_read(uint32_t address, void *data, uint32_t size);
bool eeprom_write(uint32_t address, const void *data, uint32_t size);
void eeprom_erase(void);

#endif /* MOCK_EEPROM_H */
