/**
 * Mock EEPROM implementation
 */

#include "mock_eeprom.h"
#include <string.h>

uint8_t mock_eeprom_data[MOCK_EEPROM_SIZE];

void mock_eeprom_reset(void)
{
    memset(mock_eeprom_data, 0xFF, MOCK_EEPROM_SIZE);
}

bool eeprom_init(void)
{
    return true;
}

bool eeprom_read(uint32_t address, void *data, uint32_t size)
{
    if (address + size > MOCK_EEPROM_SIZE) return false;
    memcpy(data, &mock_eeprom_data[address], size);
    return true;
}

bool eeprom_write(uint32_t address, const void *data, uint32_t size)
{
    if (address + size > MOCK_EEPROM_SIZE) return false;
    memcpy(&mock_eeprom_data[address], data, size);
    return true;
}

void eeprom_erase(void)
{
    memset(mock_eeprom_data, 0xFF, MOCK_EEPROM_SIZE);
}
