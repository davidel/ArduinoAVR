
#include <Arduino.h>
#include <EEPROM.h>
#include <stdint.h>
#include <string.h>
#include "utils.h"

uint16_t eeprom_read_le16(int address)
{
    uint8_t b1 = EEPROM.read(address);
    uint8_t b2 = EEPROM.read(address + 1);

    return ((uint16_t) b2 << 8) | b1;
}

uint32_t eeprom_read_le32(int address)
{
    uint8_t b1 = EEPROM.read(address);
    uint8_t b2 = EEPROM.read(address + 1);
    uint8_t b3 = EEPROM.read(address + 2);
    uint8_t b4 = EEPROM.read(address + 3);

    return ((uint32_t) b4 << 24) | ((uint32_t) b3 << 16) | ((uint16_t) b2 << 8) | b1;
}

void eeprom_write_le16(int address, uint16_t val)
{
    EEPROM.write(address, val);
    EEPROM.write(address + 1, val >> 8);
}

void eeprom_write_le32(int address, uint32_t val)
{
    EEPROM.write(address, val);
    EEPROM.write(address + 1, val >> 8);
    EEPROM.write(address + 2, val >> 16);
    EEPROM.write(address + 3, val >> 24);
}

