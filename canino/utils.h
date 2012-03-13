
#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include <stdint.h>
#include <string.h>

uint16_t eeprom_read_le16(int address);
uint32_t eeprom_read_le32(int address);
void eeprom_write_le16(int address, uint16_t val);
void eeprom_write_le32(int address, uint32_t val);

