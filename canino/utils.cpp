/*
 *  canino by Davide Libenzi (CAN bus interface software for AVR Arduino boards)
 *  Copyright (C) 2012  Davide Libenzi
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  Davide Libenzi <davidel@xmailserver.org>
 *
 */

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

void eeprom_write(int address, const void* data, int size)
{
    for (int i = 0; i < size; i++)
        EEPROM.write(address + i, ((const uint8_t*) data)[i]);
}

void eeprom_read(int address, void* data, int size)
{
    for (int i = 0; i < size; i++)
        ((uint8_t*) data)[i] = EEPROM.read(address + i);
}

