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
#include "config.h"

config::config() :
    uart_speed(default_uart_speed),
    mcpctrl_speed(default_mcpctrl_speed),
    data_filler(default_data_filler),
    verbose_msg(default_verbose_msg)
{

}

bool config::load()
{
    int address = config_base_address;
    uint16_t m = eeprom_read_le16(address);
    uint16_t s = eeprom_read_le16(address + 2);

    if (m != magic || s != sizeof(config))
        return false;
    address += 4;

    eeprom_read(address, this, sizeof(*this));

    return true;
}

void config::save()
{
    int address = config_base_address;

    eeprom_write_le16(address, magic);
    address += 2;

    eeprom_write_le16(address, sizeof(config));
    address += 2;

    eeprom_write(address, this, sizeof(*this));
}

