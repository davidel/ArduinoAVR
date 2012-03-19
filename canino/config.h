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

#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <string.h>

struct config
{
    static const int config_base_address = 0;
    static const uint16_t magic = 0xdade;
    static const uint32_t default_uart_speed = 115200;
    static const uint16_t default_mcpctrl_speed = 500;
    static const uint16_t default_data_filler = 0xffff;
    static const uint8_t default_verbose_msg = 0;

    config();
    bool load();
    void save();

    uint32_t uart_speed;
    uint16_t mcpctrl_speed;
    uint16_t data_filler;
    uint8_t verbose_msg;
};

