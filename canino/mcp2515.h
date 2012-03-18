/* Copyright (c) 2007 Fabian Greif
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
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

#include <inttypes.h>
#include "macros.h"
#include "mcp2515_defs.h"

#define FILTER_EID BV32(31)

struct can_message
{
    uint32_t id;
    uint8_t rtr: 1;
    uint8_t srr: 1;
    uint8_t ide: 1;
    uint8_t length;
    uint8_t data[8];
};

struct can_filter
{
    uint32_t mask;
    uint32_t filt[6];
};

enum rx_filter_mode
{
    RX_FILTER_ENABLED,
    RX_FILTER_ENABLED_SID,
    RX_FILTER_ENABLED_EID,
    RX_FILTER_DISABLED,
    RX_FILTER_INVALID
};

uint8_t spi_putc(uint8_t data);
void mcp2515_write_register(uint8_t address, uint8_t data);
void mcp2515_write_registers(uint8_t address, const uint8_t* data, uint8_t count);
uint8_t mcp2515_read_register(uint8_t address);
void mcp2515_read_registers(uint8_t address, uint8_t* data, uint8_t count);
void mcp2515_bit_modify(uint8_t address, uint8_t mask, uint8_t data);
uint8_t mcp2515_read_status(uint8_t type);
bool mcp2515_init(uint16_t speed);
bool mcp2515_check_message();
bool mcp2515_check_free_buffer();
uint8_t mcp2515_get_message(can_message* message);
uint8_t mcp2515_send_message(can_message* message);
rx_filter_mode mcp2515_get_filter_mode();
void mcp2515_set_filter_mode(rx_filter_mode mode);
void mcp2515_read_filters(can_filter* cfilt);

