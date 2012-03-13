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

#if ARDUINO >= 100
#include <Arduino.h> // Arduino 1.0
#else
#include <Wprogram.h> // Arduino 0022
#endif
#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "macros.h"
#include "can_ctrl_pins.h"
#include "mcp2515_defs.h"
#include "mcp2515.h"

struct can_timing_cfg
{
    uint8_t t_prop;
    uint8_t t_ps1;
    uint8_t t_ps2;
    uint8_t brp;
    uint16_t err;
};

#define CAN_BASE_BRP1_FREQ ((F_CPU / 1000) / 2)

// Max Time Quanta per bit: 1 Sync, Max 8 Prop, Max 8 PS1, Max 8 PS2
#define CAN_MAX_TQ_X_BIT 25

static bool mcp2515_select_timing(uint16_t speed, can_timing_cfg* tcfg)
{
    uint32_t can_cpu_freq = CAN_BASE_BRP1_FREQ;
    uint16_t tq_x_bit = CAN_BASE_BRP1_FREQ / speed;
    uint8_t brp = 1;

    while (tq_x_bit > CAN_MAX_TQ_X_BIT) {
        brp++;
        can_cpu_freq >>= 1;
        tq_x_bit = can_cpu_freq / speed;
    }
    if (brp > 8)
        return false;

    memset(tcfg, 0, sizeof(*tcfg));
    tcfg->err = can_cpu_freq % speed;
    tcfg->brp = brp;

    // Drop sync TQ.
    tq_x_bit--;

    tcfg->t_prop = tq_x_bit / 8;
    if (tcfg->t_prop == 0)
        tcfg->t_prop = 1;

    tq_x_bit -= tcfg->t_prop;
    if (tq_x_bit > 16) {
        tcfg->t_prop += tq_x_bit - 16;
        tq_x_bit = 16;
    }

    if ((tq_x_bit % 2) == 0)
        tcfg->t_ps1 = tcfg->t_ps2 = tq_x_bit / 2;
    else {
        tcfg->t_prop += 1;
        tcfg->t_ps1 = tcfg->t_ps2 = (tq_x_bit - 1) / 2;
    }

    return true;
}

uint8_t spi_putc(uint8_t data)
{
    // put byte in send-buffer
    SPDR = data;

    // wait until byte was send
    while ((SPSR & (1 << SPIF)) == 0)
        ;

    return SPDR;
}

void mcp2515_write_register(uint8_t adress, uint8_t data)
{
    RESET(MCP2515_CS);

    spi_putc(SPI_WRITE);
    spi_putc(adress);
    spi_putc(data);

    SET(MCP2515_CS);
}

uint8_t mcp2515_read_register(uint8_t adress)
{
    uint8_t data;

    RESET(MCP2515_CS);

    spi_putc(SPI_READ);
    spi_putc(adress);

    data = spi_putc(0xff);

    SET(MCP2515_CS);

    return data;
}

void mcp2515_bit_modify(uint8_t adress, uint8_t mask, uint8_t data)
{
    RESET(MCP2515_CS);

    spi_putc(SPI_BIT_MODIFY);
    spi_putc(adress);
    spi_putc(mask);
    spi_putc(data);

    SET(MCP2515_CS);
}

uint8_t mcp2515_read_status(uint8_t type)
{
    uint8_t data;

    RESET(MCP2515_CS);

    spi_putc(type);
    data = spi_putc(0xff);

    SET(MCP2515_CS);

    return data;
}

bool mcp2515_init(uint16_t speed)
{
    can_timing_cfg tcfg;

    if (!mcp2515_select_timing(speed, &tcfg))
        return false;

    SET(MCP2515_CS);
    SET_OUTPUT(MCP2515_CS);

    RESET(P_SCK);
    RESET(P_MOSI);
    RESET(P_MISO);

    SET_OUTPUT(P_SCK);
    SET_OUTPUT(P_MOSI);
    SET_INPUT(P_MISO);

    SET_INPUT(MCP2515_INT);
    SET(MCP2515_INT);

    // active SPI master interface
    SPCR = OR_BITS4(SPE, MSTR, SPR1, SPR0);
    SPSR = 0;

    // reset MCP2515 by software reset.
    // After this he is in configuration mode.
    RESET(MCP2515_CS);
    spi_putc(SPI_RESET);
    SET(MCP2515_CS);

    // wait a little bit until the MCP2515 has restarted
    _delay_us(10);

    // load CNF3..1 and CANINTE Registers
    RESET(MCP2515_CS);
    spi_putc(SPI_WRITE);
    spi_putc(CNF3);

    // CNF3
    uint8_t cnf3 = (tcfg.t_ps2 - 1) & 0x07;

    spi_putc(cnf3);

    // CNF2
    uint8_t cnf2 = (tcfg.t_prop - 1) & 0x07;

    cnf2 |= ((tcfg.t_ps1 - 1) & 0x07) << 3;
    spi_putc(cnf2 | (1 << BTLMODE));

    // CNF1
    uint8_t cnf1 = (tcfg.brp - 1) & 0x3f;

    spi_putc(cnf1);

    // CANINTE, activate interrupts
    spi_putc(OR_BITS2(RX0IE, RX1IE));
    SET(MCP2515_CS);

    // test if we could read back the value => is the chip accessible?
    if (mcp2515_read_register(CNF1) != cnf1)
        return false;

    // deaktivate the RXnBF Pins (High Impedance State)
    mcp2515_write_register(BFPCTRL, 0);

    // set TXnRTS as inputs
    mcp2515_write_register(TXRTSCTRL, 0);

    // turn off filters => receive any message
    mcp2515_write_register(RXB0CTRL, OR_BITS2(RXM0, RXM1));
    mcp2515_write_register(RXB1CTRL, OR_BITS2(RXM0, RXM1));

    // reset device to normal mode
    mcp2515_write_register(CANCTRL, 0);

    return true;
}

bool mcp2515_check_message()
{
    return !IS_SET(MCP2515_INT);
}

bool mcp2515_check_free_buffer()
{
    uint8_t status = mcp2515_read_status(SPI_READ_STATUS);

    if ((status & 0x54) == 0x54)
        // all buffers used
        return false;

    return true;
}

uint8_t mcp2515_get_message(can_message* message)
{
    uint8_t status = mcp2515_read_status(SPI_RX_STATUS);
    uint8_t addr;
    uint8_t t;

    if (bit_is_set(status,6))
        // message in buffer 0
        addr = SPI_READ_RX;
    else if (bit_is_set(status,7))
        // message in buffer 1
        addr = SPI_READ_RX | 0x04;
    else
        // Error: no message available
        return 0;

    RESET(MCP2515_CS);
    spi_putc(addr);

    // read id
    uint16_t sid = (uint16_t) spi_putc(0xff) << 3;
    uint8_t sidl = spi_putc(0xff);

    sid |= sidl >> 5;
    message->srr = (bit_is_set(sidl, 4)) ? 1: 0;
    message->ide = (bit_is_set(sidl, 3)) ? 1: 0;

    uint8_t eid_hi = spi_putc(0xff);
    uint8_t eid_lo = spi_putc(0xff);

    message->id = sid;
    if (message->ide)
        message->id |= ((uint32_t) eid_lo << 11) | ((uint32_t) eid_hi << 19) |
            ((uint32_t) (sidl & 0x03) << 27);

    // read DLC
    uint8_t rtr_dlc = spi_putc(0xff);
    uint8_t length = rtr_dlc & 0x0f;

    message->length = length;
    message->rtr = (message->ide != 0 && bit_is_set(rtr_dlc, 3)) ? 1: 0;

    // read data
    for (t = 0; t < length; t++)
        message->data[t] = spi_putc(0xff);
    SET(MCP2515_CS);

    // clear interrupt flag
    if (bit_is_set(status, 6))
        mcp2515_bit_modify(CANINTF, (1 << RX0IF), 0);
    else
        mcp2515_bit_modify(CANINTF, (1 << RX1IF), 0);

    return (status & 0x07) + 1;
}

uint8_t mcp2515_send_message(can_message* message)
{
    uint8_t status = mcp2515_read_status(SPI_READ_STATUS);

    /* Statusbyte:
     *
     * Bit  Function
     *  2   TXB0CNTRL.TXREQ
     *  4   TXB1CNTRL.TXREQ
     *  6   TXB2CNTRL.TXREQ
     */
    uint8_t address;
    uint8_t t;

    if (bit_is_clear(status, 2))
        address = 0x00;
    else if (bit_is_clear(status, 4))
        address = 0x02;
    else if (bit_is_clear(status, 6))
        address = 0x04;
    else
    {
        // all buffer used => could not send message
        return 0;
    }

    RESET(MCP2515_CS);
    spi_putc(SPI_WRITE_TX | address);

    spi_putc(message->id >> 3);

    uint8_t sidl = (uint8_t) (message->id << 5);

    if (message->ide)
        sidl |= 1 << 3;
    sidl |= (uint8_t) (message->id >> 27) & 0x03;

    spi_putc(sidl);
    if (message->ide) {
        spi_putc(message->id >> 19);
        spi_putc(message->id >> 11);
    } else {
        spi_putc(0xff);
        spi_putc(0xff);
    }

    uint8_t length = message->length & 0x0f;

    if (message->rtr)
    {
        // a rtr-frame has a length, but contains no data
        spi_putc((1 << RTR) | length);
    }
    else
    {
        // set message length
        spi_putc(length);

        // data
        for (t = 0; t < length; t++)
            spi_putc(message->data[t]);
    }
    SET(MCP2515_CS);

    _delay_us(1);

    // send message
    RESET(MCP2515_CS);
    address = (address == 0) ? 1 : address;
    spi_putc(SPI_RTS | address);
    SET(MCP2515_CS);

    return address;
}

