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
#include <string.h>
#include "macros.h"
#include "can_ctrl_pins.h"
#include "mcp2515.h"
#include "config.h"

#define STD_CAN_PIN_REQUEST 0x7df

static config cfg;
static uint8_t can_init_failed;
static uint8_t all_msgs;
static can_message message;
static uint8_t cmdbuf_count;
static char cmdbuf[80];

static bool is_tester_reponse(uint32_t id)
{
    return id >= 0x7e8 && id <= 0x7ef;
}

static void show_canctrl_status()
{
    uint8_t tec = mcp2515_read_register(TEC);
    uint8_t rec = mcp2515_read_register(REC);
    uint8_t eflg = mcp2515_read_register(EFLG);

    Serial.print("TEC:");
    Serial.print(tec, DEC);
    Serial.print(", REC:");
    Serial.print(rec, DEC);
    Serial.print(", EFL:");
    Serial.print(eflg, BIN);
    Serial.print('\n');
}

static void show_config()
{
    Serial.print("UAS:");
    Serial.print(cfg.uart_speed, DEC);
    Serial.print(", CANS:");
    Serial.print(cfg.mcpctrl_speed, DEC);
    Serial.print(", DFLR:");
    Serial.print(cfg.data_filler, HEX);
    Serial.print('\n');
}

static void send_msg()
{
    const char* tok = strtok(cmdbuf, " \t");

    memset(&message, 0, sizeof(message));
    if ((tok = strtok(NULL, " \t")) == NULL)
        return;
    if (*tok != '-')
        message.id = (uint32_t) strtol(tok, NULL, 0);
    else
        message.id = STD_CAN_PIN_REQUEST;
    if ((tok = strtok(NULL, " \t")) == NULL)
        return;
    message.rtr = strtol(tok, NULL, 0) != 0;
    if ((tok = strtok(NULL, " \t")) == NULL)
        return;
    message.srr = strtol(tok, NULL, 0) != 0;
    if ((tok = strtok(NULL, " \t")) == NULL)
        return;
    message.ide = strtol(tok, NULL, 0) != 0;

    while (message.length < 8 && (tok = strtok(NULL, " \t")) != NULL) {
        message.data[message.length] = (uint8_t) strtol(tok, NULL, 0);
        message.length++;
    }
    if (cfg.data_filler <= 255) {
        while (message.length < 8) {
            message.data[message.length] = (uint8_t) cfg.data_filler;
            message.length++;
        }
    }

    mcp2515_send_message(&message);
}

static void edit_config()
{
    const char* var;
    const char* val;

    strtok(cmdbuf, " \t");
    while ((var = strtok(NULL, " \t")) != NULL &&
           (val = strtok(NULL, " \t")) != NULL) {
        if (strcmp(var, "dfil") == 0)
            cfg.data_filler = (uint16_t) strtol(val, NULL, 0);
    }
}

static void execute_cmd()
{
    if (strncmp(cmdbuf, "smsg ", 5) == 0)
        send_msg();
    else if (strcmp(cmdbuf, "all") == 0)
        all_msgs = 1;
    else if (strcmp(cmdbuf, "nall") == 0)
        all_msgs = 0;
    else if (strcmp(cmdbuf, "st") == 0)
        show_canctrl_status();
    else if (strcmp(cmdbuf, "cfg") == 0)
        edit_config();
    else if (strcmp(cmdbuf, "scfg") == 0)
        cfg.save();
    else if (strcmp(cmdbuf, "dcfg") == 0)
        show_config();
    else if (strcmp(cmdbuf, "loop") == 0) {
        uint8_t cctrl = mcp2515_read_register(CANCTRL);

        cctrl &= ~OR_BITS3(REQOP0, REQOP1, REQOP2);
        mcp2515_write_register(CANCTRL, cctrl | _BV(REQOP1));
    } else if (strcmp(cmdbuf, "lstn") == 0) {
        uint8_t cctrl = mcp2515_read_register(CANCTRL);

        cctrl &= ~OR_BITS3(REQOP0, REQOP1, REQOP2);
        mcp2515_write_register(CANCTRL, cctrl | OR_BITS2(REQOP1, REQOP0));
    } else if (strcmp(cmdbuf, "stdm") == 0) {
        uint8_t cctrl = mcp2515_read_register(CANCTRL);

        cctrl &= ~OR_BITS3(REQOP0, REQOP1, REQOP2);
        mcp2515_write_register(CANCTRL, cctrl);
    } else {
        Serial.print("cmd?: ");
        Serial.print(cmdbuf);
        Serial.print('\n');
    }
}

static void try_read_serial()
{
    int c = Serial.read();

    if (c != -1) {
        if (c == '\n') {
            cmdbuf[cmdbuf_count++] = 0;
            execute_cmd();
            cmdbuf_count = 0;
        } else if (cmdbuf_count < sizeof(cmdbuf) - 1) {
            cmdbuf[cmdbuf_count++] = (char) c;
        }
    }
}

static void print_can_message()
{
    Serial.print("ID:");
    Serial.print(message.id, HEX);
    Serial.print(", RTR:");
    Serial.print(message.rtr, BIN);
    Serial.print(", SRR:");
    Serial.print(message.srr, BIN);
    Serial.print(", IDE:");
    Serial.print(message.ide, BIN);
    Serial.print(", LEN:");
    Serial.print(message.length, DEC);

    for (uint8_t i = 0; i < message.length; i++) {
        Serial.print(" ");
        Serial.print(message.data[i], HEX);
    }
    Serial.print('\n');
}

void setup()
{
    cfg.load();
    Serial.begin(cfg.uart_speed);
    if (!mcp2515_init(cfg.mcpctrl_speed))
        can_init_failed = 1;
}

void loop()
{
    if (can_init_failed) {
        Serial.print("CAN Init Failed!\n");
        delay(1000);
    } else {
        if (mcp2515_check_message()) {
            if (mcp2515_get_message(&message)) {
                if (all_msgs != 0 || is_tester_reponse(message.id))
                    print_can_message();
            }
        }
    }

    try_read_serial();
}

