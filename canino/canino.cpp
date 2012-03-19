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
    Serial.print(", DFIL:");
    Serial.print(cfg.data_filler, HEX);
    Serial.print(", VMSG:");
    Serial.print(cfg.verbose_msg, DEC);
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
        else if (strcmp(var, "uas") == 0)
            cfg.uart_speed = (uint32_t) strtol(val, NULL, 0);
        else if (strcmp(var, "cans") == 0)
            cfg.mcpctrl_speed = (uint16_t) strtol(val, NULL, 0);
        else if (strcmp(var, "vmsg") == 0)
            cfg.verbose_msg = (uint8_t) strtol(val, NULL, 0);
    }
}

static void read_can_register()
{
    strtok(cmdbuf, " \t");

    const char* reg = strtok(NULL, " \t");

    if (reg != NULL)
    {
        uint8_t mreg = (uint8_t) strtol(reg, NULL, 0);
        uint8_t rval = mcp2515_read_register(mreg);

        Serial.print(rval, HEX);
        Serial.print('\n');
    }
}

static void write_can_register()
{
    strtok(cmdbuf, " \t");

    const char* reg = strtok(NULL, " \t");
    const char* val;

    if (reg != NULL && (val = strtok(NULL, " \t")) != NULL)
    {
        uint8_t mreg = (uint8_t) strtol(reg, NULL, 0);
        uint8_t mval = (uint8_t) strtol(val, NULL, 0);

        mcp2515_write_register(mreg, mval);
    }
}

static void set_filters()
{
    strtok(cmdbuf, " \t");

    const char* mtok = strtok(NULL, " \t");

    if (mtok != NULL)
    {
        const char* val;
        can_filter canf;

        memset(&canf, 0, sizeof(canf));
        canf.mask = strtol(mtok, NULL, 0);

        for (uint8_t i = 0; i < COUNT_OF(canf.filt) &&
                 (val = strtok(NULL, " \t")) != NULL; i++) {
            uint32_t fval = strtol(val, NULL, 0);

            if ((fval >> 11) != 0)
                fval |= FILTER_EID;
            canf.filt[i] = fval;
        }

        operation_mode omode = mcp2515_get_operation_mode();

        mcp2515_set_operation_mode(OPMODE_CONFIG);
        mcp2515_set_filter_mode(RX_FILTER_ENABLED);
        mcp2515_write_filters(&canf);
        mcp2515_set_operation_mode(omode);
    }
}

static void clear_filters()
{
    operation_mode omode = mcp2515_get_operation_mode();

    mcp2515_set_operation_mode(OPMODE_CONFIG);
    mcp2515_set_filter_mode(RX_FILTER_DISABLED);
    mcp2515_set_operation_mode(omode);
}

static void show_filters()
{
    operation_mode omode = mcp2515_get_operation_mode();
    can_filter canf;

    mcp2515_set_operation_mode(OPMODE_CONFIG);

    mcp2515_read_filters(&canf);

    Serial.print("MSK:");
    Serial.print(canf.mask, HEX);
    for (uint8_t i = 0; i < COUNT_OF(canf.filt); i++) {
        Serial.print(", ");
        Serial.print(canf.filt[i] & ~FILTER_EID, HEX);
    }
    Serial.print('\n');
    mcp2515_set_operation_mode(omode);
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
    else if (strncmp(cmdbuf, "cfg ", 4) == 0)
        edit_config();
    else if (strncmp(cmdbuf, "rcr ", 4) == 0)
        read_can_register();
    else if (strncmp(cmdbuf, "wcr ", 4) == 0)
        write_can_register();
    else if (strcmp(cmdbuf, "scfg") == 0)
        cfg.save();
    else if (strcmp(cmdbuf, "dcfg") == 0)
        show_config();
    else if (strncmp(cmdbuf, "fset ", 5) == 0)
        set_filters();
    else if (strcmp(cmdbuf, "fclr") == 0)
        clear_filters();
    else if (strcmp(cmdbuf, "fshw") == 0)
        show_filters();
    else if (strcmp(cmdbuf, "loop") == 0)
        mcp2515_set_operation_mode(OPMODE_LOOPBACK);
    else if (strcmp(cmdbuf, "lstn") == 0)
        mcp2515_set_operation_mode(OPMODE_LISTEN);
    else if (strcmp(cmdbuf, "stdm") == 0)
        mcp2515_set_operation_mode(OPMODE_NORMAL);
    else {
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
    if (cfg.verbose_msg) {
        Serial.print(", RTR:");
        Serial.print(message.rtr, BIN);
        Serial.print(", SRR:");
        Serial.print(message.srr, BIN);
        Serial.print(", IDE:");
        Serial.print(message.ide, BIN);
        Serial.print(", LEN:");
        Serial.print(message.length, DEC);
    } else {
        Serial.print(' ');
        if (message.rtr)
            Serial.print('R');
        if (message.srr)
            Serial.print('S');
        if (message.ide)
            Serial.print('I');
    }
    for (uint8_t i = 0; i < message.length; i++) {
        Serial.print(' ');
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

