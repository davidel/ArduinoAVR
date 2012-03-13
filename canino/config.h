
#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <string.h>

struct config
{
    static const int config_base_address = 0;
    static const uint16_t config_magic = 0xdade;
    static const uint32_t default_uart_speed = 115200;
    static const uint16_t default_mcpctrl_speed = 500;
    static const uint16_t default_data_filler = 0xffff;

    config();
    bool load();
    void save();

    uint16_t magic;
    uint32_t uart_speed;
    uint16_t mcpctrl_speed;
    uint16_t data_filler;
};

