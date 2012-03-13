
#include <Arduino.h>
#include <EEPROM.h>
#include <stdint.h>
#include <string.h>
#include "utils.h"
#include "config.h"

config::config() :
    magic(config_magic),
    uart_speed(default_uart_speed),
    mcpctrl_speed(default_mcpctrl_speed),
    data_filler(default_data_filler)
{

}

bool config::load()
{
    int address = config_base_address;
    uint16_t m = eeprom_read_le16(address);
    uint16_t s = eeprom_read_le16(address + 2);

    if (m != config_magic || s != sizeof(config))
        return false;
    address += 4;

    uart_speed = eeprom_read_le32(address);
    address += 4;

    mcpctrl_speed = eeprom_read_le16(address);
    address += 2;

    data_filler = eeprom_read_le16(address);
    address += 2;

    return true;
}

void config::save()
{
    int address = config_base_address;

    eeprom_write_le16(address, magic);
    address += 2;

    eeprom_write_le16(address, sizeof(config));
    address += 2;

    eeprom_write_le32(address, uart_speed);
    address += 4;

    eeprom_write_le16(address, mcpctrl_speed);
    address += 2;

    eeprom_write_le16(address, data_filler);
    address += 2;
}

