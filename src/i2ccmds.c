// Commands for sending messages on an I2C bus
//
// Copyright (C) 2018  Florian Heilmann <Florian.Heilmann@gmx.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "basecmd.h" //oid_alloc
#include "command.h"  //sendf
#include "sched.h" //DECL_COMMAND
#include "board/gpio.h" //i2c_write/read/setup

struct i2cdev_s {
    struct i2c_config i2c_config;
};

void
command_config_i2c(uint32_t *args)
{
    uint8_t addr = args[3] & 0x7f;
    struct i2cdev_s *i2c = oid_alloc(args[0], command_config_i2c
                                     , sizeof(*i2c));
    i2c->i2c_config = i2c_setup(args[1], args[2], addr);
}
DECL_COMMAND(command_config_i2c,
             "config_i2c oid=%c i2c_bus=%u rate=%u address=%u");

void
command_i2c_write(uint32_t *args)
{
    uint8_t oid = args[0];
    struct i2cdev_s *i2c = oid_lookup(oid, command_config_i2c);
    uint16_t data_len = args[1];
    uint8_t *data = (void*)(size_t)args[2];
    i2c_write(i2c->i2c_config, data_len, data);
}
DECL_COMMAND(command_i2c_write, "i2c_write oid=%c data=%*s");

void
command_i2c_write_ok(uint32_t *args)
{
    uint8_t oid = args[0];
    struct i2cdev_s *i2c = oid_lookup(oid, command_config_i2c);
    uint16_t data_len = args[1];
    uint8_t *data = (void*)(size_t)args[2];
    uint8_t ok = i2c_write_ok(&i2c->i2c_config, data_len, data);
    sendf("i2c_write_ok_response oid=%c ok=%c response=%*s", oid, ok, data_len, data);
}
DECL_COMMAND(command_i2c_write_ok, "i2c_write_ok oid=%c data=%*s");

void
command_i2c_read(uint32_t * args)
{
    uint8_t oid = args[0];
    struct i2cdev_s *i2c = oid_lookup(oid, command_config_i2c);
    uint16_t reg_len = args[1];
    uint8_t *reg = (void*)(size_t)args[2];
    uint16_t data_len = args[3];
    uint8_t receive_array[data_len];
    uint8_t *data = (void*)(size_t)receive_array;
    i2c_read(i2c->i2c_config, reg_len, reg, data_len, data);
    sendf("i2c_read_response oid=%c response=%*s", oid, data_len, data);
}
DECL_COMMAND(command_i2c_read, "i2c_read oid=%c reg=%*s read_len=%u");

void
command_i2c_read_ok(uint32_t * args)
{
    uint8_t oid = args[0];
    struct i2cdev_s *i2c = oid_lookup(oid, command_config_i2c);
    uint16_t reg_len = args[1];
    uint8_t *reg = (void*)(size_t)args[2];
    uint16_t data_len = args[3];
    uint8_t receive_array[data_len];
    uint8_t *data = (void*)(size_t)receive_array;
    uint8_t ok = i2c_read_ok(&i2c->i2c_config, reg_len, reg, data_len, data);
    sendf("i2c_read_ok_response oid=%c ok=%c response=%*s", oid, ok, data_len, data);
}
DECL_COMMAND(command_i2c_read_ok, "i2c_read_ok oid=%c reg=%*s read_len=%u");

void
command_i2c_modify_bits(uint32_t *args)
{
    uint8_t oid = args[0];
    struct i2cdev_s *i2c = oid_lookup(oid, command_config_i2c);
    uint16_t reg_len = args[1];
    uint8_t *reg = (void*)(size_t)args[2];
    uint32_t clear_set_len = args[3];
    if (clear_set_len % 2 != 0) {
        shutdown("i2c_modify_bits: Odd number of bits!");
    }
    uint16_t data_len = clear_set_len/2;
    uint8_t *clear_set = (void*)(size_t)args[4];
    uint8_t receive_array[reg_len + data_len];
    uint8_t *receive_data = (void*)(size_t)receive_array;
    for (int i = 0; i < reg_len; i++) {
        *(receive_data + i) = *(reg + i);
    }
    i2c_read(i2c->i2c_config, reg_len, reg, data_len, receive_data + reg_len);
    for (int i = 0; i < data_len; i++) {
        *(receive_data + reg_len + i) &= ~(*(clear_set + i));
        *(receive_data + reg_len + i) |= *(clear_set + clear_set_len/2 + i);
    }
    i2c_write(i2c->i2c_config, reg_len + data_len, receive_array);
}
DECL_COMMAND(command_i2c_modify_bits,
             "i2c_modify_bits oid=%c reg=%*s clear_set_bits=%*s");
