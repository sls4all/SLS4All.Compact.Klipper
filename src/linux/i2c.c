// Linux i2c implementation
//
// Copyright (C) 2020  Eric Callahan <arksine.code@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#include <fcntl.h> // open
#include <linux/i2c-dev.h> // I2C_SLAVE
#include <stdio.h> // snprintf
#include <string.h> // memset
#include <sys/ioctl.h> // ioctl
#include <unistd.h> // write
#include "gpio.h" // i2c_setup
#include "command.h" // shutdown
#include "internal.h" // report_errno
#include "sched.h" // sched_shutdown

#include <linux/i2c-dev.h>

#ifndef I2C_FUNC_I2C
#include <linux/i2c.h>
#define I2C_MSG_FMT __u8
#endif
#define I2C_TRIES 3

// PD: increased i2c bus count for RockPi
DECL_ENUMERATION_RANGE("i2c_bus", "i2c.0", 0, 10);

struct i2c_s {
    uint32_t bus;
    uint8_t addr;
    int fd;
    int valid;
};

static struct i2c_s devices[16];
static int devices_count;

static int
i2c_open(uint32_t bus, uint8_t addr)
{
    // Find existing device (if already opened)
    int i;
    for (i=0; i<devices_count; i++) {
        if (devices[i].bus == bus && devices[i].addr == addr) {
            return devices[i].fd;
        }
    }

    char fname[256];
    snprintf(fname, sizeof(fname), "/dev/i2c-%d", bus);
    int fd = open(fname, O_RDWR|O_CLOEXEC);
    if (fd < 0) {
        report_errno("open i2c", fd);
        goto fail;
    }
    int ret = ioctl(fd, I2C_SLAVE, addr);
    if (ret < 0) {
        report_errno("ioctl i2c", ret);
        goto fail;
    }
    ret = set_non_blocking(fd);
    if (ret < 0)
    {
        report_errno("set_non_blocking i2c", ret);
        goto fail;
    }

    devices[devices_count].bus = bus;
    devices[devices_count].addr = addr;
    devices[devices_count].fd = fd;
    devices[devices_count].valid = 1;
    devices_count++;

    return fd;

fail:
    if (fd >= 0)
        close(fd);
    shutdown("Unable to open i2c device");
}

static uint8_t
i2c_try_reopen(struct i2c_config *config)
{
    // Find existing device (if already opened)
    int i;
    struct i2c_s *device = NULL;
    for (i=0; i<devices_count; i++) {
        if (devices[i].fd == config->fd) {
            device = &devices[i];
            break;
        }
    }
    if (!device)
    {
        shutdown("Unable to reopen i2c device, no existing device descriptor found");
        return 0;
    }

    uint32_t bus = device->bus;
    uint8_t addr = device->addr;

    // close existing device
    if (device->valid)
    {
        close(device->fd);
        device->valid = 0;
    }

    // reopen
    char fname[256];
    snprintf(fname, sizeof(fname), "/dev/i2c-%d", bus);
    int fd = open(fname, O_RDWR|O_CLOEXEC);
    if (fd < 0) {
        report_errno("open i2c", fd);
        goto fail;
    }
    int ret = ioctl(fd, I2C_SLAVE, addr);
    if (ret < 0) {
        report_errno("ioctl i2c", ret);
        goto fail;
    }
    ret = set_non_blocking(fd);
    if (ret < 0)
    {
        report_errno("set_non_blocking i2c", ret);
        goto fail;
    }

    device->fd = fd;
    config->fd = fd;
    device->valid = 1;

    return 1;
fail:
    if (fd >= 0)
        close(fd);
    return 0;
}

struct i2c_config
i2c_setup(uint32_t bus, uint32_t rate, uint8_t addr)
{
    // Note:  The rate is set by the kernel driver, for a Raspberry Pi this
    // is done with the following setting in /boot/config.txt:
    //
    // dtparam=i2c_baudrate=<rate>

    int fd = i2c_open(bus, addr);
    return (struct i2c_config){.fd=fd, .addr=addr};
}

void
i2c_write(struct i2c_config config, uint16_t write_len, uint8_t *data)
{
    struct i2c_msg i2c_messages[1];
    struct i2c_rdwr_ioctl_data i2c_messageset[1];
    int ret;

    for (int itry = 0; itry < I2C_TRIES; itry++)
    {
        i2c_messages[0].addr = config.addr;
        i2c_messages[0].flags = 0;
        i2c_messages[0].len = write_len;
        i2c_messages[0].buf = (I2C_MSG_FMT *)data;

        i2c_messageset[0].msgs = i2c_messages;
        i2c_messageset[0].nmsgs = 1;

        ret = ioctl(config.fd, I2C_RDWR, &i2c_messageset); 
        if (ret >= 0)
            return;
    }
    report_errno("ioctl i2c write", ret);        
    try_shutdown("Unable write i2c device");
}

void
i2c_read(struct i2c_config config, uint16_t reg_len, uint8_t *reg
         , uint16_t read_len, uint8_t *data)
{
    struct i2c_msg i2c_messages[2];
    struct i2c_rdwr_ioctl_data i2c_messageset[1];
    int ret;

    for (int itry = 0; itry < I2C_TRIES; itry++)
    {
        if (reg_len)
        {
            i2c_messages[0].addr = config.addr;
            i2c_messages[0].flags = 0;
            i2c_messages[0].len = reg_len;
            i2c_messages[0].buf = (I2C_MSG_FMT *)reg;

            i2c_messages[1].addr = config.addr;
            i2c_messages[1].flags = I2C_M_RD | I2C_M_NOSTART;
            i2c_messages[1].len = read_len;
            i2c_messages[1].buf = (I2C_MSG_FMT *)data;

            i2c_messageset[0].msgs = i2c_messages;
            i2c_messageset[0].nmsgs = 2;
        }
        else
        {
            i2c_messages[0].addr = config.addr;
            i2c_messages[0].flags = I2C_M_RD | I2C_M_NOSTART;
            i2c_messages[0].len = read_len;
            i2c_messages[0].buf = (I2C_MSG_FMT *)data;

            i2c_messageset[0].msgs = i2c_messages;
            i2c_messageset[0].nmsgs = 1;
        }

        memset(data, 0, read_len);

        ret = ioctl(config.fd, I2C_RDWR, &i2c_messageset);
        if (ret >= 0)
            return;
    }
    report_errno("ioctl i2c read/write", ret);        
    try_shutdown("Unable read/write i2c device");
}

uint8_t
i2c_write_ok(struct i2c_config *config, uint16_t write_len, uint8_t *data)
{
    struct i2c_msg i2c_messages[1];
    struct i2c_rdwr_ioctl_data i2c_messageset[1];
    int ret;

    i2c_messages[0].addr = config->addr;
    i2c_messages[0].flags = 0;
    i2c_messages[0].len = write_len;
    i2c_messages[0].buf = (I2C_MSG_FMT *)data;

    i2c_messageset[0].msgs = i2c_messages;
    i2c_messageset[0].nmsgs = 1;

    ret = ioctl(config->fd, I2C_RDWR, &i2c_messageset); 
    if (ret >= 0)
        return 1;
    report_errno("ioctl i2c read/write (ok variant) will try reopen", ret);        
    i2c_try_reopen(config);
    return 0;
}

uint8_t
i2c_read_ok(struct i2c_config *config, uint16_t reg_len, uint8_t *reg
         , uint16_t read_len, uint8_t *data)
{
    struct i2c_msg i2c_messages[2];
    struct i2c_rdwr_ioctl_data i2c_messageset[1];
    int ret;

    if (reg_len)
    {
        i2c_messages[0].addr = config->addr;
        i2c_messages[0].flags = 0;
        i2c_messages[0].len = reg_len;
        i2c_messages[0].buf = (I2C_MSG_FMT *)reg;

        i2c_messages[1].addr = config->addr;
        i2c_messages[1].flags = I2C_M_RD | I2C_M_NOSTART;
        i2c_messages[1].len = read_len;
        i2c_messages[1].buf = (I2C_MSG_FMT *)data;

        i2c_messageset[0].msgs = i2c_messages;
        i2c_messageset[0].nmsgs = 2;
    }
    else
    {
        i2c_messages[0].addr = config->addr;
        i2c_messages[0].flags = I2C_M_RD | I2C_M_NOSTART;
        i2c_messages[0].len = read_len;
        i2c_messages[0].buf = (I2C_MSG_FMT *)data;

        i2c_messageset[0].msgs = i2c_messages;
        i2c_messageset[0].nmsgs = 1;
    }

    memset(data, 0, read_len);

    ret = ioctl(config->fd, I2C_RDWR, &i2c_messageset);
    if (ret >= 0)
        return 1;
    report_errno("ioctl i2c read/write (ok variant) will try reopen", ret);        
    i2c_try_reopen(config);
    return 0;
}
