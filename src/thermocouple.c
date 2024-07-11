// Basic support for common SPI controlled thermocouple chips
//
// Copyright (C) 2018  Petri Honkala <cruwaller@gmail.com>
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
#include "board/irq.h" // irq_disable
#include "basecmd.h" // oid_alloc
#include "byteorder.h" // be32_to_cpu
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "spicmds.h" // spidev_transfer

enum {
    TS_CHIP_MAX31855, TS_CHIP_MAX31856, TS_CHIP_MAX31865, TS_CHIP_MAX6675, TS_CHIP_INOVAV1, TS_CHIP_INOVAGATE1, TS_CHIP_LAST
};

DECL_ENUMERATION("thermocouple_type", "MAX31855", TS_CHIP_MAX31855);
DECL_ENUMERATION("thermocouple_type", "MAX31856", TS_CHIP_MAX31856);
DECL_ENUMERATION("thermocouple_type", "MAX31865", TS_CHIP_MAX31865);
DECL_ENUMERATION("thermocouple_type", "MAX6675", TS_CHIP_MAX6675);
DECL_ENUMERATION("thermocouple_type", "InovaV1", TS_CHIP_INOVAV1);
DECL_ENUMERATION("thermocouple_type", "InovaGATE1", TS_CHIP_INOVAGATE1);

struct thermocouple_spi {
    struct timer timer;
    uint32_t rest_time;
    uint32_t min_value;           // Min allowed ADC value
    uint32_t max_value;           // Max allowed ADC value
    struct spidev_s *spi;
    uint8_t chip_type, flags;
    uint8_t channel;
};

enum {
    TS_PENDING = 1,
};

static struct task_wake thermocouple_wake;

static uint_fast8_t thermocouple_event(struct timer *timer) {
    struct thermocouple_spi *spi = container_of(
            timer, struct thermocouple_spi, timer);
    // Trigger task to read and send results
    sched_wake_task(&thermocouple_wake);
    spi->flags |= TS_PENDING;
    spi->timer.waketime += spi->rest_time;
    return SF_RESCHEDULE;
}

void
command_config_thermocouple(uint32_t *args)
{
    uint8_t chip_type = args[2];
    if (chip_type >= TS_CHIP_LAST)
        shutdown("Invalid thermocouple chip type");
    struct thermocouple_spi *spi = oid_alloc(
        args[0], command_config_thermocouple, sizeof(*spi));
    spi->timer.func = thermocouple_event;
    spi->spi = spidev_oid_lookup(args[1]);
    spi->chip_type = chip_type;
    spi->channel = args[3];
}
DECL_COMMAND(command_config_thermocouple,
             "config_thermocouple oid=%c spi_oid=%c thermocouple_type=%c channel=%c");

void
command_query_thermocouple(uint32_t *args)
{
    struct thermocouple_spi *spi = oid_lookup(
        args[0], command_config_thermocouple);

    sched_del_timer(&spi->timer);
    spi->timer.waketime = args[1];
    spi->rest_time = args[2];
    if (! spi->rest_time)
        return;
    spi->min_value = args[3];
    spi->max_value = args[4];
    sched_add_timer(&spi->timer);
}
DECL_COMMAND(command_query_thermocouple,
             "query_thermocouple oid=%c clock=%u rest_ticks=%u"
             " min_value=%u max_value=%u");

static void
thermocouple_respond(struct thermocouple_spi *spi, uint32_t next_begin_time
                     , uint32_t value, uint8_t fault, uint8_t oid)
{
    sendf("thermocouple_result oid=%c next_clock=%u value=%u fault=%c",
          oid, next_begin_time, value, fault);
    /* check the result and stop if below or above allowed range */
    if (value < spi->min_value || value > spi->max_value)
        try_shutdown("Thermocouple ADC out of range");
}

static void
thermocouple_handle_max31855(struct thermocouple_spi *spi
                             , uint32_t next_begin_time, uint8_t oid)
{
    uint8_t msg[4] = { 0x00, 0x00, 0x00, 0x00 };
    spidev_transfer(spi->spi, 1, sizeof(msg), msg);
    uint32_t value;
    memcpy(&value, msg, sizeof(value));
    value = be32_to_cpu(value);
    thermocouple_respond(spi, next_begin_time, value, 0, oid);
    // Kill after data send, host decode an error
    if (value & 0x04)
        try_shutdown("Thermocouple reader fault");
}

#define MAX31856_LTCBH_REG 0x0C
#define MAX31856_SR_REG 0x0F

static void
thermocouple_handle_max31856(struct thermocouple_spi *spi
                             , uint32_t next_begin_time, uint8_t oid)
{
    uint8_t msg[4] = { MAX31856_LTCBH_REG, 0x00, 0x00, 0x00 };
    spidev_transfer(spi->spi, 1, sizeof(msg), msg);
    uint32_t value;
    memcpy(&value, msg, sizeof(value));
    value = be32_to_cpu(value) & 0x00ffffff;
    // Read faults
    msg[0] = MAX31856_SR_REG;
    msg[1] = 0x00;
    spidev_transfer(spi->spi, 1, 2, msg);
    thermocouple_respond(spi, next_begin_time, value, msg[1], oid);
}

#define MAX31865_RTDMSB_REG 0x01
#define MAX31865_FAULTSTAT_REG 0x07

static void
thermocouple_handle_max31865(struct thermocouple_spi *spi
                             , uint32_t next_begin_time, uint8_t oid)
{
    uint8_t msg[4] = { MAX31865_RTDMSB_REG, 0x00, 0x00, 0x00 };
    spidev_transfer(spi->spi, 1, 3, msg);
    uint32_t value;
    memcpy(&value, msg, sizeof(value));
    value = (be32_to_cpu(value) >> 8) & 0xffff;
    // Read faults
    msg[0] = MAX31865_FAULTSTAT_REG;
    msg[1] = 0x00;
    spidev_transfer(spi->spi, 1, 2, msg);
    thermocouple_respond(spi, next_begin_time, value, msg[1], oid);
    // Kill after data send, host decode an error
    if (value & 0x0001)
        try_shutdown("Thermocouple reader fault");
}

static void
thermocouple_handle_max6675(struct thermocouple_spi *spi
                            , uint32_t next_begin_time, uint8_t oid)
{
    uint8_t msg[2] = { 0x00, 0x00};
    spidev_transfer(spi->spi, 1, sizeof(msg), msg);
    uint16_t value;
    memcpy(&value, msg, sizeof(msg));
    value = be16_to_cpu(value);
    thermocouple_respond(spi, next_begin_time, value, 0, oid);
    // Kill after data send, host decode an error
    if (value & 0x04)
        try_shutdown("Thermocouple reader fault");
}

static void
thermocouple_handle_inovaV1(struct thermocouple_spi *spi
                            , uint32_t next_begin_time, uint8_t oid)
{
    uint8_t msg[3] = { 4 | 2 | (spi->channel >> 2), spi->channel << 6, 0x00 };
    spidev_transfer(spi->spi, 1, sizeof(msg), msg);
    uint16_t value = (((uint16_t)msg[2]) | (((uint16_t)msg[1]) << 8)) & 4095;
    // TODO: check ranges with thermocouple_respond
    sendf("thermocouple_result oid=%c next_clock=%u value=%u fault=%c",
          oid, next_begin_time, value, 0);
    //thermocouple_respond(spi, next_begin_time, value, 0, oid);
}

static void
thermocouple_handle_inovaGATE1(struct thermocouple_spi *spi
                            , uint32_t next_begin_time, uint8_t oid)
{
    // NOTE: ADC128S102 will take address of *NEXT* conversion as an input.
    //       To get around that, we run the conversion twice. 
    //       (CS also needs to be reset on second try to apply the address)
    uint8_t msg[2] = { (spi->channel << 3), 0 };
    spidev_transfer(spi->spi, 1, sizeof(msg), msg);
    spidev_transfer(spi->spi, 1, sizeof(msg), msg);
    uint16_t value = (((uint16_t)msg[1]) | (((uint16_t)msg[0]) << 8)) & 4095;
    // TODO: check ranges with thermocouple_respond
    sendf("thermocouple_result oid=%c next_clock=%u value=%u fault=%c",
          oid, next_begin_time, value, 0);
    //thermocouple_respond(spi, next_begin_time, value, 0, oid);
}

void thermocouple_read(uint8_t oid, struct thermocouple_spi *spi, uint8_t force)
{
    if (!force && !(spi->flags & TS_PENDING))
        return;
    irq_disable();
    uint32_t next_begin_time = spi->timer.waketime;
    if (!force)
        spi->flags &= ~TS_PENDING;
    irq_enable();

    switch (spi->chip_type) {
    case TS_CHIP_MAX31855:
        thermocouple_handle_max31855(spi, next_begin_time, oid);
        break;
    case TS_CHIP_MAX31856:
        thermocouple_handle_max31856(spi, next_begin_time, oid);
        break;
    case TS_CHIP_MAX31865:
        thermocouple_handle_max31865(spi, next_begin_time, oid);
        break;
    case TS_CHIP_MAX6675:
        thermocouple_handle_max6675(spi, next_begin_time, oid);
        break;
    case TS_CHIP_INOVAV1:
        thermocouple_handle_inovaV1(spi, next_begin_time, oid);
        break;
    case TS_CHIP_INOVAGATE1:
        thermocouple_handle_inovaGATE1(spi, next_begin_time, oid);
        break;
    }
}

void
command_read_thermocouple(uint32_t *args)
{
    uint8_t oid = args[0];
    struct thermocouple_spi *spi = oid_lookup(
        oid, command_config_thermocouple);

    thermocouple_read(oid, spi, 1);
}
DECL_COMMAND_FLAGS(command_read_thermocouple, HF_IN_SHUTDOWN, "read_thermocouple oid=%c");

// task to read thermocouple and send response
void
thermocouple_task(void)
{
    if (!sched_check_wake(&thermocouple_wake))
        return;
    uint8_t oid;
    struct thermocouple_spi *spi;
    foreach_oid(oid, spi, command_config_thermocouple) {
        thermocouple_read(oid, spi, 0);
    }
}
DECL_TASK(thermocouple_task);
