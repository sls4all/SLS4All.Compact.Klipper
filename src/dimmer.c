// Dimming functions
//
// Copyright (C) 2021-2022  Pavel Dyntera (dyntera@anyteq.eu)
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/gpio.h" // gpio_out_setup
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // sched_shutdown
#include "board/misc.h" // timer_from_us
#include "board/irq.h" // irq_disable

#define DIMMER_SENSOR_DEBOUNCE 5
#define DIMMER_SENSOR_PERIOD_US 25
#define DIMMER_WAIT_PERIOD_US 1000
#define DIMMER_MIN_PERIOD_US 7142 // AC~70Hz
#define DIMMER_MAX_PERIOD_US 12500 // AC~40Hz
#define DIMMER_MAX 255
#define DIMMER_POWER_SHIFT 10
#define DIMMER_POWER_PULSE_SHIFT 3
DECL_CONSTANT("DIMMER_MAX", DIMMER_MAX);

static const uint16_t dimmer_power_table[] = {
    1024, 983, 966, 953, 942, 932, 924, 915, 908, 901, 894, 888, 881, 876, 870, 864, 859, 854, 849, 844, 839, 834, 830, 825, 821, 816, 812, 808, 804, 800, 796, 792, 788, 784, 780, 777, 773, 769, 766, 762, 759, 755, 752, 748, 745, 741, 738, 735, 731, 728, 725, 722, 719, 715, 712, 709, 706, 703, 700, 697, 694, 691, 688, 685, 682, 679, 676, 673, 670, 667, 664, 662, 659, 656, 653, 650, 648, 645, 642, 639, 636, 634, 631, 628, 625, 623, 620, 617, 615, 612, 609, 607, 604, 601, 599, 596, 593, 591, 588, 585, 583, 580, 578, 575, 572, 570, 567, 565, 562, 559, 557, 554, 552, 549, 547, 544, 541, 539, 536, 534, 531, 529, 526, 524, 521, 518, 516, 513, 511, 508, 506, 503, 500, 498, 495, 493, 490, 488, 485, 483, 480, 477, 475, 472, 470, 467, 465, 462, 459, 457, 454, 452, 449, 446, 444, 441, 439, 436, 433, 431, 428, 425, 423, 420, 417, 415, 412, 409, 407, 404, 401, 399, 396, 393, 390, 388, 385, 382, 379, 376, 374, 371, 368, 365, 362, 360, 357, 354, 351, 348, 345, 342, 339, 336, 333, 330, 327, 324, 321, 318, 315, 312, 309, 305, 302, 299, 296, 293, 289, 286, 283, 279, 276, 272, 269, 265, 262, 258, 255, 251, 247, 244, 240, 236, 232, 228, 224, 220, 216, 212, 208, 203, 199, 194, 190, 185, 180, 175, 170, 165, 160, 154, 148, 143, 136, 130, 123, 116, 109, 100, 92, 82, 71, 58, 41, 0
};

static uint32_t dimmer_sensor_period;
static uint32_t dimmer_wait_period;
static uint32_t dimmer_min_period;
static uint32_t dimmer_max_period;

static struct timer dimmer_sensor_timer;
static struct gpio_in dimmer_sensor_pin;
static uint32_t dimmer_sensor_pin_number;
static uint32_t sensor_state_zero_cross_time;
static uint_fast8_t sensor_state_zero_cross_time_valid;
static uint32_t sensor_state_on_time;
static uint32_t sensor_state_on_time_candidate;
static uint32_t sensor_state_off_time_candidate;
static uint_fast8_t sensor_state_on_time_valid;
static uint32_t sensor_state_period;
static uint_fast8_t sensor_state_on_mode;
static uint_fast8_t sensor_state_off_mode;

struct dimmer_s {
    struct timer schedule_timer;
    uint_fast8_t schedule_value;
    struct timer set_timer;
    struct gpio_out pin;
    uint_fast8_t value;
    uint_fast8_t on_value;
    uint_fast8_t off_value;
    uint_fast8_t flags;
    uint32_t max_duration;
};

enum {
    DIMMER_SHOULD_TURN_ON=1<<0,
};

static uint32_t fix_period(uint32_t start, uint32_t time)
{
    // subtract one period, on case we are just after threshold
    start -= sensor_state_period;
    // ensure time is in the future
    while (1)
    {
        int32_t diff = start - time;
        if (diff > 0)
            break;
        start += sensor_state_period;
    }
    return start;
}

static void fix_overflow(uint32_t *time, uint32_t waketime)
{
    const uint32_t future_reserve = 8;
    while (1)
    {
        int32_t diff = (*time + future_reserve * sensor_state_period) - waketime;
        if (diff >= 0) // future_reserve periods are in the future
            break;
        // fix potential overflow by moving old values
        *time += sensor_state_period;
    }
}

static uint_fast8_t dimmer_sensor_event(struct timer *t)
{
    uint_fast8_t sensor_state = gpio_in_read(dimmer_sensor_pin);
    dimmer_sensor_timer.waketime = timer_read_time() + dimmer_sensor_period; // NOTE: use current time instead of addition to avoid timer debt
    if (sensor_state)
    {
        if (sensor_state_off_mode <= DIMMER_SENSOR_DEBOUNCE)
            sensor_state_off_mode = 0;

        if (sensor_state_on_mode < DIMMER_SENSOR_DEBOUNCE) // debounce
        {
            if (sensor_state_on_mode++ == 0)
                sensor_state_on_time_candidate = dimmer_sensor_timer.waketime;
        }
        else if (sensor_state_on_mode == DIMMER_SENSOR_DEBOUNCE)
        {            
            if (sensor_state_on_time_valid)
            {
                uint32_t candidate_period = sensor_state_on_time_candidate - sensor_state_on_time;
                if (candidate_period >= dimmer_min_period && candidate_period <= dimmer_max_period) // filter out bullshit
                    sensor_state_period = candidate_period;
            }

            sensor_state_on_mode = DIMMER_SENSOR_DEBOUNCE + 1;
            sensor_state_on_time = sensor_state_on_time_candidate;
            sensor_state_on_time_valid = 1;
            sensor_state_off_mode = 0;
        }
    }
    else
    {
        if (sensor_state_on_mode <= DIMMER_SENSOR_DEBOUNCE)
            sensor_state_on_mode = 0;

        if (sensor_state_off_mode < DIMMER_SENSOR_DEBOUNCE) // debounce
        {
            if (sensor_state_off_mode++ == 0)
                sensor_state_off_time_candidate = dimmer_sensor_timer.waketime;
        }
        else if (sensor_state_off_mode == DIMMER_SENSOR_DEBOUNCE)
        {
            uint_fast8_t old_on_mode = sensor_state_on_mode;

            if (sensor_state_on_time_valid)
            {
                uint32_t sensor_state_off_length = sensor_state_off_time_candidate - sensor_state_on_time;
                sensor_state_zero_cross_time = sensor_state_on_time + (sensor_state_off_length >> 1);
                sensor_state_zero_cross_time_valid = 1;
            }
            
            sensor_state_off_mode = DIMMER_SENSOR_DEBOUNCE + 1;
            sensor_state_on_mode = 0;

            if (old_on_mode > DIMMER_SENSOR_DEBOUNCE)
            {
                // skip checking most of off states
                uint32_t next = sensor_state_on_time + dimmer_min_period - DIMMER_SENSOR_DEBOUNCE * dimmer_sensor_period;
                int32_t diff = next - (dimmer_sensor_timer.waketime + dimmer_sensor_period);
                if (diff > 0)
                    dimmer_sensor_timer.waketime = next;
            }
        }
    }
    if (sensor_state_period)
    {
        if (sensor_state_on_time_valid)
            fix_overflow(&sensor_state_on_time, dimmer_sensor_timer.waketime);
        if (sensor_state_zero_cross_time_valid)
            fix_overflow(&sensor_state_zero_cross_time, dimmer_sensor_timer.waketime);
    }
    return SF_RESCHEDULE;
}

static uint_fast8_t dimmer_out_event(struct timer *t)
{
    struct dimmer_s *s = container_of(t, struct dimmer_s, set_timer);
    if (unlikely(s->value == 0 || !sensor_state_period || !sensor_state_zero_cross_time_valid)) // off or unknown zero-cross info
    {
        gpio_out_write(s->pin, s->off_value);
        s->flags &= ~DIMMER_SHOULD_TURN_ON;
    }
    else if (unlikely(s->value >= DIMMER_MAX)) // always on
    {
        gpio_out_write(s->pin, s->on_value);
        s->flags &= ~DIMMER_SHOULD_TURN_ON;
    }
    else if (s->flags & DIMMER_SHOULD_TURN_ON) // turn on signal
    {
        gpio_out_write(s->pin, s->on_value);
        s->flags &= ~DIMMER_SHOULD_TURN_ON;
        s->set_timer.waketime += sensor_state_period >> (DIMMER_POWER_SHIFT - DIMMER_POWER_PULSE_SHIFT); // just pulse
        return SF_RESCHEDULE;
    }
    else
    {
        gpio_out_write(s->pin, s->off_value);
        uint32_t turnOnTime = sensor_state_zero_cross_time + ((sensor_state_period * dimmer_power_table[s->value]) >> DIMMER_POWER_SHIFT);
        s->flags |= DIMMER_SHOULD_TURN_ON;
        s->set_timer.waketime = fix_period(turnOnTime, timer_read_time());
        return SF_RESCHEDULE;
    }
    // just wait a while for something to happen
    s->set_timer.waketime += dimmer_wait_period;
    return SF_RESCHEDULE;
}

uint_fast8_t dimmer_end_event(struct timer *timer)
{
    shutdown("Missed scheduling of next dimmer event");
}

static uint_fast8_t dimmer_load_event(struct timer *t)
{
    struct dimmer_s *s = container_of(t, struct dimmer_s, schedule_timer);
    s->value = s->schedule_value;
    if (s->value == s->off_value || !s->max_duration)
        return SF_DONE;
    s->schedule_timer.waketime += s->max_duration;
    s->schedule_timer.func = dimmer_end_event;
    return SF_RESCHEDULE;
}

void command_config_dimmer_out(uint32_t *args)
{
    struct dimmer_s *s = oid_alloc(args[0], command_config_dimmer_out, sizeof(*s));
    uint32_t sensor_pin_number = args[1];
    uint8_t invert = !!args[4];
    uint8_t value = args[3];
    struct gpio_out pin = gpio_out_setup(args[2], invert);

    irq_disable();
    if (dimmer_sensor_pin_number != 0 && dimmer_sensor_pin_number != sensor_pin_number)
    {
        irq_enable();
        shutdown("Changing dimmer sensor pin is not supported");
        return;
    }
    else if (dimmer_sensor_pin_number == 0)
    {
        dimmer_sensor_pin_number = sensor_pin_number;
        dimmer_sensor_pin = gpio_in_setup(sensor_pin_number, 1);
        dimmer_sensor_timer.func = dimmer_sensor_event;
        dimmer_sensor_timer.waketime = timer_read_time() + dimmer_sensor_period;
        sched_add_timer(&dimmer_sensor_timer);
    }
    s->pin = pin;
    s->flags = 0;
    s->on_value = !invert;
    s->off_value = invert;
    s->value = value;
    s->set_timer.func = dimmer_out_event;
    s->set_timer.waketime = timer_read_time() + dimmer_wait_period;
    s->max_duration = args[5];
    sched_add_timer(&s->set_timer);
    irq_enable();
}
DECL_COMMAND(command_config_dimmer_out,
             "config_dimmer_out oid=%c sensor_pin=%u pin=%u value=%hu invert=%c max_duration=%u");

void command_schedule_dimmer_out(uint32_t *args)
{
    struct dimmer_s *s = oid_lookup(args[0], command_config_dimmer_out);
    uint32_t time = args[1];
    uint8_t value = args[2];
    irq_disable();
    sched_del_timer(&s->schedule_timer);
    s->schedule_timer.waketime = time;
    s->schedule_timer.func = dimmer_load_event;
    s->schedule_value = value;
    sched_add_timer(&s->schedule_timer);
    irq_enable();
}
DECL_COMMAND(command_schedule_dimmer_out,
             "schedule_dimmer_out oid=%c clock=%u value=%hu");

void
command_update_dimmer_out(uint32_t *args)
{
    int ms10 = timer_from_us(10000);
    uint32_t newArgs[3] = { args[0], timer_read_time() + ms10, args[1] };
    command_schedule_dimmer_out(newArgs);
}
DECL_COMMAND(command_update_dimmer_out, "update_dimmer_out oid=%c value=%hu");

void
dimmer_init(void)
{
    dimmer_sensor_period = timer_from_us(DIMMER_SENSOR_PERIOD_US);
    dimmer_wait_period = timer_from_us(DIMMER_WAIT_PERIOD_US);
    dimmer_min_period = timer_from_us(DIMMER_MIN_PERIOD_US);
    dimmer_max_period = timer_from_us(DIMMER_MAX_PERIOD_US);

    dimmer_sensor_pin_number = 0;
    sensor_state_zero_cross_time = 0;
    sensor_state_zero_cross_time_valid = 0;
    sensor_state_on_time = 0;
    sensor_state_on_time_candidate = 0;
    sensor_state_off_time_candidate = 0;
    sensor_state_period = 0;
    sensor_state_on_time_valid = 0;
    sensor_state_on_mode = 0;
    sensor_state_off_mode = 0;
}
DECL_INIT(dimmer_init);

void
dimmer_shutdown(void)
{
    uint8_t i;
    struct dimmer_s *p;
    foreach_oid(i, p, command_config_dimmer_out) {
        gpio_out_write(p->pin, p->off_value);
    }
}
DECL_SHUTDOWN(dimmer_shutdown);
