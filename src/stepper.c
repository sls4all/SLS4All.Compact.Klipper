// Handling of stepper drivers.
//
// Copyright (C) 2016-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_*
#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_is_before
#include "command.h" // DECL_COMMAND
#include "sched.h" // struct timer
#include "stepper.h" // command_config_stepper

DECL_CONSTANT("STEP_DELAY", CONFIG_STEP_DELAY);

//#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"

/****************************************************************
 * Steppers
 ****************************************************************/

struct __attribute__((__packed__)) step_move_v1 {
    uint32_t interval;
    int16_t count;
    int16_t add;
};

struct __attribute__((__packed__)) step_move_v2 {
    uint32_t interval;
    int16_t count;
};

struct __attribute__((__packed__)) step_move_v3 {
    uint16_t interval;
    int16_t count;
};

struct __attribute__((__packed__)) step_move_v4 {
    int32_t delay;
    int16_t pwm;
};

struct __attribute__((__packed__)) stepper_move {
    union {
        struct {
            uint32_t interval;
            int16_t add;
            uint16_t count;
        };
        uint64_t interval64;
    };
    int32_t next_relative : 24;
    uint8_t flags : 8;
};

#define get_stepper_move_next(pmove) ((pmove)->next_relative ? (struct stepper_move*)((size_t)(pmove) + (pmove)->next_relative) : NULL)
#define set_stepper_move_next(pmove, pnext) (pmove)->next_relative = pnext ? (size_t)(pnext) - (size_t)(pmove) : 0

enum { MF_DIR=1<<0, MF_DWELL=1<<1, MF_PWM=1<<2, MF_POSITIVE_DIR=1<<3, MF_VERIFY=1<<4 };

struct stepper {
    struct timer time;
    uint32_t interval;
    int16_t add;
    uint8_t interval_precision;
    uint32_t count;
    uint32_t pwm;
    uint64_t next_step_time;
    struct gpio_out step_pin, dir_pin;
    uint32_t position;
    uint32_t min_interval;
    struct stepper_move *first, *last;
    uint32_t flags;
    uint16_t step_add;
    uint16_t step_add_rest;
    uint8_t pwm_oid;
};

enum { POSITION_BIAS=0x40000000 };

enum {
    SF_LAST_DIR=1<<0, SF_NEXT_DIR=1<<1, SF_INVERT_STEP=1<<2, SF_NEED_RESET=1<<3,
    SF_SINGLE_SCHED=1<<4, SF_HAVE_ADD=1<<5, SF_DWELL=1<<6, SF_NO_DELAY=1<<7,
    SF_PWM_STEPPER=1<<8, SF_PWM_SET=1<<9
};

// Setup a stepper for the next move in its queue
static uint_fast8_t
stepper_load_next(struct stepper *s)
{
LoadNext:;
    struct stepper_move *m = s->first;
    if (!m) {
        // There is no next move - the queue is empty
        s->count = 0;
        return SF_DONE;
    }

    // Load next 'struct stepper_move' into 'struct stepper'
    if (unlikely(m->flags & (MF_DWELL | MF_PWM | MF_VERIFY)))
    {
        if (likely(m->flags & (MF_DWELL | MF_PWM)))
        {
            if (likely((m->flags & (MF_DWELL | MF_PWM)) == (MF_DWELL | MF_PWM)))
            {
                s->next_step_time += m->interval;
                s->time.waketime = s->next_step_time >> s->interval_precision;
                s->flags |= SF_DWELL | SF_PWM_SET;
                s->pwm = m->count;
                s->count = 1;
            }
            else if (unlikely(m->flags & MF_DWELL))
            {
                s->next_step_time += m->interval;
                s->time.waketime = s->next_step_time >> s->interval_precision;
                s->flags |= SF_DWELL;
                s->count = 1;
            }
            else if (unlikely(m->flags & MF_PWM))
            {
                gpio_oid_pwm_set_noirq(s->pwm_oid, (uint16_t)m->count);
                s->first = get_stepper_move_next(m);
                move_free(m);
                goto LoadNext;
            }
        }
        else // verify
        {
            if (s->next_step_time != m->interval64)
            {
                if (s->next_step_time < m->interval64)
                    shutdown("Stepper next_step_time verification failed, MCU value is before the expected host value");
                else
                    shutdown("Stepper next_step_time verification failed, MCU value is after the expected host value");
            }
            s->first = get_stepper_move_next(m);
            move_free(m);
            goto LoadNext;
        }
    }
    else
    {
        uint16_t old_count = m->count;
        uint32_t delay = 0;
        if (m->interval < s->min_interval)
        {
            if (m->add != 0)
                shutdown("Stepper min_interval is supported only with add=0");
            uint16_t step_add = (s->min_interval + m->interval - 1) / m->interval;
            uint16_t new_count = old_count / step_add;
            if (new_count)
            {
                uint16_t count_remainder = old_count % step_add;
                delay = m->interval * count_remainder;
                s->step_add = step_add + count_remainder;
                s->step_add_rest = step_add;
                m->count = new_count;
                m->interval = m->interval * step_add;
            }
            else
            {
                s->step_add = old_count;
                s->step_add_rest = old_count;
                m->count = 1;
                m->interval = m->interval * old_count;
            }
        }
        else
        {
            s->step_add_rest = 1;
            s->step_add = 1;       
        }
        s->add = m->add;
        s->interval = m->interval + m->add;
        if (CONFIG_STEP_DELAY <= 0) {
            s->next_step_time = s->next_step_time + m->interval + delay;
            s->time.waketime = s->next_step_time >> s->interval_precision;
            if (CONFIG_MACH_AVR)
                s->flags = m->add ? s->flags|SF_HAVE_ADD : s->flags & ~SF_HAVE_ADD;
            s->count = m->count;
        } else {
            // It is necessary to schedule unstep events and so there are
            // twice as many events.
            s->next_step_time = s->next_step_time + m->interval + delay;
            s->time.waketime = s->next_step_time >> s->interval_precision;
            if (s->flags & SF_NO_DELAY)
                s->count = m->count;
            else
                s->count = (uint32_t)m->count * 2;
        }
        // Add all steps to s->position (stepper_get_position() can calc mid-move)
        if (m->flags & MF_DIR) {
            s->position = -s->position + old_count;
            gpio_out_toggle_noirq(s->dir_pin);
        } else {
            s->position += old_count;
        }
    }

    s->first = get_stepper_move_next(m);
    move_free(m);
    return SF_RESCHEDULE;
}

// AVR optimized step function
uint_fast8_t
stepper_event_avr(struct stepper *s)
{
    gpio_out_toggle_noirq(s->step_pin);
    uint32_t *pcount = (void*)&s->count, count = *pcount - 1;
    if (likely(count)) {
        *pcount = count;
        s->next_step_time = s->next_step_time + s->interval;
        s->time.waketime = s->next_step_time >> s->interval_precision;
        gpio_out_toggle_noirq(s->step_pin);
        if (s->flags & SF_HAVE_ADD)
            s->interval += s->add;
        return SF_RESCHEDULE;
    }
    uint_fast8_t ret = stepper_load_next(s);
    gpio_out_toggle_noirq(s->step_pin);
    return ret;
}

// Optimized step function for stepping and unstepping in same function using specialized GPIO method
uint_fast8_t
stepper_event_nodelay_double_toggle(struct stepper *s)
{
    gpio_out_double_toggle_count_noirq(s->step_pin, s->step_add);
    uint32_t *pcount = (void*)&s->count, count = *pcount - 1;
    if (likely(count)) {
        *pcount = count;
        s->next_step_time += s->interval;
        s->time.waketime = s->next_step_time >> s->interval_precision;
        s->interval += s->add;
        s->step_add = s->step_add_rest;
        return SF_RESCHEDULE;
    }
    uint_fast8_t ret = stepper_load_next(s);
    return ret;
}

// Regular "double scheduled" step function
uint_fast8_t
stepper_event_full(struct stepper *s)
{
    gpio_out_toggle_noirq(s->step_pin);
    uint32_t curtime = timer_read_time();
    uint32_t min_next_time = curtime + timer_from_us(CONFIG_STEP_DELAY);
    s->count--;
    if (likely(s->count & 1))
        // Schedule unstep event
        goto reschedule_min;
    if (likely(s->count)) {
        s->next_step_time += s->interval;
        s->interval += s->add;
        s->time.waketime = s->next_step_time >> s->interval_precision;
        if (unlikely(timer_is_before(s->time.waketime, min_next_time)))
            // The next step event is too close - push it back
            goto reschedule_min;
        return SF_RESCHEDULE;
    }
    uint_fast8_t ret = stepper_load_next(s);
    if (ret == SF_DONE || !timer_is_before(s->time.waketime, min_next_time))
        return ret;
    // Next step event is too close to the last unstep
    int32_t diff = s->time.waketime - min_next_time;
    if (diff < (int32_t)-timer_from_us(1000))
        shutdown("Stepper too far in past");
reschedule_min:
    s->time.waketime = min_next_time;
    return SF_RESCHEDULE;
}

// Timer callback - step the given stepper.
uint_fast8_t
stepper_event(struct timer *t)
{
    struct stepper *s = container_of(t, struct stepper, time);
    if (unlikely(s->flags & (SF_DWELL | SF_PWM_SET)))
    {
        if (likely(s->flags & SF_PWM_SET))
            gpio_oid_pwm_set_noirq(s->pwm_oid, s->pwm);
        s->flags &= ~(SF_DWELL | SF_PWM_SET);
        return stepper_load_next(s);
    }
    else if (likely(s->flags & SF_NO_DELAY))
        return stepper_event_nodelay_double_toggle(s);
    else
    {
        if (CONFIG_STEP_DELAY <= 0 && CONFIG_MACH_AVR)
            return stepper_event_avr(s);        
        else
            return stepper_event_full(s);
    }
}

void
command_config_stepper(uint32_t *args)
{
    struct stepper *s = oid_alloc(args[0], command_config_stepper, sizeof(*s));
    int_fast8_t invert_step = args[4];
    s->flags = invert_step > 0 ? SF_INVERT_STEP : 0;
    s->step_pin = gpio_out_setup(args[1], s->flags & SF_INVERT_STEP);
    s->dir_pin = gpio_out_setup(args[2], 0);
    s->position = -POSITION_BIAS;
    s->min_interval = args[3];
    if (args[5])
        s->flags |= SF_NO_DELAY;
    s->interval_precision = args[6];
    move_request_size(sizeof(struct stepper_move));
    if (!CONFIG_INLINE_STEPPER_HACK)
        s->time.func = stepper_event;
}
DECL_COMMAND(command_config_stepper,
             "config_stepper oid=%c step_pin=%c dir_pin=%c"
             " min_interval=%u invert_step=%c no_delay=%c interval_precision=%c");

void
command_config_pwm_stepper(uint32_t *args)
{
    struct stepper *s = oid_alloc(args[0], command_config_stepper, sizeof(*s));
    s->flags = SF_NO_DELAY | SF_PWM_STEPPER;
    s->position = -POSITION_BIAS;
    s->pwm_oid = args[1];
    s->interval_precision = args[2];
    move_request_size(sizeof(struct stepper_move));
    if (!CONFIG_INLINE_STEPPER_HACK)
        s->time.func = stepper_event;
}
DECL_COMMAND(command_config_pwm_stepper,
             "config_pwm_stepper oid=%c pwm_oid=%c interval_precision=%c");

// Return the 'struct stepper' for a given stepper oid
struct stepper *
stepper_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_stepper);
}

// Schedule a set of steps with a given timing
void
command_queue_step(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    struct stepper_move *m1 = move_alloc();
    m1->interval = args[1];
    m1->count = args[2];
    m1->add = args[3];
    set_stepper_move_next(m1, NULL);
    m1->flags = 0;

    if (!m1->count)
        m1->flags |= MF_DWELL;

    irq_disable();
    uint32_t flags = s->flags;
    if (!!(flags & SF_LAST_DIR) != !!(flags & SF_NEXT_DIR)) {
        flags ^= SF_LAST_DIR;
        m1->flags |= MF_DIR;
    }
    if (s->count) {
        s->flags = flags;
        if (s->first)
            set_stepper_move_next(s->last, m1);
        else
            s->first = m1;
        s->last = m1;
    } else if (flags & SF_NEED_RESET) {
        move_free(m1);
    } else {
        s->flags = flags;
        s->first = m1;
        s->last = m1;
        if (stepper_load_next(s) == SF_RESCHEDULE)
        {
            if (timer_is_before(s->time.waketime, timer_read_time()))
                shutdown("First scheduled stepper event is in the past");
            sched_add_timer(&s->time);
        }
    }
    irq_enable();
}
DECL_COMMAND(command_queue_step,
             "queue_step oid=%c interval=%u count=%hu add=%hi");

// Set the direction of the next queued step
void
command_set_next_step_dir(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    uint8_t nextdir = args[1] ? SF_NEXT_DIR : 0;
    irq_disable();
    s->flags = (s->flags & ~SF_NEXT_DIR) | nextdir;
    irq_enable();
}
DECL_COMMAND(command_set_next_step_dir, "set_next_step_dir oid=%c dir=%c");

// Set an absolute time that the next step will be relative to
void
command_reset_step_clock(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    uint64_t clock64 = (uint64_t)args[1] | (((uint64_t)args[2] << 32));
    irq_disable();
    if (s->count)
        shutdown("Can't reset time when stepper active");
    s->next_step_time = clock64;
    s->time.waketime = clock64 >> s->interval_precision;
    s->flags &= ~SF_NEED_RESET;
    irq_enable();
}
DECL_COMMAND(command_reset_step_clock, "reset_step_clock oid=%c clock=%u clockhi=%u");

// Return the current stepper position.  Caller must disable irqs.
static uint32_t
stepper_get_position(struct stepper *s)
{
    uint32_t position = s->position;
    // If stepper is mid-move, subtract out steps not yet taken
    if (CONFIG_STEP_DELAY <= 0 || (s->flags & SF_NO_DELAY))
        position -= s->count;
    else
        position -= s->count / 2;
    // The top bit of s->position is an optimized reverse direction flag
    if (position & 0x80000000)
        return -position;
    return position;
}

// Report the current position of the stepper
void
command_stepper_get_position(uint32_t *args)
{
    uint8_t oid = args[0];
    struct stepper *s = stepper_oid_lookup(oid);
    irq_disable();
    uint32_t position = stepper_get_position(s);
    irq_enable();
    sendf("stepper_position oid=%c pos=%i", oid, position - POSITION_BIAS);
}
DECL_COMMAND(command_stepper_get_position, "stepper_get_position oid=%c");

// Stop all moves for a given stepper (used in end stop homing).  IRQs
// must be off.
void
stepper_stop(struct stepper *s)
{
    sched_del_timer(&s->time);
    s->next_step_time = s->time.waketime = 0;
    s->position = -stepper_get_position(s);
    s->count = 0;
    s->flags = (s->flags & (SF_INVERT_STEP | SF_NO_DELAY | SF_DWELL | SF_PWM_STEPPER | SF_PWM_SET)) | SF_NEED_RESET;
    if (!(s->flags & SF_PWM_STEPPER))
    {
        gpio_out_write(s->dir_pin, 0);
        gpio_out_write(s->step_pin, s->flags & SF_INVERT_STEP);
    }
    while (s->first) {
        struct stepper_move *next = get_stepper_move_next(s->first);
        move_free(s->first);
        s->first = next;
    }
}

void
stepper_shutdown(void)
{
    uint8_t i;
    struct stepper *s;
    foreach_oid(i, s, command_config_stepper) {
        s->first = NULL;
        stepper_stop(s);
    }
}
DECL_SHUTDOWN(stepper_shutdown);

static void
queue_steps_finalize(struct stepper *s, struct stepper_move **pmx, uint32_t count)
{
    struct stepper_move *m1 = pmx[0];
    struct stepper_move *mx = pmx[count - 1];

    irq_disable();
    uint32_t flags = s->flags;
    for (int32_t i = 0; i < count; i++)
    {
        struct stepper_move *m = pmx[i];
        if (!(m->flags & (MF_DWELL | MF_PWM | MF_VERIFY)))
        {
            if (!!(flags & SF_LAST_DIR) != !!(m->flags & MF_POSITIVE_DIR)) {
                flags ^= SF_LAST_DIR;
                m->flags |= MF_DIR;
            }
        }
    }
    if (s->count) {
        s->flags = flags;
        if (s->first)
            set_stepper_move_next(s->last, m1);
        else
            s->first = m1;
        s->last = mx;
    } else if (flags & SF_NEED_RESET) {
        for (int32_t i = 0; i < count; i++)
            move_free(pmx[i]);
    } else {
        s->flags = flags;
        s->first = m1;
        s->last = mx;
        if (stepper_load_next(s) == SF_RESCHEDULE)
        {
            if (timer_is_before(s->time.waketime, timer_read_time()))
                shutdown("First scheduled stepper event is in the past");
            sched_add_timer(&s->time);
        }
    }
    irq_enable();
}

void
command_queue_steps_v1(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    uint32_t count = ((size_t)args[1]) / sizeof(struct step_move_v1);
    struct step_move_v1 *data = (void*)(size_t)args[2];
    struct stepper_move *pmx[32];

    for (int32_t i = 0; i < count; i++)
    {
        struct step_move_v1 *item = &data[i];
        struct stepper_move *m = move_alloc();
        int16_t count_signed = item->count;
        pmx[i] = m;

        set_stepper_move_next(m, NULL);
        m->flags = 0;
        if (unlikely(item->interval == UINT32_MAX))
        {
            m->flags |= MF_PWM;
            m->count = item->count; // power
        }
        else if (unlikely(!count_signed))
        {
            m->flags |= MF_DWELL;
            m->interval = item->interval; // delay
        }
        else
        {
            m->interval = item->interval;
            m->add = item->add;
            if (count_signed > 0)
            {
                m->count = count_signed;
                m->flags |= MF_POSITIVE_DIR;
            }
            else
            {
                m->count = -count_signed;
            }
        }
        
        if (i > 0)
            set_stepper_move_next(pmx[i - 1], m);
    }

    queue_steps_finalize(s, pmx, count);
}
DECL_COMMAND(command_queue_steps_v1, "queue_steps_v1 oid=%c steps=%*s");

void
command_queue_steps_v2(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    uint32_t count = ((size_t)args[1]) / sizeof(struct step_move_v2);
    struct step_move_v2 *data = (void*)(size_t)args[2];
    struct stepper_move *pmx[32];

    for (int32_t i = 0; i < count; i++)
    {
        struct step_move_v2 *item = &data[i];
        struct stepper_move *m = move_alloc();
        int16_t count_signed = item->count;
        pmx[i] = m;

        set_stepper_move_next(m, NULL);
        m->flags = 0;
        if (unlikely(item->interval == UINT32_MAX))
        {
            m->flags |= MF_PWM;
            m->count = item->count; // power
        }
        else if (unlikely(!count_signed))
        {
            m->flags |= MF_DWELL;
            m->interval = item->interval; // delay
        }
        else
        {
            m->interval = item->interval;
            m->add = 0;
            if (count_signed > 0)
            {
                m->count = count_signed;
                m->flags |= MF_POSITIVE_DIR;
            }
            else
            {
                m->count = -count_signed;
            }
        }
        
        if (i > 0)
            set_stepper_move_next(pmx[i - 1], m);
    }

    queue_steps_finalize(s, pmx, count);
}
DECL_COMMAND(command_queue_steps_v2, "queue_steps_v2 oid=%c steps=%*s");

void
command_queue_steps_v3(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    uint32_t count = ((size_t)args[1]) / sizeof(struct step_move_v3);
    struct step_move_v3 *data = (void*)(size_t)args[2];
    struct stepper_move *pmx[32];

    for (int32_t i = 0; i < count; i++)
    {
        struct step_move_v3 *item = &data[i];
        struct stepper_move *m = move_alloc();
        int16_t count_signed = item->count;
        pmx[i] = m;

        set_stepper_move_next(m, NULL);
        m->flags = 0;
        if (unlikely(item->interval == UINT16_MAX))
        {
            m->flags |= MF_PWM;
            m->count = item->count; // power
        }
        else if (unlikely(!count_signed))
        {
            m->flags |= MF_DWELL;
            m->interval = item->interval; // delay
        }
        else
        {
            m->interval = item->interval;
            m->add = 0;
            if (count_signed > 0)
            {
                m->count = count_signed;
                m->flags |= MF_POSITIVE_DIR;
            }
            else
            {
                m->count = -count_signed;
            }
        }
        
        if (i > 0)
            set_stepper_move_next(pmx[i - 1], m);
    }

    queue_steps_finalize(s, pmx, count);
}
DECL_COMMAND(command_queue_steps_v3, "queue_steps_v3 oid=%c steps=%*s");

void
command_queue_steps_v4(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    uint32_t count = ((size_t)args[1]) / sizeof(struct step_move_v4);
    struct step_move_v4 *data = (void*)(size_t)args[2];
    struct stepper_move *pmx[32];

    for (int32_t i = 0; i < count; i++)
    {
        struct step_move_v4 *item = &data[i];
        struct stepper_move *m = move_alloc();
        pmx[i] = m;
        set_stepper_move_next(m, NULL);
        if (item->delay != 0)
        {
            m->flags = MF_DWELL | MF_PWM;
            m->count = item->pwm;
            m->interval = item->delay;
        }
        else
        {
            m->flags = MF_PWM;
            m->count = item->pwm;
        }
        if (i > 0)
            set_stepper_move_next(pmx[i - 1], m);
    }

    queue_steps_finalize(s, pmx, count);
}
DECL_COMMAND(command_queue_steps_v4, "queue_steps_v4 oid=%c steps=%*s");

void
command_get_next_step_waketime(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    uint64_t next_step_time;
    irq_disable();
    next_step_time = s->next_step_time;
    irq_enable();
    sendf("next_step_waketime oid=%c clock=%u clockhi=%u", args[0], (uint32_t)next_step_time, (uint32_t)(next_step_time >> 32));
}
DECL_COMMAND(command_get_next_step_waketime, "get_next_step_waketime oid=%c");

void
command_verify_next_step_waketime(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    struct stepper_move *m1 = move_alloc();
    m1->interval64 = (uint64_t)args[1] | (((uint64_t)args[2] << 32));
    set_stepper_move_next(m1, NULL);
    m1->flags = MF_VERIFY;

    irq_disable();
    uint32_t flags = s->flags;
    if (s->count) {
        if (s->first)
            set_stepper_move_next(s->last, m1);
        else
            s->first = m1;
        s->last = m1;
    } else if (flags & SF_NEED_RESET) {
        move_free(m1);
    } else {
        s->first = m1;
        s->last = m1;
        if (stepper_load_next(s) == SF_RESCHEDULE)
        {
            if (timer_is_before(s->time.waketime, timer_read_time()))
                shutdown("First scheduled stepper event is in the past");
            sched_add_timer(&s->time);
        }
    }
    irq_enable();
}
DECL_COMMAND(command_verify_next_step_waketime, "verify_next_step_waketime oid=%c clock=%u clockhi=%u");

void
command_update_dac_reset(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    gpio_dac_reset(s->step_pin);
}
DECL_COMMAND(command_update_dac_set, "reset_stepper_dac oid=%c");

void
command_update_dac_set(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    gpio_dac_set(s->step_pin, args[1]);
}
DECL_COMMAND(command_update_dac_set, "update_stepper_dac oid=%c value=%u");