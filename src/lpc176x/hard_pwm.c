// Hardware PWM pin support
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <limits.h> // CONFIG_MACH_atmega644p
#include "autoconf.h" // CONFIG_MACH_atmega644p
#include "command.h" // shutdown
#include "gpio.h" // gpio_pwm_write
#include "internal.h" // GPIO2REGS
#include "sched.h" // sched_shutdown

#define SBIT_CNTEN     0 
#define SBIT_PWMEN     2

#define SBIT_PWMMR0R   1

#define SBIT_LEN0      0
#define SBIT_LEN1      1
#define SBIT_LEN2      2
#define SBIT_LEN3      3
#define SBIT_LEN4      4
#define SBIT_LEN5      5
#define SBIT_LEN6      6

#define SBIT_PWMENA1   9
#define SBIT_PWMENA2   10
#define SBIT_PWMENA3   11
#define SBIT_PWMENA4   12
#define SBIT_PWMENA5   13
#define SBIT_PWMENA6   14


#define PWM_1          0  //P2_0 (0-1 Bits of PINSEL4)
#define PWM_2          2  //P2_1 (2-3 Bits of PINSEL4)
#define PWM_3          4  //P2_2 (4-5 Bits of PINSEL4)
#define PWM_4          6  //P2_3 (6-7 Bits of PINSEL4)
#define PWM_5          8  //P2_4 (8,9 bits of PINSEL4)
#define PWM_6          10 //P2_5 (10,11 bits of PINSEL4)

struct gpio_pwm_info {
    uint8_t pin;
    volatile uint32_t *mr;
    uint32_t sel;
    uint32_t latch;
    uint32_t enable;
};

static const struct gpio_pwm_info pwm_regs[] = {
    { GPIO(2, 0), &LPC_PWM1->MR1, 1<<PWM_1, 1<<SBIT_LEN1, 1<<SBIT_PWMENA1 },
    { GPIO(2, 1), &LPC_PWM1->MR2, 1<<PWM_2, 1<<SBIT_LEN2, 1<<SBIT_PWMENA2 },
    { GPIO(2, 2), &LPC_PWM1->MR3, 1<<PWM_3, 1<<SBIT_LEN3, 1<<SBIT_PWMENA3 },
    { GPIO(2, 3), &LPC_PWM1->MR4, 1<<PWM_4, 1<<SBIT_LEN4, 1<<SBIT_PWMENA4 },
    { GPIO(2, 4), &LPC_PWM1->MR5, 1<<PWM_5, 1<<SBIT_LEN5, 1<<SBIT_PWMENA5 },
    { GPIO(2, 5), &LPC_PWM1->MR6, 1<<PWM_6, 1<<SBIT_LEN6, 1<<SBIT_PWMENA6 },
};

#define PWM_MAX 65535
DECL_CONSTANT("PWM_MAX", PWM_MAX);

inline void
gpio_pwm_set(const struct gpio_pwm_info *p, uint32_t val, uint32_t mr0)
{
    uint32_t target;
    if (val == 0) // off
        target = mr0;
    else if (val >= PWM_MAX) // full power
        target = mr0 - 1;
    else // ~ 0-99%
    {
        target = (uint32_t)((uint64_t)val * mr0 / PWM_MAX) - 1;
        if (target == UINT_MAX) // off at mr0
            target = mr0;
    }
    *p->mr = target;    
}

struct gpio_pwm
gpio_pwm_setup(uint8_t pin, uint32_t cycle_time, uint32_t val)
{
    // Find pin in pwm_regs table
    const struct gpio_pwm_info *p = pwm_regs;
    for (; ; p++) {
        if (p >= &pwm_regs[ARRAY_SIZE(pwm_regs)])
            shutdown("Not a valid PWM pin");
        if (p->pin == pin)
            break;
    }

    // Setup PWM timer

    /* Cofigure pin for PWM mode. */
    LPC_PINCON->PINSEL4 |= p->sel;

    /* Enable Counters,PWM module */ 
    LPC_PWM1->TCR = (1<<SBIT_CNTEN) | (1<<SBIT_PWMEN);

    LPC_PWM1->PR  =  0x0;               /* No Prescalar */
    LPC_PWM1->MCR = (1<<SBIT_PWMMR0R);  /* Reset on PWMMR0, reset TC if it matches MR0 */

    uint32_t mr0 = cycle_time / 4;
    if (LPC_PWM1->MR0 && LPC_PWM1->MR0 != mr0)
        shutdown("PWM cycle time already configured to a different value");
    LPC_PWM1->MR0 = mr0;                /* set PWM cycle(Ton+Toff) */

    gpio_pwm_set(p, val, mr0);

    /* Trigger the latch Enable Bits to load the new Match Values */
    LPC_PWM1->LER |= (1<<SBIT_LEN0) | p->latch; 

    LPC_PWM1->PCR |= p->enable;

    struct gpio_pwm g = (struct gpio_pwm) { (void*)p };

    return g;
}

void
gpio_pwm_write(struct gpio_pwm g, uint32_t val)
{
    const struct gpio_pwm_info *p = g.reg;
    uint32_t mr0 = LPC_PWM1->MR0;
    gpio_pwm_set(p, val, mr0);

    /* Trigger the latch Enable Bits to load the new Match Values */
    LPC_PWM1->LER |= p->latch; 
}
