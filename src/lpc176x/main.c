// Main starting point for LPC176x boards.
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_CLOCK_FREQ
#include "board/armcm_boot.h" // armcm_main
#include "basecmd.h" // command_poweroff
#include "gpio.h" // gpio_out_setup
#include "internal.h" // enable_pclock
#include "sched.h" // sched_main


/****************************************************************
 * watchdog handler
 ****************************************************************/

void
watchdog_reset(void)
{
    LPC_WDT->WDFEED = 0xaa;
    LPC_WDT->WDFEED = 0x55;
}
DECL_TASK(watchdog_reset);

void
watchdog_init(void)
{
    LPC_WDT->WDTC = 4000000 / 2; // 500ms timeout
    LPC_WDT->WDCLKSEL = 1<<31; // Lock to internal RC
    LPC_WDT->WDMOD = 0x03; // select reset and enable
    watchdog_reset();
}
DECL_INIT(watchdog_init);


/****************************************************************
 * misc functions
 ****************************************************************/

// Check if a peripheral clock has been enabled
int
is_enabled_pclock(uint32_t pclk)
{
    return !!(LPC_SC->PCONP & (1<<pclk));
}

// Enable a peripheral clock
void
enable_pclock(uint32_t pclk)
{
    LPC_SC->PCONP |= 1<<pclk;
    if (pclk < 16) {
        uint32_t shift = pclk * 2;
        LPC_SC->PCLKSEL0 = (LPC_SC->PCLKSEL0 & ~(0x3<<shift)) | (0x1<<shift);
    } else {
        uint32_t shift = (pclk - 16) * 2;
        LPC_SC->PCLKSEL1 = (LPC_SC->PCLKSEL1 & ~(0x3<<shift)) | (0x1<<shift);
    }
}

// Return the frequency of the given peripheral clock
uint32_t
get_pclock_frequency(uint32_t pclk)
{
    return CONFIG_CLOCK_FREQ;
}

// Main entry point - called from armcm_boot.c:ResetHandler()
void
armcm_main(void)
{
    SystemInit();
    sched_main();
}

#define POWERPIN1 (2 * 32 + 4) // P2.4 (HE1)
#define POWERPIN2 (1 * 32 + 23) // P1.23

static struct gpio_out power_out1;
static struct gpio_out power_out2;

void system_poweron(void)
{
    power_out1 = gpio_out_setup(POWERPIN1, 1);
    power_out2 = gpio_out_setup(POWERPIN2, 1);
}
DECL_INIT(system_poweron);

void system_poweroff(void)
{
    gpio_out_write(power_out1, 0);
    gpio_out_write(power_out2, 0);
}
