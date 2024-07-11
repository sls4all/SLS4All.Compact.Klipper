// GPIO functions on lpc176x
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // ffs
#include "board/irq.h" // irq_save
#include "board/misc.h" // timer_read_time
#include "command.h" // shutdown
#include "gpio.h" // gpio_out_setup
#include "internal.h" // gpio_peripheral
#include "sched.h" // sched_shutdown


/****************************************************************
 * Pin mappings
 ****************************************************************/

DECL_ENUMERATION_RANGE("pin", "P0.0", GPIO(0, 0), 32);
DECL_ENUMERATION_RANGE("pin", "P1.0", GPIO(1, 0), 32);
DECL_ENUMERATION_RANGE("pin", "P2.0", GPIO(2, 0), 32);
DECL_ENUMERATION_RANGE("pin", "P3.0", GPIO(3, 0), 32);
DECL_ENUMERATION_RANGE("pin", "P4.0", GPIO(4, 0), 32);
DECL_ENUMERATION_RANGE("pin", "PDAC.0", GPIO(5, 0), 6);

#define DAC_RATE 30000000 // DAC8562SDGSR is capable of max 50MHz, we however cannot divide 120MHZ of LPC1769 closer than 30MHz

struct dac_dev_s {
    uint8_t channel;
    uint32_t value;
    uint32_t value_max;
    uint8_t dir_state;
    uint8_t step_state;
    uint8_t stop_state;
    uint8_t cs_pin;

    struct gpio_out cs;
    struct spi_config spi;
};

struct dac_reg_s {
    struct dac_dev_s *dac;
    uint8_t *value;
};

static struct dac_dev_s dac_devs[] = {
    { .channel = 0, .cs_pin = GPIO(0, 16), .step_state = 0, .dir_state = 0, .value = 0, .stop_state = 1, .value_max = 65535 },
    { .channel = 1, .cs_pin = GPIO(0, 16), .step_state = 0, .dir_state = 0, .value = 0, .stop_state = 1, .value_max = 65535 },
};

#define DAC_DEVS_SIZE (sizeof(dac_devs) / sizeof(struct dac_dev_s))

static struct dac_reg_s dac_regs[] = {
    // DAC 0 channel 0
    { .dac = &dac_devs[0], .value = &dac_devs[0].dir_state }, 
    { .dac = &dac_devs[0], .value = &dac_devs[0].step_state },
    { .dac = &dac_devs[0], .value = &dac_devs[0].stop_state },
    // DAC 0 channel 1
    { .dac = &dac_devs[1], .value = &dac_devs[1].dir_state }, 
    { .dac = &dac_devs[1], .value = &dac_devs[1].step_state },
    { .dac = &dac_devs[1], .value = &dac_devs[1].stop_state },
};

static void * const digital_regs[] = {
    LPC_GPIO0, LPC_GPIO1, LPC_GPIO2, LPC_GPIO3, LPC_GPIO4, dac_regs
};

void
command_config_digital_out(uint32_t *args);

// Set the mode and extended function of a pin
void
gpio_peripheral(uint32_t gpio, int func, int pull_up)
{
    uint32_t bank_pos = GPIO2PORT(gpio) * 2, pin_pos = (gpio % 32) * 2;
    if (pin_pos >= 32) {
        pin_pos -= 32;
        bank_pos++;
    }
    uint32_t sel_bits = (func & 0x03) << pin_pos, mask = ~(0x03 << pin_pos);
    uint32_t mode = (pull_up ? (pull_up > 0 ? 0x00 : 0x03) : 0x02) << pin_pos;
    volatile uint32_t *pinsel = &LPC_PINCON->PINSEL0;
    volatile uint32_t *pinmode = &LPC_PINCON->PINMODE0;
    irqstatus_t flag = irq_save();
    pinsel[bank_pos] = (pinsel[bank_pos] & mask) | sel_bits;
    pinmode[bank_pos] = (pinmode[bank_pos] & mask) | mode;
    irq_restore(flag);
}

// Convert a register and bit location back to an integer pin identifier
static int
regs_to_pin(LPC_GPIO_TypeDef *regs, uint32_t bit)
{
    int i;
    for (i=0; i<ARRAY_SIZE(digital_regs); i++)
        if (digital_regs[i] == regs)
            return GPIO(i, ffs(bit)-1);
    return 0;
}


/****************************************************************
 * General Purpose Input Output (GPIO) pins
 ****************************************************************/

struct gpio_out
gpio_out_setup(uint8_t pin, uint8_t val)
{
    if (GPIO2PORT(pin) >= ARRAY_SIZE(digital_regs))
        goto fail;
    LPC_GPIO_TypeDef *regs = digital_regs[GPIO2PORT(pin)];
    struct gpio_out g = { .regs=regs, .bit=GPIO2BIT(pin), .index=GPIO2INDEX(pin) };
    gpio_out_reset(g, val);
    return g;
fail:
    shutdown("Not an output pin");
}

void
dac_set(struct dac_dev_s *dev, uint32_t value, uint8_t noirq)
{
    // MCP4822
    // uint8_t data[] = {
    //     (dev->channel << 7)     // channel
    //     | (0 << 5)              // !gain
    //     | (1 << 4)              // !shutdown
    //     | ((value >> 8) & 15),  // 4 high bits
    //     (value & 255),          // 8 low bits
    // };

    // MAX541
    // uint8_t data[] = {
    //     (value >> 8),              // 8 high bits
    //     (value & 255),             // 8 low bits
    // };

    // struct gpio_out *cs = &dev->cs;
    // LPC_GPIO_TypeDef *cs_regs = cs->regs;
    // irqstatus_t flag;
    // if (!noirq)
    //     flag = irq_save();
    // spi_prepare(dev->spi);
    // cs_regs->FIOCLR = cs->bit; //gpio_out_write(dev->cs, 0);
    // spi_transfer(dev->spi, 0, sizeof(data), data);
    // cs_regs->FIOSET = cs->bit; //gpio_out_write(dev->cs, 1);
    // if (!noirq)
    //     irq_restore(flag);

    // DAC8562SDGSR
    uint8_t c = dev->channel;
    uint8_t data[] = {
        (0 << 5) | (1 << 4) | (1 << 3) |    
        (0 << 2) | (0 << 1) | (c << 0),     // X X 0 1 1 0 0 c - Write to input register A|B and update DAC register A|B
        (value >> 8),                       // 8 high bits
        (value & 255),                      // 8 low bits
    };

    struct gpio_out *cs = &dev->cs;
    LPC_GPIO_TypeDef *cs_regs = cs->regs;
    irqstatus_t flag;
    if (!noirq)
        flag = irq_save();
    spi_prepare(dev->spi);
    cs_regs->FIOCLR = cs->bit; //gpio_out_write(dev->cs, 0);
    spi_transfer(dev->spi, 0, sizeof(data), data);
    cs_regs->FIOSET = cs->bit; //gpio_out_write(dev->cs, 1);
    if (!noirq)
        irq_restore(flag);
}

void
dac_init(struct dac_dev_s *dev)
{
    dev->cs = gpio_out_setup(dev->cs_pin, 1);
    dev->spi = spi_setup(0, 1 /* DAC8562SDGSR */, DAC_RATE);

    // DAC8562SDGSR
    uint8_t data_reset[] = {
        (1 << 5) | (0 << 4) | (1 << 3),     // X X 1 0 1 X X X - Software reset
        0,                                  // X X X X X X X X    
        (1 << 0),                           // X X X X X X X 1 - Reset all registers and update all DACs (Power-on-reset update)
    };
    uint8_t int_ref_gain2[] = {
        (1 << 5) | (1 << 4) | (1 << 3),     // X X 1 1 1 X X X - Enable or disable the internal reference
        0,                                  // X X X X X X X X    
        (1 << 0),                           // X X X X X X X 1 - Enable internal reference and reset DACs to gain = 2
    };
    uint8_t ab_gain1[] = {
        (0 << 5) | (0 << 4) | (0 << 3) |     
        (0 << 2) | (1 << 1) | (0 << 0),     // X X 0 0 0 X X X - Write to input register [GAIN]
        0,                                  // X X X X X X X X    
        (1 << 1) | (1 << 0),                // X X X X X X 1 1 - Gain: DAC-B gain = 1, DAC-A gain = 1
    };
    uint8_t synch_mode_ab[] = {
        (1 << 5) | (1 << 4) | (0 << 3),     // X X 1 1 0 X X X - Set LDAC register
        (0 << 2) | (1 << 1) | (1 << 0),     // X X X X X X 1 1 - LDAC pin inactive for DAC-B and DAC-A
    };
    uint8_t dac_ab_to_zero[] = {
        (0 << 5) | (1 << 4) | (1 << 3) |    
        (1 << 2) | (1 << 1) | (1 << 0),     // X X 0 1 1 1 1 1 - Write to input registers A&B and update DAC registers A&B
        0,                                  // 0 0 0 0 0 0 0 0 
        0,                                  // 0 0 0 0 0 0 0 0 = 0 = 0V
    };
    uint8_t power_up_ab[] = {
        (1 << 5) | (0 << 4) | (0 << 3) |    // X X 1 0 0 X X X - Set DAC power up or down mode
        0,                                  // X X X X X X X X    
        (0 << 5) | (0 << 4) | (0 << 3) |    
        (0 << 2) | (1 << 1) | (1 << 0),     // X X 0 0 X X 1 1 - Power up DAC-A and DAC-B
    };

    irqstatus_t flag;
    flag = irq_save();
    spi_prepare(dev->spi);

    gpio_out_write(dev->cs, 0);
    spi_transfer(dev->spi, 0, sizeof(data_reset), data_reset);
    gpio_out_write(dev->cs, 1);

    gpio_out_write(dev->cs, 0);
    spi_transfer(dev->spi, 0, sizeof(int_ref_gain2), int_ref_gain2);
    gpio_out_write(dev->cs, 1);

    gpio_out_write(dev->cs, 0);
    spi_transfer(dev->spi, 0, sizeof(int_ref_gain2), ab_gain1);
    gpio_out_write(dev->cs, 1);

    gpio_out_write(dev->cs, 0);
    spi_transfer(dev->spi, 0, sizeof(synch_mode_ab), synch_mode_ab);
    gpio_out_write(dev->cs, 1);

    gpio_out_write(dev->cs, 0);
    spi_transfer(dev->spi, 0, sizeof(dac_ab_to_zero), dac_ab_to_zero);
    gpio_out_write(dev->cs, 1);

    gpio_out_write(dev->cs, 0);
    spi_transfer(dev->spi, 0, sizeof(power_up_ab), power_up_ab);
    gpio_out_write(dev->cs, 1);

    irq_restore(flag);
}

void
gpio_out_reset(struct gpio_out g, uint8_t val)
{
    if (g.regs == dac_regs)
    {
        //sendf("gpio_out_reset dac_reg=%c", g.bit); // TODO: remove

        struct dac_reg_s *reg = &dac_regs[g.index];
        *reg->value = val;
        dac_init(reg->dac);
    }
    else
    {
        LPC_GPIO_TypeDef *regs = g.regs;
        int pin = regs_to_pin(regs, g.bit);
        irqstatus_t flag = irq_save();
        regs->FIOPIN = (regs->FIOSET & ~g.bit) | (val ? g.bit : 0);
        regs->FIODIR |= g.bit;
        gpio_peripheral(pin, 0, 0);
        irq_restore(flag);
    }
}

void
dac_double_toggle_count(struct gpio_out g, uint8_t noirq, uint32_t count)
{
    struct dac_reg_s *reg = &dac_regs[g.index];
    struct dac_dev_s *dac = reg->dac;
    if (reg->value == &dac->step_state) // touching "step" pin (not the "dir" pin)
    {
        if (dac->dir_state) // increment, endstop does NOT trigger here
        {
            uint32_t newValue = dac->value + count;
            if (likely(newValue <= dac->value_max))
                dac->value = newValue;
            else
                dac->value = dac->value_max;
            dac->stop_state = 0;
        }
        else // decrement, endstop triggers here
        {
            uint32_t oldValue = dac->value;
            if (likely(oldValue > count))
            {
                dac->value = oldValue - count;
                dac->stop_state = 0;
            }
            else
            {
                dac->value = 0;
                dac->stop_state = 1;
            }
        }
        dac_set(dac, dac->value, noirq);
    }
}

void
dac_toggle_or_write(struct gpio_out g, uint8_t toggle, uint8_t write_value, uint8_t noirq)
{
    struct dac_reg_s *reg = &dac_regs[g.index];
    struct dac_dev_s *dac = reg->dac;
    uint8_t old_value = *reg->value;
    if (toggle)
        write_value = !old_value;
    else if (old_value == write_value)
        return;
    *reg->value = write_value;
    if (reg->value == &dac->step_state) // touching "step" pin (not the "dir" pin)
    {
        if (!old_value) // step from 0 -> 1
        {
            if (dac->dir_state) // increment, endstop does NOT trigger here
            {
                uint32_t newValue = dac->value++;
                if (likely(newValue <= dac->value_max))
                    dac->value = newValue;
                else
                    dac->value = dac->value_max;
                dac->stop_state = 0;
            }
            else // decrement, endstop triggers here
            {
                if (likely(dac->value > 1))
                {
                    dac->value--;
                    dac->stop_state = 0;
                }
                else
                {
                    dac->value = 0;
                    dac->stop_state = 1;
                }
            }
            dac_set(dac, dac->value, noirq);
        }
    }
}

void
gpio_out_toggle_noirq(struct gpio_out g)
{
    if (g.regs == dac_regs)
    {
        dac_toggle_or_write(g, 1, 0, 1);
    }
    else
    {
        LPC_GPIO_TypeDef *regs = g.regs;
        regs->FIOPIN = regs->FIOSET ^ g.bit;
    }
}

void
gpio_out_double_toggle_count_noirq(struct gpio_out g, uint32_t count)
{
    if (g.regs == dac_regs)
    {
        dac_double_toggle_count(g, 1, count);
    }
    else
    {
        gpio_out_toggle_noirq(g);
        gpio_out_toggle_noirq(g);
    }
}

void
gpio_out_toggle(struct gpio_out g)
{
    if (g.regs == dac_regs)
    {
        dac_toggle_or_write(g, 1, 0, 0);
    }
    else
    {
        irqstatus_t flag = irq_save();
        gpio_out_toggle_noirq(g);
        irq_restore(flag);
    }
}

void
gpio_out_write(struct gpio_out g, uint8_t val)
{
    if (g.regs == dac_regs)
    {
        dac_toggle_or_write(g, 0, val, 0);
    }
    else
    {
        LPC_GPIO_TypeDef *regs = g.regs;
        if (val)
            regs->FIOSET = g.bit;
        else
            regs->FIOCLR = g.bit;
    }
}


struct gpio_in
gpio_in_setup(uint8_t pin, int8_t pull_up)
{
    if (GPIO2PORT(pin) >= ARRAY_SIZE(digital_regs))
        goto fail;
    LPC_GPIO_TypeDef *regs = digital_regs[GPIO2PORT(pin)];
    struct gpio_in g = { .regs=regs, .bit=GPIO2BIT(pin), .index=GPIO2INDEX(pin) };
    gpio_in_reset(g, pull_up);
    return g;
fail:
    shutdown("Not an input pin");
}

void
gpio_in_reset(struct gpio_in g, int8_t pull_up)
{
    if (g.regs == dac_regs)
    {
        // nothing to do
    }
    else
    {
        LPC_GPIO_TypeDef *regs = g.regs;
        int pin = regs_to_pin(regs, g.bit);
        irqstatus_t flag = irq_save();
        gpio_peripheral(pin, 0, pull_up);
        regs->FIODIR &= ~g.bit;
        irq_restore(flag);
    }
}

uint8_t
gpio_in_read(struct gpio_in g)
{
    if (g.regs == dac_regs)
    {
        struct dac_reg_s *reg = &dac_regs[g.index];
        return *reg->value;
    }
    else
    {
        LPC_GPIO_TypeDef *regs = g.regs;
        return !!(regs->FIOPIN & g.bit);
    }
}

void
gpio_dac_reset(struct gpio_out g)
{
    struct dac_reg_s *reg = &dac_regs[g.index];
    struct dac_dev_s *dac = reg->dac;
    dac_init(dac);
    // NOTE: do not try so set dac->value to the DAC since reset of another axis/gpio can reset the same chip
}

void
gpio_dac_set(struct gpio_out g, uint32_t value)
{
    struct dac_reg_s *reg = &dac_regs[g.index];
    struct dac_dev_s *dac = reg->dac;
    dac->value = value;
    dac_set(dac, dac->value, 0);
}
