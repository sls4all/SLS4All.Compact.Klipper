#ifndef __LPC176X_GPIO_H
#define __LPC176X_GPIO_H

#include <stdint.h>

struct gpio_out {
    void *regs;
    uint32_t bit;
    uint32_t index;
};
struct gpio_out gpio_out_setup(uint8_t pin, uint8_t val);
void gpio_out_reset(struct gpio_out g, uint8_t val);
void gpio_out_toggle_noirq(struct gpio_out g);
void gpio_out_double_toggle_count_noirq(struct gpio_out g, uint32_t count);
void gpio_out_toggle(struct gpio_out g);
void gpio_out_write(struct gpio_out g, uint8_t val);

struct gpio_in {
    void *regs;
    uint32_t bit;
    uint32_t index;
};
struct gpio_in gpio_in_setup(uint8_t pin, int8_t pull_up);
void gpio_in_reset(struct gpio_in g, int8_t pull_up);
uint8_t gpio_in_read(struct gpio_in g);

struct gpio_pwm {
    void *reg;
    uint8_t channel;
};
struct gpio_pwm gpio_pwm_setup(uint8_t pin, uint32_t cycle_time, uint32_t val);
void gpio_pwm_write(struct gpio_pwm g, uint32_t val);

struct gpio_adc {
    uint32_t chan;
};
struct gpio_adc gpio_adc_setup(uint8_t pin);
uint32_t gpio_adc_sample(struct gpio_adc g);
uint16_t gpio_adc_read(struct gpio_adc g);
void gpio_adc_cancel_sample(struct gpio_adc g);

struct spi_config {
    void *spi;
    uint32_t cr0, cpsr;
};
struct spi_config spi_setup(uint32_t bus, uint8_t mode, uint32_t rate);
void spi_prepare(struct spi_config config);
void spi_transfer(struct spi_config config, uint8_t receive_data
                  , uint8_t len, uint8_t *data);

struct i2c_config {
    void *i2c;
    uint8_t addr;
};

struct i2c_config i2c_setup(uint32_t bus, uint32_t rate, uint8_t addr);
void i2c_write(struct i2c_config config, uint8_t write_len, uint8_t *write);
void i2c_read(struct i2c_config config, uint8_t reg_len, uint8_t *reg
              , uint8_t read_len, uint8_t *read);

void gpio_oid_pwm_set_noirq(uint8_t oid, uint32_t val);

uint8_t i2c_write_ok(struct i2c_config *config, uint16_t write_len, uint8_t *write);
uint8_t i2c_read_ok(struct i2c_config *config, uint16_t reg_len, uint8_t *reg
              , uint16_t read_len, uint8_t *read);
              
void gpio_dac_reset(struct gpio_out g);
void gpio_dac_set(struct gpio_out g, uint32_t value);

#endif // gpio.h
