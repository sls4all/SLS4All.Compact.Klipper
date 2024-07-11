# Main code build rules

src-y += sched.c command.c basecmd.c debugcmds.c
src-$(CONFIG_HAVE_GPIO) += initial_pins.c gpiocmds.c
src-$(CONFIG_HAVE_STEPPER) += stepper.c endstop.c
src-$(CONFIG_HAVE_GPIO_ADC) += adccmds.c
src-$(CONFIG_HAVE_GPIO_SPI) += spicmds.c thermocouple.c
src-$(CONFIG_HAVE_GPIO_I2C) += i2ccmds.c
src-$(CONFIG_HAVE_GPIO_HARD_PWM) += pwmcmds.c
src-$(CONFIG_HAVE_GPIO_TMCUART) += tmcuart.c
src-$(CONFIG_HAVE_GPIO_BUTTONS) += buttons.c
src-$(CONFIG_HAVE_GPIO_BITBANGING) += lcd_st7920.c lcd_hd44780.c \
    spi_software.c neopixel.c sensor_adxl345.c
src-$(CONFIG_HAVE_GPIO_DIMMER) += dimmer.c
