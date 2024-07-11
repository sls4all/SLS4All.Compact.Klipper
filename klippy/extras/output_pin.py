# Code to configure miscellaneous chips
#
# Copyright (C) 2017,2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

RESEND_HOST_TIME = 0.300
PRIORITY_DELAY = 0.005
MAX_SCHEDULE_TIME = 5.0
SET_PIN_DELAY = 0.00001
#SET_PIN_DELAY = 0.0004

class PrinterOutputPin:
    def __init__(self, config):
        self.printer = config.get_printer()
        ppins = self.printer.lookup_object('pins')
        self.is_dimmer = config.getboolean('dimmer', False)
        self.is_pwm = config.getboolean('pwm', False)
        if self.is_dimmer:
            self.mcu_pin = ppins.setup_pin(
                'dimmer', 
                config.get('pin'), 
                sensor_pin_desc=config.get('sensor'),
                control_periods=config.getint('control_periods', 1, minval=0))
            self.scale = 1.
            self.last_cycle_time = self.default_cycle_time = 0.
        elif self.is_pwm:
            self.mcu_pin = ppins.setup_pin('pwm', config.get('pin'))
            cycle_time = config.getfloat('cycle_time', 0.100, above=0.,
                                         maxval=MAX_SCHEDULE_TIME)
            hardware_pwm = config.getboolean('hardware_pwm', False)
            self.mcu_pin.setup_cycle_time(cycle_time, hardware_pwm)
            self.scale = config.getfloat('scale', 1., above=0.)
            self.last_cycle_time = self.default_cycle_time = cycle_time
        else:
            self.mcu_pin = ppins.setup_pin('digital_out', config.get('pin'))
            self.scale = 1.
            self.last_cycle_time = self.default_cycle_time = 0.
        self.last_is_priority = False
        self.last_is_precise = False
        self.last_print_time = 0.
        static_value = config.getfloat('static_value', None,
                                       minval=0., maxval=self.scale)
        self.reactor = self.printer.get_reactor()
        self.resend_timer = None
        self.resend_interval = 0.
        if static_value is not None:
            self.mcu_pin.setup_max_duration(0.)
            self.last_value = static_value / self.scale
            self.mcu_pin.setup_start_value(
                self.last_value, self.last_value, True)
        else:
            max_mcu_duration = config.getfloat('maximum_mcu_duration', 0.,
                                               minval=0.500,
                                               maxval=MAX_SCHEDULE_TIME)
            self.mcu_pin.setup_max_duration(max_mcu_duration)
            if max_mcu_duration:
                self.resend_interval = max_mcu_duration - RESEND_HOST_TIME

            self.last_value = config.getfloat(
                'value', 0., minval=0., maxval=self.scale) / self.scale
            self.shutdown_value = config.getfloat(
                'shutdown_value', 0., minval=0., maxval=self.scale) / self.scale
            self.mcu_pin.setup_start_value(self.last_value, self.shutdown_value)
            pin_name = config.get_name().split()[1]
            gcode = self.printer.lookup_object('gcode')
            gcode.register_mux_command("SET_PIN", "PIN", pin_name,
                                       self.cmd_SET_PIN,
                                       desc=self.cmd_SET_PIN_help)
            wh = self.printer.lookup_object('webhooks')
            wh.register_endpoint("gcode/set_pin/" + pin_name, self.cmd_SET_PIN_webhooks)

    def get_status(self, eventtime):
        return {'value': self.last_value}
    def _set_pin(self, print_time, value, cycle_time, is_resend=False, is_precise=False, is_priority=True):
        if value == self.last_value and cycle_time == self.last_cycle_time:
            if not is_resend:
                return
        set_time = max(print_time + SET_PIN_DELAY * 0.5, self.last_print_time)
        if self.is_dimmer:
            self.mcu_pin.set_dimmer(set_time, value)
        elif self.is_pwm:
            self.mcu_pin.set_pwm(set_time, value, cycle_time)
        else:
            self.mcu_pin.set_digital(set_time, value)
        self.last_is_precise = is_precise
        self.last_is_priority = is_priority
        self.last_value = value
        self.last_cycle_time = cycle_time
        self.last_print_time = set_time
        if self.resend_interval and self.resend_timer is None:
            self.resend_timer = self.reactor.register_timer(
                self._resend_current_val, self.reactor.NOW)
        return SET_PIN_DELAY 
    cmd_SET_PIN_help = "Set the value of an output pin"

    def cmd_SET_PIN_inner(self, value, is_precise, is_priority, cycle_time):
        if not is_priority:
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.register_lookahead_callback(
                lambda print_time: self._set_pin(print_time, value, cycle_time, is_precise=is_precise, is_priority=is_priority))
        else:
            systime = self.reactor.monotonic()
            print_time = self.mcu_pin.get_mcu().estimated_print_time(systime) + PRIORITY_DELAY # priority pins are scheduled with host time, make MCU have bit of delay to schedule it
            self._set_pin(print_time, value, cycle_time, is_precise=is_precise, is_priority=is_priority)

    def cmd_SET_PIN(self, gcmd):
        value = gcmd.get_float('VALUE', minval=0., maxval=self.scale)
        is_precise = gcmd.get_int('PRECISE', 0) != 0
        is_priority = gcmd.get_int('PRIORITY', 0) != 0        
        value /= self.scale
        cycle_time = gcmd.get_float('CYCLE_TIME', self.default_cycle_time,
                                    above=0., maxval=MAX_SCHEDULE_TIME)
        if not self.is_pwm and not self.is_dimmer and value not in [0., 1.]:
            raise gcmd.error("Invalid pin value")
        self.cmd_SET_PIN_inner(value, is_precise, is_priority, cycle_time)

    def cmd_SET_PIN_webhooks(self, web_request):
        value = web_request.get_float('VALUE')
        if value < 0. or value > self.scale:
            web_request.set_error("Invalid pin value")
            return
        is_precise = web_request.get_int('PRECISE', 0) != 0
        is_priority = web_request.get_int('PRIORITY', 0) != 0        
        value /= self.scale
        cycle_time = web_request.get_float('CYCLE_TIME', self.default_cycle_time)
        if cycle_time < 0 or cycle_time > MAX_SCHEDULE_TIME:
            web_request.set_error("Invalid cycle_time value")
            return
        if not self.is_pwm and not self.is_dimmer and value not in [0., 1.]:
            web_request.set_error("Invalid pin value")
            return
        self.cmd_SET_PIN_inner(value, is_precise, is_priority, cycle_time)

    def _resend_current_val(self, eventtime):
        if self.last_value == self.shutdown_value:
            self.reactor.unregister_timer(self.resend_timer)
            self.resend_timer = None
            return self.reactor.NEVER

        systime = self.reactor.monotonic()
        print_time = self.mcu_pin.get_mcu().estimated_print_time(systime)
        time_diff = (self.last_print_time + self.resend_interval) - print_time
        if time_diff > 0.:
            # Reschedule for resend time
            return systime + time_diff
        if self.last_is_priority: # priority pins are scheduled with host time, make MCU have bit of delay to schedule it
            print_time = print_time + PRIORITY_DELAY
        self._set_pin(print_time,
                      self.last_value, self.last_cycle_time, True, is_precise=self.last_is_precise, is_priority=self.last_is_priority)
        return systime + self.resend_interval

def load_config_prefix(config):
    return PrinterOutputPin(config)
