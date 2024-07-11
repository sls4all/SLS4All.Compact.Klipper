# Support a fan for cooling the MCU whenever a stepper or heater is on
#
# Copyright (C) 2019  Nils Friedchen <nils.friedchen@googlemail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import fan

PIN_MIN_TIME = 0.100

class ControllerFan:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.stepper_names = []
        self.stepper_enable = self.printer.load_object(config, 'stepper_enable')
        self.printer.load_object(config, 'heaters')
        self.heaters = []
        self.fan = fan.Fan(config)
        self.fan_speed = config.getfloat('fan_speed', default=1.,
                                         minval=0., maxval=1.)
        self.idle_speed = config.getfloat(
            'idle_speed', default=self.fan_speed, minval=0., maxval=1.)
        self.idle_timeout = config.getint("idle_timeout", default=30, minval=0)
        self.heater_name = config.get("heater", "extruder")
        self.last_on = self.idle_timeout
    def handle_ready(self):
        pheaters = self.printer.lookup_object('heaters')
        self.heaters = [pheaters.lookup_heater(n.strip())
                        for n in self.heater_name.split(',')]
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        self.stepper_names = [s.get_name() for s in kin.get_steppers()]
        reactor = self.printer.get_reactor()
        reactor.register_timer(self.callback, reactor.NOW)
    def callback(self, eventtime):
        power = 0.
        active = False
        for name in self.stepper_names:
            active |= self.stepper_enable.lookup_enable(name).is_motor_enabled()
        for heater in self.heaters:
            _, target_temp = heater.get_temp(eventtime)
            if target_temp:
                active = True
        if active:
            self.last_on = 0
            power = self.fan_speed
        elif self.last_on < self.idle_timeout:
            power = self.idle_speed
            self.last_on += 1
        print_time = self.fan.get_mcu().estimated_print_time(eventtime)
        self.fan.set_speed(print_time + PIN_MIN_TIME, power)
        return eventtime + 1.

def load_config_prefix(config):
    return ControllerFan(config)
