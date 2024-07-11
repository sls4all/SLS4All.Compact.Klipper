# Code for managing power draw
#
# Copyright (C) 2021-2022  Pavel Dyntera (yourself@yourself.cz)
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging, importlib, threading

PIN_DELAY = 0.01

class error(Exception):
    pass

class powerman_PIN:
    def __init__(self, name, power):
        self.name = name
        self.power = power
        self.pin = None
        self.on_time = None
        self.power_duration = 0
        self.power_time = None
        self.on_value = 1
        self.cycle_time = None

class powerman:
    def __init__(self, config):
        self.pins = {}
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.lock = threading.Lock()
        gcode = self.printer.lookup_object('gcode')
        powers_parts = [x.strip() for x in config.get("pin_powers", "").split(',')]
        self.required_power = 0.
        self.current_power = 0.
        self.max_power = config.getfloat("max_power", 0, minval = 0)
        self.switch_period = config.getfloat("switch_period", 1, minval = 0)
        for pair in powers_parts:
            parts = pair.split("=")
            if len(parts) != 2:
                raise error("Invalid pin power: " + pair)
            name = parts[0].upper()
            power = float(parts[1])
            self.pins[name] = powerman_PIN(name, power)
        gcode.register_command('SET_POWERMAN_MAX', self.cmd_SET_POWERMAN_MAX)

        self.timer = self.reactor.register_timer(self.switch_power, self.reactor.NOW)

    def cmd_SET_POWERMAN_MAX(self, gcmd):
        with self.lock:
            toolhead = self.printer.lookup_object('toolhead')
            self.max_power = gcmd.get_float('POWER', self.max_power, minval = 0)
            time = self.reactor.monotonic()
            print_time = toolhead.get_last_move_time()
            for info in map(lambda x: x[1], sorted(self.pins.items(), reverse=True, key=lambda item: item[1].power)):
                if self.current_power <= self.max_power:
                    break
                self.try_power_off(time, print_time, info)

    def get_status(self, eventtime):
        with self.lock:
            powered_pins = []
            for pair in self.pins.items():
                if pair[1].power_time is not None:   # powered on
                    powered_pins.append(pair[0])
            return {
                'max_power': self.max_power, 
                'current_power': self.current_power,
                'required_power': self.required_power,
                'powered_pins': ','.join(powered_pins)
            }
            
    def set_value(self, pin, print_time, value, cycle_time=None):
        name = pin._pin.upper()
        if name not in self.pins:
            pin.set_value_callback(print_time, value, cycle_time=cycle_time)
            return

        with self.lock:
            info = self.pins[name]
            time = self.reactor.monotonic()
            info.pin = pin

            if value:
                info.on_value = value
                info.cycle_time = cycle_time
                if info.on_time is None:            # switch on
                    info.on_time = time
                    info.power_duration = 0.
                    info.power_time = None
                    self.required_power = self.required_power + info.power
                    self.try_power_on(time, print_time, info)
                elif info.power_time is not None:   # powered on
                    pin.set_value_callback(print_time, value, cycle_time=cycle_time)
            else:
                info.cycle_time = cycle_time
                if info.on_time is not None:         # switch off
                    self.try_power_off(time, print_time, info)
                    info.on_time = None
                    self.required_power = self.required_power - info.power

    def try_power_on(self, time, print_time_or_none, info):
        if info.power_time is not None: # already powered on
            return True
        if self.current_power + info.power > self.max_power: # power exceeded
            return False
        self.current_power = self.current_power + info.power
        info.power_time = time
        if print_time_or_none is None:
            print_time_or_none = info.pin.get_mcu().estimated_print_time(self.reactor.monotonic()) + PIN_DELAY
        info.pin.set_value_callback(print_time_or_none, info.on_value, cycle_time=info.cycle_time)
        return True

    def try_power_off(self, time, print_time_or_none, info):
        if info.power_time is None: # not powered on
            return
        assert self.current_power >= info.power 
        self.current_power = self.current_power - info.power
        info.power_duration = info.power_duration + (time - info.power_time)
        info.power_time = None
        if print_time_or_none is None:
            print_time_or_none = info.pin.get_mcu().estimated_print_time(self.reactor.monotonic()) + PIN_DELAY
        info.pin.set_value_callback(print_time_or_none, 0., cycle_time=info.cycle_time)

    def switch_power(self, eventtime):
        with self.lock:
            powered_on_debts = {}
            powered_off_debts = {}
            all_debts = {}
            time = self.reactor.monotonic()

            for name in self.pins:
                info = self.pins[name]
                if info.on_time is None: # switched off
                    continue
                on_duration = time - info.on_time
                power_duration = info.power_duration
                if info.power_time is not None: # powered on
                    power_duration = power_duration + (time - info.power_time)
                power_debt = on_duration - power_duration
                
                if info.power_time is not None: # powered on
                    powered_on_debts[info] = power_debt
                else:
                    powered_off_debts[info] = power_debt
                all_debts[info] = power_debt

            if powered_off_debts: # something is switched on byt powered off -> rebalance
                for item in map(lambda x: x[0], sorted(all_debts.items(), reverse=True, key=lambda item: item[1])):
                    if item.power_time is not None: # powered on
                        # since we are going from items with most debt to least
                        # remove this item from power-off-candidates
                        if item in powered_on_debts:
                            del powered_on_debts[item]
                        continue

                    available = self.max_power - self.current_power
                    assert available >= 0
                    remaining = item.power - available
                    will_power_off = []
                    for powered_on in map(lambda x: x[0], sorted(powered_on_debts.items(), reverse=False, key=lambda item: (item[1], -item[0].power))):
                        if remaining <= 0:
                            break
                        remaining = remaining - powered_on.power
                        will_power_off.append(powered_on)
                    if remaining > 0: # nothing can be done
                        continue
                    for powered_on in will_power_off:
                        self.try_power_off(time, None, powered_on)
                        del powered_on_debts[powered_on]
                    assert self.try_power_on(time, None, item)

                msg = "Pin debts:"
                for item in all_debts:
                    msg = msg + " " + item.name + "=" + str(all_debts[item])
                logging.info(msg)
        return self.reactor.monotonic() + self.switch_period

def add_printer_objects(config):
    printer = config.get_printer()
    printer.add_object('powerman', powerman(config.getsection('powerman')))
