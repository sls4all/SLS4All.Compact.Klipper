# Tracking of PWM controlled heaters and their temperature control
#
# Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2021-2022  Pavel Dyntera (yourself@yourself.cz)
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, threading
from xml.etree.ElementTree import tostring

class error(Exception):
    pass

######################################################################
# Heater
######################################################################

KELVIN_TO_CELSIUS = -273.15
MAX_HEAT_TIME = 10.0
MIN_HEAT_PERIOD = 1.0
AMBIENT_TEMP = 25.
PID_PARAM_BASE = 255.
PIN_DELAY = 0.01

class Heater:
    def __init__(self, config, sensor):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        # Setup sensor
        self.sensor = sensor
        self.min_temp = config.getfloat('min_temp', minval=KELVIN_TO_CELSIUS)
        self.max_temp = config.getfloat('max_temp', above=self.min_temp)
        self.sensor.setup_minmax(self.min_temp, self.max_temp)
        self.sensor.setup_callback(self.temperature_callback)
        self.pwm_delay = self.sensor.get_report_time_delta()
        # Setup temperature checks
        self.min_extrude_temp = config.getfloat(
            'min_extrude_temp', 170.,
            minval=self.min_temp, maxval=self.max_temp)
        is_fileoutput = (self.printer.get_start_args().get('debugoutput')
                         is not None)
        self.can_extrude = self.min_extrude_temp <= 0. or is_fileoutput
        self.max_power = config.getfloat('max_power', 1., above=0., maxval=1.)
        self.smooth_time = config.getfloat('smooth_time', 2., above=0.)
        self.inv_smooth_time = 1. / self.smooth_time
        self.lock = threading.Lock()
        self.last_temp = self.smoothed_temp = self.target_temp = 0.
        self.last_temp_time = 0.
        # pwm caching
        self.next_pwm_time = 0.
        self.last_pwm_value = 0.
        # Setup control algorithm sub-class
        algos = {'watermark': ControlBangBang, 'pid': ControlPID}
        algo = config.getchoice('control', algos)
        self.control = algo(self, config)
        # Setup output heater pin
        heater_pin = config.get('heater_pin')
        ppins = self.printer.lookup_object('pins')
        if algo is ControlBangBang and self.max_power == 1.:
            self.mcu_pwm = ppins.setup_pin('digital_out', heater_pin)
        else:
            self.mcu_pwm = ppins.setup_pin('pwm', heater_pin)
            pwm_cycle_time = config.getfloat(
                'pwm_cycle_time', 0.100, above=0., maxval=self.pwm_delay)
            self.mcu_pwm.setup_cycle_time(pwm_cycle_time)
        self.mcu_pwm.setup_max_duration(MAX_HEAT_TIME)
        # Load additional modules
        self.printer.load_object(config, "verify_heater %s" % (self.name,))
        self.printer.load_object(config, "pid_calibrate")
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("SET_HEATER_TEMPERATURE", "HEATER",
                                   self.name, self.cmd_SET_HEATER_TEMPERATURE,
                                   desc=self.cmd_SET_HEATER_TEMPERATURE_help)
    def set_pwm(self, read_time, value):
        if self.target_temp <= 0.:
            value = 0.
        measured_time = self.printer.reactor.monotonic()
        estimated_print_time = self.mcu_pwm.get_mcu().estimated_print_time(measured_time) + PIN_DELAY
        self.set_pwm_inner(estimated_print_time, value)
    
    def set_pwm_inner(self, read_time, value):
        if ((read_time < self.next_pwm_time or not self.last_pwm_value)
            and abs(value - self.last_pwm_value) < 0.05):
            # No significant change in value - can suppress update
            return
        self.next_pwm_time = read_time + MIN_HEAT_PERIOD
        self.last_pwm_value = value
        self.mcu_pwm.set_pwm(read_time, value)
        #logging.debug("%s: pwm=%.3f@%.3f (from %.3f@%.3f [%.3f])",
        #              self.name, value, pwm_time,
        #              self.last_temp, self.last_temp_time, self.target_temp)
    def temperature_callback(self, read_time, temp):
        with self.lock:
            time_diff = read_time - self.last_temp_time
            self.last_temp = temp
            self.last_temp_time = read_time
            self.control.temperature_update(read_time, temp, self.target_temp)
            temp_diff = temp - self.smoothed_temp
            adj_time = min(time_diff * self.inv_smooth_time, 1.)
            self.smoothed_temp += temp_diff * adj_time
            self.can_extrude = (self.smoothed_temp >= self.min_extrude_temp)
        #logging.debug("temp: %.3f %f = %f", read_time, temp)
    # External commands
    def get_pwm_delay(self):
        return self.pwm_delay
    def get_max_power(self):
        return self.max_power
    def get_smooth_time(self):
        return self.smooth_time
    def set_temp(self, degrees):
        if degrees and (degrees < self.min_temp or degrees > self.max_temp):
            raise self.printer.command_error(
                "Requested temperature (%.1f) out of range (%.1f:%.1f)"
                % (degrees, self.min_temp, self.max_temp))
        with self.lock:
            self.target_temp = degrees
    def get_temp(self, eventtime):
        print_time = self.mcu_pwm.get_mcu().estimated_print_time(eventtime) - 5.
        with self.lock:
            if self.last_temp_time < print_time:
                return 0., self.target_temp
            return self.smoothed_temp, self.target_temp
    def check_busy(self, eventtime):
        with self.lock:
            return self.control.check_busy(
                eventtime, self.smoothed_temp, self.target_temp)
    def set_control(self, control):
        with self.lock:
            old_control = self.control
            self.control = control
            self.target_temp = 0.
        return old_control
    def alter_target(self, target_temp):
        if target_temp:
            target_temp = max(self.min_temp, min(self.max_temp, target_temp))
        self.target_temp = target_temp
    def stats(self, eventtime):
        with self.lock:
            target_temp = self.target_temp
            last_temp = self.last_temp
            last_pwm_value = self.last_pwm_value
        is_active = target_temp or last_temp > 50.
        return is_active, '%s: target=%.0f temp=%.1f pwm=%.3f' % (
            self.name, target_temp, last_temp, last_pwm_value)
    def get_status(self, eventtime):
        with self.lock:
            target_temp = self.target_temp
            smoothed_temp = self.smoothed_temp
        return {'temperature': smoothed_temp, 'target': target_temp}
    cmd_SET_HEATER_TEMPERATURE_help = "Sets a heater temperature"
    def cmd_SET_HEATER_TEMPERATURE(self, gcmd):
        temp = gcmd.get_float('TARGET', 0.)
        self.set_temp(temp)


######################################################################
# Bang-bang control algo
######################################################################

class ControlBangBang:
    def __init__(self, heater, config):
        self.heater = heater
        self.heater_max_power = heater.get_max_power()
        self.max_delta = config.getfloat('max_delta', 2.0, above=0.)
        self.heating = False
    def temperature_update(self, read_time, temp, target_temp):
        if self.heating and temp >= target_temp+self.max_delta:
            self.heating = False
        elif not self.heating and temp <= target_temp-self.max_delta:
            self.heating = True
        if self.heating:
            self.heater.set_pwm(read_time, self.heater_max_power)
        else:
            self.heater.set_pwm(read_time, 0.)
    def check_busy(self, eventtime, smoothed_temp, target_temp):
        return smoothed_temp < target_temp-self.max_delta


######################################################################
# Proportional Integral Derivative (PID) control algo
######################################################################

PID_SETTLE_DELTA = 1.
PID_SETTLE_SLOPE = .1

class ControlPID:
    def __init__(self, heater, config):
        self.heater = heater
        self.heater_max_power = heater.get_max_power()
        self.Kp = config.getfloat('pid_Kp') / PID_PARAM_BASE
        self.Ki = config.getfloat('pid_Ki') / PID_PARAM_BASE
        self.Kd = config.getfloat('pid_Kd') / PID_PARAM_BASE
        self.min_deriv_time = heater.get_smooth_time()
        imax = config.getfloat('pid_integral_max', self.heater_max_power,
                               minval=0.)
        self.temp_integ_max = 0.
        if self.Ki:
            self.temp_integ_max = imax / self.Ki
        self.prev_temp = AMBIENT_TEMP
        self.prev_temp_time = 0.
        self.prev_temp_deriv = 0.
        self.prev_temp_integ = 0.
    def temperature_update(self, read_time, temp, target_temp):
        time_diff = read_time - self.prev_temp_time
        # Calculate change of temperature
        temp_diff = temp - self.prev_temp
        if time_diff >= self.min_deriv_time:
            temp_deriv = temp_diff / time_diff
        else:
            temp_deriv = (self.prev_temp_deriv * (self.min_deriv_time-time_diff)
                          + temp_diff) / self.min_deriv_time
        # Calculate accumulated temperature "error"
        temp_err = target_temp - temp
        temp_integ = self.prev_temp_integ + temp_err * time_diff
        temp_integ = max(0., min(self.temp_integ_max, temp_integ))
        # Calculate output
        co = self.Kp*temp_err + self.Ki*temp_integ - self.Kd*temp_deriv
        #logging.debug("pid: %f@%.3f -> diff=%f deriv=%f err=%f integ=%f co=%d",
        #    temp, read_time, temp_diff, temp_deriv, temp_err, temp_integ, co)
        bounded_co = max(0., min(self.heater_max_power, co))
        self.heater.set_pwm(read_time, bounded_co)
        # Store state for next measurement
        self.prev_temp = temp
        self.prev_temp_time = read_time
        self.prev_temp_deriv = temp_deriv
        if co == bounded_co:
            self.prev_temp_integ = temp_integ
    def check_busy(self, eventtime, smoothed_temp, target_temp):
        temp_diff = target_temp - smoothed_temp
        return (abs(temp_diff) > PID_SETTLE_DELTA
                or abs(self.prev_temp_deriv) > PID_SETTLE_SLOPE)

######################################################################
# Inova bed heater
######################################################################

class InovaSurfaceHeater:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        # Setup sensor
        self.mode = config.get('mode', "max")
        self.mode_perc = config.getfloat('mode_perc', 0.5)
        self.count = config.getint('count')
        self.minx = []
        self.miny = []
        self.maxx = []
        self.maxy = []
        self.next_pwm_time = []
        self.last_pwm_value = []
        self.last_pwm_time = []
        for i in range(0, self.count):
            self.minx.append(config.getint('minx' + str(i)))
            self.miny.append(config.getint('miny' + str(i)))
            self.maxx.append(config.getint('maxx' + str(i)))
            self.maxy.append(config.getint('maxy' + str(i)))
            self.next_pwm_time.append(0.)
            self.last_pwm_time.append(0.)
            self.last_pwm_value.append(0.)
        self.soft_delta = config.getfloat('soft_delta', 2.0, above=0.)
        self.hard_delta = config.getfloat('hard_delta', 2.0, above=0.)
        self.sensor_name = config.get('sensor')
        self.sensor = self.printer.lookup_object("mlx90640 " + self.sensor_name)
        self.min_temp = config.getfloat('min_temp', minval=KELVIN_TO_CELSIUS)
        self.max_temp = config.getfloat('max_temp', above=self.min_temp)
        self.min_pwm = config.getfloat('min_pwm')
        self.max_pwm = config.getfloat('max_pwm')
        self.lights_enabled = False
        self.lights_count = config.getint('lights_count', 2)
        self.lights_pwm = config.getfloat('lights_pwm', self.min_pwm)
        self.sensor.setup_callback(self.temperature_callback)
        self.pwm_delay = self.sensor.get_report_time_delta()
        self.can_extrude = False
        self.smooth_time = config.getfloat('smooth_time', 2., above=0.)
        self.inv_smooth_time = 1. / self.smooth_time
        self.lock = threading.Lock()
        self.last_temp = self.smoothed_temp = self.target_temp = 0.
        self.target_temp_reached = [False] * self.count
        self.last_temp_time = 0.
        # Setup output heater pins
        ppins = self.printer.lookup_object('pins')
        self.pwm_cycle_time = config.getfloat(
            'pwm_cycle_time', 0.100, above=0., maxval=self.pwm_delay)
        self.mcu_pwms = []
        for i in range(0, self.count):
            pin = ppins.setup_pin('pwm', config.get('heater_pin' + str(i)))
            pin.setup_cycle_time(self.pwm_cycle_time)
            pin.setup_max_duration(MAX_HEAT_TIME)
            self.mcu_pwms.append(pin)
        # Load additional modules
        self.printer.load_object(config, "verify_heater %s" % (self.name,))
        self.printer.load_object(config, "pid_calibrate")
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("SET_HEATER_TEMPERATURE", "HEATER",
                                   self.name, self.cmd_SET_HEATER_TEMPERATURE,
                                   desc=self.cmd_SET_HEATER_TEMPERATURE_help)
        gcode.register_mux_command("SET_LIGHTS", "HEATER",
                                   self.name, self.cmd_SET_LIGHTS)
        wh = self.printer.lookup_object('webhooks')
        wh.register_endpoint("gcode/set_lights/" + self.name, self.cmd_SET_LIGHTS_webhooks)

    def set_pwm(self, read_time, value, index):
        if self.target_temp <= 0. and not self.lights_enabled:
            value = 0.
        measured_time = self.printer.reactor.monotonic()
        estimated_print_time = self.mcu_pwms[index].get_mcu().estimated_print_time(measured_time) + PIN_DELAY
        self.set_pwm_inner(estimated_print_time, value, index)
        #logging.debug("%s: pwm=%.3f@%.3f (from %.3f@%.3f [%.3f])",
        #              self.name, value, pwm_time,
        #              self.last_temp, self.last_temp_time, self.target_temp)

    def set_pwm_inner(self, read_time, value, index):
        if ((read_time < self.next_pwm_time[index] or not self.last_pwm_value[index])
            and abs(value - self.last_pwm_value[index]) <= 0.05):
            # No significant change in value - can suppress update
            return
        if (read_time - self.last_pwm_time[index] <= self.pwm_cycle_time):
            # too fast for possible soft_pwm
            return
        self.next_pwm_time[index] = read_time + MIN_HEAT_PERIOD
        self.last_pwm_time[index] = read_time
        self.last_pwm_value[index] = value
        self.mcu_pwms[index].set_pwm(read_time, value)

    def calc_temp(self, index):
        temp_matrix = self.sensor.temp_matrix
        values = []
        for y in range(self.miny[index], self.maxy[index] + 1):
            for x in range(self.minx[index], self.maxx[index] + 1):
                v = temp_matrix[y * 32 + x]
                values.append(v)
        if self.mode == "max":
            temp = max(values)
        elif self.mode == "min":
            temp = min(values)
        elif self.mode == "avg":
            temp = sum(values) / len(values)
        elif self.mode == "perc":
            sorted_values = sorted(values)
            index = int(self.mode_perc * (len(sorted_values) - 1))
            temp = sorted_values[index]
        else:
            raise error("Invalid mode")
        if temp < self.min_temp or temp > self.max_temp:
            self.printer.invoke_shutdown(
                "MLX90640 temperature %0.1f outside range of %0.1f:%.01f"
                % (temp, self.min_temp, self.max_temp))
        return temp
    
    def temperature_callback(self, read_time, temp):
        action = None
        with self.lock:
            temps = []
            for i in range(0, self.count):
                temps.append(self.calc_temp(i))
            min_temp = min(temps)
            temp = sum(temps) / len(temps)     # override sensor temp with average over all areas

            time_diff = read_time - self.last_temp_time
            self.last_temp = temp
            self.last_temp_time = read_time
            temp_diff = temp - self.smoothed_temp
            adj_time = min(time_diff * self.inv_smooth_time, 1.)
            self.smoothed_temp += temp_diff * adj_time

            # check targets
            for i in range(0, self.count):
                if temps[i] >= self.target_temp:
                    self.target_temp_reached[i] = True

            # control outputs
            if min_temp >= self.target_temp + self.soft_delta:
                # target exceeded
                self.can_extrude = True
                action = "TargetExceeded"
            elif min_temp <= self.target_temp - self.hard_delta:
                # warming up
                self.can_extrude = False
                action = "WarmingUp"
            else:
                # keeping warm using single output
                self.can_extrude = True
                min_temp_index = temps.index(min_temp)
                action = "KeepWarm"

            # lights mode - override
            if self.lights_enabled:
                action = "Lights"

        if action == "Lights":
            for i in range(0, self.count):
                if i < self.lights_count:
                    self.set_pwm(read_time, self.lights_pwm, i)
                else:
                    self.set_pwm(read_time, 0., i)
        elif action == "TargetExceeded":
            for i in range(0, self.count):
                self.set_pwm(read_time, 0.0, i)
        elif action == "WarmingUp":
            for i in range(0, self.count):
                self.set_pwm(read_time, 1.0, i)
        elif action == "KeepWarm":
            for i in range(0, self.count):
                if i == min_temp_index:
                    self.set_pwm(read_time, self.max_pwm, i)
                else:
                    self.set_pwm(read_time, self.min_pwm, i)
        else:
            assert(False)
            
    # External commands
    def get_pwm_delay(self):
        return self.pwm_delay
    def get_max_power(self):
        return self.max_power
    def get_smooth_time(self):
        return self.smooth_time
    def set_temp(self, degrees):
        if degrees and (degrees < self.min_temp or degrees > self.max_temp):
            raise self.printer.command_error(
                "Requested temperature (%.1f) out of range (%.1f:%.1f)"
                % (degrees, self.min_temp, self.max_temp))
        with self.lock:
            self.lights_on = False
            self.target_temp = degrees
            self.target_temp_reached = [False] * self.count
    def set_lights(self, enabled):
        with self.lock:
            self.lights_enabled = enabled
    def get_temp(self, eventtime):
        print_time = self.mcu_pwms[0].get_mcu().estimated_print_time(eventtime) - 5.
        with self.lock:
            if self.last_temp_time < print_time:
                return 0., self.target_temp
            return self.smoothed_temp, self.target_temp
    def check_busy(self, eventtime):
        return False
    def set_control(self, control):
        return None
    def alter_target(self, target_temp):
        if target_temp:
            target_temp = max(self.min_temp, min(self.max_temp, target_temp))
        self.target_temp = target_temp
    def stats(self, eventtime):
        with self.lock:
            target_temp = self.target_temp
            last_temp = self.last_temp
        is_active = target_temp or last_temp > 50.
        return is_active, '%s: target=%.0f temp=%.1f' % (
            self.name, target_temp, last_temp)
    def get_status(self, eventtime):
        with self.lock:
            target_temp = self.target_temp
            smoothed_temp = self.smoothed_temp
            target_reached = all(self.target_temp_reached)
        return {'temperature': smoothed_temp, 'target': target_temp, 'target_reached': target_reached}
    cmd_SET_HEATER_TEMPERATURE_help = "Sets a heater temperature"
    def cmd_SET_HEATER_TEMPERATURE(self, gcmd):
        temp = gcmd.get_float('TARGET', 0.)
        self.set_temp(temp)
    def cmd_SET_LIGHTS(self, gcmd):
        enabled = gcmd.get_int('ENABLED', 1)
        self.set_lights(not not enabled)
    def cmd_SET_LIGHTS_webhooks(self, web_request):
        enabled = web_request.get_int('ENABLED', 1)
        self.set_lights(not not enabled)

######################################################################
# Inova bed heater v2
######################################################################

class InovaSurfaceHeaterv2_Pin:
    def __init__(self, pin, name):
        self.next_pwm_time = 0.
        self.last_pwm_time = 0.
        self.last_pwm_value = 0.
        self.pin = pin
        self.name = name

class InovaSurfaceHeaterv2_Detector:
    def __init__(self, minx, miny, maxx, maxy, index):
        self.pins = []
        self.minx = minx
        self.miny = miny
        self.maxx = maxx
        self.maxy = maxy
        self.next_pwm_time = 0.
        self.last_pwm_time = 0.
        self.last_pwm_value = 0.
        self.neighbours = []
        self.index = index
        self.temp = 0.

class InovaSurfaceHeaterv2:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        # Setup sensor
        self.mode = config.get('mode', "max")
        self.mode_perc = config.getfloat('mode_perc', 0.5)
        self.mode_percmax = config.getfloat('mode_percmax', 5)
        self.detectors = []
        self.all_pins = []
        self.soft_delta = config.getfloat('soft_delta', 2.0, above=0.)
        self.hard_delta = config.getfloat('hard_delta', 2.0, above=0.)
        self.sensor_name = config.get('sensor')
        self.sensor = self.printer.lookup_object("mlx90640 " + self.sensor_name)
        self.min_temp = config.getfloat('min_temp', minval=KELVIN_TO_CELSIUS)
        self.max_temp = config.getfloat('max_temp', above=self.min_temp)
        self.min_pwm = config.getfloat('min_pwm')
        self.max_pwm = config.getfloat('max_pwm')
        self.factor_min_pwm = config.getfloat('factor_min_pwm', self.min_pwm)
        self.factor_pwm = config.getfloat('factor_pwm', 1.)
        self.target_reached_tolerance = config.getfloat('target_reached_tolerance', 0.)
        section_count = config.getint('count')
        self.lights_enabled = False
        self.lights_mask = 0
        if config.getboolean("factored", False):
            self.sensor.setup_callback(self.temperature_callback_factored)
        else:
            self.sensor.setup_callback(self.temperature_callback)
        self.pwm_delay = self.sensor.get_report_time_delta()
        self.can_extrude = False
        self.smooth_time = config.getfloat('smooth_time', 2., above=0.)
        self.inv_smooth_time = 1. / self.smooth_time
        self.lock = threading.Lock()
        self.last_temp = self.smoothed_temp = self.target_temp = 0.
        self.last_temp_time = 0.
        # Setup output heater pins
        ppins = self.printer.lookup_object('pins')
        self.pwm_cycle_time = config.getfloat('pwm_cycle_time', 0.02, above=0., maxval=self.pwm_delay) # defaults to 0.02 => 50Hz for dimming
        self.dimmer_enable = config.getboolean('dimmer', False)
        if self.dimmer_enable:
            dimmer_sensor_pin = config.get('dimmer_sensor_pin')
        self.all_pins_dict={}
        for i in range(0, section_count):
            dminx = config.getint('minx' + str(i), -1)
            dminy = config.getint('miny' + str(i), -1)
            dmaxx = config.getint('maxx' + str(i), -1)
            dmaxy = config.getint('maxy' + str(i), -1)
            if dminx != -1 or dminy != -1 or dmaxx != -1 or dmaxy != -1:
                detector = InovaSurfaceHeaterv2_Detector(dminx, dminy, dmaxx, dmaxy, i)
                self.detectors.append(detector)
            else:
                detector = None
            heater_pins = [x.strip() for x in config.get('heater_pins' + str(i)).split(',')]
            for pin in heater_pins:
                if pin not in self.all_pins_dict:
                    if self.dimmer_enable:
                        pwm_pin = ppins.setup_pin('dimmer', pin, dimmer_sensor_pin)
                    else:
                        pwm_pin = ppins.setup_pin('pwm', pin)
                        pwm_pin.setup_cycle_time(self.pwm_cycle_time)
                    pwm_pin.setup_max_duration(MAX_HEAT_TIME)
                    my_pin = InovaSurfaceHeaterv2_Pin(pwm_pin, pin)
                    self.all_pins_dict[pin] = my_pin
                    self.all_pins.append(my_pin)
                if detector is not None:
                    detector.pins.append(self.all_pins_dict[pin])
        self.pin_count = len(self.all_pins)
        self.lights_pwm_current = self.lights_pwm = [config.getfloat('lights_pwm', self.min_pwm)] * self.pin_count
        self.count = len(self.detectors)
        self.target_temp_reached = [False] * self.count
        for detector in self.detectors:
            for pin in detector.pins:
                for other_detector in self.detectors:
                    if other_detector == detector:
                        continue
                    if pin in other_detector.pins:
                        detector.neighbours.append(other_detector)
        self.lights_pins = []
        light_pins_names = [x.strip() for x in config.get('lights_pins').split(',')]
        if not light_pins_names:
            self.lights_pins = self.all_pins[0:2]
        else:
            for name in light_pins_names:
                self.lights_pins.append(self.all_pins_dict[name])        
        # Load additional modules
        self.printer.load_object(config, "verify_heater %s" % (self.name,))
        self.printer.load_object(config, "pid_calibrate")
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("SET_HEATER_TEMPERATURE", "HEATER",
                                   self.name, self.cmd_SET_HEATER_TEMPERATURE,
                                   desc=self.cmd_SET_HEATER_TEMPERATURE_help)
        gcode.register_mux_command("SET_LIGHTS", "HEATER",
                                   self.name, self.cmd_SET_LIGHTS)
        gcode.register_mux_command("GET_LIGHT_COUNT", "HEATER",
                                   self.name, self.cmd_GET_LIGHT_COUNT)
        wh = self.printer.lookup_object('webhooks')
        wh.register_endpoint("gcode/set_lights/" + self.name, self.cmd_SET_LIGHTS_webhooks)
        wh.register_endpoint("gcode/set_light/" + self.name, self.cmd_SET_LIGHT_webhooks)

    def set_pwm(self, read_time, value, pin):
        if self.target_temp <= 0. and not self.lights_enabled:
            value = 0.
        measured_time = self.printer.reactor.monotonic()
        estimated_print_time = pin.pin.get_mcu().estimated_print_time(measured_time) + PIN_DELAY
        self.set_pwm_inner(estimated_print_time, value, pin)
        #logging.debug("%s: pwm=%.3f@%.3f (from %.3f@%.3f [%.3f])",
        #              self.name, value, pwm_time,
        #              self.last_temp, self.last_temp_time, self.target_temp)

    def set_pwm_inner(self, read_time, value, pin):
        if ((read_time < pin.next_pwm_time or not pin.last_pwm_value)
            and abs(value - pin.last_pwm_value) <= 0.01):
            # No significant change in value - can suppress update
            return
        if (read_time - pin.last_pwm_time <= self.pwm_cycle_time):
            # too fast for possible soft_pwm
            return
        pin.next_pwm_time = read_time + MIN_HEAT_PERIOD
        pin.last_pwm_time = read_time
        pin.last_pwm_value = value
        pin.pin.set_pwm(read_time, value)

    def calc_temp(self, index):
        temp_matrix = self.sensor.temp_matrix
        values = []
        detector = self.detectors[index]
        for y in range(detector.miny, detector.maxy + 1):
            for x in range(detector.minx, detector.maxx + 1):
                v = temp_matrix[y * 32 + x]
                values.append(v)
        if self.mode == "max":
            temp = max(values)
        elif self.mode == "min":
            temp = min(values)
        elif self.mode == "avg":
            temp = sum(values) / len(values)
        elif self.mode == "perc":
            sorted_values = sorted(values)
            index = int(self.mode_perc * (len(sorted_values) - 1))
            temp = sorted_values[index]
        elif self.mode == "percmax":
            sorted_values = sorted(values)
            index = int(self.mode_perc * (len(sorted_values) - 1))
            candidate1 = sorted_values[index]
            candidate2 = sorted_values[len(sorted_values) - 1]
            if candidate2 - candidate1 < self.mode_percmax:
                temp = candidate2
            else:
                temp = candidate1
        else:
            raise error("Invalid mode")
        if temp < self.min_temp or temp > self.max_temp:
            self.printer.invoke_shutdown(
                "MLX90640 temperature %0.1f outside range of %0.1f:%.01f"
                % (temp, self.min_temp, self.max_temp))
        return temp

    def temperature_callback_factored(self, read_time, temp):
        action = None
        with self.lock:
            temps = []
            for detector in self.detectors:
                detector.temp = self.calc_temp(detector.index)
                temps.append(detector.temp)
            min_temp = min(temps)
            temp = sum(temps) / len(temps)     # override sensor temp with average over all areas

            time_diff = read_time - self.last_temp_time
            self.last_temp = temp
            self.last_temp_time = read_time
            temp_diff = temp - self.smoothed_temp
            adj_time = min(time_diff * self.inv_smooth_time, 1.)
            self.smoothed_temp += temp_diff * adj_time

            # check targets
            for i in range(0, self.count):
                if temps[i] >= self.target_temp - self.target_reached_tolerance:
                    self.target_temp_reached[i] = True

            # control outputs
            if min_temp >= self.target_temp + self.soft_delta:
                # target exceeded
                self.can_extrude = True
                action = "TargetExceeded"
            elif min_temp <= self.target_temp - self.hard_delta:
                # warming up
                self.can_extrude = False
                action = "WarmingUp"
            elif min_temp > self.target_temp:
                # slightly over, turn off the max quadrant
                self.can_extrude = True
                action = "SlowingDown"
                max_temp = max(temps)
                detector = self.detectors[temps.index(max_temp)]
                slowing_pins = detector.pins 
            else:
                # keeping warm using single pin
                self.can_extrude = True
                action = "KeepWarm"
                detector = self.detectors[temps.index(min_temp)]
                temps = []
                neighbours = []
                for other in detector.neighbours:
                    temps.append(other.temp)
                    neighbours.append(other)
                detector2 = neighbours[temps.index(min(temps))]
                warming_pwm = min(self.factor_min_pwm + (max(self.target_temp - min_temp, 0) / self.hard_delta) * self.factor_pwm * (self.max_pwm - self.factor_min_pwm), self.max_pwm)
                warming_pins = []
                for pin1 in detector.pins:
                    for pin2 in detector2.pins:
                        if pin1 == pin2:
                            warming_pins.append(pin1)

            # lights mode - override
            if self.lights_enabled:
                action = "Lights"
                lights_mask = self.lights_mask
                if lights_mask == 0:
                    lights_pwm = self.lights_pwm
                else:
                    lights_pwm = self.lights_pwm_current or self.lights_pwm

        if action == "Lights":
            if not lights_mask:
                for i in range(0, self.pin_count):
                    pin = self.all_pins[i]
                    if pin in self.lights_pins:
                        self.set_pwm(read_time, lights_pwm[i], pin)
                    else:
                        self.set_pwm(read_time, 0., pin)
            else:
                for i in range(0, self.pin_count):
                    pin = self.all_pins[i]
                    if lights_mask & (1 << i):
                        self.set_pwm(read_time, lights_pwm[i], pin)
                    else:
                        self.set_pwm(read_time, 0., pin)
        elif action == "TargetExceeded":
            for pin in self.all_pins:
                self.set_pwm(read_time, 0.0, pin)
        elif action == "WarmingUp":
            for pin in self.all_pins:
                self.set_pwm(read_time, 1.0, pin)
        elif action == "SlowingDown":
            for pin in self.all_pins:
                if pin in slowing_pins:
                    self.set_pwm(read_time, 0.0, pin)
                else:
                    self.set_pwm(read_time, self.min_pwm, pin)
        elif action == "KeepWarm":
            for pin in self.all_pins:
                if pin in warming_pins:
                    self.set_pwm(read_time, warming_pwm, pin)
                else:
                    self.set_pwm(read_time, self.min_pwm, pin)
        else:
            assert(False)

    def temperature_callback(self, read_time, temp):
        action = None
        with self.lock:
            temps = []
            for detector in self.detectors:
                detector.temp = self.calc_temp(detector.index)
                temps.append(detector.temp)
            min_temp = min(temps)
            temp = sum(temps) / len(temps)     # override sensor temp with average over all areas

            time_diff = read_time - self.last_temp_time
            self.last_temp = temp
            self.last_temp_time = read_time
            temp_diff = temp - self.smoothed_temp
            adj_time = min(time_diff * self.inv_smooth_time, 1.)
            self.smoothed_temp += temp_diff * adj_time

            # check targets
            for i in range(0, self.count):
                if temps[i] >= self.target_temp:
                    self.target_temp_reached[i] = True

            # control outputs
            if min_temp >= self.target_temp + self.soft_delta:
                # target exceeded
                self.can_extrude = True
                action = "TargetExceeded"
            elif min_temp <= self.target_temp - self.hard_delta:
                # warming up
                self.can_extrude = False
                action = "WarmingUp"
            else:
                # keeping warm using single pin
                self.can_extrude = True
                action = "KeepWarm"
                detector = self.detectors[temps.index(min_temp)]
                temps = []
                neighbours = []
                for other in detector.neighbours:
                    temps.append(other.temp)
                    neighbours.append(other)
                detector2 = neighbours[temps.index(min(temps))]
                warming_pins = []
                for pin1 in detector.pins:
                    for pin2 in detector2.pins:
                        if pin1 == pin2:
                            warming_pins.append(pin1)

            # lights mode - override
            if self.lights_enabled:
                action = "Lights"
                lights_mask = self.lights_mask
                if lights_mask == 0:
                    lights_pwm = self.lights_pwm
                else:
                    lights_pwm = self.lights_pwm_current or self.lights_pwm

        if action == "Lights":
            if not lights_mask:
                for i in range(0, self.pin_count):
                    pin = self.all_pins[i]
                    if pin in self.lights_pins:
                        self.set_pwm(read_time, lights_pwm[i], pin)
                    else:
                        self.set_pwm(read_time, 0., pin)
            else:
                for i in range(0, self.pin_count):
                    pin = self.all_pins[i]
                    if lights_mask & (1 << i):
                        self.set_pwm(read_time, lights_pwm[i], pin)
                    else:
                        self.set_pwm(read_time, 0., pin)
        elif action == "TargetExceeded":
            for pin in self.all_pins:
                self.set_pwm(read_time, 0.0, pin)
        elif action == "WarmingUp":
            for pin in self.all_pins:
                self.set_pwm(read_time, 1.0, pin)
        elif action == "KeepWarm":
            for pin in self.all_pins:
                if pin in warming_pins:
                    self.set_pwm(read_time, self.max_pwm, pin)
                else:
                    self.set_pwm(read_time, self.min_pwm, pin)
        else:
            assert(False)
            
    # External commands
    def get_pwm_delay(self):
        return self.pwm_delay
    def get_max_power(self):
        return self.max_power
    def get_smooth_time(self):
        return self.smooth_time
    def set_temp(self, degrees):
        if degrees and (degrees < self.min_temp or degrees > self.max_temp):
            raise self.printer.command_error(
                "Requested temperature (%.1f) out of range (%.1f:%.1f)"
                % (degrees, self.min_temp, self.max_temp))
        with self.lock:
            self.lights_on = False
            self.target_temp = degrees
            self.target_temp_reached = [False] * self.count
    def set_light(self, enabled, index, pwm = 0.):
        with self.lock:
            if enabled:
                self.lights_mask = self.lights_mask | (1 << index)
            else:
                self.lights_mask = self.lights_mask & ~(1 << index)
            self.lights_pwm_current[index] = pwm
            self.lights_enabled = self.lights_mask != 0
    def set_lights(self, enabled, mask = 0, pwm = 0.):
        with self.lock:
            self.lights_mask = mask
            self.lights_pwm_current = [ pwm ] * self.pin_count
            self.lights_enabled = enabled
    def get_temp(self, eventtime):
        print_time = self.all_pins[0].pin.get_mcu().estimated_print_time(eventtime) - 5.
        with self.lock:
            if self.last_temp_time < print_time:
                return 0., self.target_temp
            return self.smoothed_temp, self.target_temp
    def check_busy(self, eventtime):
        return False
    def set_control(self, control):
        return None
    def alter_target(self, target_temp):
        if target_temp:
            target_temp = max(self.min_temp, min(self.max_temp, target_temp))
        self.target_temp = target_temp
    def stats(self, eventtime):
        with self.lock:
            target_temp = self.target_temp
            last_temp = self.last_temp
        is_active = target_temp or last_temp > 50.
        return is_active, '%s: target=%.0f temp=%.1f' % (
            self.name, target_temp, last_temp)
    def get_status(self, eventtime):
        with self.lock:
            target_temp = self.target_temp
            smoothed_temp = self.smoothed_temp
            target_reached = all(self.target_temp_reached)
        return {'temperature': smoothed_temp, 'target': target_temp, 'target_reached': target_reached}
    cmd_SET_HEATER_TEMPERATURE_help = "Sets a heater temperature"
    def cmd_SET_HEATER_TEMPERATURE(self, gcmd):
        temp = gcmd.get_float('TARGET', 0.)
        self.set_temp(temp)
    def cmd_GET_LIGHT_COUNT(self, gcmd):
        msg = str(self.pin_count)
        did_ack = gcmd.ack(msg)
        if not did_ack:
            gcmd.respond_raw(msg)
    def cmd_SET_LIGHTS(self, gcmd):
        enabled = gcmd.get_int('ENABLED', 1)
        mask = gcmd.get_int('MASK', 0, minval=0)
        pwm = gcmd.get_float('POWER', 0., minval=0., maxval=1.)
        self.set_lights(not not enabled, mask, pwm)
    def cmd_SET_LIGHTS_webhooks(self, web_request):
        enabled = web_request.get_int('ENABLED', 1)
        mask = web_request.get_int('MASK', 0)
        if mask < 0:
            web_request.set_error("Invalid mask")
            return
        pwm = web_request.get_float('POWER', 0.)
        if pwm < 0 or pwm > 1:
            web_request.set_error("Invalid power")
            return
        self.set_lights(not not enabled, mask, pwm)
    def cmd_SET_LIGHT_webhooks(self, web_request):
        count = web_request.get_int('COUNT')
        for i in range(0, count):
            index = web_request.get_int('INDEX' + str(i))
            enabled = web_request.get_int('ENABLED' + str(i), 1)
            pwm = web_request.get_float('POWER' + str(i), 0.)
            self.set_light(not not enabled, index, pwm)

######################################################################
# Sensor and heater lookup
######################################################################

class PrinterHeaters:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.sensor_factories = {}
        self.heaters = {}
        self.gcode_id_to_sensor = {}
        self.available_heaters = []
        self.available_sensors = []
        self.has_started = False
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler("gcode:request_restart",
                                            self.turn_off_all_heaters)
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("TURN_OFF_HEATERS", self.cmd_TURN_OFF_HEATERS,
                               desc=self.cmd_TURN_OFF_HEATERS_help)
        gcode.register_command("M105", self.cmd_M105, when_not_ready=True)
    def add_sensor_factory(self, sensor_type, sensor_factory):
        self.sensor_factories[sensor_type] = sensor_factory
    def setup_heater(self, config, gcode_id=None):
        heater_name = config.get_name().split()[-1]
        if heater_name in self.heaters:
            raise config.error("Heater %s already registered" % (heater_name,))
        # Setup sensor
        sensor = self.setup_sensor(config)
        # Create heater
        self.heaters[heater_name] = heater = Heater(config, sensor)
        self.register_sensor(config, heater, gcode_id)
        self.available_heaters.append(config.get_name())
        return heater
    def setup_inova_surface_heater(self, config, gcode_id=None):
        heater_name = config.get_name().split()[-1]
        if heater_name in self.heaters:
            raise config.error("Heater %s already registered" % (heater_name,))
        # Create heater
        self.heaters[heater_name] = heater = InovaSurfaceHeater(config)
        self.register_sensor(config, heater, gcode_id)
        self.available_heaters.append(config.get_name())
        return heater
    def setup_inova_surface_heaterv2(self, config, gcode_id=None):
        heater_name = config.get_name().split()[-1]
        if heater_name in self.heaters:
            raise config.error("Heater %s already registered" % (heater_name,))
        # Create heater
        self.heaters[heater_name] = heater = InovaSurfaceHeaterv2(config)
        self.register_sensor(config, heater, gcode_id)
        self.available_heaters.append(config.get_name())
        return heater
    def get_all_heaters(self):
        return self.available_heaters
    def lookup_heater(self, heater_name):
        if heater_name not in self.heaters:
            raise self.printer.config_error(
                "Unknown heater '%s'" % (heater_name,))
        return self.heaters[heater_name]
    def setup_sensor(self, config):
        modules = ["thermistor", "adc_temperature", "spi_temperature",
                   "bme280", "htu21d", "lm75", "mlx90614"]
        for module_name in modules:
            self.printer.load_object(config, module_name)
        sensor_type = config.get('sensor_type')
        if sensor_type not in self.sensor_factories:
            raise self.printer.config_error(
                "Unknown temperature sensor '%s'" % (sensor_type,))
        return self.sensor_factories[sensor_type](config)
    def register_sensor(self, config, psensor, gcode_id=None):
        if gcode_id is None:
            gcode_id = config.get('gcode_id', None)
            if gcode_id is None:
                return
        if gcode_id in self.gcode_id_to_sensor:
            raise self.printer.config_error(
                "G-Code sensor id %s already registered" % (gcode_id,))
        self.gcode_id_to_sensor[gcode_id] = psensor
        self.available_sensors.append(config.get_name())
    def get_status(self, eventtime):
        return {'available_heaters': self.available_heaters,
                'available_sensors': self.available_sensors}
    def turn_off_all_heaters(self, print_time=0.):
        for heater in self.heaters.values():
            heater.set_temp(0.)
    cmd_TURN_OFF_HEATERS_help = "Turn off all heaters"
    def cmd_TURN_OFF_HEATERS(self, gcmd):
        self.turn_off_all_heaters()
    # G-Code M105 temperature reporting
    def _handle_ready(self):
        self.has_started = True
    def _get_temp(self, eventtime):
        # Tn:XXX /YYY B:XXX /YYY
        out = []
        if self.has_started:
            for gcode_id, sensor in sorted(self.gcode_id_to_sensor.items()):
                cur, target = sensor.get_temp(eventtime)
                out.append("%s:%.1f /%.1f" % (gcode_id, cur, target))
        if not out:
            return "T:0"
        return " ".join(out)
    def cmd_M105(self, gcmd):
        # Get Extruder Temperature
        reactor = self.printer.get_reactor()
        msg = self._get_temp(reactor.monotonic())
        did_ack = gcmd.ack(msg)
        if not did_ack:
            gcmd.respond_raw(msg)
    def wait_for_temperature(self, heater):
        # Helper to wait on heater.check_busy() and report M105 temperatures
        if self.printer.get_start_args().get('debugoutput') is not None:
            return
        toolhead = self.printer.lookup_object("toolhead")
        gcode = self.printer.lookup_object("gcode")
        reactor = self.printer.get_reactor()
        eventtime = reactor.monotonic()
        while not self.printer.is_shutdown() and heater.check_busy(eventtime):
            print_time = toolhead.get_last_move_time()
            gcode.respond_raw(self._get_temp(eventtime))
            eventtime = reactor.pause(eventtime + 1.)

def load_config(config):
    return PrinterHeaters(config)
