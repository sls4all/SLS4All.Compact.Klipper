import logging
import crc8
from . import bus

MLX90614_CHIP_ADDR = 0x5A
MLX90614_I2C_SPEED = 100000
MLX90614_OBJECT1_REG = 7
#     'AMBIENT'   : 6,
#     'OBJECT1'   : 7,
#     'OBJECT2'   : 8,
#     'FLAGS'     : 0xf0,
#     'SLEEP'     : 0xff
# }
MLX90614_ERROR_TEMP = 1000
MLX90614_REPORT_TIME = 1.0
MLX90614_MIN_REPORT_TIME = 0.1

class MLX90614:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.i2c = bus.MCU_I2C_from_config(
            config,
            default_addr=MLX90614_CHIP_ADDR,
            default_speed=MLX90614_I2C_SPEED
        )
        self.mcu = self.i2c.get_mcu()
        self.report_time = config.getfloat(
            'mlx90614_report_time',
            MLX90614_REPORT_TIME,
            minval=MLX90614_MIN_REPORT_TIME
        )
        self.temp = 0.0
        self.sample_timer = self.reactor.register_timer(self._sample_mlx90614)
        self.printer.add_object("MLX90614 " + self.name, self)
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

    def handle_ready(self):
        self._init_mlx90614()
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def setup_minmax(self, min_temp, max_temp):
        pass

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return self.report_time

    def _init_mlx90614(self):
        pass

    def _sample_mlx90614(self, eventtime):
        try:
            data = self.read_register(MLX90614_OBJECT1_REG, 3)
            crc = crc8.crc8()
            crc.update("".join(map(chr, [
                MLX90614_CHIP_ADDR << 1,
                MLX90614_OBJECT1_REG,
                (MLX90614_CHIP_ADDR << 1) + 1,
                data[0],
                data[1],
                data[2]
            ])))
            if crc.digest() == b'\x00':
                word = data[0] + (data[1] << 8)
                if (word & 0x8000) == 0:
                    self.temp = ((word << 1) - 27315) / 100.0
    
        except Exception:
            logging.exception("MLX90614: Error reading data")
            self.temp = 0.0
            return self.reactor.NEVER

        measured_time = self.reactor.monotonic()
        self._callback(self.mcu.estimated_print_time(measured_time), self.temp)
        return measured_time + self.report_time

    def read_register(self, reg, read_len):
        params = self.i2c.i2c_read([reg], read_len)
        return bytearray(params['response'])

    def write_register(self, reg_name, data):
        if type(data) is not list:
            data = [data]
        reg = MLX90614_REGS[reg_name]
        data.insert(0, reg)
        self.i2c.i2c_write(data)

    def get_status(self, eventtime):
        return {
            'temperature': self.temp,
        }


def load_config(config):
    # Register sensor
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory("MLX90614", MLX90614)
