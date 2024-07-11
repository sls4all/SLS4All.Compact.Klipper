# Inova surface heater v2
#
# Copyright (C) 2021-2022  Pavel Dyntera (yourself@yourself.cz)

def load_config_prefix(config):
    pheaters = config.get_printer().load_object(config, 'heaters')
    return pheaters.setup_inova_surface_heaterv2(config)
