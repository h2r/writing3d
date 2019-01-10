# Defines parameters for various kinds of pens.
from pprint import pprint

# Generic pen
class Pen:
    CONFIG = {
        "RESOLUTION": 0.0002,
        "Z_RESOLUTION": 0.0002,
        "Z_MIN": -0.05,  # This depends on the size of the pen
        "Z_MAX": 0.05,
        "Z_LIFT": 0.05,
    }

    @classmethod
    def param(cls, name):
        return cls.CONFIG.get(name, None)
    
    @classmethod
    def get_config(cls):
        return cls.CONFIG

    @classmethod
    def print_config(cls):
        print("%s:" % cls.__name__)
        pprint(cls.CONFIG)

class BrushSmall(Pen):
    CONFIG = {
        "RESOLUTION": 0.0002,
        "Z_RESOLUTION": 0.0002,
        "Z_MIN": -0.03,  # This depends on the size of the pen
        "Z_MAX": 0.03,
        "Z_LIFT": 0.03,    
    }

class Sharpe:
    CONFIG = {
        "RESOLUTION": 0.0002,
        "Z_LIFT": 0.03,
    }

