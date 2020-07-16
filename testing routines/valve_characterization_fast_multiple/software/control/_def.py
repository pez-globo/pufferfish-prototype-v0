class TriggerMode:
    SOFTWARE = 'Software Trigger'
    HARDWARE = 'Hardware Trigger'
    CONTINUOUS = 'Continuous Acqusition'
    def __init__(self):
        pass

class MicroscopeMode:
    BFDF = 'BF/DF'
    FLUORESCENCE = 'Fluorescence'
    FLUORESCENCE_PREVIEW = 'Fluorescence Preview'
    def __init__(self):
        pass

class WaitTime:
    BASE = 0.0
    X = 0.0     # per mm
    Y = 0.0	 # per mm
    Z = 0.0     # per mm
    def __init__(self):
        pass

class AF:
    STOP_THRESHOLD = 0.85
    CROP_WIDTH = 500
    CROP_HEIGHT = 500
    def __init__(self):
        pass

class Motion:
    MICROSTEPS_X = 2
    MICROSTEPS_Y = 2
    MICROSTEPS_Z = 2

    STEPS_PER_MM = 78.74*MICROSTEPS_Y # change the unit from per mm to per step (NMB)
    # STEPS_PER_MM_XY = 19.68*MICROSTEPS_Y # change the unit from per mm to per step (Dings linear actuator (J))

    # STEPS_PER_MM_Z = 82.02*MICROSTEPS_Z  # change the unit from per mm to per step

    def __init__(self):
        pass
'''
# for octopi-malaria
class Motion:
    STEPS_PER_MM_XY = 40
    STEPS_PER_MM_Z = 5333
    def __init__(self):
        pass
'''

class Acquisition:
    CROP_WIDTH = 2200
    CROP_HEIGHT = 2200
    NUMBER_OF_FOVS_PER_AF = 3
    IMAGE_FORMAT = 'bmp'
    IMAGE_DISPLAY_SCALING_FACTOR = 0.25
    DX = 1
    DY = 1
    DZ = 3

    def __init__(self):
        pass

class PosUpdate:
    INTERVAL_MS = 25



class Force:

    DRIVE_VOLTAGE = 3.3     
    VOLT_MIN_LOAD = 0.33    # Volts
    VOLT_MAX_LOAD =2.97     # Volts
    ADC_RESOLUTION = 1023
    MAX_RATED_LOAD = 45.3   # Newtons




FLOW_FS = 200
PRESSURE_FS = 60
# Encoder Counter per mm (1um per count RLS miniature linear encoder)
ENCODER_COUNTS_PER_MM = 500

class MicrocontrollerDef:
    N_VALVES = 4
    N_BYTES_PER_RECORD = 16
    N_RECORDS_PER_MSG = 60
    MSG_LENGTH = N_RECORDS_PER_MSG*N_BYTES_PER_RECORD 
    FIELD_NUM = 7
    CMD_LENGTH = 4
    N_BYTES_POS = 3
    CYCLES_PER_VALVE = 100      # Default value of timer cycles per valve.

    N_VALVES_LIST = [str(ii+1) for ii in range(N_VALVES)]

    DEFAULT_ACTIVE_VALVE = 0


