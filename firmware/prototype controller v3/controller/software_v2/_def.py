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
    BASE = 0.001
    X = 0.004     # per mm
    Y = 0.004	 # per mm
    Z = 0.002     # per mm
    def __init__(self):
        pass

class AF:
    STOP_THRESHOLD = 0.85
    CROP_WIDTH = 500
    CROP_HEIGHT = 500
    def __init__(self):
        pass

class Motion:
    STEPS_PER_MM_XY = 1600 # microsteps
    STEPS_PER_MM_Z = 5333  # microsteps
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

class MicrocontrollerDef:
    MSG_LENGTH = 8
    CMD_LENGTH = 3
    N_BYTES_DATA = 2
    FLOW_FS = 200.0
    VOLUME_FS = 1500.0
    PAW_FS = 50.0
    TIMER_PERIOD_ms = 0.5
    VT_FS = 1500.0
    PEEP_FS = 30.0
    TI_FS = 5.0
    RR_FS = 60.0
    
    VALVE_POS_OPEN_STEPS_FS = 125;
    PC_RISE_TIME_MS_FS = 500;
    PID_COEFFICIENT_P_FS = 0.1;
    PID_COEFFICIENT_I_FRAC_FS = 1;
    
    CMD_Vt = 0
    CMD_Ti = 1
    CMD_RR = 2
    CMD_PEEP = 3
    CMD_Flow = 4
    CMD_Pinsp = 5
    CMD_RiseTime = 6
    CMD_PID_P = 7
    CMD_PID_I_frac = 8

    CMD_FlowDeceleratingSlope = 5

    Ti_DEFAULT = 1.2
    RR_DEFAULT = 18
    PEEP_DEFAULT = 5
    Vt_DEFAULT = 300


class WAVEFORMS:
    UPDATE_INTERVAL_MS = 25 # In milliseconds
    DISPLAY_RANGE_S = 10 # In seconds
    DISPLAY_DOWNSAMPLING_FACTOR = 20

PLOTS = ['Airway Pressure', 'Flow Rate', 'Volume']
PLOT_VARIABLES = {'Airway Pressure':'P_aw', 'Flow Rate':'Flow_rate', 'Volume':'Volume'}
PLOT_UNITS = {'Airway Pressure':'cmH20', 'Flow Rate':'L/min', 'Volume':'mL'}

SIMULATION = True

class MCU:
    TIMER_PERIOD_ms = 1.25