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
    STEPS_PER_MM_XY = 1 # microsteps
    STEPS_PER_MM_Z = 1  # microsteps
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

class MCU:
    
    CMD_LENGTH = 4
    N_BYTES_DATA = 2

    # full scale
    flow_FS = 200
    paw_FS = 100

    TIMER_PERIOD_ms = 1
    TIMEPOINT_PER_UPDATE = 50
    DATA_INTERVAL_ms = TIMER_PERIOD_ms*TIMEPOINT_PER_UPDATE
    RECORD_LENGTH_BYTE = 8
    MSG_LENGTH = TIMEPOINT_PER_UPDATE*RECORD_LENGTH_BYTE

class WAVEFORMS:
    # UPDATE_INTERVAL_MS = 25 # make sure this equals MCU.DATA_INTERVAL_ms when MCU is connected and MCU.DATA_INTERVAL_ms/2 when in simulation
    DISPLAY_RANGE_S = 10 # In seconds
    CYCLE_GAP = 5 # In sample num
    PAW_MIN = 0
    PAW_MAX = 30
    FLOW_MIN = -150
    FLOW_MAX = 150
    V_MIN = 0
    V_MAX = 1000

PLOTS = ['Flow 1', 'P']
# PLOT_VARIABLES = {'Airway Pressure':'P_aw', 'Flow Rate':'Flow_rate', 'Volume':'Volume'}
PLOT_UNITS = {'Flow 1':'L/min', 'P':'cmH2O'}

SIMULATION = False    


if SIMULATION:
    WAVEFORMS.UPDATE_INTERVAL_MS = MCU.DATA_INTERVAL_ms/2
else:
    WAVEFORMS.UPDATE_INTERVAL_MS = MCU.DATA_INTERVAL_ms

