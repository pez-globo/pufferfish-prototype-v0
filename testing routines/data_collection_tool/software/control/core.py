# set QT_API environment variable
import os 
os.environ["QT_API"] = "pyqt5"
import qtpy

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *

import control.utils as utils
from control._def import *

from queue import Queue
import time
import numpy as np
import pyqtgraph as pg
from datetime import datetime
from pathlib import Path

class Waveforms(QObject):

    signal_Paw = Signal(float,float)
    signal_Flow1 = Signal(float,float)

    signal_flow_1 = Signal(str)
    signal_p_airway = Signal(str)

    def __init__(self,microcontroller):
        QObject.__init__(self)
        self.file = open(str(Path.home()) + "/Downloads/" + datetime.now().strftime('%Y-%m-%d %H-%M-%-S.%f') + ".csv", "w+")
        # self.file.write('Time (s),Paw (cmH2O),Flow (l/min),Volume (ml),Vt (ml),Ti (s),RR (/min),PEEP (cmH2O)\n')
        self.microcontroller = microcontroller
        self.pressure_aw_cmH2O = 0
        self.flow_1 = 0
        self.time = 0
        self.timer_update_waveform = QTimer()
        self.timer_update_waveform.setInterval(MCU.DATA_INTERVAL_ms/2)
        self.timer_update_waveform.timeout.connect(self.update_waveforms)
        self.timer_update_waveform.start()

        self.first_run = True
        self.time_ticks_start = 0

        self.time_now = 0
        self.time_diff = 0
        self.time_prev = time.time()

        self.counter_display = 0
        self.counter_file_flush = 0

        self.logging_is_on = True

    def logging_onoff(self,state,experimentID):
        self.logging_is_on = state
        if state == False:
            self.file.close()
        else:
            self.experimentID = experimentID
            self.file = open(str(Path.home()) + "/Downloads/" + self.experimentID + '_' + datetime.now().strftime('%Y-%m-%d %H-%M-%-S.%f') + ".csv", "w+")

    def update_waveforms(self):
        # self.time = self.time + (1/1000)*WAVEFORMS.UPDATE_INTERVAL_MS
      
        if SIMULATION:
            # test plotting multiple data points at a time
            #for i in range(MCU.TIMEPOINT_PER_UPDATE):
            for i in range(MCU.TIMEPOINT_PER_UPDATE):
                # Use the processor clock to determine elapsed time since last function call
                self.time_now = time.time()
                self.time_diff = self.time_now - self.time_prev
                self.time_prev = self.time_now
                self.time += self.time_diff
                self.flow_1 = (self.flow_1 + 0.2/MCU.TIMEPOINT_PER_UPDATE)%5
                self.pressure_aw_cmH2O = (self.pressure_aw_cmH2O + 0.1//MCU.TIMEPOINT_PER_UPDATE)%5

            self.signal_Flow1.emit(self.time,self.flow_1)
            self.signal_Paw.emit(self.time,self.pressure_aw_cmH2O)
            # self.signal_Paw.emit(self.time,self.flow_3)

            self.signal_flow_1.emit("{:.2f}".format(self.flow_1))
            self.signal_p_airway.emit("{:.2f}".format(self.pressure_aw_cmH2O)) 
            # self.signal_p_airway.emit("{:.2f}".format(self.flow_3))           

        else:
            readout = self.microcontroller.read_received_packet_nowait()
            if readout is not None:

                # self.time_now = time.time()
                # self.time_diff = self.time_now - self.time_prev
                # self.time_prev = self.time_now
                # self.time += self.time_diff
                self.time_now = time.time()

                for i in range(MCU.TIMEPOINT_PER_UPDATE):
                    # time
                    self.time_ticks = int.from_bytes(readout[i*MCU.RECORD_LENGTH_BYTE:i*MCU.RECORD_LENGTH_BYTE+4], byteorder='big', signed=False)
                    if self.first_run:
                        self.time_ticks_start = self.time_ticks
                        self.first_run = False
                    self.time = (self.time_ticks - self.time_ticks_start)*MCU.TIMER_PERIOD_ms/1000
                    self.flow_1 = utils.unsigned_to_signed(readout[i*MCU.RECORD_LENGTH_BYTE+4:i*MCU.RECORD_LENGTH_BYTE+6],2)/(65536/2)*MCU.flow_FS
                    self.pressure_aw_cmH2O = utils.unsigned_to_signed(readout[i*MCU.RECORD_LENGTH_BYTE+6:i*MCU.RECORD_LENGTH_BYTE+8],2)/(65536/2)*MCU.paw_FS

                    record_from_MCU = (
                        str(self.time_ticks) + '\t' + str(self.flow_1) + '\t' + "{:.2f}".format(self.pressure_aw_cmH2O) )
                    record_settings = (str(self.time_now))
                   
                    # saved variables
                    if self.logging_is_on:
                        self.file.write(record_from_MCU + '\t' + record_settings + '\n')

                # reduce display refresh rate
                self.counter_display = self.counter_display + 1
                if self.counter_display>=1:
                    self.counter_display = 0
                    self.signal_Flow1.emit(self.time,self.flow_1)
                    self.signal_Paw.emit(self.time,self.pressure_aw_cmH2O)

                    self.signal_flow_1.emit("{:.2f}".format(self.flow_1))
                    self.signal_p_airway.emit("{:.2f}".format(self.pressure_aw_cmH2O))

        # file flushing
        if self.logging_is_on:
            self.counter_file_flush = self.counter_file_flush + 1
            if self.counter_file_flush>=500:
                self.counter_file_flush = 0
                self.file.flush()

    def close(self):
        self.file.close()