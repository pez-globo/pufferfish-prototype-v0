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

class ValveController(QObject):

    xPos = Signal(float)
    yPos = Signal(float)

    def __init__(self,microcontroller):
        QObject.__init__(self)
        self.microcontroller = microcontroller
        self.x_pos = 0
        self.y_pos = 0

    def move_x(self,delta):
        self.microcontroller.move_x(delta)
        self.x_pos = self.x_pos + delta
        self.xPos.emit(self.x_pos)
        QApplication.processEvents()
        #print(self.x_pos)

    def move_y(self,delta):
        self.microcontroller.move_y(delta)
        self.y_pos = self.y_pos + delta
        self.yPos.emit(self.y_pos)
        QApplication.processEvents()

    def open_valve_1(self):
        self.microcontroller.toggle_valve_1(1)

    def open_valve_2(self):
        self.microcontroller.toggle_valve_2(1)

    def close_valve_1(self):
        self.microcontroller.toggle_valve_1(0)

    def close_valve_2(self):
        self.microcontroller.toggle_valve_2(0)

class VentController(QObject):
    def __init__(self,microcontroller):
        QObject.__init__(self)
        self.microcontroller = microcontroller
        self.Vt = MicrocontrollerDef.Vt_DEFAULT
        self.Ti = MicrocontrollerDef.Ti_DEFAULT
        self.RR = MicrocontrollerDef.RR_DEFAULT
        self.PEEP = MicrocontrollerDef.PEEP_DEFAULT
        self.mode = MODE_PC_AC

    def setVT(self,value):
        self.microcontroller.set_parameter(MicrocontrollerDef.CMD_Vt,value/MicrocontrollerDef.VT_FS)
        self.Vt = value

    def setTi(self,value):
        self.microcontroller.set_parameter(MicrocontrollerDef.CMD_Ti,value/MicrocontrollerDef.TI_FS)
        self.Ti = value

    def setRR(self,value):
        self.microcontroller.set_parameter(MicrocontrollerDef.CMD_RR,value/MicrocontrollerDef.RR_FS)
        self.RR = value

    def setPEEP(self,value):
        self.microcontroller.set_parameter(MicrocontrollerDef.CMD_PEEP,value/MicrocontrollerDef.PEEP_FS)
        self.PEEP = value

    def setFlow(self,value):
        self.microcontroller.set_parameter(MicrocontrollerDef.CMD_Flow,value/MicrocontrollerDef.VALVE_POS_OPEN_STEPS_FS)

    # def setFlowDeceleratingSlope(self,value):
    #     self.microcontroller.set_parameter(MicrocontrollerDef.CMD_FlowDeceleratingSlope,value/100)

    def setPinsp(self,value):
        print('setting P_insp')
        self.microcontroller.set_parameter(MicrocontrollerDef.CMD_Pinsp,value/MicrocontrollerDef.PAW_FS)

    def setRiseTime(self,value):
        self.microcontroller.set_parameter(MicrocontrollerDef.CMD_RiseTime,value/MicrocontrollerDef.PC_RISE_TIME_MS_FS)

    def setPID_P(self,value):
        self.microcontroller.set_parameter(MicrocontrollerDef.CMD_PID_P,value/MicrocontrollerDef.PID_COEFFICIENT_P_FS)

    def setPID_I_frac(self,value):
        self.microcontroller.set_parameter(MicrocontrollerDef.CMD_PID_I_frac,value/MicrocontrollerDef.PID_COEFFICIENT_I_FRAC_FS)
    
    def setONOFF(self,state):
        if state == False:
            self.microcontroller.set_mode(MicrocontrollerDef.CMD_MODE,0) # to add mode selection
            print('stop breathing')
        else:
            #TODO: add mode selection
            self.microcontroller.set_mode(MicrocontrollerDef.CMD_MODE,self.mode) 

    def updateMode(self,mode):
        print('update mode')
        if mode == MODE_PC_AC_STRING:
            self.mode = MODE_PC_AC
            # self.microcontroller.set_mode(MicrocontrollerDef.CMD_MODE,self.mode) 
        if mode == MODE_VC_AC_STRING:
            self.mode = MODE_VC_AC
            # self.microcontroller.set_mode(MicrocontrollerDef.CMD_MODE,self.mode) 
        if mode == MODE_PSV_STRING:
            self.mode = MODE_PSV
            # self.microcontroller.set_mode(MicrocontrollerDef.CMD_MODE,self.mode) 

# class DataLogger(QObject):
#     def __init__(self,microcontroller):
#         QObject.__init__(self)
#         self.file = open("~/Downloads/" + datetime.now().strftime('%Y-%m-%d %H-%M-%-S.%f') + ".csv", "w")

#     def log_data(self,time,paw,flow,volume):
#         self.file.write(str(time)+','+str(paw)+','+str(flow)+','+str(volume))

#     def close(self):
#         self.file.close()

class Waveforms(QObject):

    signal_Paw = Signal(float,float)
    signal_Volume = Signal(float,float)
    signal_Flow = Signal(float,float)

    def __init__(self,microcontroller,ventController):
        QObject.__init__(self)
        self.file = open(str(Path.home()) + "/Downloads/" + datetime.now().strftime('%Y-%m-%d %H-%M-%-S.%f') + ".csv", "w+")
        self.file.write('Time (s),Paw (cmH2O),Flow (l/min),Volume (ml),Vt (ml),Ti (s),RR (/min),PEEP (cmH2O)\n')
        self.microcontroller = microcontroller
        self.ventController = ventController
        self.Paw = 0
        self.Volume = 0
        self.Flow = 0
        self.time = 0
        self.timer_update_waveform = QTimer()
        self.timer_update_waveform.setInterval(MCU.DATA_INTERVAL_ms/2)
        self.timer_update_waveform.timeout.connect(self.update_waveforms)
        self.timer_update_waveform.start()

        self.time_now = 0
        self.time_diff = 0
        self.time_prev = time.time()

        self.counter_display = 0
        self.counter_file_flush = 0

    def update_waveforms(self):
        # self.time = self.time + (1/1000)*WAVEFORMS.UPDATE_INTERVAL_MS
      
        if SIMULATION:
            # test plotting multiple data points at a time
            for i in range(MCU.TIMEPOINT_PER_UPDATE):
                # Use the processor clock to determine elapsed time since last function call
                self.time_now = time.time()
                self.time_diff = self.time_now - self.time_prev
                self.time_prev = self.time_now
                self.time += self.time_diff
                self.Paw = (self.Paw + 0.2/MCU.TIMEPOINT_PER_UPDATE)%5
                self.Volume = (self.Volume + 0.2/MCU.TIMEPOINT_PER_UPDATE)%5
                self.Flow = (self.Flow + 0.2/MCU.TIMEPOINT_PER_UPDATE)%5
                # self.file.write(str(self.time_now)+','+str(self.Paw)+','+str(self.Flow)+','+str(self.Volume)+'\n')
                self.signal_Paw.emit(self.time,self.Paw)
                self.signal_Flow.emit(self.time,self.Flow)
                self.signal_Volume.emit(self.time,self.Volume)

        else:
            readout = self.microcontroller.read_received_packet_nowait()
            if readout is not None:
                self.time_now = time.time()
                self.time_diff = self.time_now - self.time_prev
                self.time_prev = self.time_now
                self.time += self.time_diff
                self.Paw = (utils.unsigned_to_signed(readout[0:2],MicrocontrollerDef.N_BYTES_DATA)/(65536/2))*MicrocontrollerDef.PAW_FS 
                self.Flow = (utils.unsigned_to_signed(readout[2:4],MicrocontrollerDef.N_BYTES_DATA)/(65536/2))*MicrocontrollerDef.FLOW_FS
                self.Volume = (utils.unsigned_to_unsigned(readout[4:6],MicrocontrollerDef.N_BYTES_DATA)/65536)*MicrocontrollerDef.VOLUME_FS
                # self.time = float(utils.unsigned_to_unsigned(readout[6:8],MicrocontrollerDef.N_BYTES_DATA))*MicrocontrollerDef.TIMER_PERIOD_ms/1000
                self.file.write(str(self.time_now)+','+"{:.2f}".format(self.Paw)+','+"{:.2f}".format(self.Flow)+','+"{:.2f}".format(self.Volume)+','+str(self.ventController.Vt)+','+str(self.ventController.Ti)+','+str(self.ventController.RR)+','+str(self.ventController.PEEP) +'\n')
                print("{:.2f}".format(self.Paw)+'\t'+"{:.2f}".format(self.Flow)+'\t'+"{:.2f}".format(self.Volume))

            # reduce display refresh rate
            self.counter_display = self.counter_display + 1
            if self.counter_display>=1:
                self.counter_display = 0
                self.signal_Paw.emit(self.time,self.Paw)
                self.signal_Flow.emit(self.time,self.Flow)
                self.signal_Volume.emit(self.time,self.Volume)

        # file flushing
        self.counter_file_flush = self.counter_file_flush + 1
        if self.counter_file_flush>=500:
            self.counter_file_flush = 0
            self.file.flush()

    def close(self):
        self.file.close()

class NavigationController(QObject):

    xPos = Signal(float)
    yPos = Signal(float)
    zPos = Signal(float)

    def __init__(self,microcontroller):
        QObject.__init__(self)
        self.microcontroller = microcontroller
        self.x_pos = 0
        self.y_pos = 0
        self.z_pos = 0
        
    def move_x(self,delta):
        self.microcontroller.move_x(delta)
        self.x_pos = self.x_pos + delta
        print('X: ' + str(self.x_pos))
        self.xPos.emit(self.x_pos)

    def move_y(self,delta):
        self.microcontroller.move_y(delta)
        self.y_pos = self.y_pos + delta
        print('Y: ' + str(self.y_pos))
        self.yPos.emit(self.y_pos)

    def move_z(self,delta):
        self.microcontroller.move_z(delta)
        self.z_pos = self.z_pos + delta
        print('Z: ' + str(self.z_pos))
        self.zPos.emit(self.z_pos)

    def close_x(self):
        self.microcontroller.close_x()
        self.x_pos = 0
        self.xPos.emit(self.x_pos)

    def close_y(self):
        self.microcontroller.close_y()
        self.y_pos = 0
        self.yPos.emit(self.y_pos)

    def close_z(self):
        self.microcontroller.close_z()
        self.z_pos = 0
        self.zPos.emit(self.z_pos)

    def setbias_x(self):
        self.microcontroller.setbias_x()
        self.x_pos = 0
        self.xPos.emit(self.x_pos)

    def setbias_y(self):
        self.microcontroller.setbias_y()
        self.y_pos = 0
        self.yPos.emit(self.y_pos)

    def setbias_z(self):
        self.microcontroller.setbias_z()
        self.z_pos = 0
        self.zPos.emit(self.z_pos)
