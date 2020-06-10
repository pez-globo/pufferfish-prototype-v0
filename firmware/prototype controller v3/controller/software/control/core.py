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
        self.Vt = MCU.Vt_DEFAULT
        self.Ti = MCU.Ti_DEFAULT
        self.RR = MCU.RR_DEFAULT
        self.PEEP = MCU.PEEP_DEFAULT
        self.mode = MODE_PC_AC

    def setVT(self,value):
        self.microcontroller.set_parameter(MCU.CMD_Vt,value/MCU.VT_FS)
        self.Vt = value

    def setTi(self,value):
        self.microcontroller.set_parameter(MCU.CMD_Ti,value/MCU.TI_FS)
        self.Ti = value

    def setRR(self,value):
        self.microcontroller.set_parameter(MCU.CMD_RR,value/MCU.RR_FS)
        self.RR = value

    def setPEEP(self,value):
        self.microcontroller.set_parameter(MCU.CMD_PEEP,value/MCU.PEEP_FS)
        self.PEEP = value

    def setFlow(self,value):
        self.microcontroller.set_parameter(MCU.CMD_Flow,value/MCU.VALVE_POS_OPEN_STEPS_FS)

    # def setFlowDeceleratingSlope(self,value):
    #     self.microcontroller.set_parameter(MCU.CMD_FlowDeceleratingSlope,value/100)

    def setPinsp(self,value):
        print('setting P_insp')
        self.microcontroller.set_parameter(MCU.CMD_Pinsp,value/MCU.PAW_FS)

    def setRiseTime(self,value):
        self.microcontroller.set_parameter(MCU.CMD_RiseTime,value/MCU.PC_RISE_TIME_MS_FS)

    def setPID_P(self,value):
        self.microcontroller.set_parameter(MCU.CMD_PID_P,value/MCU.PID_COEFFICIENT_P_FS)

    def setPID_I_frac(self,value):
        self.microcontroller.set_parameter(MCU.CMD_PID_I_frac,value/MCU.PID_COEFFICIENT_I_FRAC_FS)
    
    def setONOFF(self,state):
        if state == False:
            self.microcontroller.set_mode(MCU.CMD_MODE,0) # to add mode selection
            print('stop breathing')
        else:
            #TODO: add mode selection
            self.microcontroller.set_mode(MCU.CMD_MODE,self.mode) 

    def updateMode(self,mode):
        print('update mode')
        if mode == MODE_PC_AC_STRING:
            self.mode = MODE_PC_AC
            # self.microcontroller.set_mode(MCU.CMD_MODE,self.mode) 
        if mode == MODE_VC_AC_STRING:
            self.mode = MODE_VC_AC
            # self.microcontroller.set_mode(MCU.CMD_MODE,self.mode) 
        if mode == MODE_PSV_STRING:
            self.mode = MODE_PSV
            # self.microcontroller.set_mode(MCU.CMD_MODE,self.mode) 

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

        self.first_run = True
        self.time_ticks_start = 0

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

                # self.time_now = time.time()
                # self.time_diff = self.time_now - self.time_prev
                # self.time_prev = self.time_now
                # self.time += self.time_diff

                for i in range(MCU.TIMEPOINT_PER_UPDATE):
                    # time
                    self.time_ticks = int.from_bytes(readout[i*MCU.RECORD_LENGTH_BYTE:i*MCU.RECORD_LENGTH_BYTE+4], byteorder='big', signed=False)
                    if self.first_run:
                        self.time_ticks_start = self.time_ticks
                        self.first_run = False
                    self.time = (self.time_ticks - self.time_ticks_start)*MCU.TIMER_PERIOD_ms/1000
                    # stepper air pos
                    self.stepper_air_pos = utils.unsigned_to_signed(readout[i*MCU.RECORD_LENGTH_BYTE+4:i*MCU.RECORD_LENGTH_BYTE+6],2)
                    # stepper oxygen pos
                    self.stepper_oxygen_pos = utils.unsigned_to_signed(readout[i*MCU.RECORD_LENGTH_BYTE+6:i*MCU.RECORD_LENGTH_BYTE+8],2)
                    # flow air
                    self.flow_air = utils.unsigned_to_signed(readout[i*MCU.RECORD_LENGTH_BYTE+8:i*MCU.RECORD_LENGTH_BYTE+10],2)/(65536/2)*MCU.flow_FS
                    # flow oxygen
                    self.flow_oxygen = utils.unsigned_to_signed(readout[i*MCU.RECORD_LENGTH_BYTE+10:i*MCU.RECORD_LENGTH_BYTE+12],2)/(65536/2)*MCU.flow_FS
                    # flow proximal
                    self.flow_proximal = utils.unsigned_to_signed(readout[i*MCU.RECORD_LENGTH_BYTE+12:i*MCU.RECORD_LENGTH_BYTE+14],2)/(65536/2)*MCU.flow_FS
                    # upstream pressure air
                    self.pressure_upstream_air_psi = utils.unsigned_to_signed(readout[i*MCU.RECORD_LENGTH_BYTE+14:i*MCU.RECORD_LENGTH_BYTE+16],2)/(65536/2)*MCU.psupply_FS
                    # upstream pressure oxygen
                    self.pressure_upstream_oxygen_psi = utils.unsigned_to_signed(readout[i*MCU.RECORD_LENGTH_BYTE+16:i*MCU.RECORD_LENGTH_BYTE+18],2)/(65536/2)*MCU.psupply_FS
                    # exhalation valve pressure
                    self.pressure_exhalation_control_cmH2O = utils.unsigned_to_signed(readout[i*MCU.RECORD_LENGTH_BYTE+18:i*MCU.RECORD_LENGTH_BYTE+20],2)/(65536/2)*MCU.paw_FS
                    # patient pressure
                    self.pressure_patient_cmH2O = utils.unsigned_to_signed(readout[i*MCU.RECORD_LENGTH_BYTE+20:i*MCU.RECORD_LENGTH_BYTE+22],2)/(65536/2)*MCU.paw_FS
                    # airway pressure
                    self.pressure_aw_cmH2O = utils.unsigned_to_signed(readout[i*MCU.RECORD_LENGTH_BYTE+22:i*MCU.RECORD_LENGTH_BYTE+24],2)/(65536/2)*MCU.paw_FS
                    # volume
                    self.volume = utils.unsigned_to_signed(readout[i*MCU.RECORD_LENGTH_BYTE+24:i*MCU.RECORD_LENGTH_BYTE+26],2)/(65536/2)*MCU.volume_FS
                    # FiO2
                    # humidity

                    # self.Paw = (utils.unsigned_to_signed(readout[0:2],MCU.N_BYTES_DATA)/(65536/2))*MCU.PAW_FS 
                    # self.Flow = (utils.unsigned_to_signed(readout[2:4],MCU.N_BYTES_DATA)/(65536/2))*MCU.FLOW_FS
                    # self.Volume = (utils.unsigned_to_unsigned(readout[4:6],MCU.N_BYTES_DATA)/65536)*MCU.VOLUME_FS

                    # saved variables
                    self.file.write(str(self.time_now)+','+"{:.2f}".format(self.Paw)+','+"{:.2f}".format(self.Flow)+','+"{:.2f}".format(self.Volume)+','+str(self.ventController.Vt)+','+str(self.ventController.Ti)+','+str(self.ventController.RR)+','+str(self.ventController.PEEP) +'\n')
                    print(str(self.time_ticks) + '\t' + str(self.stepper_air_pos) + '\t' + "{:.2f}".format(self.flow_air) + '\t' + "{:.2f}".format(self.flow_proximal) + '\t' +  "{:.2f}".format(self.pressure_exhalation_control_cmH2O) + '\t' + "{:.2f}".format(self.pressure_aw_cmH2O))

            # reduce display refresh rate
            self.counter_display = self.counter_display + 1
            if self.counter_display>=1:
                self.counter_display = 0
                self.signal_Paw.emit(self.time,self.pressure_aw_cmH2O)
                self.signal_Flow.emit(self.time,self.flow_proximal)
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
