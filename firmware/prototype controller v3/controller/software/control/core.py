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
        self.Pinsp = MCU.pinsp_DEFAULT
        self.riseTime = MCU.pc_rise_time_ms_DEFAULT
        self.P = MCU.P_default
        self.I_frac = MCU.I_frac_default
        self.trigger_th = MCU.paw_trigger_th_DEFAULT
        self.exhalationControlPRiseTime = MCU.rise_time_ms_exhalation_control_DEFAULT

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
        self.Pinsp = value

    def setRiseTime(self,value):
        self.microcontroller.set_parameter(MCU.CMD_RiseTime,value/MCU.PC_RISE_TIME_MS_FS)
        self.riseTime = value

    def setPID_P(self,value):
        self.microcontroller.set_parameter(MCU.CMD_PID_P,value/MCU.PID_COEFFICIENT_P_FS)
        self.P = value

    def setPID_I_frac(self,value):
        self.microcontroller.set_parameter(MCU.CMD_PID_I_frac,value/MCU.PID_COEFFICIENT_I_FRAC_FS)
        self.I_frac = value

    def setTriggerTh(self,value):
        self.microcontroller.set_parameter(MCU.CMD_Trigger_th,-value/MCU.PAW_FS)
        self.I_frac = value

    def setONOFF(self,state):
        self.is_breathing = state
        if self.is_breathing == False:
            self.microcontroller.set_mode(MCU.CMD_ONOFF,0) # to add mode selection
            print('stop breathing')
        else:
            self.microcontroller.set_mode(MCU.CMD_ONOFF,1) 
            print('start breathing')
        
    def updateMode(self,mode):
        print('update mode')
        if mode == MODE_PC_AC_STRING:
            self.mode = MODE_PC_AC
            self.microcontroller.set_mode(MCU.CMD_MODE,self.mode) 
        if mode == MODE_VC_AC_STRING:
            self.mode = MODE_VC_AC
            self.microcontroller.set_mode(MCU.CMD_MODE,self.mode) 
        if mode == MODE_PSV_STRING:
            self.mode = MODE_PSV
            self.microcontroller.set_mode(MCU.CMD_MODE,self.mode) 

    def setExhalationControlPRiseTime(self,value):
        self.microcontroller.set_parameter(MCU.CMD_Exhalation_Control_RiseTime,value/MCU.PC_RISE_TIME_MS_FS)
        self.exhalationControlPRiseTime = value

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
    signal_print = Signal(str)

    signal_stepper_pos_air = Signal(str)
    signal_stepper_pos_oxygen = Signal(str)
    signal_flow_air = Signal(str)
    signal_flow_proximal = Signal(str)
    signal_p_exhalation_control = Signal(str)
    signal_p_airway = Signal(str)
    signal_p_aux = Signal(str)
    signal_dP = Signal(str)
    signal_p_supply_air = Signal(str)
    signal_p_supply_oxygen = Signal(str)
    signal_fio2 = Signal(str)
    signal_flow_oxygen = Signal(str)

    def __init__(self,microcontroller,ventController):
        QObject.__init__(self)
        self.file = open(str(Path.home()) + "/Downloads/" + datetime.now().strftime('%Y-%m-%d %H-%M-%-S.%f') + ".csv", "w+")
        # self.file.write('Time (s),Paw (cmH2O),Flow (l/min),Volume (ml),Vt (ml),Ti (s),RR (/min),PEEP (cmH2O)\n')
        self.microcontroller = microcontroller
        self.ventController = ventController
        self.Paw = 0
        self.Volume = 0
        self.volume = 0
        self.Flow = 0
        self.time = 0
        self.fio2 = 0
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
                    tmp_volume_total = self.volume # for FiO2 calculation
                    self.volume = utils.unsigned_to_signed(readout[i*MCU.RECORD_LENGTH_BYTE+24:i*MCU.RECORD_LENGTH_BYTE+26],2)/(65536/2)*MCU.volume_FS
                    # FiO2
                    self.dP = utils.unsigned_to_signed(readout[i*MCU.RECORD_LENGTH_BYTE+26:i*MCU.RECORD_LENGTH_BYTE+28],2)/(65536/2)*MCU.dP_FS
                    # humidity
                    self.volume_oxygen = utils.unsigned_to_signed(readout[i*MCU.RECORD_LENGTH_BYTE+28:i*MCU.RECORD_LENGTH_BYTE+30],2)/(65536/2)*MCU.volume_FS

                    # fio2 calculation; note here volume is actually mass
                    if (self.volume >= tmp_volume_total) and (self.volume >= 100):
                        if(self.volume==0):
                            self.fio2 = 0
                        else:
                            self.fio2 = ( (self.volume - self.volume_oxygen)*0.2314 + self.volume_oxygen*1 ) / (self.volume)
                        
                    record_from_MCU = (
                        str(self.time_ticks) + '\t' + str(self.stepper_air_pos) + '\t' + str(self.stepper_oxygen_pos) + '\t' + "{:.2f}".format(self.flow_air) + '\t' + 
                        "{:.2f}".format(self.flow_oxygen) + '\t' + "{:.2f}".format(self.flow_proximal) + '\t' +  "{:.2f}".format(self.pressure_exhalation_control_cmH2O) + '\t' + 
                        "{:.2f}".format(self.pressure_patient_cmH2O) + '\t' + "{:.2f}".format(self.pressure_aw_cmH2O) + '\t' + "{:.2f}".format(self.volume) ) + '\t' + "{:.2f}".format(self.dP) + '\t' + "{:.2f}".format(self.volume_oxygen)
                    record_settings = (
                        str(self.time_now) + '\t' + str(self.ventController.Vt) + '\t' + str(self.ventController.Ti) + '\t' + str(self.ventController.RR) + '\t' + 
                        str(self.ventController.PEEP) + '\t' + str(self.ventController.PEEP) + '\t' + str(self.ventController.Pinsp) + '\t' + str(self.ventController.riseTime) + '\t' + 
                        str(self.ventController.P) + '\t' + str(self.ventController.I_frac) + '\t' + str(self.ventController.trigger_th) + '\t' + str(self.ventController.mode) + '\t' + 
                        str(self.ventController.exhalationControlPRiseTime) )
                    # self.signal_print.emit(record_from_MCU)
                   
                    # saved variables
                    if self.logging_is_on:
                        self.file.write(record_from_MCU + '\t' + record_settings + '\n')
                    # print(record_from_MCU)
                    # record_from_MCU_debug = 'sterpper pos: ' + str(self.stepper_air_pos) + '\t\t flow_air: ' + "{:.2f}".format(self.flow_air) + '\t flow_proximal: ' + "{:.2f}".format(self.flow_proximal) + '\t p_exhalation control: ' +  "{:.2f}".format(self.pressure_exhalation_control_cmH2O) + '\t p_airway: ' + "{:.2f}".format(self.pressure_aw_cmH2O)
                    # self.signal_print.emit(record_from_MCU_debug)

                # reduce display refresh rate
                self.counter_display = self.counter_display + 1
                if self.counter_display>=1:
                    self.counter_display = 0
                    self.signal_Paw.emit(self.time,self.pressure_aw_cmH2O)
                    self.signal_Flow.emit(self.time,self.flow_proximal)
                    self.signal_Volume.emit(self.time,self.volume)
                    self.signal_stepper_pos_air.emit("{:.2f}".format(self.stepper_air_pos))
                    self.signal_stepper_pos_oxygen.emit("{:.2f}".format(self.stepper_oxygen_pos))
                    self.signal_flow_air.emit("{:.2f}".format(self.flow_air))
                    self.signal_flow_proximal.emit("{:.2f}".format(self.flow_proximal))
                    self.signal_p_exhalation_control.emit("{:.2f}".format(self.pressure_exhalation_control_cmH2O))
                    self.signal_p_airway.emit("{:.2f}".format(self.pressure_aw_cmH2O))
                    self.signal_p_aux.emit("{:.2f}".format(self.pressure_patient_cmH2O))
                    self.signal_dP.emit("{:.2f}".format(self.dP))
                    self.signal_p_supply_air.emit("{:.2f}".format(self.pressure_upstream_air_psi))
                    self.signal_p_supply_oxygen.emit("{:.2f}".format(self.pressure_upstream_oxygen_psi))
                    if self.fio2 >= 0.21 and self.fio2 <=1:
                        self.signal_fio2.emit("{:.0f}".format(self.fio2*100))
                    else:
                        self.signal_fio2.emit("-")
                    self.signal_flow_oxygen.emit("{:.2f}".format(self.flow_oxygen))

        # file flushing
        if self.logging_is_on:
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
