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
import control.CSV_Tool as CSV_Tool


from queue import Queue
from threading import Thread, Lock
import time
import numpy as np
import pyqtgraph as pg
from datetime import datetime
from pathlib import Path

class NavigationController(QObject):

    ValvePosition = Signal(float)
    ValveCycle =  Signal(int)
    Pressure =  Signal(float)
    FlowRate = Signal(float)
    ActiveValveID = Signal(int)
    Force = Signal(float)


    def __init__(self,microcontroller):
        QObject.__init__(self)
        self.microcontroller = microcontroller
        
        # Internal copy of the data
        self.valve_pos = np.zeros(MicrocontrollerDef.N_VALVES)
        self.valve_cycles = 0
        self.valve_temperature = np.zeros(MicrocontrollerDef.N_VALVES)

        # self.timer_read_pos = QTimer()
        # self.timer_read_pos.setInterval(PosUpdate.INTERVAL_MS)
        # self.timer_read_pos.timeout.connect(self.update_pos)
        # self.timer_read_pos.start()

        self.file_counter = 0
        self.time_elapsed_since_save = 0
        self.time_last_saved = 0

        self.file_header = 'Time (s)'+','+'Active valve'+','+'cycles'+','+'stepper position (mm)'+','+'pressure (psi)'+','+'flow (slm)'+','+'force (N)'+','+'temperature (C)'+'\n'
        self.file = open(str(Path.home()) + "/Downloads/" + datetime.now().strftime('%Y-%m-%d %H-%M-%-S') + '_{:5d}'.format(self.file_counter)+".csv", "w+")
        
        # cycles_header = list()
        # position_header = list()
        # temperatures_header = list()

        # for ii in range(MicrocontrollerDef.N_VALVES):
        #     cycles_header = cycles_header.append(['valve {} cycles,'.format(ii)])
        #     position_header = position_header.append(['valve {} position,'.format(ii)])
        #     temperatures_header = temperatures_header.append(['valve {} temperatures,'.format(ii)])

        self.file.write(self.file_header)
        # self.file.write('Time (s)'+','+cycles_header+','+position_header+','+temperatures_header+'Force (N)'+'\n')
        # self.file.write('stepper 1 pos,stepper 2 pos,stepper 3 pos,flow (slm),pressure 1 (psi),pressure 2 (psi),pressure 3 (psi)\n')

        # Initialize arrays to hold recd data from uController
        self.valve_cycles_rec = 0
        self.valve_pos_rec = 0
        self.valve_temperature_rec = 0
        self.force = 0

        self.current_active_valve = 0

        self.timer_collect_data = QTimer()
        self.timer_collect_data.setInterval(10) # check every 100 ms
        self.timer_collect_data.timeout.connect(self.collect_data)
        self.timer_collect_data.start()

        self.time_start = time.time()

    def move_valve(self, delta):

        self.microcontroller.move_valve(delta)

        self.valve_pos[self.current_active_valve] += delta

        # Update the local copy of the valve position
        
        # if(valve_id > MicrocontrollerDef.N_VALVES):
        #     self.valve_pos = self.valve_pos + delta
        # else:
        #     self.valve_pos[valve_id] = self.valve_pos[valve_id] + delta

        # self.ValvePositions.emit(self.valve_pos)

    def start_cycle_valve(self):
        self.microcontroller.start_cycle_valve()

    def stop_cycle_valve(self):
        self.microcontroller.stop_cycle_valve()

    def close_valve(self):
        self.microcontroller.close_valve()


    def update_active_valve(self, valve_id):

        self.current_active_valve = valve_id

        self.microcontroller.update_active_valve(valve_id)

    def set_valve_cycles(self, valve_cycles):

        self.microcontroller.enable_valve_measurement_cycling(valve_cycles)


    def start_new_file(self):

        self.file.close()
        self.file_counter+=1
        self.file = open(str(Path.home()) + "/Downloads/" + datetime.now().strftime('%Y-%m-%d %H-%M-%-S') + '_{:05d}'.format(self.file_counter)+".csv", "w+")
        self.file.write(self.file_header)

    def collect_data(self):

        self.time_now = time.time() - self.time_start

        self.time_elapsed_since_save = time.time() - self.time_last_saved

        if(self.time_elapsed_since_save > SAVE_TIME_PER_FILE):
            self.start_new_file()
            self.time_last_saved = time.time()
            self.time_elapsed_since_save = 0


        data = self.microcontroller.read_received_packet_nowait()
        
        if data is not None:
            # print('data collected')
            # print('No:of bytes: {}'.format(len(data)))
            ''' 
            Message format:
             - No:of cycles (4 bytes x N_valves)
             - Actuator position (2 bytes x N_valves)
             - Temperature (2 bytes x N_valves)
            '''

            for ii in range(int(MicrocontrollerDef.MSG_LENGTH/MicrocontrollerDef.N_BYTES_PER_RECORD)):

               
                start_index = ii*MicrocontrollerDef.N_BYTES_PER_RECORD

                self.valve_cycles_rec = int.from_bytes(data[start_index: start_index + 4], byteorder='big', signed=False)
                # self.valve_cycles_rec[jj] = utils.data4byte_to_int(data[start_index: start_index + 4])
                self.valve_pos_rec = utils.unsigned_to_signed(data[start_index+4: start_index+6], 2)

                self.flow_rate = utils.unsigned_to_signed(data[start_index+6: start_index+8], 2)

                self.upstream_pressure = int.from_bytes(data[start_index + 8: start_index + 10], byteorder='big', signed=False)

                self.active_valve_id = int.from_bytes(data[start_index + 10: start_index + 12], byteorder='big', signed=False)

                self.force = int.from_bytes(data[start_index + 12: start_index + 14], byteorder='big', signed=False)

                self.valve_temperature_rec = int.from_bytes(data[start_index + 14: start_index + 16], byteorder='big', signed=False)
                
                # Convert raw reading for force to Newtons:
                force_voltage = (Force.DRIVE_VOLTAGE)*(self.force/Force.ADC_RESOLUTION)

                force_newtons = Force.MAX_RATED_LOAD*(force_voltage - Force.VOLT_MIN_LOAD)/(Force.VOLT_MAX_LOAD - Force.VOLT_MIN_LOAD)

                # ['Time (s)'+','+'Active valve'+','+'cycles'+','+'stepper position (mm)'+','+'pressure (psi)'+','+'flow (slm)','force (N)','temperature (C)']
                self.file.write(str(round(self.time_now,3)) + ',' +str(self.active_valve_id)+','+str(self.valve_cycles)+','+str(self.valve_pos_rec)+','+str(self.upstream_pressure)+','+str(self.flow_rate)+','+str(self.force)+','+str(self.valve_temperature_rec)+'\n')

                print('cycles: '+ str(self.valve_cycles_rec)+','+ str(round(self.time_now,2)) +',' + 'active valve: '+ str(self.active_valve_id)+','+'valve pos: '+str(self.valve_pos_rec)+','+ 'pressure: '+ str(round(self.upstream_pressure,2)) +','+ 'flow rate: '+ str(round(self.flow_rate,2))+ ',' + str(round(self.force,2)) + '\n')
            
            

            self.ActiveValveID.emit(self.active_valve_id)
            self.ValveCycle.emit(self.valve_cycles_rec)
            self.Pressure.emit(self.upstream_pressure)
            self.FlowRate.emit(self.flow_rate)
            self.ValvePosition.emit(self.valve_pos_rec)
            self.Force.emit(self.force)
            self.file.flush()


# from gravity machine
class ImageDisplayWindow(QMainWindow):

    def __init__(self, window_title=''):
        super().__init__()
        self.setWindowTitle(window_title)
        self.setWindowFlags(self.windowFlags() | Qt.CustomizeWindowHint)
        self.setWindowFlags(self.windowFlags() & ~Qt.WindowCloseButtonHint)
        self.widget = QWidget()

        self.graphics_widget = pg.GraphicsLayoutWidget()
        self.graphics_widget.view = self.graphics_widget.addViewBox()
        
        ## lock the aspect ratio so pixels are always square
        self.graphics_widget.view.setAspectLocked(True)
        
        ## Create image item
        self.graphics_widget.img = pg.ImageItem(border='w')
        self.graphics_widget.view.addItem(self.graphics_widget.img)

        layout = QGridLayout()
        layout.addWidget(self.graphics_widget, 0, 0) 
        self.widget.setLayout(layout)
        self.setCentralWidget(self.widget)

        # set window size
        desktopWidget = QDesktopWidget();
        width = min(desktopWidget.height()*0.9,1000) #@@@TO MOVE@@@#
        height = width
        self.setFixedSize(width,height)

    def display_image(self,image):
        self.graphics_widget.img.setImage(image,autoLevels=False)
        # print('display image')
