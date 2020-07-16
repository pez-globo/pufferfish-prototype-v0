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

        # self.file_header = ['stepper 1 pos,stepper 2 pos,stepper 3 pos,flow (slm),pressure 1 (psi),pressure 2 (psi),pressure 3 (psi)\n']


        # self.file = open(str(Path.home()) + "/Downloads/" + datetime.now().strftime('%Y-%m-%d %H-%M-%-S') + ".csv", "w+")
        
        # cycles_header = list()
        # position_header = list()
        # temperatures_header = list()

        # for ii in range(MicrocontrollerDef.N_VALVES):
        #     cycles_header = cycles_header.append(['valve {} cycles,'.format(ii)])
        #     position_header = position_header.append(['valve {} position,'.format(ii)])
        #     temperatures_header = temperatures_header.append(['valve {} temperatures,'.format(ii)])

        # self.file.write('Time (s)'+','+cycles_header+','+position_header+','+temperatures_header+'Force (N)'+'\n')
        # self.file.write('stepper 1 pos,stepper 2 pos,stepper 3 pos,flow (slm),pressure 1 (psi),pressure 2 (psi),pressure 3 (psi)\n')

        # Initialize arrays to hold recd data from uController
        self.valve_cycles_rec = 0
        self.valve_pos_rec = 0
        self.valve_temperature_rec = 0

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





    # def move_x(self,delta):
    #     self.microcontroller.move_x(delta)
    #     self.x_pos = self.x_pos + delta
    #     print('X: ' + str(self.x_pos))
    #     self.xPos.emit(self.x_pos)

    # def move_y(self,delta):
    #     self.microcontroller.move_y(delta)
    #     self.y_pos = self.y_pos + delta
    #     print('Y: ' + str(self.y_pos))
    #     self.yPos.emit(self.y_pos)

    # def move_z(self,delta):
    #     self.microcontroller.move_z(delta)
    #     self.z_pos = self.z_pos + delta
    #     print('Z: ' + str(self.z_pos))
    #     self.zPos.emit(self.z_pos)

    # def close_x(self):
    #     self.microcontroller.close_x()
    #     self.x_pos = 0
    #     self.xPos.emit(self.x_pos)

    # def close_y(self):
    #     self.microcontroller.close_y()
    #     self.y_pos = 0
    #     self.yPos.emit(self.y_pos)

    # def close_z(self):
    #     self.microcontroller.close_z()
    #     self.z_pos = 0
    #     self.zPos.emit(self.z_pos)

    # def cycle_x(self):
    #     self.microcontroller.cycle_x()
    #     self.x_pos = 0
    #     self.xPos.emit(self.x_pos)

    # def cycle_y(self):
    #     self.microcontroller.cycle_y()
    #     self.y_pos = 0
    #     self.yPos.emit(self.y_pos)

    # def cycle_z(self):
    #     self.microcontroller.cycle_z()
    #     self.z_pos = 0
    #     self.zPos.emit(self.z_pos)

    # def update_pos(self):
    #     pos = self.microcontroller.read_received_packet_nowait()
    #     if pos is None:
    #         return
    #     self.x_pos = utils.unsigned_to_signed(pos[0:3],MicrocontrollerDef.N_BYTES_POS)/Motion.STEPS_PER_MM_XY # @@@TODO@@@: move to microcontroller?
    #     self.y_pos = utils.unsigned_to_signed(pos[3:6],MicrocontrollerDef.N_BYTES_POS)/Motion.STEPS_PER_MM_XY # @@@TODO@@@: move to microcontroller?
    #     self.z_pos = utils.unsigned_to_signed(pos[6:9],MicrocontrollerDef.N_BYTES_POS)/Motion.STEPS_PER_MM_Z  # @@@TODO@@@: move to microcontroller?
    #     self.xPos.emit(self.x_pos)
    #     self.yPos.emit(self.y_pos)
    #     self.zPos.emit(self.z_pos*1000)

  

    def collect_data(self):

        self.time_now = time.time() - self.time_start
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
                
                # self.file.write(str(self.time_now) + ',' +str(self.valve_cycles_rec)+','+str(self.valve_pos_rec)+','+str(self.valve_temperature_rec) + str(0) +'\n')

                print(str(round(self.time_now,2)) +',' + 'commmanded pos: '+ str(self.upstream_pressure)+','+'valve pos: '+str(self.valve_pos_rec)+','+ 'valve_id: '+str(self.active_valve_id) + ',' + 'cycles: '+ str(self.valve_cycles_rec) + ',' + str(self.force) + ','  + str(self.valve_temperature_rec) + '\n')
                # print('Open loop pos (mm)'+'\t'+ 'Force (N)') 
                # print(str(stepper1_openLoop_pos)+'\t'+str(force_newtons))
            
            self.ActiveValveID.emit(self.active_valve_id)
            self.ValveCycle.emit(self.valve_cycles_rec)
            self.Pressure.emit(self.upstream_pressure)
            self.FlowRate.emit(self.flow_rate)
            self.ValvePosition.emit(self.valve_pos_rec)
            # self.file.flush()


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
