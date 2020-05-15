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
from threading import Thread, Lock
import time
import numpy as np
import pyqtgraph as pg
from datetime import datetime
from pathlib import Path

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

        # self.timer_read_pos = QTimer()
        # self.timer_read_pos.setInterval(PosUpdate.INTERVAL_MS)
        # self.timer_read_pos.timeout.connect(self.update_pos)
        # self.timer_read_pos.start()

        self.file = open(str(Path.home()) + "/Downloads/" + datetime.now().strftime('%Y-%m-%d %H-%M-%-S.%f') + ".csv", "w+")
        self.file.write('stepper 1 pos,stepper 2 pos,stepper 3 pos,flow (slm),pressure 1 (psi),pressure 2 (psi),pressure 3 (psi)\n')

        self.timer_collect_data = QTimer()
        self.timer_collect_data.setInterval(10) # check every 100 ms
        self.timer_collect_data.timeout.connect(self.collect_data)
        self.timer_collect_data.start()

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

    def cycle_x(self):
        self.microcontroller.cycle_x()
        self.x_pos = 0
        self.xPos.emit(self.x_pos)

    def cycle_y(self):
        self.microcontroller.cycle_y()
        self.y_pos = 0
        self.yPos.emit(self.y_pos)

    def cycle_z(self):
        self.microcontroller.cycle_z()
        self.z_pos = 0
        self.zPos.emit(self.z_pos)

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

    def home(self):
        # self.microcontroller.move_x(-self.x_pos)
        # self.microcontroller.move_y(-self.y_pos)
        pass

    def collect_data(self):
        data = self.microcontroller.read_received_packet_nowait()
        if data is not None:
            print('data collected')
            for i in range(int(MicrocontrollerDef.MSG_LENGTH_USED/N_BYTES_PER_RECORD)):
                stepper1_pos = utils.unsigned_to_signed(data[i*N_BYTES_PER_RECORD+0:i*N_BYTES_PER_RECORD*2+2],2)
                stepper2_pos = utils.unsigned_to_signed(data[i*N_BYTES_PER_RECORD+2:i*N_BYTES_PER_RECORD*2+4],2)
                stepper3_pos = utils.unsigned_to_signed(data[i*N_BYTES_PER_RECORD+4:i*N_BYTES_PER_RECORD*2+6],2)
                flow = (utils.unsigned_to_signed(data[i*N_BYTES_PER_RECORD+6:i*N_BYTES_PER_RECORD*2+8],2)/(65536/2))*FLOW_FS
                pressure_1 = ((data[i*N_BYTES_PER_RECORD+8]*256+data[i*N_BYTES_PER_RECORD+9])/65536.0)*PRESSURE_FS
                pressure_2 = ((data[i*N_BYTES_PER_RECORD+10]*256+data[i*N_BYTES_PER_RECORD+11])/65536.0)*PRESSURE_FS
                pressure_3 = ((data[i*N_BYTES_PER_RECORD+12]*256+data[i*N_BYTES_PER_RECORD+13])/65536.0)*PRESSURE_FS
                self.file.write(str(stepper1_pos)+','+str(stepper2_pos)+','+str(stepper3_pos)+','+str(flow)+','+str(pressure_1)+','+str(pressure_2)+','+str(pressure_3)+'\n')
                print(str(stepper1_pos)+'\t'+str(stepper2_pos)+'\t'+str(stepper3_pos)+'\t'+"{:.2f}".format(flow)+'\t'+"{:.2f}".format(pressure_1)+'\t'+"{:.2f}".format(pressure_2)+'\t'+"{:.2f}".format(pressure_3))
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
