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

    def __init__(self,microcontroller):
        QObject.__init__(self)
        self.microcontroller = microcontroller

        self.timer_collect_data = QTimer()
        self.timer_collect_data.setInterval(10) # check every 10 ms
        self.timer_collect_data.timeout.connect(self.collect_data)
        self.timer_collect_data.start()

        # self.file = open(str(Path.home()) + "/Downloads/" + datetime.now().strftime('%Y-%m-%d %H-%M-%-S.%f') + ".csv", "w+")
        # self.file.write('stepper 1 pos,stepper 2 pos,stepper 3 pos,flow (slm),pressure 1 (psi),pressure 2 (psi),pressure 3 (psi)\n')

    def update_PWM(self,cmd):
        self.microcontroller.send_command(cmd)

    def cycle_PWM(self,cmd,filename):
        self.file = open(str(Path.home()) + "/Downloads/" + filename + '_' + datetime.now().strftime('%Y-%m-%d %H-%M-%-S.%f') + ".csv", "w+")
        self.microcontroller.send_command(cmd)

    def collect_data(self):
        data = self.microcontroller.read_received_packet_nowait()
        if data is not None:
            for i in range(int(MicrocontrollerDef.MSG_LENGTH/N_BYTES_PER_RECORD)):
                timestamp = int.from_bytes(data[i*N_BYTES_PER_RECORD:i*N_BYTES_PER_RECORD+4], byteorder='big', signed=False)
                cycle_count = int.from_bytes(data[i*N_BYTES_PER_RECORD+4:i*N_BYTES_PER_RECORD+8], byteorder='big', signed=False)
                pwm = data[i*N_BYTES_PER_RECORD+8]*256+data[i*N_BYTES_PER_RECORD+9]
                flow = (utils.unsigned_to_signed(data[i*N_BYTES_PER_RECORD+10:i*N_BYTES_PER_RECORD+12],2)/(65536/2))*FLOW_FS
                pressure = ((data[i*N_BYTES_PER_RECORD+12]*256+data[i*N_BYTES_PER_RECORD+13])/65536.0)*PRESSURE_FS
                is_cycling = data[i*N_BYTES_PER_RECORD+14]
                if(is_cycling):
                    self.file.write(str(timestamp)+','+str(cycle_count)+','+str(pwm)+','+"{:.2f}".format(flow)+','+"{:.2f}".format(pressure)+'\n')
                print(str(timestamp)+'\t'+str(cycle_count)+'\t'+str(pwm)+'\t'+"{:.2f}".format(flow)+'\t'+"{:.2f}".format(pressure)+'\t'+ str(is_cycling))
            if(is_cycling):
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
