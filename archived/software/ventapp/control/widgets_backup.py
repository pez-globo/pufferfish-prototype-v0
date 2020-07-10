# set QT_API environment variable
import os
os.environ["QT_API"] = "pyqt5"
import qtpy

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *

from ventapp.control._def import *

import numpy as np
import pyqtgraph as pg
from collections import deque
import time

class stepperMotorWidget(QFrame):
    def __init__(self, stepperMotorController, main=None, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.stepperMotorController = stepperMotorController
        self.add_components()
        self.setFrameStyle(QFrame.Panel | QFrame.Raised)

    def add_components(self):
        self.label_Xpos = QLabel()
        self.label_Xpos.setNum(0)
        self.label_Xpos.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.entry_dX = QDoubleSpinBox()
        self.entry_dX.setMinimum(0)
        self.entry_dX.setMaximum(5)
        self.entry_dX.setSingleStep(0.2)
        self.entry_dX.setValue(0)
        self.btn_moveX_forward = QPushButton('Forward')
        self.btn_moveX_forward.setDefault(False)
        self.btn_moveX_backward = QPushButton('Backward')
        self.btn_moveX_backward.setDefault(False)

        self.label_Ypos = QLabel()
        self.label_Ypos.setNum(0)
        self.label_Ypos.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.entry_dY = QDoubleSpinBox()
        self.entry_dY.setMinimum(0)
        self.entry_dY.setMaximum(5)
        self.entry_dY.setSingleStep(0.2)
        self.entry_dY.setValue(0)
        self.btn_moveY_forward = QPushButton('Forward')
        self.btn_moveY_forward.setDefault(False)
        self.btn_moveY_backward = QPushButton('Backward')
        self.btn_moveY_backward.setDefault(False)

        grid_line0 = QGridLayout()
        grid_line0.addWidget(QLabel('X (mm)'), 0,0)
        grid_line0.addWidget(self.label_Xpos, 0,1)
        grid_line0.addWidget(self.entry_dX, 0,2)
        grid_line0.addWidget(self.btn_moveX_forward, 0,3)
        grid_line0.addWidget(self.btn_moveX_backward, 0,4)

        grid_line1 = QGridLayout()
        grid_line1.addWidget(QLabel('Y (mm)'), 0,0)
        grid_line1.addWidget(self.label_Ypos, 0,1)
        grid_line1.addWidget(self.entry_dY, 0,2)
        grid_line1.addWidget(self.btn_moveY_forward, 0,3)
        grid_line1.addWidget(self.btn_moveY_backward, 0,4)

        self.grid = QGridLayout()
        self.grid.addLayout(grid_line0,0,0)
        self.grid.addLayout(grid_line1,1,0)
        self.setLayout(self.grid)

        self.btn_moveX_forward.clicked.connect(self.move_x_forward)
        self.btn_moveX_backward.clicked.connect(self.move_x_backward)
        self.btn_moveY_forward.clicked.connect(self.move_y_forward)
        self.btn_moveY_backward.clicked.connect(self.move_y_backward)

    def move_x_forward(self):
        self.stepperMotorController.move_x(self.entry_dX.value())
    def move_x_backward(self):
        self.stepperMotorController.move_x(-self.entry_dX.value())
    def move_y_forward(self):
        self.stepperMotorController.move_y(self.entry_dY.value())
    def move_y_backward(self):
        self.stepperMotorController.move_y(-self.entry_dY.value())


# from Deepak
class PlotWidget(pg.GraphicsLayoutWidget):

    def __init__(self,title, color = 'w', parent=None):
        super().__init__(parent)
        #pg.setConfigOption('background', 'w')
        self.title = title
        self.maxLen = int(1000*WAVEFORMS.DISPLAY_RANGE_S/WAVEFORMS.UPDATE_INTERVAL_MS)
        self.X_data = deque(maxlen = self.maxLen)
        self.Y_data = deque(maxlen = self.maxLen)
        self.Abs = []
        self.Ord = []
        self.plot1 = self.addPlot(title = self.title + ' ' + PLOT_UNITS[self.title])
        self.setBackground('w')
        # self.curve = self.plot1.plot(self.Abs, self.Ord, pen=pg.mkPen(color, width=3), fillLevel=-0.3, brush=(50,50,200,100))
        self.curve = self.plot1.plot(self.Abs, self.Ord, pen=pg.mkPen(color, width=3), brush=(50,50,200,100))
        self.curve.setClipToView(True)
        self.plot1.enableAutoRange('y', True)
        self.plot1.showGrid(x=True, y=True)
        self.ptr = 0
        #pg.setConfigOption('background', 'w')

    def update_plot(self, time, data):
        self.X_data.append(time)
        self.Y_data.append(data)
        self.label = PLOT_UNITS[self.title]
        self.Abs = np.array(self.X_data)
        self.Ord = np.array(self.Y_data)
        self.curve.setData(self.Abs-self.Abs[-1], self.Ord)
        self.curve.setPos(self.Abs[-1],0)

    def initialise_plot(self):
        self.X_data = deque(maxlen = self.maxLen)
        self.Y_data = deque(maxlen = self.maxLen)
        self.Abs = []
        self.Ord = []
        self.curve.setData(self.Abs,self.Ord)


class WaveformDisplay(QFrame):

    def __init__(self, main=None, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.add_components()
        self.setFrameStyle(QFrame.Panel | QFrame.Raised)

    def add_components(self):
        self.plotWidgets = {key: PlotWidget(title = key, color = 'b') for key in PLOTS}
        grid = QGridLayout()
        for ii, key in enumerate(PLOTS):
            grid.addWidget(self.plotWidgets[key], ii, 0,1,2)
        self.setLayout(grid)
