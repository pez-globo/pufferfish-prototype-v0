# set QT_API environment variable
import os 
os.environ["QT_API"] = "pyqt5"
import qtpy

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *

from _def import *

import numpy as np
import pyqtgraph as pg
from collections import deque
import time

from plot4vent import plot_4vent
import constants as constants

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

class ControlPanel(QFrame):
	def __init__(self, ventController, main=None, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.font = QFont()
		self.font.setPixelSize(16)
		self.ventController = ventController
		self.add_components()
		self.setFrameStyle(QFrame.Panel | QFrame.Raised)

	def add_components(self):

		self.entry_VT = QDoubleSpinBox()
		self.entry_VT.setFont(self.font)
		self.entry_VT.setMinimum(100)
		self.entry_VT.setMaximum(500)
		self.entry_VT.setSingleStep(10)
		self.entry_VT.setValue(MicrocontrollerDef.Vt_DEFAULT)

		self.entry_Ti = QDoubleSpinBox()
		self.entry_Ti.setFont(self.font)
		self.entry_Ti.setMinimum(0.3)
		self.entry_Ti.setMaximum(5)
		self.entry_Ti.setSingleStep(0.1)
		self.entry_Ti.setValue(MicrocontrollerDef.Ti_DEFAULT)

		self.entry_RR = QDoubleSpinBox()
		self.entry_RR.setFont(self.font)
		self.entry_RR.setMinimum(5)
		self.entry_RR.setMaximum(60)
		self.entry_RR.setSingleStep(1)
		self.entry_RR.setValue(MicrocontrollerDef.RR_DEFAULT)

		self.entry_PEEP = QDoubleSpinBox()
		self.entry_PEEP.setFont(self.font)
		self.entry_PEEP.setMinimum(0)
		self.entry_PEEP.setMaximum(20)
		self.entry_PEEP.setSingleStep(0.5)
		self.entry_PEEP.setValue(MicrocontrollerDef.PEEP_DEFAULT)

		self.entry_Pi = QDoubleSpinBox()
		self.entry_Pi.setFont(self.font)
		self.entry_Pi.setMinimum(5)
		self.entry_Pi.setMaximum(50)
		self.entry_Pi.setSingleStep(1)
		self.entry_Pi.setValue(15)

		self.entry_RiseTime = QDoubleSpinBox()
		self.entry_RiseTime.setFont(self.font)
		self.entry_RiseTime.setMinimum(50)
		self.entry_RiseTime.setMaximum(500)
		self.entry_RiseTime.setSingleStep(1)
		self.entry_RiseTime.setValue(100)

		self.entry_Flow = QDoubleSpinBox()
		self.entry_Flow.setFont(self.font)
		self.entry_Flow.setMinimum(20)
		self.entry_Flow.setMaximum(100)
		self.entry_Flow.setSingleStep(5)
		self.entry_Flow.setValue(100)

		self.entry_PID_P = QDoubleSpinBox()
		self.entry_PID_P.setDecimals(5)
		self.entry_PID_P.setFont(self.font)
		self.entry_PID_P.setMinimum(0.000001)
		self.entry_PID_P.setMaximum(0.1)
		self.entry_PID_P.setSingleStep(0.0001)
		self.entry_PID_P.setValue(0.01)

		self.entry_PID_I_frac = QDoubleSpinBox()
		self.entry_PID_I_frac.setDecimals(5)
		self.entry_PID_I_frac.setFont(self.font)
		self.entry_PID_I_frac.setMinimum(0.000001)
		self.entry_PID_I_frac.setMaximum(1)
		self.entry_PID_I_frac.setSingleStep(0.001)
		self.entry_PID_I_frac.setValue(0.1)

		self.entry_Flow = QDoubleSpinBox()
		self.entry_Flow.setFont(self.font)
		self.entry_Flow.setMinimum(20)
		self.entry_Flow.setMaximum(125)
		self.entry_Flow.setSingleStep(5)
		self.entry_Flow.setValue(100)

		self.entry_FlowDeceleratingSlope = QDoubleSpinBox()
		self.entry_FlowDeceleratingSlope.setFont(self.font)
		self.entry_FlowDeceleratingSlope.setMinimum(0)
		self.entry_FlowDeceleratingSlope.setMaximum(100)
		self.entry_FlowDeceleratingSlope.setSingleStep(5)
		self.entry_FlowDeceleratingSlope.setValue(0)

		grid_line0 = QGridLayout()
		grid_line0.addWidget(QLabel('VT (ml)',font=self.font), 0,0)
		grid_line0.addWidget(self.entry_VT, 0,1)

		grid_line1 = QGridLayout()
		grid_line1.addWidget(QLabel('Ti (s)',font=self.font), 0,0)
		grid_line1.addWidget(self.entry_Ti, 0,1)

		grid_line2 = QGridLayout()
		grid_line2.addWidget(QLabel('RR (/min)',font=self.font), 0,0)
		grid_line2.addWidget(self.entry_RR, 0,1)

		grid_line3 = QGridLayout()
		grid_line3.addWidget(QLabel('PEEP (cmH2O)',font=self.font), 0,0)
		grid_line3.addWidget(self.entry_PEEP, 0,1)

		grid_line4 = QGridLayout()
		grid_line4.addWidget(QLabel('Flow',font=self.font), 0,0)
		grid_line4.addWidget(self.entry_Flow, 0,1)

		grid_line5 = QGridLayout()
		grid_line5.addWidget(QLabel('FlowDec',font=self.font), 0,0)
		grid_line5.addWidget(self.entry_FlowDeceleratingSlope, 0,1)


		grid_line6 = QGridLayout()
		grid_line6.addWidget(QLabel('P_Insp',font=self.font), 0,0)
		grid_line6.addWidget(self.entry_Pi, 0,1)

		grid_line7 = QGridLayout()
		grid_line7.addWidget(QLabel('Rise Time (ms)',font=self.font), 0,0)
		grid_line7.addWidget(self.entry_RiseTime, 0,1)

		grid_line8 = QGridLayout()
		grid_line8.addWidget(QLabel('P Gain',font=self.font), 0,0)
		grid_line8.addWidget(self.entry_PID_P, 0,1)

		grid_line9 = QGridLayout()
		grid_line9.addWidget(QLabel('I Gain (frac)',font=self.font), 0,0)
		grid_line9.addWidget(self.entry_PID_I_frac, 0,1)


		self.grid = QGridLayout()
		self.grid.addLayout(grid_line0,0,0)
		self.grid.addLayout(grid_line1,0,1)
		self.grid.addLayout(grid_line2,0,2)
		self.grid.addLayout(grid_line3,0,3)
		self.grid.addLayout(grid_line5,0,5)

		self.grid.addLayout(grid_line6,1,0)
		self.grid.addLayout(grid_line7,1,1)
		self.grid.addLayout(grid_line8,1,2)
		self.grid.addLayout(grid_line9,1,3)
		self.grid.addLayout(grid_line4,1,4)

		self.setLayout(self.grid)

		self.entry_VT.valueChanged.connect(self.ventController.setVT)
		self.entry_Ti.valueChanged.connect(self.ventController.setTi)
		self.entry_RR.valueChanged.connect(self.ventController.setRR)
		self.entry_PEEP.valueChanged.connect(self.ventController.setPEEP)
		self.entry_Flow.valueChanged.connect(self.ventController.setFlow)
		# self.entry_FlowDeceleratingSlope.valueChanged.connect(self.ventController.setFlowDeceleratingSlope)
		self.entry_Pi.valueChanged.connect(self.ventController.setPinsp)
		self.entry_RiseTime.valueChanged.connect(self.ventController.setRiseTime)
		self.entry_PID_P.valueChanged.connect(self.ventController.setPID_P)
		self.entry_PID_I_frac.valueChanged.connect(self.ventController.setPID_I_frac)


# from Deepak
class PlotWidget(pg.GraphicsLayoutWidget):

	def __init__(self,title, color = 'w', parent=None):
		super().__init__(parent)
		#pg.setConfigOption('background', 'w')

		self.font = QFont()
		self.font.setPixelSize(25)

		self.title = title
		self.maxLen = int(1000*WAVEFORMS.DISPLAY_RANGE_S/WAVEFORMS.UPDATE_INTERVAL_MS)
		self.left_X_data = deque(maxlen = self.maxLen)
		self.left_Y_data = deque(maxlen = self.maxLen)
		self.right_Abs = []
		self.right_Ord = []
		self.right_X_data = deque(maxlen = self.maxLen)
		self.right_Y_data = deque(maxlen = self.maxLen)
		self.left_Abs = []
		self.left_Ord = []
		self.plot1 = self.addPlot(title = self.title + ' ' + PLOT_UNITS[self.title])
		self.plot1.setTitle(title = self.title + ' [' + PLOT_UNITS[self.title] + ']',size = '30pt')
		self.plot1.getAxis("bottom").tickFont = self.font
		self.plot1.getAxis("left").tickFont = self.font
		self.setBackground('w')
		# self.curve = self.plot1.plot(self.Abs, self.Ord, pen=pg.mkPen(color, width=3), fillLevel=-0.3, brush=(50,50,200,100))
		self.left_curve = self.plot1.plot(self.left_Abs, self.left_Ord, pen=pg.mkPen(color, width=3), fillLevel=-0.3, brush=(50,50,200,100))
		self.right_curve = self.plot1.plot(self.right_Abs, self.right_Ord, pen=pg.mkPen(color, width=3), brush=(50,50,200,100))
		self.left_curve.setClipToView(True)
		self.right_curve.setClipToView(True)
		# self.plot1.enableAutoRange('y',True)
		self.plot1.setXRange(min=0,max=WAVEFORMS.DISPLAY_RANGE_S)
		self.plot1.showGrid(x=True, y=True)
		self.ptr = 0
		self.cycleGap = WAVEFORMS.CYCLE_GAP
		#pg.setConfigOption('background', 'w')

	def update_plot(self, time_data, data):

		timestamp = time_data%WAVEFORMS.DISPLAY_RANGE_S
		
		# Wraparound condition
		if len(self.left_X_data) > 0 and timestamp < self.left_X_data[-1]:
			self.right_X_data = self.left_X_data
			self.right_Y_data = self.left_Y_data
			self.left_X_data = deque(maxlen = self.maxLen)
			self.left_Y_data = deque(maxlen = self.maxLen)

		# Add new data to the right end of the left waveform
		self.left_X_data.append(timestamp)        
		self.left_Y_data.append(data)

		# Remove overlapping samples by popping left from the right waveform.
		while (len(self.right_X_data) > 0 and len(self.left_X_data) + len(self.right_X_data) >= self.maxLen - self.cycleGap):
			self.right_X_data.popleft()
			self.right_Y_data.popleft()
		
		# Set the data 
		self.label = PLOT_UNITS[self.title]
		self.left_Abs = np.array(self.left_X_data)
		self.left_Ord = np.array(self.left_Y_data)
		self.right_Abs = np.array(self.right_X_data)
		self.right_Ord = np.array(self.right_Y_data)

		# Update the plot.
		if len(self.left_Abs):
			self.left_curve.setData(self.left_Abs-self.left_Abs[-1], self.left_Ord)
			self.left_curve.setPos(self.left_Abs[-1],0)
		if len(self.right_Abs):
			self.right_curve.setData(self.right_Abs-self.right_Abs[-1], self.right_Ord)
			self.right_curve.setPos(self.right_Abs[-1],0)

	def initialise_plot(self):
		self.left_X_data = deque(maxlen = self.maxLen)
		self.left_Y_data = deque(maxlen = self.maxLen)
		self.right_X_data = deque(maxlen = self.maxLen)
		self.right_Y_data = deque(maxlen = self.maxLen)
		self.left_Abs = []
		self.left_Ord = []
		self.right_Abs = []
		self.right_Ord = []
		self.left_curve.setData(self.left_Abs,self.left_Ord)
		self.right_curve.setData(self.right_Abs,self.right_Ord)


class WaveformDisplay(QFrame):

	def __init__(self, main=None, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.row_idx = 0
		self.plots  = []
		self.add_components()
		self.setFrameStyle(QFrame.Panel | QFrame.Raised)

	def add_components(self):

		plot_layout = QGridLayout()
		self.plot_grp_box = QGroupBox()
		r = 0
		for dict_item in constants.__PLOTS__:
			r = r+1
			plot = plot_4vent(dict_item['name'], dict_item['xrange'], dict_item['yrange'], size=constants.__SIZE__)
			plot.add_series(dict_item['name'], dict_item['callback'])
			#plot_layout.setVerticalSpacing(20)
			#plot_layout.setOriginCorner(50)
			plot_layout.addWidget(plot.get(),r+self.row_idx,0)
			self.plots.append(plot)
		self.plot_grp_box.setLayout(plot_layout)

		window_layout = QGridLayout()
		window_layout.addWidget(self.plot_grp_box,1,0,3,5)
		self.setLayout(window_layout)

		# self.plotWidgets = {key: PlotWidget(title = key, color = 'b') for key in PLOTS}
		# self.plotWidgets['Airway Pressure'].plot1.setYRange(min=0,max=50)
		# self.plotWidgets['Flow Rate'].plot1.setYRange(min=-100,max=100)
		# self.plotWidgets['Volume'].plot1.setYRange(min=0,max=600)

		# grid = QGridLayout() 
		# for ii, key in enumerate(PLOTS):
		# 	grid.addWidget(self.plotWidgets[key], ii, 0,1,2)
		# self.setLayout(grid)
		
	def plot_values(self):
		## Start a timer to rapidly update the plot in pw
		#print ("plot_values")
		for p in self.plots:
			p.run_series(constants.__SIZE__) # TODO Make sure the plots are of same size for now
		''' TODO Convert to a multi threaded application for these processing
			Not only for the plots but also for the labels
		'''
