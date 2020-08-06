# set QT_API environment variable
import os 
os.environ["QT_API"] = "pyqt5"
import qtpy

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *

from control._def import *

import numpy as np
import pyqtgraph as pg
from collections import deque
import time

class ControlPanel(QFrame):

	signal_logging_onoff = Signal(bool,str)

	def __init__(self, main=None, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.font = QFont()
		self.font.setPixelSize(16)
		self.add_components()
		self.setFrameStyle(QFrame.Panel | QFrame.Raised)

	def add_components(self):

		self.lineEdit_experimentID = QLineEdit()

		self.btn_logging_onoff = QPushButton('Logging On/Off')
		self.btn_logging_onoff.setDefault(False)
		self.btn_logging_onoff.setCheckable(True)
		self.btn_logging_onoff.setChecked(True)

		# self.label_print = QLabel()
		# self.label_print.setFrameStyle(QFrame.Panel | QFrame.Sunken)

		grid_line2 = QHBoxLayout()
		grid_line2.addWidget(QLabel('File Prefix'))
		grid_line2.addWidget(self.lineEdit_experimentID)
		grid_line2.addWidget(self.btn_logging_onoff)

		# grid_line11 = QGridLayout()
		# grid_line11.addWidget(self.label_print,0,0,10,0)

		# for displaying stepper position and flow/pressure measurements
		self.label_flow_1 = QLabel()
		self.label_flow_1.setFrameStyle(QFrame.Panel | QFrame.Sunken)
		self.label_flow_1.setFixedWidth(50)
		self.label_p_airway = QLabel()
		self.label_p_airway.setFrameStyle(QFrame.Panel | QFrame.Sunken)
		self.label_p_airway.setFixedWidth(50)

		self.label_fio2 = QLabel()
		self.label_fio2.setFrameStyle(QFrame.Panel | QFrame.Sunken)
		self.label_fio2.setFixedWidth(50)

		# self.label_print = QLabel()
		# self.label_print.setFrameStyle(QFrame.Panel | QFrame.Sunken)

		grid_line3 = QHBoxLayout()
		grid_line3.addWidget(QLabel('flow_1 (slm)'))
		grid_line3.addWidget(self.label_flow_1)
		grid_line3.addWidget(QLabel('p airway (cmH2O)'))
		grid_line3.addWidget(self.label_p_airway)
		
		self.grid = QGridLayout()
		self.grid.addLayout(grid_line2,2,0)
		self.grid.addLayout(grid_line3,3,0)
		# self.grid.addWidget(self.label_print,3,0,1,8)

		self.setLayout(self.grid)
		self.btn_logging_onoff.clicked.connect(self.logging_onoff)

	def logging_onoff(self,state):
		self.signal_logging_onoff.emit(state,self.lineEdit_experimentID.text())
		

# from Deepak
class PlotWidget(pg.GraphicsLayoutWidget):

	def __init__(self,title, color = 'w', parent=None):
		super().__init__(parent)
		#pg.setConfigOption('background', 'w')

		self.font = QFont()
		self.font.setPixelSize(16)

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
		self.plot1.setTitle(title = self.title + ' [' + PLOT_UNITS[self.title] + ']',size = '25pt')
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
		self.CYCLE_GAP = WAVEFORMS.CYCLE_GAP
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
		while (len(self.right_X_data) > 0 and len(self.left_X_data) + len(self.right_X_data) >= self.maxLen - self.CYCLE_GAP):
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
		self.add_components()
		self.setFrameStyle(QFrame.Panel | QFrame.Raised)

	def add_components(self):
		self.plotWidgets = {key: PlotWidget(title = key, color = 'b') for key in PLOTS}
		# self.plotWidgets['Airway Pressure'].plot1.setYRange(min=WAVEFORMS.PAW_MIN,max=WAVEFORMS.PAW_MAX)
		# self.plotWidgets['Flow Rate'].plot1.setYRange(min=WAVEFORMS.FLOW_MIN,max=WAVEFORMS.FLOW_MAX)
		# self.plotWidgets['Volume'].plot1.setYRange(min=WAVEFORMS.V_MIN,max=WAVEFORMS.V_MAX)

		grid = QGridLayout() 
		for ii, key in enumerate(PLOTS):
			grid.addWidget(self.plotWidgets[key], ii, 0,1,2)
		self.setLayout(grid)
