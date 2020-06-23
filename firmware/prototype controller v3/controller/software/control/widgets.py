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

class NavigationWidget(QFrame):
    def __init__(self, navigationController, main=None, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.navigationController = navigationController
        self.add_components()
        self.setFrameStyle(QFrame.Panel | QFrame.Raised)

    def add_components(self):
        self.label_Xpos = QLabel()
        self.label_Xpos.setNum(0)
        self.label_Xpos.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.entry_dX = QDoubleSpinBox()
        self.entry_dX.setMinimum(0) 
        self.entry_dX.setMaximum(2000) 
        self.entry_dX.setSingleStep(1)
        self.entry_dX.setValue(0)
        self.btn_moveX_forward = QPushButton('Forward')
        self.btn_moveX_forward.setDefault(False)
        self.btn_moveX_backward = QPushButton('Backward')
        self.btn_moveX_backward.setDefault(False)
        self.btn_close_valveX = QPushButton('close the valve')
        self.btn_close_valveX.setDefault(False)
        self.btn_setbias_valveX = QPushButton('set as bias')
        self.btn_setbias_valveX.setDefault(False)
        
        self.label_Ypos = QLabel()
        self.label_Ypos.setNum(0)
        self.label_Ypos.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.entry_dY = QDoubleSpinBox()
        self.entry_dY.setMinimum(0)
        self.entry_dY.setMaximum(2000)
        self.entry_dY.setSingleStep(1)
        self.entry_dY.setValue(0)
        self.btn_moveY_forward = QPushButton('Forward')
        self.btn_moveY_forward.setDefault(False)
        self.btn_moveY_backward = QPushButton('Backward')
        self.btn_moveY_backward.setDefault(False)
        self.btn_close_valveY = QPushButton('close the valve')
        self.btn_close_valveY.setDefault(False)
        self.btn_setbias_valveY = QPushButton('set as bias')
        self.btn_setbias_valveY.setDefault(False)

        self.label_Zpos = QLabel()
        self.label_Zpos.setNum(0)
        self.label_Zpos.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.entry_dZ = QDoubleSpinBox()
        self.entry_dZ.setMinimum(0) 
        self.entry_dZ.setMaximum(2000) 
        self.entry_dZ.setSingleStep(1)
        self.entry_dZ.setValue(0)
        self.btn_moveZ_forward = QPushButton('Forward')
        self.btn_moveZ_forward.setDefault(False)
        self.btn_moveZ_backward = QPushButton('Backward')
        self.btn_moveZ_backward.setDefault(False)
        self.btn_close_valveZ = QPushButton('close the valve')
        self.btn_close_valveZ.setDefault(False)
        self.btn_setbias_valveZ = QPushButton('set as bias')
        self.btn_setbias_valveZ.setDefault(False)
        
        grid_line0 = QGridLayout()
        grid_line0.addWidget(QLabel('X (mm)'), 0,0)
        grid_line0.addWidget(self.label_Xpos, 0,1)
        grid_line0.addWidget(self.entry_dX, 0,2)
        grid_line0.addWidget(self.btn_moveX_forward, 0,3)
        grid_line0.addWidget(self.btn_moveX_backward, 0,4)
        grid_line0.addWidget(self.btn_close_valveX, 0,5)
        grid_line0.addWidget(self.btn_setbias_valveX, 0,6)

        grid_line1 = QGridLayout()
        grid_line1.addWidget(QLabel('Y (mm)'), 0,0)
        grid_line1.addWidget(self.label_Ypos, 0,1)
        grid_line1.addWidget(self.entry_dY, 0,2)
        grid_line1.addWidget(self.btn_moveY_forward, 0,3)
        grid_line1.addWidget(self.btn_moveY_backward, 0,4)
        grid_line1.addWidget(self.btn_close_valveY, 0,5)
        grid_line1.addWidget(self.btn_setbias_valveY, 0,6)

        grid_line2 = QGridLayout()
        grid_line2.addWidget(QLabel('Z (um)'), 0,0)
        grid_line2.addWidget(self.label_Zpos, 0,1)
        grid_line2.addWidget(self.entry_dZ, 0,2)
        grid_line2.addWidget(self.btn_moveZ_forward, 0,3)
        grid_line2.addWidget(self.btn_moveZ_backward, 0,4)
        grid_line2.addWidget(self.btn_close_valveZ, 0,5)
        grid_line2.addWidget(self.btn_setbias_valveZ, 0,6)

        self.grid = QGridLayout()
        # self.grid.addLayout(grid_line0,0,0)
        # self.grid.addLayout(grid_line1,1,0)
        self.grid.addLayout(grid_line2,2,0)
        self.setLayout(self.grid)

        self.btn_moveX_forward.clicked.connect(self.move_x_forward)
        self.btn_moveX_backward.clicked.connect(self.move_x_backward)
        self.btn_moveY_forward.clicked.connect(self.move_y_forward)
        self.btn_moveY_backward.clicked.connect(self.move_y_backward)
        self.btn_moveZ_forward.clicked.connect(self.move_z_forward)
        self.btn_moveZ_backward.clicked.connect(self.move_z_backward)
        self.btn_close_valveX.clicked.connect(self.navigationController.close_x)
        self.btn_close_valveY.clicked.connect(self.navigationController.close_y)
        self.btn_close_valveZ.clicked.connect(self.navigationController.close_z)
        self.btn_setbias_valveX.clicked.connect(self.navigationController.setbias_x)
        self.btn_setbias_valveY.clicked.connect(self.navigationController.setbias_y)
        self.btn_setbias_valveZ.clicked.connect(self.navigationController.setbias_z)
        
    def move_x_forward(self):
        self.navigationController.move_x(self.entry_dX.value())
    def move_x_backward(self):
        self.navigationController.move_x(-self.entry_dX.value())
    def move_y_forward(self):
        self.navigationController.move_y(self.entry_dY.value())
    def move_y_backward(self):
        self.navigationController.move_y(-self.entry_dY.value())
    def move_z_forward(self):
        self.navigationController.move_z(self.entry_dZ.value())
    def move_z_backward(self):
        self.navigationController.move_z(-self.entry_dZ.value())


class ControlPanel(QFrame):

	signal_logging_onoff = Signal(bool,str)

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
		self.entry_VT.setValue(MCU.Vt_DEFAULT)

		self.entry_Ti = QDoubleSpinBox()
		self.entry_Ti.setFont(self.font)
		self.entry_Ti.setMinimum(0.3)
		self.entry_Ti.setMaximum(5)
		self.entry_Ti.setSingleStep(0.1)
		self.entry_Ti.setValue(MCU.Ti_DEFAULT)

		self.entry_RR = QDoubleSpinBox()
		self.entry_RR.setFont(self.font)
		self.entry_RR.setMinimum(5)
		self.entry_RR.setMaximum(60)
		self.entry_RR.setSingleStep(1)
		self.entry_RR.setValue(MCU.RR_DEFAULT)

		self.entry_PEEP = QDoubleSpinBox()
		self.entry_PEEP.setFont(self.font)
		self.entry_PEEP.setMinimum(0)
		self.entry_PEEP.setMaximum(30)
		self.entry_PEEP.setSingleStep(0.5)
		self.entry_PEEP.setValue(MCU.PEEP_DEFAULT)

		self.entry_Pi = QDoubleSpinBox()
		self.entry_Pi.setFont(self.font)
		self.entry_Pi.setMinimum(5)
		self.entry_Pi.setMaximum(50)
		self.entry_Pi.setSingleStep(1)
		self.entry_Pi.setValue(MCU.pinsp_DEFAULT)

		self.entry_RiseTime = QDoubleSpinBox()
		self.entry_RiseTime.setFont(self.font)
		self.entry_RiseTime.setMinimum(50)
		self.entry_RiseTime.setMaximum(500)
		self.entry_RiseTime.setSingleStep(1)
		self.entry_RiseTime.setValue(MCU.pc_rise_time_ms_DEFAULT)

		self.entry_Flow = QDoubleSpinBox()
		self.entry_Flow.setFont(self.font)
		self.entry_Flow.setMinimum(20)
		self.entry_Flow.setMaximum(100)
		self.entry_Flow.setSingleStep(5)
		self.entry_Flow.setValue(MCU.valve_pos_open_steps_DEFAULT_abs)

		self.entry_PID_P = QDoubleSpinBox()
		self.entry_PID_P.setDecimals(5)
		self.entry_PID_P.setFont(self.font)
		self.entry_PID_P.setMinimum(0.000001)
		self.entry_PID_P.setMaximum(0.1)
		self.entry_PID_P.setSingleStep(0.0001)
		self.entry_PID_P.setValue(MCU.P_default)

		self.entry_PID_I_frac = QDoubleSpinBox()
		self.entry_PID_I_frac.setDecimals(5)
		self.entry_PID_I_frac.setFont(self.font)
		self.entry_PID_I_frac.setMinimum(0.000001)
		self.entry_PID_I_frac.setMaximum(1)
		self.entry_PID_I_frac.setSingleStep(0.001)
		self.entry_PID_I_frac.setValue(MCU.I_frac_default)

		self.entry_FlowDeceleratingSlope = QDoubleSpinBox()
		self.entry_FlowDeceleratingSlope.setFont(self.font)
		self.entry_FlowDeceleratingSlope.setMinimum(0)
		self.entry_FlowDeceleratingSlope.setMaximum(100)
		self.entry_FlowDeceleratingSlope.setSingleStep(5)
		self.entry_FlowDeceleratingSlope.setValue(0)

		self.entry_TriggerTh = QDoubleSpinBox()
		self.entry_TriggerTh.setFont(self.font)
		self.entry_TriggerTh.setMinimum(-5)
		self.entry_TriggerTh.setMaximum(0)
		self.entry_TriggerTh.setSingleStep(0.1)
		self.entry_TriggerTh.setValue(MCU.paw_trigger_th_DEFAULT)

		self.mode = MODE_PC_AC
		self.dropdown_modeManu = QComboBox()
		self.dropdown_modeManu.addItems([MODE_VC_AC_STRING,MODE_PC_AC_STRING,MODE_PSV_STRING])
		self.dropdown_modeManu.setCurrentText(MODE_PC_AC_STRING)

		self.btn_onoff = QPushButton('On/Off')
		self.btn_onoff.setDefault(False)
		self.btn_onoff.setCheckable(True)
		self.btn_onoff.setChecked(False)

		self.entry_Exhalation_Control_P_RiseTime = QDoubleSpinBox()
		self.entry_Exhalation_Control_P_RiseTime.setFont(self.font)
		self.entry_Exhalation_Control_P_RiseTime.setMinimum(0)
		self.entry_Exhalation_Control_P_RiseTime.setMaximum(1000)
		self.entry_Exhalation_Control_P_RiseTime.setSingleStep(1)
		self.entry_Exhalation_Control_P_RiseTime.setValue(MCU.rise_time_ms_exhalation_control_DEFAULT)

		self.lineEdit_experimentID = QLineEdit()

		self.btn_logging_onoff = QPushButton('Logging On/Off')
		self.btn_logging_onoff.setDefault(True)
		self.btn_logging_onoff.setCheckable(True)
		self.btn_logging_onoff.setChecked(False)

		# self.label_print = QLabel()
		# self.label_print.setFrameStyle(QFrame.Panel | QFrame.Sunken)

		grid_line0 = QHBoxLayout()
		grid_line0.addWidget(QLabel('Ti (s)',font=self.font))
		grid_line0.addWidget(self.entry_Ti)
		grid_line0.addWidget(QLabel('RR (/min)',font=self.font))
		grid_line0.addWidget(self.entry_RR)
		grid_line0.addWidget(QLabel('PEEP (cmH2O)',font=self.font))
		grid_line0.addWidget(self.entry_PEEP)
		grid_line0.addWidget(QLabel('Flow',font=self.font))
		grid_line0.addWidget(self.entry_Flow)
		grid_line0.addWidget(QLabel('Trigger th (cmH2O)',font=self.font))
		grid_line0.addWidget(self.entry_TriggerTh)

		grid_line1 = QHBoxLayout()
		grid_line1.addWidget(QLabel('P_Insp',font=self.font))
		grid_line1.addWidget(self.entry_Pi)
		grid_line1.addWidget(QLabel('Rise Time (ms)',font=self.font))
		grid_line1.addWidget(self.entry_RiseTime)
		grid_line1.addWidget(QLabel('P Gain',font=self.font))
		grid_line1.addWidget(self.entry_PID_P)
		grid_line1.addWidget(QLabel('I Gain (frac)',font=self.font))
		grid_line1.addWidget(self.entry_PID_I_frac)
		grid_line1.addWidget(QLabel('VT (ml)',font=self.font))
		grid_line1.addWidget(self.entry_VT)

		grid_line2 = QHBoxLayout()
		grid_line2.addWidget(self.dropdown_modeManu)
		grid_line2.addWidget(self.btn_onoff)
		grid_line2.addWidget(QLabel('exhalation control pressure rise time (set 0 for open loop control)'))
		grid_line2.addWidget(self.entry_Exhalation_Control_P_RiseTime)
		grid_line2.addWidget(QLabel('File Prefix'))
		grid_line2.addWidget(self.lineEdit_experimentID)
		grid_line2.addWidget(self.btn_logging_onoff)

		# grid_line11 = QGridLayout()
		# grid_line11.addWidget(self.label_print,0,0,10,0)

		# for displaying stepper position and flow/pressure measurements
		self.label_stepper_pos = QLabel()
		self.label_stepper_pos.setFrameStyle(QFrame.Panel | QFrame.Sunken)
		self.label_stepper_pos.setFixedWidth(50)
		self.label_flow_air = QLabel()
		self.label_flow_air.setFrameStyle(QFrame.Panel | QFrame.Sunken)
		self.label_flow_air.setFixedWidth(50)
		self.label_flow_oxygen = QLabel()
		self.label_flow_oxygen.setFrameStyle(QFrame.Panel | QFrame.Sunken)
		self.label_flow_oxygen.setFixedWidth(50)
		self.label_flow_proximal = QLabel()
		self.label_flow_proximal.setFrameStyle(QFrame.Panel | QFrame.Sunken)
		self.label_flow_proximal.setFixedWidth(50)
		self.label_p_exhalation_control = QLabel()
		self.label_p_exhalation_control.setFrameStyle(QFrame.Panel | QFrame.Sunken)
		self.label_p_exhalation_control.setFixedWidth(50)
		self.label_p_airway = QLabel()
		self.label_p_airway.setFrameStyle(QFrame.Panel | QFrame.Sunken)
		self.label_p_airway.setFixedWidth(50)
		self.label_p_aux = QLabel()
		self.label_p_aux.setFrameStyle(QFrame.Panel | QFrame.Sunken)
		self.label_p_aux.setFixedWidth(50)
		self.label_dP = QLabel()
		self.label_dP.setFrameStyle(QFrame.Panel | QFrame.Sunken)
		self.label_dP.setFixedWidth(50)
		# self.label_print = QLabel()
		# self.label_print.setFrameStyle(QFrame.Panel | QFrame.Sunken)

		grid_line3 = QHBoxLayout()
		grid_line3.addWidget(QLabel('stepper pos'))
		grid_line3.addWidget(self.label_stepper_pos)
		grid_line3.addWidget(QLabel('flow_air'))
		grid_line3.addWidget(self.label_flow_air)
		grid_line3.addWidget(QLabel('flow_oxygen'))
		grid_line3.addWidget(self.label_flow_oxygen)
		grid_line3.addWidget(QLabel('flow_proximal'))
		grid_line3.addWidget(self.label_flow_proximal)
		grid_line3.addWidget(QLabel('p exhalation control'))
		grid_line3.addWidget(self.label_p_exhalation_control)
		grid_line3.addWidget(QLabel('p airway'))
		grid_line3.addWidget(self.label_p_airway)
		grid_line3.addWidget(QLabel('p aux'))
		grid_line3.addWidget(self.label_p_aux)
		grid_line3.addWidget(QLabel('dP'))
		grid_line3.addWidget(self.label_dP)

		self.grid = QGridLayout()
		self.grid.addLayout(grid_line0,0,0)
		self.grid.addLayout(grid_line1,1,0)
		self.grid.addLayout(grid_line2,2,0)
		self.grid.addLayout(grid_line3,3,0)
		# self.grid.addWidget(self.label_print,3,0,1,8)

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
		self.entry_TriggerTh.valueChanged.connect(self.ventController.setTriggerTh)

		self.btn_onoff.clicked.connect(self.ventController.setONOFF)
		self.dropdown_modeManu.currentTextChanged.connect(self.ventController.updateMode)
		self.entry_Exhalation_Control_P_RiseTime.valueChanged.connect(self.ventController.setExhalationControlPRiseTime)

		self.btn_logging_onoff.clicked.connect(self.logging_onoff)

	def logging_onoff(self,state):
		self.signal_logging_onoff.emit(state,self.lineEdit_experimentID.text())
		

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
