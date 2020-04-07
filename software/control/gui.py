# set QT_API environment variable
import os 
os.environ["QT_API"] = "pyqt5"
import qtpy

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *

# app specific libraries
import control.widgets as widgets
import control.core as core
import control.microcontroller as microcontroller

class VentDevGUI(QMainWindow):

	# variables
	fps_software_trigger = 100

	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)

		# load objects
		# self.microcontroller = microcontroller.Microcontroller_Simulation()
		self.microcontroller = microcontroller.Microcontroller()
		self.stepperMotorController = core.ValveController(self.microcontroller)
		self.waveforms = core.Waveforms(self.microcontroller)

		# load widgets
		self.navigationWidget = widgets.stepperMotorWidget(self.stepperMotorController)
		self.waveformDisplay = widgets.WaveformDisplay()
		
		# layout widgets
		layout = QGridLayout() #layout = QStackedLayout()
		# layout.addWidget(self.navigationWidget,0,0)
		layout.addWidget(self.waveformDisplay,1,0)

		# transfer the layout to the central widget
		self.centralWidget = QWidget()
		self.centralWidget.setLayout(layout)
		self.setCentralWidget(self.centralWidget)

		# make connections
		self.stepperMotorController.xPos.connect(self.navigationWidget.label_Xpos.setNum)
		self.stepperMotorController.yPos.connect(self.navigationWidget.label_Ypos.setNum)
		self.waveforms.signal_Paw.connect(self.waveformDisplay.plotWidgets['Airway Pressure'].update_plot)
		self.waveforms.signal_Flow.connect(self.waveformDisplay.plotWidgets['Flow Rate'].update_plot)
		self.waveforms.signal_Volume.connect(self.waveformDisplay.plotWidgets['Volume'].update_plot)


	def closeEvent(self, event):
		event.accept()