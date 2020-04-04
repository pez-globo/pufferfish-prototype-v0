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
		self.microcontroller = microcontroller.Microcontroller_Simulation()
		self.stepperMotorController = core.ValveController(self.microcontroller)
		self.waveforms = core.Waveforms(self.microcontroller)

		# load widgets
		self.navigationWidget = widgets.stepperMotorWidget(self.stepperMotorController)
		
		# layout widgets
		layout = QGridLayout() #layout = QStackedLayout()
		layout.addWidget(self.navigationWidget,2,0)

		# transfer the layout to the central widget
		self.centralWidget = QWidget()
		self.centralWidget.setLayout(layout)
		self.setCentralWidget(self.centralWidget)

		# make connections
		self.stepperMotorController.xPos.connect(self.navigationWidget.label_Xpos.setNum)
		self.stepperMotorController.yPos.connect(self.navigationWidget.label_Ypos.setNum)

	def closeEvent(self, event):
		event.accept()