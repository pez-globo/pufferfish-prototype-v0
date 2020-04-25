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
from control._def import *

class VentDevGUI(QMainWindow):

	# variables
	fps_software_trigger = 100

	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)

		# load objects
		if SIMULATION:
			self.microcontroller = microcontroller.Microcontroller_Simulation()
		else:
			self.microcontroller = microcontroller.Microcontroller()
		self.stepperMotorController = core.ValveController(self.microcontroller)
		self.ventController = core.VentController(self.microcontroller)
		self.waveforms = core.Waveforms(self.microcontroller,self.ventController)
		
		# load widgets
		self.navigationWidget = widgets.stepperMotorWidget(self.stepperMotorController)
		self.waveformDisplay = widgets.WaveformDisplay()
		self.controlPanel = widgets.ControlPanel(self.ventController)

		# layout widgets
		layout = QGridLayout() #layout = QStackedLayout()
		# layout.addWidget(self.navigationWidget,0,0)
		layout.addWidget(self.waveformDisplay,0,0)
		layout.addWidget(self.controlPanel,1,0)

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
		self.waveforms.close()
		event.accept()