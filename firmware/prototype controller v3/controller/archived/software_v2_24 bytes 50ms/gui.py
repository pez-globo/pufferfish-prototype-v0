# set QT_API environment variable
import os 
os.environ["QT_API"] = "pyqt5"
import qtpy
import numpy           as     np
# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *

# app specific libraries
import widgets as widgets
import core as core
import constants as constants
import microcontroller as microcontroller
from _def import *
from utils import *

class VentDevGUI(QMainWindow):

	# variables
	fps_software_trigger = 100

	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)
        
		super().setStyleSheet(open(css_file()).read())
		# load objects
		if SIMULATION:
			self.microcontroller = microcontroller.Microcontroller_Simulation()
		else:
			self.microcontroller = microcontroller.Microcontroller()
		self.stepperMotorController = core.ValveController(self.microcontroller)
		self.ventController = core.VentController(self.microcontroller)
		self.waveforms = core.Waveforms(self.microcontroller,self.ventController, size=constants.__SIZE__)

		self.gui_plots = [{ 'name': 'Paw ', 'callback' : self.plot_Paw, 'xrange':constants.__XRANGE__, 'yrange':constants.__YRANGE_Paw__ },
				{ 'name': 'Flow L/min', 'callback' : self.plot_Flow, 'xrange':constants.__XRANGE__, 'yrange':constants.__YRANGE_Flow__ },
				{ 'name': 'Volume mL', 'callback' : self.plot_Volume, 'xrange':constants.__XRANGE__, 'yrange':constants.__YRANGE_V__ },]
		# load widgets
		self.navigationWidget = widgets.stepperMotorWidget(self.stepperMotorController)
		self.waveformDisplay = widgets.WaveformDisplay()
		self.waveformDisplay.add_components(gui_plots=self.gui_plots)
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
		self.waveforms.signal_updatePlot.connect(self.waveformDisplay.plot_values)
		# self.waveforms.signal_updatePlot_withValues.connect(self.waveformDisplay.plot_receiced_values)
		# self.waveforms.signal_Paw.connect(self.waveformDisplay.plotWidgets['Airway Pressure'].update_plot)
		# self.waveforms.signal_Flow.connect(self.waveformDisplay.plotWidgets['Flow Rate'].update_plot)
		# self.waveforms.signal_Volume.connect(self.waveformDisplay.plotWidgets['Volume'].update_plot)

	def plot_Paw(self, size, counter):
		''' returns a scalar value of data for counter range '''
		#x = np.linspace(constants.__XRANGE__[0], constants.__XRANGE__[1], size)
		# We should block here if the values are not yet read
		y = self.waveforms.Paw[counter]
		return y, counter

	def plot_Flow(self, size, counter):
		''' returns a scalar value of data for counter range '''
		#x = np.linspace(constants.__XRANGE__[0], constants.__XRANGE__[1], size)
		# We should block here if the values are not yet read
		y = self.waveforms.Flow[counter]
		return y, counter
		
	def plot_Volume(self, size, counter):
		''' returns a scalar value of data for counter range '''
		#x = np.linspace(constants.__XRANGE__[0], constants.__XRANGE__[1], size)
		# We should block here if the values are not yet read
		y = self.waveforms.Volume[counter]
		return y, counter

	def closeEvent(self, event):
		self.waveforms.close()
		event.accept()
