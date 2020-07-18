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
		self.navigationController = core.NavigationController(self.microcontroller)
		self.ventController = core.VentController(self.microcontroller)
		self.waveforms = core.Waveforms(self.microcontroller,self.ventController)
		
		# load widgets
		self.navigationWidget = widgets.NavigationWidget(self.navigationController)
		self.waveformDisplay = widgets.WaveformDisplay()
		self.controlPanel = widgets.ControlPanel(self.ventController)

		# layout widgets
		layout = QGridLayout() #layout = QStackedLayout()
		# layout.addWidget(self.navigationWidget,0,0)
		layout.addWidget(self.waveformDisplay,0,0)
		layout.addWidget(self.controlPanel,1,0)
		layout.addWidget(self.navigationWidget,2,0)

		# transfer the layout to the central widget
		self.centralWidget = QWidget()
		self.centralWidget.setLayout(layout)
		self.setCentralWidget(self.centralWidget)

		# make connections
		self.controlPanel.signal_logging_onoff.connect(self.waveforms.logging_onoff)
		self.waveforms.signal_Paw.connect(self.waveformDisplay.plotWidgets['Airway Pressure'].update_plot)
		self.waveforms.signal_Flow.connect(self.waveformDisplay.plotWidgets['Flow Rate'].update_plot)
		self.waveforms.signal_Volume.connect(self.waveformDisplay.plotWidgets['Volume'].update_plot)
		# self.waveforms.signal_print.connect(self.controlPanel.label_print.setText)
		self.waveforms.signal_stepper_pos_air.connect(self.controlPanel.label_stepper_pos_air.setText)
		self.waveforms.signal_stepper_pos_oxygen.connect(self.controlPanel.label_stepper_pos_oxygen.setText)
		self.waveforms.signal_flow_air.connect(self.controlPanel.label_flow_air.setText)
		self.waveforms.signal_flow_proximal.connect(self.controlPanel.label_flow_proximal.setText)
		self.waveforms.signal_p_exhalation_control.connect(self.controlPanel.label_p_exhalation_control.setText)
		self.waveforms.signal_p_airway.connect(self.controlPanel.label_p_airway.setText)
		self.waveforms.signal_p_aux.connect(self.controlPanel.label_p_aux.setText)
		self.waveforms.signal_dP.connect(self.controlPanel.label_dP.setText)
		self.waveforms.signal_p_supply_air.connect(self.controlPanel.label_p_supply_air.setText)
		self.waveforms.signal_p_supply_oxygen.connect(self.controlPanel.label_p_supply_oxygen.setText)
		self.waveforms.signal_fio2.connect(self.controlPanel.label_fio2.setText)
		self.waveforms.signal_flow_oxygen.connect(self.controlPanel.label_flow_oxygen.setText)
		self.waveforms.signal_vt_internal.connect(self.controlPanel.label_Vt_internal.setText)

	def closeEvent(self, event):
		self.waveforms.close()
		event.accept()