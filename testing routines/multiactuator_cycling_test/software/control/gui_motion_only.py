# set QT_API environment variable
import os 
os.environ["QT_API"] = "pyqt5"
import qtpy

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *

from control._def import *


# app specific libraries
import control.widgets as widgets
import control.core as core
import control.microcontroller as microcontroller

class OctopiGUI(QMainWindow):

	# variables
	fps_software_trigger = 100

	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)

		# load objects
		self.microcontroller = microcontroller.Microcontroller()
		self.navigationController = core.NavigationController(self.microcontroller)

		# load widgets
		self.navigationWidget = widgets.NavigationWidget(self.navigationController)
		self.DataDisplayWidget = widgets.DataDisplayWidget()

		
		# layout widgets
		layout = QGridLayout() #layout = QStackedLayout()
		layout.addWidget(self.navigationWidget,0,0)
		layout.addWidget(self.DataDisplayWidget,0,1)

		# transfer the layout to the central widget
		self.centralWidget = QWidget()
		self.centralWidget.setLayout(layout)
		self.setCentralWidget(self.centralWidget)

		# make connections
		self.navigationController.ValvePositions.connect(self.navigationWidget.set_position_labels)

		self.navigationController.ValveCycles.connect(self.DataDisplayWidget.set_cycle_labels)

		self.navigationController.ValveTemperatures.connect(self.DataDisplayWidget.set_temperature_labels)




	def closeEvent(self, event):
		event.accept()
		# self.navigationController.home()