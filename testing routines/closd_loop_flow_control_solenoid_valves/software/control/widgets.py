# set QT_API environment variable
import os 
os.environ["QT_API"] = "pyqt5"
import qtpy

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *

from control._def import *

class NavigationWidget(QFrame):
    def __init__(self, navigationController, main=None, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.navigationController = navigationController
        self.add_components()
        self.setFrameStyle(QFrame.Panel | QFrame.Raised)

    def add_components(self):
        self.label_PWM = QLabel()
        self.label_PWM.setNum(0)
        self.label_PWM.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        
        self.entry_PWM_start = QDoubleSpinBox()
        self.entry_PWM_start.setMinimum(0) 
        self.entry_PWM_start.setMaximum(10000) 
        self.entry_PWM_start.setSingleStep(0.1)
        self.entry_PWM_start.setValue(0)

        self.entry_PWM_end = QDoubleSpinBox()
        self.entry_PWM_end.setMinimum(0) 
        self.entry_PWM_end.setMaximum(10000) 
        self.entry_PWM_end.setSingleStep(0.1)
        self.entry_PWM_end.setValue(0)

        self.entry_PWM_step_increase = QDoubleSpinBox()
        self.entry_PWM_step_increase.setMinimum(0) 
        self.entry_PWM_step_increase.setMaximum(10000) 
        self.entry_PWM_step_increase.setSingleStep(0.1)
        self.entry_PWM_step_increase.setValue(0)

        self.entry_PWM_step_decrease = QDoubleSpinBox()
        self.entry_PWM_step_decrease.setMinimum(0) 
        self.entry_PWM_step_decrease.setMaximum(10000) 
        self.entry_PWM_step_decrease.setSingleStep(0.1)
        self.entry_PWM_step_decrease.setValue(0)

        self.entry_PWM_max_hold = QSpinBox()
        self.entry_PWM_max_hold.setMinimum(0) 
        self.entry_PWM_max_hold.setMaximum(10000) 
        self.entry_PWM_max_hold.setSingleStep(1)
        self.entry_PWM_max_hold.setValue(0)

        self.entry_PWM_min_hold = QSpinBox()
        self.entry_PWM_min_hold.setMinimum(0) 
        self.entry_PWM_min_hold.setMaximum(10000) 
        self.entry_PWM_min_hold.setSingleStep(1)
        self.entry_PWM_min_hold.setValue(0)

        self.entry_N = QSpinBox()
        self.entry_N.setMinimum(0) 
        self.entry_N.setMaximum(10000) 
        self.entry_N.setSingleStep(1)
        self.entry_N.setValue(0)

        self.updatePWM = QPushButton('update PWM')
        self.updatePWM.setDefault(False)
        self.cyclePWM = QPushButton('cycle the valve')
        self.cyclePWM.setDefault(False)
        
        grid_line0 = QHBoxLayout()
        grid_line0.addWidget(QLabel('PWM'))
        grid_line0.addWidget(self.label_PWM)
        grid_line0.addWidget(QLabel('PWM start'))
        grid_line0.addWidget(self.entry_PWM_start)
        grid_line0.addWidget(QLabel('PWM end'))
        grid_line0.addWidget(self.entry_PWM_end)
        grid_line0.addWidget(QLabel('PWM + step'))
        grid_line0.addWidget(self.entry_PWM_step_increase)
        grid_line0.addWidget(QLabel('PWM - step'))
        grid_line0.addWidget(self.entry_PWM_step_decrease)
        grid_line0.addWidget(QLabel('PWM max hold'))
        grid_line0.addWidget(self.entry_PWM_max_hold)
        grid_line0.addWidget(QLabel('PWM min hold'))
        grid_line0.addWidget(self.entry_PWM_min_hold)
        grid_line0.addWidget(QLabel('N'))
        grid_line0.addWidget(self.entry_N)

        grid_line0.addWidget(self.updatePWM)
        grid_line0.addWidget(self.cyclePWM)

        self.grid = QGridLayout()
        self.grid.addLayout(grid_line0,0,0)
        self.setLayout(self.grid)

        self.updatePWM.clicked.connect(self.update_PWM)
        self.cyclePWM.clicked.connect(self.cycle_PWM)
        
    def update_PWM(self):
        cmd = bytearray(MicrocontrollerDef.CMD_LENGTH)
        tmp = (float(self.entry_PWM_start.value())/FLOW_FS)*65535
        cmd[0] = int(tmp/256)
        cmd[1] = int(tmp%256)
        self.navigationController.update_PWM(cmd)

    def cycle_PWM(self):
        cmd = bytearray(MicrocontrollerDef.CMD_LENGTH)
        tmp = (float(self.entry_PWM_start.value())/FLOW_FS)*65535
        cmd[0] = int(tmp/256)
        cmd[1] = int(tmp%256)
        tmp = (float(self.entry_PWM_end.value())/FLOW_FS)*65535
        cmd[2] = int(tmp/256)
        cmd[3] = int(tmp%256)
        tmp = (float(self.entry_PWM_step_increase.value())/FLOW_FS)*65535
        cmd[4] = int(tmp/256)
        cmd[5] = int(tmp%256)
        tmp = (float(self.entry_PWM_step_decrease.value())/FLOW_FS)*65535
        cmd[6] = int(tmp/256)
        cmd[7] = int(tmp%256)
        tmp = self.entry_PWM_max_hold.value()
        cmd[8] = int(tmp/256)
        cmd[9] = int(tmp%256)
        tmp = self.entry_PWM_min_hold.value()
        cmd[10] = int(tmp/256)
        cmd[11] = int(tmp%256)
        cmd[12] = int(self.entry_N.value()>>24)
        cmd[13] = int(self.entry_N.value()>>16)
        cmd[14] = int(self.entry_N.value()>>8)
        cmd[15] = int(self.entry_N.value()%256)
        filename = str(self.entry_PWM_start.value()) + '_' + str(self.entry_PWM_end.value()) + '_' + str(self.entry_PWM_step_increase.value()) + '_' + str(self.entry_PWM_step_decrease.value()) + '_' +str(self.entry_PWM_max_hold.value()) + '_' + str(self.entry_PWM_min_hold.value()) + '_' + str(self.entry_N.value())
        self.navigationController.cycle_PWM(cmd,filename)
