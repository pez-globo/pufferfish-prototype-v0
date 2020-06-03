# set QT_API environment variable
import os 
os.environ["QT_API"] = "pyqt5"
import qtpy

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *

# app specific libraries
import gui as gui
import pyqtgraph                           as     pg

pg.setConfigOption('background', pg.mkColor(186, 227, 230))
pg.setConfigOption('foreground', 'k')

if __name__ == "__main__":

    app = QApplication([])
    win = gui.VentDevGUI()
    win.show()
    app.exec_() #sys.exit(app.exec_())
