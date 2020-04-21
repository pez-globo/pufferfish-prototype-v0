# set QT_API environment variable
import os
os.environ["QT_API"] = "pyqt5"
import qtpy

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *

# app specific libraries
import ventapp.control.gui as gui


def main():
    app = QApplication([])
    win = gui.VentDevGUI()
    win.show()
    app.exec_() #sys.exit(app.exec_())


if __name__ == "__main__":
    main()
