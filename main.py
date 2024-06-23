
# -*- coding: utf-8 -*-
import sys

from serial_frame import SerialFrame
from PyQt5.QtWidgets import QApplication,QMainWindow

if __name__ == "__main__":
    app = QApplication(sys.argv)

    m = SerialFrame()
    m.mainwindow.show()
    app.exec_()
