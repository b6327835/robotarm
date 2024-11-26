
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt
from ros_gui import Ui_MainWindow
from video_thread import VideoThread
from ros_operations import MoveRobotThread

class RobotArmGUI(Ui_MainWindow):
    def __init__(self) -> None:
        super().setupUi(MainWindow)
        self.gnc()
        # ...existing code...
        # (Keep the GUI initialization and methods)

    # ...existing code...
    # (Keep all the GUI-related methods)