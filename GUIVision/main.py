
from PyQt5 import QtWidgets
import sys
from gui_main import RobotArmGUI

def main():
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    gui = RobotArmGUI()
    MainWindow.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()