import sys
import os

# Add parent directory to Python path dynamically
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(os.path.dirname(current_dir))
sys.path.append(os.path.dirname(current_dir))

from PyQt5 import QtWidgets
from robot_control import RobotControl

# Set QT environment variables
os.environ["QT_QPA_PLATFORM"] = "xcb"

if __name__ == "__main__":
    try:
        app = QtWidgets.QApplication(sys.argv)
        MainWindow = QtWidgets.QMainWindow()
        robot_control = RobotControl()
        MainWindow.show()
        sys.exit(app.exec_())
    except Exception as e:
        print(f"Error: {e}")
