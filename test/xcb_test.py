import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread

app = QtWidgets.QApplication(sys.argv)
MainWindow = QtWidgets.QMainWindow()

from serial import Serial
from time import sleep, ctime
import cv2
import numpy as np
from ultralytics import YOLO
import logging
from UI_TESTTING import Ui_MainWindow


# Set up logging
logging.basicConfig(filename='robot_control.log', level=logging.INFO,
                    format='%(asctime)s:%(levelname)s:%(message)s')

class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)
    object_detected_signal = pyqtSignal(int, int, int, float)  # x, y, priority, confidence

    def __init__(self):
        super().__init__()
        self.running = True

    def run(self):
        model = YOLO('yolov8n.pt')
        cap = cv2.VideoCapture(0)

        if not cap.isOpened():
            logging.error("Error: Could not open webcam.")
            return

        x_limit, y_limit = 300, 200
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, x_limit)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, y_limit)

        while self.running:
            ret, frame = cap.read()
            if not ret:
                logging.error("Error: Failed to grab frame.")
                break

            cv_img = cv2.resize(cv_img, (x_limit, y_limit))
            results = model(cv_img)

            detected_objects = []
            for result in results:
                for box in result.boxes:
                    conf = box.conf[0].item() * 100
                    if conf > 60:
                        x1, y1, x2, y2 = box.xyxy[0].int().tolist()
                        target_x, target_y = (x1 + x2) // 2, (y1 + y2) // 2
                        detected_objects.append((x1, y1, x2, y2, target_x, target_y, conf))

            detected_objects.sort(key=lambda obj: obj[0])

            for priority, (x1, y1, x2, y2, target_x, target_y, conf) in enumerate(detected_objects, start=1):
                cv2.circle(cv_img, (target_x, target_y), 5, (0, 0, 255), -1)
                cv2.putText(cv_img, f"P:{priority},X:{target_x},Y:{target_y}", (target_x - 10, target_y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Emit signal with object information
                self.object_detected_signal.emit(target_x, target_y, priority, conf)

            self.change_pixmap_signal.emit(cv_img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()

    def stop(self):
        self.running = False
        self.wait()

class RobotControl:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0

    def move(self, axis, value):
        if axis == 'x':
            self.x = max(0, min(300, self.x + value))
        elif axis == 'y':
            self.y = max(0, min(200, self.y + value))
        elif axis == 'z':
            self.z = max(0, min(200, self.z + value))
        logging.info(f"Moved {axis} by {value}. New position: X:{self.x}, Y:{self.y}, Z:{self.z}")

    def set_position(self, x, y, z):
        self.x = max(0, min(300, x))
        self.y = max(0, min(200, y))
        self.z = max(0, min(200, z))
        logging.info(f"Set position to X:{self.x}, Y:{self.y}, Z:{self.z}")

    def get_position(self):
        return self.x, self.y, self.z

class MyClass(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().setupUi(MainWindow)
        self.robot = RobotControl()
        self.setup_connections()
        self.setup_video_thread()

    def setup_connections(self):
        self.mannualMode.clicked.connect(lambda: self.stackedWidget.setCurrentWidget(self.manpage))
        self.jogMode.clicked.connect(lambda: self.stackedWidget.setCurrentWidget(self.jogpage))
        self.VSMode.clicked.connect(lambda: self.stackedWidget.setCurrentWidget(self.visionpage))

        self.Initial_bottom.clicked.connect(self.initial_set)
        self.home_bottom.clicked.connect(self.home_set)
        self.start_bottom.clicked.connect(self.start_set)
        self.Gopoint.clicked.connect(self.vision_go)

        self.horizontalSlider_1.sliderReleased.connect(self.x_set)
        self.horizontalSlider_2.sliderReleased.connect(self.y_set)
        self.horizontalSlider_3.sliderReleased.connect(self.z_set)

        self.jogxu.clicked.connect(lambda: self.jog('x', 1))
        self.jogxd.clicked.connect(lambda: self.jog('x', -1))
        self.jogyu.clicked.connect(lambda: self.jog('y', 1))
        self.jogyd.clicked.connect(lambda: self.jog('y', -1))
        self.jogzu.clicked.connect(lambda: self.jog('z', 1))
        self.jogzd.clicked.connect(lambda: self.jog('z', -1))

        self.selectbot.clicked.connect(self.setting)
        self.Restsetbot.clicked.connect(self.reset)
        self.lineEdit.returnPressed.connect(self.take_val)

    def setup_video_thread(self):
        self.thread = VideoThread()
        self.thread.change_pixmap_signal.connect(self.update_image)
        self.thread.object_detected_signal.connect(self.handle_detected_object)
        self.thread.start()

    @pyqtSlot(np.ndarray)
    def update_image(self, cv_img):
        qt_img = self.convert_cv_qt(cv_img, 400, 200)
        self.Display.setPixmap(qt_img)

    def convert_cv_qt(self, cv_img, w, h):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        return QPixmap.fromImage(convert_to_Qt_format.scaled(w, h, Qt.KeepAspectRatio))

    @pyqtSlot(int, int, int, float)
    def handle_detected_object(self, x, y, priority, confidence):
        logging.info(f"Detected object - Priority: {priority}, X: {x}, Y: {y}, Confidence: {confidence:.2f}%")
        # Update UI elements with detected object information
        self.Vision_X.setNum(x)
        self.Vision_Y.setNum(y)
        self.Vision_Priority.setNum(priority)
        self.Vision_Confidence.setText(f"{confidence:.2f}%")

    def jog(self, axis, direction):
        try:
            step = int(self.lineEdit.text()) * direction
            self.robot.move(axis, step)
            self.update_position_display()
        except ValueError:
            logging.error("Invalid jog step value")
            QtWidgets.QMessageBox.warning(self, "Error", "Invalid jog step value")

    def update_position_display(self):
        x, y, z = self.robot.get_position()
        self.X_core_j.setNum(x)
        self.Y_core_j.setNum(y)
        self.Z_core_j.setNum(z)
        self.horizontalSlider_1.setValue(x)
        self.horizontalSlider_2.setValue(y)
        self.horizontalSlider_3.setValue(z)

    def initial_set(self):
        self.robot.set_position(0, 0, 0)
        self.update_position_display()

    def home_set(self):
        self.robot.set_position(150, 100, 100)
        self.update_position_display()

    def start_set(self):
        x = self.horizontalSlider_1.value()
        y = self.horizontalSlider_2.value()
        z = self.horizontalSlider_3.value()
        self.robot.set_position(x, y, z)
        self.update_position_display()

    def vision_go(self):
        try:
            x = int(self.Vision_X.text())
            y = int(self.Vision_Y.text())
            self.robot.set_position(x, y, self.robot.z)  # Keep current Z position
            self.update_position_display()
            logging.info(f"Moving to vision-based position: X:{x}, Y:{y}")
        except ValueError:
            logging.error("Invalid vision position values")
            QtWidgets.QMessageBox.warning(self, "Error", "Invalid vision position values")

    def x_set(self):
        self.robot.set_position(self.horizontalSlider_1.value(), self.robot.y, self.robot.z)
        self.update_position_display()

    def y_set(self):
        self.robot.set_position(self.robot.x, self.horizontalSlider_2.value(), self.robot.z)
        self.update_position_display()

    def z_set(self):
        self.robot.set_position(self.robot.x, self.robot.y, self.horizontalSlider_3.value())
        self.update_position_display()

    def setting(self):
        try:
            value = int(self.lineEdit.text())
            if 0 <= value <= 10:
                self.selectbot.setDisabled(True)
                self.lineEdit.setDisabled(True)
            else:
                raise ValueError
        except ValueError:
            logging.error("Invalid setting value")
            QtWidgets.QMessageBox.warning(self, "Error", "Invalid setting value. Please enter a number between 0 and 10.")
            self.reset()

    def reset(self):
        self.lineEdit.setText('0')
        self.selectbot.setEnabled(True)
        self.lineEdit.setEnabled(True)
        for button in [self.jogxu, self.jogxd, self.jogyu, self.jogyd, self.jogzu, self.jogzd]:
            button.setEnabled(True)

    def take_val(self):
        try:
            value = int(self.lineEdit.text())
            if 0 <= value <= 10:
                self.lineEdit.setText(str(value))
            else:
                raise ValueError
        except ValueError:
            logging.error("Invalid input value")
            QtWidgets.QMessageBox.warning(self, "Error", "Please enter a number between 0 and 10.")
            self.lineEdit.setText('0')

    def closeEvent(self, event):
        self.thread.stop()
        event.accept()

if __name__ == "__main__":
    window = MyClass()
    MainWindow.show()
    sys.exit(app.exec_())
