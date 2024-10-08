import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread
import cv2
from ultralytics import YOLO
from UI_TESTTING import Ui_MainWindow
app = QtWidgets.QApplication(sys.argv)
MainWindow = QtWidgets.QMainWindow()
class YOLOThread(QThread):
    detection_signal = pyqtSignal(QImage)

    def __init__(self):
        super().__init__()
        self.model = YOLO("yolov8n.pt")  # Load YOLOv8 model

    def run(self):
        cap = cv2.VideoCapture(0)  # Use default camera
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            # Perform YOLOv8 detection
            results = self.model(frame)
            
            # Draw bounding boxes
            for result in results:
                boxes = result.boxes.xyxy.cpu().numpy()
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box[:4])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Convert frame to QImage
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)

            self.detection_signal.emit(qt_image)

class UpdateThread(QThread):
    update_signal = pyqtSignal(QPixmap)

    def __init__(self):
        super().__init__()
        self.pixmap = None

    def set_pixmap(self, pixmap):
        self.pixmap = pixmap

    def run(self):
        while True:
            if self.pixmap:
                self.update_signal.emit(self.pixmap)
            self.msleep(30)  # Update every 30ms (approx. 33 fps)

class MyClass(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.yolo_thread = YOLOThread()
        self.update_thread = UpdateThread()

        self.yolo_thread.detection_signal.connect(self.update_image)
        self.update_thread.update_signal.connect(self.display_image)

        self.yolo_thread.start()
        self.update_thread.start()

    def update_image(self, image):
        pixmap = QPixmap.fromImage(image)
        self.update_thread.set_pixmap(pixmap)

    def display_image(self, pixmap):
        self.label1.setPixmap(pixmap.scaled(self.label1.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

if __name__ == "__main__":
    window = MyClass()
    MainWindow.show()
    sys.exit(app.exec_())
