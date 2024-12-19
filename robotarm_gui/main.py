from PyQt5 import QtCore, QtGui, QtWidgets
import sys
import time
app = QtWidgets.QApplication(sys.argv)
MainWindow = QtWidgets.QMainWindow()
# from ros_gui import Ui_MainWindow
from ros_gui_2 import Ui_MainWindow
from serial import Serial
from time import sleep, ctime
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread

import cv2
import numpy as np
from ultralytics import YOLO
from threading import Thread
from move_robot_thread import MoveRobotThread
from video.video_thread import VideoThread
import queue
from auto_pnp_thread import AutoPnPThread
from gui_init import GUIInitializer
from jog_controls import JogControls

class myclass(Ui_MainWindow, GUIInitializer, JogControls):
    def __init__(self) -> None:
        super().setupUi(MainWindow)
        self.gnc()
        
        self.thread = VideoThread()
        self.thread.start()
        self.thread.change_pixmap_signal.connect(self.update_image)
        self.thread.target_signal.connect(self.target_xy)
        self.thread.grid_start_signal.connect(self.update_grid_start)
        self.disply_width = self.Display.width()
        self.display_height = self.Display.height()

        # Rest of initialization
        self.current_joint_positions = [0.0, 0.0, 0.0]

        self.x_def = 0
        self.y_def = 0
        self.z_def = 0

        self.x_post = 0
        self.y_post = 0
        self.z_post = 0
        self.GetVal = 0
        
        self.tar_x = 0.0
        self.tar_y = 0.0
        self.tar_z = 0.0

        self.tm1tick = QtCore.QTimer()
        self.tm1tick.timeout.connect(self.checkbottom)
        self.tm1tick.setInterval(100)
        self.tm1tick.start()
        
        self.is_move_running = False
        self.is_auto_pnp_running = False
        self.object_detected = False
        self.grid_rows = 4
        self.grid_cols = 3
        self.cell_size = 0.03
        self.placed_count = 0
        self.auto_pnp_thread = None
        self.grid_start_x = 0.0
        self.grid_start_y = 0.0
        self.object_queue = queue.Queue()
        self.first_pnp_completed = False

    def update_image(self, cv_img):
        qt_img = self.convert_cv_qt(cv_img, 640, 480)
        self.Display.setPixmap(qt_img)

    def convert_cv_qt(self, cv_img, Wx, Hx):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        self.h, self.w, self.ch = rgb_image.shape
        bytes_per_line = self.ch * self.w
        convert_to_Qt_format = QtGui.QImage(
            rgb_image.data, self.w, self.h, bytes_per_line, QtGui.QImage.Format_RGB888
        )
        p = convert_to_Qt_format.scaled(Wx, Hx, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)
    def target_xy(self, target_x, target_y, z=0.12):
        self.tar_x = target_x
        self.tar_y = target_y
        self.Vision_X.setText(f"{self.tar_x * 0.001:.3f}")
        self.Vision_Y.setText(f"{self.tar_y * 0.001:.3f}")
        self.object_queue.put((self.tar_x, self.tar_y))
            
    def is_operation_running(self):
        return self.is_move_running or self.is_auto_pnp_running

    def start_move_thread(self, mode, dest_x=0.0, dest_y=0.0):
        print(f"move mode: {mode}")
        self.move_mode = mode
        if not self.is_move_running:
            print(f"Starting {mode} with X={self.tar_x * 0.001}, Y={self.tar_y * 0.001}, Z={self.tar_z * 0.001}")
            if mode == "pnp":
                row = self.placed_count // self.grid_cols
                col = self.placed_count % self.grid_cols
                dest_x = self.grid_start_x + col * self.cell_size
                dest_y = self.grid_start_y + row * self.cell_size
                self.placed_count += 1
            self.move_thread = MoveRobotThread(self.tar_x * 0.001, self.tar_y * 0.001, self.tar_z * 0.001, self.move_mode, dest_x=dest_x, dest_y=dest_y)
            self.move_thread.movement_status.connect(self.handle_movement_status)
            self.move_thread.finished.connect(self.on_move_finished)
            self.is_move_running = True
            self.move_thread.start()
        else:
            print("An operation is already running, please wait for it to finish.")

    def handle_movement_status(self, status):
        print(f"Robot Status: {status}")
        # You could also update a status label in the GUI here if desired
        # self.status_label.setText(status)

    def to_pnp(self):
        self.start_move_thread("pnp")

    def move_to(self):
        self.start_move_thread("move")

    def picktarget(self):
        self.start_move_thread("pick")

    def home(self):
        self.start_move_thread("home")
        
    def auto_pnp(self):
        if not self.is_auto_pnp_running:
            self.auto_pnp_thread = AutoPnPThread(
                self.grid_rows,
                self.grid_cols,
                self.cell_size,
                self,
                self.grid_start_x,
                self.grid_start_y
            )
            self.auto_pnp_thread.finished.connect(self.on_auto_pnp_finished)
            self.is_auto_pnp_running = True
            self.auto_pnp_thread.start()
        else:
            print("Auto pick-and-place operation is already running, please wait.")

    def on_auto_pnp_finished(self):
        self.is_auto_pnp_running = False
        print("Auto pick-and-place completed.")

    def on_move_finished(self):
        self.is_move_running = False
        if self.move_mode == "pnp":
            self.first_pnp_completed = True
        print("Move finished. is_move_running set to False.")
        print("All operations finished. You can start again.")

    def update_grid_start(self, x, y):
        if not self.first_pnp_completed:
            self.grid_start_x = x
            self.grid_start_y = y
            print(f"Updated grid start position to X={x:.3f}, Y={y:.3f}")

if __name__ == "__main__":
    myobj = myclass()
    MainWindow.show()
    sys.exit(app.exec_())
