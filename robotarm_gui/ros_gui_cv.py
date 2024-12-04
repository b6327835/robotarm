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
from video_thread import VideoThread  # Add this import
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2, MoveIt2State
from pymoveit2.robots import cartesian as panda # type: ignore

import cv2
import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal
from move_robot_thread import MoveRobotThread
from auto_pnp_thread import AutoPnPThread  # Add this import
import queue  # Add this import

class myclass(Ui_MainWindow):
    def __init__(self) -> None:
        super().setupUi(MainWindow)
        self.gnc()
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

        # self.ser = Serial('COM12',115200)
        # self.ser.setDTR(1)
        # self.ser.setRTS(0)

        self.tm1tick = QtCore.QTimer()
        self.tm1tick.timeout.connect(self.checkbottom)
        self.tm1tick.setInterval(100)
        self.tm1tick.start()
        
        # self.move_timer = QtCore.QTimer()
        # self.move_timer.timeout.connect(self.execute_move)
        self.is_move_running = False
        self.is_auto_pnp_running = False  # Add this line
        self.object_detected = False  # Flag to indicate if an object is detected
        self.grid_rows = 4
        self.grid_cols = 3
        self.cell_size = 0.03  # 10mm in meters
        self.placed_count = 0
        self.auto_pnp_thread = None
        self.grid_start_x = 0.0  # Default starting X position for grid
        self.grid_start_y = 0.0  # Default starting Y position for grid
        self.object_queue = queue.Queue()  # Queue for detected objects
        self.first_pnp_completed = False

    def gnc(self):
        self.stackedWidget.addWidget(self.manpage)
        self.stackedWidget.addWidget(self.jogpage)
        self.stackedWidget.addWidget(self.visionpage)

        self.mannualMode.clicked.connect(
            lambda: self.stackedWidget.setCurrentWidget(self.manpage)
        )
        self.jogMode.clicked.connect(
            lambda: self.stackedWidget.setCurrentWidget(self.jogpage)
        )
        self.VSMode.clicked.connect(
            lambda: self.stackedWidget.setCurrentWidget(self.visionpage)
        )

        self.Initial_bottom.clicked.connect(self.INITIAL_SET)
        self.home_bottom.clicked.connect(self.HOME_SET)
        self.start_bottom.clicked.connect(self.START_SET)
        # self.Gopoint.clicked.connect(self.to_pnp)

        self.horizontalSlider_1.sliderReleased.connect(self.X_SET)
        self.horizontalSlider_2.sliderReleased.connect(self.Y_SET)
        self.horizontalSlider_3.sliderReleased.connect(self.Z_SET)

        self.jogxu.clicked.connect(self.Ux)
        self.jogxd.clicked.connect(self.Dx)

        self.jogyu.clicked.connect(self.Uy)
        self.jogyd.clicked.connect(self.Dy)

        self.jogzu.clicked.connect(self.Uz)
        self.jogzd.clicked.connect(self.Dz)

        self.selectbot.clicked.connect(self.Setting)
        self.Restsetbot.clicked.connect(self.Rest)
        self.lineEdit.returnPressed.connect(self.takeVal)
        
        self.Gopoint.clicked.connect(self.move_to)
        self.pick_target_bt.clicked.connect(self.picktarget)
        self.pnp_bt.clicked.connect(self.to_pnp)
        self.home_bt.clicked.connect(self.home)
        self.auto_bt.clicked.connect(self.auto_pnp)

        self.thread = VideoThread()
        self.thread.start()
        self.thread.change_pixmap_signal.connect(self.update_image)
        self.thread.target_signal.connect(self.target_xy)
        self.disply_width = self.Display.width()
        self.display_height = self.Display.height()
        self.thread.grid_start_signal.connect(self.update_grid_start)
        
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
        self.object_queue.put((self.tar_x, self.tar_y))  # Enqueue detected object
        # print(f"P1 coordinates: X={self.tar_x}, Y={self.tar_y}, Z={z}")
    
    def checkbottom(self):
        if self.x_post == 0:
            self.jogxd.setDisabled(1)
        else:
            self.jogxd.setEnabled(1)

        if self.y_post == 0:
            self.jogyd.setDisabled(1)
        else:
            self.jogyd.setEnabled(1)

        if self.z_post == 0:
            self.jogzd.setDisabled(1)
        else:
            self.jogzd.setEnabled(1)

        if self.x_post >= 300:
            self.jogxu.setDisabled(1)
            self.x_def = 300
            self.X_core_j.setNum(self.x_def)

        else:
            self.jogxu.setEnabled(1)

        if self.y_post >= 200:
            self.jogyu.setDisabled(1)
            self.y_def = 200
            self.Y_core_j.setNum(self.x_def)
        else:
            self.jogyu.setEnabled(1)

        if self.z_post >= 200:
            self.jogzu.setDisabled(1)
            self.z_def = 200
            self.Z_core_j.setNum(self.x_def)
        else:
            self.jogzu.setEnabled(1)

    def Rest(self):
        self.GetVal = 0
        self.lineEdit.setText("0")
        self.jogxu.setEnabled(1)
        self.jogxd.setEnabled(1)
        self.jogyu.setEnabled(1)
        self.jogyd.setEnabled(1)
        self.jogzu.setEnabled(1)
        self.jogzd.setEnabled(1)
        self.selectbot.setEnabled(1)
        self.lineEdit.setEnabled(1)

    def takeVal(self):
        try:
            value = int(self.lineEdit.text())
            if value > 10:
                value = 10
            elif value < 0:
                value = 0
            self.lineEdit.setText(str(value))
            self.GetVal = value

        except ValueError:
            self.GetVal = None

    def Setting(self):
        self.takeVal()
        # print(type(self.GetVal))

        if isinstance(self.GetVal, int):
            self.selectbot.setDisabled(1)
            self.lineEdit.setDisabled(1)

        else:
            self.lineEdit.setText("Try again.")
            self.jogxu.setDisabled(1)
            self.jogxu.setDisabled(1)
            self.jogxd.setDisabled(1)
            self.jogyu.setDisabled(1)
            self.jogyd.setDisabled(1)
            self.jogzu.setDisabled(1)
            self.jogzd.setDisabled(1)

    def Ux(self):
        self.x_def += int(self.GetVal)
        self.X_core_j.setNum(self.x_def)
        self.x_post = self.x_def
        print(self.x_def)
        self.horizontalSlider_1.setValue(self.x_def)

    def Dx(self):
        self.x_def -= int(self.GetVal)
        self.X_core_j.setNum(self.x_def)
        self.x_post = self.x_def
        print(self.x_def)
        self.horizontalSlider_1.setValue(self.x_def)

    def Uy(self):
        self.y_def += int(self.GetVal)
        self.Y_core_j.setNum(self.y_def)
        self.y_post = self.y_def
        print(self.y_def)
        self.horizontalSlider_2.setValue(self.y_def)

    def Dy(self):
        self.y_def -= int(self.GetVal)
        self.Y_core_j.setNum(self.y_def)
        self.y_post = self.y_def
        print(self.y_def)
        self.horizontalSlider_2.setValue(self.y_def)

    def Uz(self):
        self.z_def += int(self.GetVal)
        self.Z_core_j.setNum(self.z_def)
        self.z_post = self.z_def
        print(self.z_def)
        self.horizontalSlider_3.setValue(self.z_def)

    def Dz(self):
        self.z_def -= int(self.GetVal)
        self.Z_core_j.setNum(self.z_def)
        self.z_post = self.z_def
        print(self.z_def)
        self.horizontalSlider_3.setValue(self.z_def)

    def INITIAL_SET(self):
        self.horizontalSlider_1.setValue(0)
        self.horizontalSlider_2.setValue(0)
        self.horizontalSlider_3.setValue(0)

        self.x_post = self.horizontalSlider_1.value()
        self.y_post = self.horizontalSlider_2.value()
        self.z_post = self.horizontalSlider_3.value()

        self.x_post = 0
        self.y_post = 0
        self.z_post = 0

        self.x_def = 0
        self.y_def = 0
        self.z_def = 0

        self.X_core_j.setText(str(0))
        self.Y_core_j.setText(str(0))
        self.Z_core_j.setText(str(0))
        print(" ")
        # print(f'X:{x_post} ,Y:{y_post} ,Z:{z_post}')
        print(f"X,Y,Z: {self.x_post},{self.y_post},{self.z_post}")
        print("INITIAL Check")
        # self.xpoint()

    def HOME_SET(self):
        self.horizontalSlider_1.setValue(150)
        self.horizontalSlider_2.setValue(100)
        self.horizontalSlider_3.setValue(100)

        self.x_post = self.horizontalSlider_1.value()
        self.y_post = self.horizontalSlider_2.value()
        self.z_post = self.horizontalSlider_3.value()

        self.x_post = 150
        self.y_post = 100
        self.z_post = 100

        self.X_core_j.setText(str(150))
        self.Y_core_j.setText(str(150))
        self.Z_core_j.setText(str(150))

        print(" ")
        # print(f'X:{x_post} ,Y:{y_post} ,Z:{z_post}')
        print(f"X,Y,Z: {self.x_post},{self.y_post},{self.z_post}")
        print("HOME Check")
        # self.xpoint()

    def START_SET(self):
        self.x_post = self.horizontalSlider_1.value()
        self.y_post = self.horizontalSlider_2.value()
        self.z_post = self.horizontalSlider_3.value()
        # print(f'X:{x_post} ,Y:{y_post} ,Z:{z_post}')
        print(" ")
        if self.x_post == 0 & self.y_post == 0 & self.z_post == 0:
            # self.ser.write(f'u'.encode())
            self.xp = "3x" + str(0)
            self.yp = "3y" + str(0)
            self.zp = "3z" + str(0)
            self.setINITIAL = self.xp + "," + self.yp + "," + self.zp
            sleep(0.2)
            # self.ser.write(f'{self.setINITIAL}'.encode())
            print(self.setINITIAL)
            print("Set Initial")
        else:
            self.xpoint()
            self.ypoint()
            self.zpoint()
            self.setpost = self.xp + "," + self.yp + "," + self.zp
            sleep(0.2)
            # self.ser.write(f'{self.setpost}'.encode())
            print(self.setpost)

            print("Set Positions")

        # print(f'X,Y,Z: {self.x_post},{self.y_post},{self.z_post}')
        # print("START Check")

    def X_SET(self):
        self.x_vale = self.horizontalSlider_1.value()
        self.X_core_j.setText(str(self.x_vale))
        print(f"X : {self.x_vale}")

    def Y_SET(self):
        self.y_vale = self.horizontalSlider_2.value()
        self.Y_core_j.setText(str(self.y_vale))
        print(f"Y : {self.y_vale}")

    def Z_SET(self):
        self.z_vale = self.horizontalSlider_3.value()
        self.Z_core_j.setText(str(self.z_vale))
        print(f"Z : {self.z_vale}")

    def xpoint(self):
        # if(self.x_post==self.x_def):
        #     print(0)

        if self.x_post >= self.x_def:
            self.x_def = self.x_post - self.x_def
            print(f"X : +{self.x_def}")
            self.x_def = self.x_post
            self.xp = "1x" + str(self.x_def)

        elif self.x_post <= self.x_def:
            self.x_def = abs(self.x_post - self.x_def)
            print(f"X : -{self.x_def}")
            self.x_def = self.x_post
            self.xp = "2x" + str(self.x_def)

    def ypoint(self):
        # if(self.y_post==self.y_def):
        #     print(0)

        if self.y_post >= self.y_def:
            self.y_def = self.y_post - self.y_def
            print(f"Y : +{self.y_def}")
            self.y_def = self.y_post
            self.yp = "1y" + str(self.y_def)

        elif self.y_post <= self.y_def:
            self.y_def = abs(self.y_post - self.y_def)
            print(f"Y : -{self.y_def}")
            self.y_def = self.y_post
            self.yp = "2y" + str(self.y_def)

    def zpoint(self):
        # if(self.z_post==self.z_def):
        #     print(0)

        if self.z_post >= self.z_def:
            self.z_def = self.z_post - self.z_def
            print(f"Z : +{self.z_def}")
            self.z_def = self.z_post
            self.zp = "1z" + str(self.z_def)

        elif self.z_post <= self.x_def:
            self.z_def = abs(self.z_post - self.z_def)
            print(f"Z : -{self.z_def}")
            self.z_def = self.z_post
            self.zp = "2z" + str(self.z_def)
            
    def is_operation_running(self):
        return self.is_move_running or self.is_auto_pnp_running

    def start_move_thread(self, mode, dest_x=0.0, dest_y=0.0):
        print(f"move mode: {mode}")
        self.move_mode = mode
        if not self.is_move_running:
            print(f"Starting {mode} with X={self.tar_x * 0.001}, Y={self.tar_y * 0.001}, Z={self.tar_z * 0.001}")
            if mode == "pnp":
                # Compute destination positions on grid
                row = self.placed_count // self.grid_cols
                col = self.placed_count % self.grid_cols
                dest_x = self.grid_start_x + col * self.cell_size
                dest_y = self.grid_start_y + row * self.cell_size
                self.placed_count += 1
            self.move_thread = MoveRobotThread(self.tar_x * 0.001, self.tar_y * 0.001, self.tar_z * 0.001, self.move_mode, dest_x=dest_x, dest_y=dest_y)
            self.move_thread.finished.connect(self.on_move_finished)
            self.is_move_running = True
            self.move_thread.start()
        else:
            print("An operation is already running, please wait for it to finish.")

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
                self.grid_start_x,  # Pass starting X position
                self.grid_start_y   # Pass starting Y position
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
