from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt
import sys
from ros_gui import Ui_MainWindow
from video_thread import VideoThread
from move_robot_thread import MoveRobotThread

import cv2
import numpy as np
from ultralytics import YOLO
from threading import Thread
from time import time,sleep
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2.pymoveit2 import MoveIt2, MoveIt2State
from pymoveit2.pymoveit2.robots import cartesian as panda

import cv2
import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal

class RobotControl(Ui_MainWindow):
    def __init__(self):
        super().setupUi(MainWindow) # type: ignore
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
        # Update the instantiation of VideoThread
        self.thread = VideoThread()
        self.thread.start()
        self.thread.change_pixmap_signal.connect(self.update_image)
        self.thread.target_signal.connect(self.target_xy)
        self.disply_width = self.Display.width()
        self.display_height = self.Display.height()

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
        # print(f"P1 coordinates: X={self.tar_x}, Y={self.tar_y}, Z={z}")
    # def execute_move(self):
    #     print(f"Calling move_follow with X={self.tar_x * 0.001}, Y={self.tar_y* 0.001}, Z={self.tar_z* 0.001}")
    #     self.follow(self.tar_x * 0.001,self.tar_y * 0.001,self.tar_z * 0.001)
    # @pyqtSlot(int,int)
    
    def pnp_to(self):
        if not self.is_pnp_running:
            print(f"Starting pnp with X={self.tar_x * 0.001}, Y={self.tar_y * 0.001}, Z={self.tar_z * 0.001}")
            self.pnp_thread = MoveRobotThread(self.tar_x * 0.001,self.tar_y * 0.001,self.tar_z * 0.001)
            self.pnp_thread.finished.connect(self.on_move_finished)  # Connect finished signal
            self.is_pnp_running = True  # Set the flag to indicate pnp is running
            self.pnp_thread.start()
        else:
            print("an operation is already running, please wait for it to finish.")

        # Increment all axes by 10 units
        # self.current_joint_positions = [
        #     pos + 0.01 for pos in self.current_joint_positions
        # ]
        # self.move_robot()
        # self.pnp(self.tar_x,self.tar_y,self.tar_z)
        # self.pnp(0.12,0.06,0.12)
        # self.execute_move()
        # self.follow(self.tar_x * 0.001,self.tar_y * 0.001,self.tar_z * 0.001)
        # self.pnp(0.24,0.15,0.12)
        # print(f"Calling pnp with X={self.tar_x * 0.001}, Y={self.tar_y* 0.001}, Z={self.tar_z* 0.001}")
    def on_move_finished(self):
        self.is_pnp_running = False  # Reset the flag when the thread finishes
        print("all operation finished. You can start again.")

    def follow(self,x,y,z):
        # Initialize ROS for the pick and place operation
        rclpy.init()
        node = Node("follow_goal")

        # Create callback group for MoveIt2
        callback_group = ReentrantCallbackGroup()
        moveit2 = MoveIt2(
            node=node,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )

        # Spin the node and allow for execution of the motion
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(node)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        node.create_rate(1.0).sleep()
        moveit2.max_velocity = 0.1
        moveit2.max_acceleration = 0.1

        def move_follow(positions):
            # Move to a given joint configuration twice
            for _ in range(2):
                node.get_logger().info(f"Moving to joint positions: {list(positions)}")
                moveit2.move_to_configuration(positions)
                moveit2.wait_until_executed()

        # 1. Move to x, y position twice
        joint_positions_xy = [x, y, 0.0]
        move_follow(joint_positions_xy)


        # Shutdown ROS after movement
        rclpy.shutdown()
        executor_thread.join()

    def move_robot(self):
        rclpy.init()

        # Create node for this example
        node = Node("ex_joint_goal")

        # Declare parameter for joint positions
        node.declare_parameter("joint_positions", self.current_joint_positions)
        node.declare_parameter("synchronous", True)
        node.declare_parameter("cancel_after_secs", 0.0)
        node.declare_parameter("planner_id", "RRTConnectkConfigDefault")

        # Create callback group that allows execution of callbacks in parallel
        callback_group = ReentrantCallbackGroup()

        # Update this part with your robot's specific joint names and configuration
        moveit2 = MoveIt2(
            node=node,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )
        moveit2.planner_id = (
            node.get_parameter("planner_id").get_parameter_value().string_value
        )

        # Spin the node in background thread(s) and wait a bit for initialization
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(node)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        node.create_rate(1.0).sleep()

        # Scale down velocity and acceleration of joints (percentage of maximum)
        moveit2.max_velocity = 0.1
        moveit2.max_acceleration = 0.1

        # Move to joint configuration
        joint_positions = (
            node.get_parameter("joint_positions").get_parameter_value().double_array_value
        )
        synchronous = node.get_parameter("synchronous").get_parameter_value().bool_value

        node.get_logger().info(f"Moving to joint positions: {list(joint_positions)}")
        moveit2.move_to_configuration(joint_positions)

        if synchronous:
            moveit2.wait_until_executed()

        # Shutdown ROS after movement
        rclpy.shutdown()
        executor_thread.join()
    def pnp(self, x, y, z):
        # Initialize ROS for the pick and place operation
        rclpy.init()
        node = Node("pnp_goal")

        # Create callback group for MoveIt2
        callback_group = ReentrantCallbackGroup()
        moveit2 = MoveIt2(
            node=node,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )

        # Spin the node and allow for execution of the motion
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(node)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        node.create_rate(1.0).sleep()
        moveit2.max_velocity = 0.1
        moveit2.max_acceleration = 0.1

        # Execute the movement sequence

        def move_twice(positions):
            # Move to a given joint configuration twice
            for _ in range(2):
                node.get_logger().info(f"Moving to joint positions: {list(positions)}")
                moveit2.move_to_configuration(positions)
                moveit2.wait_until_executed()
                time.sleep(3)

        # 1. Move to x, y position twice
        joint_positions_xy = [x, y, 0.0]
        move_twice(joint_positions_xy)

        # 2. Move to z position (pick) twice
        joint_positions_pick = [x, y, z]
        move_twice(joint_positions_pick)

        # 3. Wait for 2 seconds
        node.get_logger().info("Waiting for 2 seconds...")
        time.sleep(2)

        # 4. Move z back to 0 (place) twice
        joint_positions_place = [x, y, 0.0]
        move_twice(joint_positions_place)

        # 5. Move xy back to 0 twice
        joint_positions_home = [0.0, 0.0, 0.0]
        move_twice(joint_positions_home)
        
        joint_positions_home = [0.0, 0.0, 0.12]
        move_twice(joint_positions_home)
        
        joint_positions_home = [0.0, 0.0, 0.0]
        move_twice(joint_positions_home)

        # Shutdown ROS after movement
        rclpy.shutdown()
        executor_thread.join()


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
        if self.is_move_running:
            return True
        else:
            return False

    def start_move_thread(self, mode):
        print(f'Move mode: {mode}')
        self.move_mode = mode
        if not self.is_operation_running():
            print(f"Starting {mode} with X={self.tar_x * 0.001}, Y={self.tar_y * 0.001}, Z={self.tar_z * 0.001}")
            self.move_thread = MoveRobotThread(self.tar_x * 0.001, self.tar_y * 0.001, self.tar_z * 0.001, self.move_mode)
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

    def on_move_finished(self):
        self.is_move_running = False  # Reset the flag when the thread finishes
        print("all operation finished. You can start again.")
