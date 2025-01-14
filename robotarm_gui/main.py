from PyQt5 import QtCore, QtGui, QtWidgets
import sys
import time
app = QtWidgets.QApplication(sys.argv)
MainWindow = QtWidgets.QMainWindow()
# from ros_gui import Ui_MainWindow
from ros_gui_3 import Ui_MainWindow
from serial import Serial
from time import sleep, ctime
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread
from PyQt5.QtWidgets import QDialog, QVBoxLayout, QListWidget, QPushButton, QLabel

import cv2
import numpy as np
from ultralytics import YOLO
from threading import Thread
from move_robot_thread import MoveRobotThread
from video.video_thread import VideoThread
import queue
from auto_pnp_thread import AutoPnPThread
from gui_init import GUIInitializer
from jog_controls import JogControls, MoveLControls
from utils.coordinate_converter import CoordinateConverter

class PositionSelectorDialog(QDialog):
    def __init__(self, positions, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Select Position")
        self.setModal(True)
        
        layout = QVBoxLayout()
        
        # Source (Objects) list
        if positions['objects']:
            layout.addWidget(QLabel("Select Source Object:"))
            self.objects_list = QListWidget()
            t_count = 1
            b_count = 1
            for x, y in positions['objects']:
                if y < 240:
                    label = f"T{t_count}"
                    t_count += 1
                else:
                    label = f"B{b_count}"
                    b_count += 1
                self.objects_list.addItem(f"{label}: ({x:.2f}, {y:.2f})")
            layout.addWidget(self.objects_list)
        
        # Destination (Grid) list
        if positions['grid']:
            layout.addWidget(QLabel("Select Destination Grid:"))
            self.grid_list = QListWidget()
            # Only show unoccupied grid positions
            for grid_id, (x, y) in positions['grid'].items():
                # Skip any positions marked as occupied
                if not grid_id.startswith('occupied_'):
                    self.grid_list.addItem(f"{grid_id}: ({x:.2f}, {y:.2f})")
            layout.addWidget(self.grid_list)
        
        self.select_btn = QPushButton("Select")
        self.select_btn.clicked.connect(self.accept)
        layout.addWidget(self.select_btn)
        
        self.setLayout(layout)
    
    def get_selected_positions(self):
        source = None
        destination = None
        
        if hasattr(self, 'objects_list') and self.objects_list.currentItem():
            idx = self.objects_list.currentRow()
            source = ('object', idx)
            
        if hasattr(self, 'grid_list') and self.grid_list.currentItem():
            text = self.grid_list.currentItem().text()
            grid_id = text.split(':')[0].strip()
            destination = ('grid', grid_id)
            
        return source, destination

class myclass(Ui_MainWindow, GUIInitializer, JogControls, MoveLControls):
    def __init__(self) -> None:
        super().setupUi(MainWindow)
        self.gnc()
        
        self.thread = VideoThread()
        self.thread.start()
        self.thread.change_pixmap_signal.connect(self.update_image)
        self.thread.target_signal.connect(self.target_xy)
        self.thread.grid_position_signal.connect(self.handle_grid_position)
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
        self.object_queue = queue.Queue()
        self.first_pnp_completed = False
        self.grid_target = "L1"  # Default grid target

        self.available_positions = {
            'objects': [],
            'grid': {}
        }
        self.thread.available_positions_signal.connect(self.update_available_positions)

        # Add grid initialization
        self.grid_cols = 4  # Number of columns in the grid
        self.grid_rows = 3  # Number of rows in the grid
        self.placed_count = 0  # Counter for placed objects
        self.grid_start_x = 0.0  # Starting X coordinate for grid
        self.grid_start_y = 0.0  # Starting Y coordinate for grid
        self.cell_size = 0.02  # Size of each grid cell in meters

        self.available_positions = {
            'objects': [],
            'grid': {}
        }

        self.workspace_bounds = {
            'x_fixed': 0,
            'y_fixed': 0,
            'box_width': 0,
            'box_height': 0
        }
        self.thread.workspace_bounds_signal.connect(self.update_workspace_bounds)

        self.is_auto_mode = False
        self.auto_check_timer = QtCore.QTimer()
        self.auto_check_timer.timeout.connect(self.auto_pick_and_place)
        self.auto_check_timer.setInterval(10000)  # 10 seconds

        self.coordinate_converter = CoordinateConverter()

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

    def update_positions(self, x, y, z):
        # Update your sliders or other UI components
        self.xpos_current_slider.setValue(int(x))
        self.ypos_current_slider.setValue(int(y))
        self.zpos_current_slider.setValue(int(z))

    def start_move_thread(self, mode, dest_x=0.0, dest_y=0.0, dest_id=None):
        print(f"move mode: {mode}")
        self.move_mode = mode
        if not self.is_move_running:
            print(f"Starting {mode} with X={self.tar_x * 0.001}, Y={self.tar_y * 0.001}, Z={self.tar_z * 0.001}")
            if mode == "pnp" and dest_id:
                # Get grid coordinates from available_positions
                dest_x, dest_y = self.available_positions['grid'][dest_id]
                # Convert to robot coordinates using coordinate converter
                dest_x, dest_y = self.coordinate_converter.grid_to_robot_coordinates(
                    dest_x, dest_y, self.workspace_bounds
                )
            
            self.move_thread = MoveRobotThread(self.tar_x * 0.001, self.tar_y * 0.001, self.tar_z * 0.001, self.move_mode, dest_x=dest_x, dest_y=dest_y)
            self.move_thread.movement_status.connect(self.handle_movement_status)
            self.move_thread.positions_update.connect(self.update_positions)
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
        
    def on_move_finished(self):
        self.is_move_running = False
        if self.move_mode == "pnp":
            self.first_pnp_completed = True
        print("Move finished. is_move_running set to False.")
        print("All operations finished. You can start again.")
        if self.is_auto_mode:
            # Schedule next pick and place operation
            QtCore.QTimer.singleShot(1000, self.auto_pick_and_place)

    def handle_grid_position(self, x, y):
        """Handle grid position signal from video thread"""
        self.tar_x = x
        self.tar_y = y
        self.Vision_X.setText(f"{self.tar_x * 0.001:.3f}")
        self.Vision_Y.setText(f"{self.tar_y * 0.001:.3f}")
        self.start_move_thread("move")  # Start movement to target position
        
    def move_to_grid_position(self):
        """Called when auto_bt is clicked"""
        if not self.is_auto_mode:
            # Start auto mode
            self.is_auto_mode = True
            self.auto_bt.setText("Stop Auto")
            self.auto_check_timer.start()
            self.auto_pick_and_place()  # Start immediately
        else:
            # Stop auto mode
            self.is_auto_mode = False
            self.auto_bt.setText("Auto")
            self.auto_check_timer.stop()

    def get_first_available_cell(self):
        """Get the first available cell in the right side"""
        for i in range(1, 13):  # Total 12 cells (3x4)
            cell_id = f"R{i}"
            if cell_id in self.available_positions['grid']:
                return cell_id
        return None

    def calculate_middle_placement(self):
        """Calculate coordinates for middle of bottom-half of top rectangle"""
        if not self.workspace_bounds:
            return None
        
        x_fixed = self.workspace_bounds['x_fixed']
        y_fixed = self.workspace_bounds['y_fixed']
        box_width = self.workspace_bounds['box_width']
        
        # Calculate center X and Y for placement
        center_x = x_fixed + (box_width / 2)
        # Y coordinate for bottom half of top rectangle (50 is the gap)
        center_y = y_fixed - 50 - (box_width / 4)
        
        return center_x, center_y

    def auto_pick_and_place(self):
        """Auto pick and place logic"""
        if not self.is_auto_mode or self.is_move_running:
            return

        # Check for T objects first
        t_objects = [(i, pos) for i, pos in enumerate(self.available_positions['objects']) 
                    if pos[1] < 240]  # Y < 240 means top rectangle
        
        # Then check for B objects
        b_objects = [(i, pos) for i, pos in enumerate(self.available_positions['objects']) 
                    if pos[1] >= 240]  # Y >= 240 means bottom rectangle

        if t_objects:
            # Pick T object and place in available R cell
            obj_idx, (x, y) = t_objects[0]
            available_cell = self.get_first_available_cell()
            
            if available_cell:
                self.tar_x = x
                self.tar_y = y
                self.Vision_X.setText(f"{self.tar_x * 0.001:.3f}")
                self.Vision_Y.setText(f"{self.tar_y * 0.001:.3f}")
                self.start_move_thread("pnp", dest_id=available_cell)
                
        elif b_objects:
            # Pick B object and place in middle of top rectangle
            obj_idx, (x, y) = b_objects[0]
            middle_coords = self.calculate_middle_placement()
            
            if middle_coords:
                self.tar_x = x
                self.tar_y = y
                self.Vision_X.setText(f"{self.tar_x * 0.001:.3f}")
                self.Vision_Y.setText(f"{self.tar_y * 0.001:.3f}")
                # Convert middle coordinates to robot coordinates
                dest_x, dest_y = self.coordinate_converter.grid_to_robot_coordinates(
                    middle_coords[0], middle_coords[1], self.workspace_bounds
                )
                self.start_move_thread("pnp", dest_x=dest_x, dest_y=dest_y)

    def update_available_positions(self, positions):
        self.available_positions = positions

    def update_workspace_bounds(self, x_fixed, y_fixed, box_width, box_height):
        self.workspace_bounds = {
            'x_fixed': x_fixed,
            'y_fixed': y_fixed,
            'box_width': box_width,
            'box_height': box_height
        }

    def show_position_selector(self):
        dialog = PositionSelectorDialog(self.available_positions, MainWindow)
        if dialog.exec_() == QDialog.Accepted:
            source, destination = dialog.get_selected_positions()
            if source and destination:
                # Handle source (object to pick)
                if source[0] == 'object':
                    x, y = self.available_positions['objects'][source[1]]
                    self.tar_x = x
                    self.tar_y = y
                    self.Vision_X.setText(f"{self.tar_x * 0.001:.3f}")
                    self.Vision_Y.setText(f"{self.tar_y * 0.001:.3f}")
                    
                    # Store destination grid position and start PnP
                    if destination[0] == 'grid':
                        self.grid_target = destination[1]
                        # Pass destination ID to start_move_thread
                        self.start_move_thread("pnp", dest_id=destination[1])
            elif source:  # Only object selected
                x, y = self.available_positions['objects'][source[1]]
                self.tar_x = x
                self.tar_y = y
                self.Vision_X.setText(f"{self.tar_x * 0.001:.3f}")
                self.Vision_Y.setText(f"{self.tar_y * 0.001:.3f}")
                self.start_move_thread("move")
            elif destination:  # Only grid selected
                self.grid_target = destination[1]
                self.thread.set_grid_target(destination[1])

if __name__ == "__main__":
    myobj = myclass()
    MainWindow.show()
    sys.exit(app.exec_())
