from PyQt5 import QtCore, QtGui, QtWidgets
import sys
import time
app = QtWidgets.QApplication(sys.argv)
MainWindow = QtWidgets.QMainWindow()
# from ros_gui import Ui_MainWindow
from gui.gui_4 import Ui_MainWindow
from serial import Serial
from time import sleep, ctime
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread
from PyQt5.QtWidgets import QDialog, QVBoxLayout, QListWidget, QPushButton, QLabel

import cv2
import numpy as np
from ultralytics import YOLO
from threading import Thread
from controls.move_robot_thread import MoveRobotThread
from video.video_thread import VideoThread
import queue
from controls.auto_pnp_thread import AutoPnPThread
from gui.gui_init import GUIInitializer
from controls.jog_controls import JogControls, MoveLControls
from utils.coordinate_converter import CoordinateConverter
from controls.pnp2 import PnP2Operations

class PositionSelectorDialog(QDialog):
    def __init__(self, positions, workspace_bounds, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Select Position")
        self.setModal(True)
        self.workspace_bounds = workspace_bounds
        self.positions = positions
        
        layout = QVBoxLayout()
        
        # Clear existing object counters
        self.pickable_start_idx = 0
        self.non_pickable_start_idx = 0
        
        # Initialize object lists with averaged positions
        if positions['pickable_objects'] or positions['non_pickable_objects']:
            layout.addWidget(QLabel("Available Objects:"))
            self.objects_list = QListWidget()
            
            # Add pickable objects with their averaged positions
            for i, (x, y) in enumerate(positions['pickable_objects']):
                robot_x, robot_y = CoordinateConverter.to_robot_coordinates(
                    x, y, 
                    self.workspace_bounds['x_fixed'],
                    self.workspace_bounds['y_fixed'],
                    self.workspace_bounds['box_width'],
                    self.workspace_bounds['box_height']
                )
                # Use averaged y coordinate for top/bottom determination
                label = f"{'T' if y < 240 else 'B'}{i+1}"
                self.objects_list.addItem(f"P-{label}: Cam({x:.3f}, {y:.3f}) → Real({robot_x:.3f}, {robot_y:.3f})")
            
            # Update non-pickable start index
            self.non_pickable_start_idx = len(positions['pickable_objects'])
            
            # Add non-pickable objects with their actual positions
            for i, (x, y) in enumerate(positions['non_pickable_objects']):
                robot_x, robot_y = CoordinateConverter.to_robot_coordinates(
                    x, y, 
                    self.workspace_bounds['x_fixed'],
                    self.workspace_bounds['y_fixed'],
                    self.workspace_bounds['box_width'],
                    self.workspace_bounds['box_height']
                )
                # Determine if object is in top or bottom half
                label = f"{'T' if y < 240 else 'B'}{i+1}"
                self.objects_list.addItem(f"NP-{label}: Cam({x:.3f}, {y:.3f}) → Real({robot_x:.3f}, {robot_y:.3f})")
            
            layout.addWidget(self.objects_list)

        # Add grid positions
        if positions['grid']:
            layout.addWidget(QLabel("Select Destination Grid:"))
            self.grid_list = QListWidget()
            # Sort grid positions by ID for consistent ordering
            sorted_grid = sorted(positions['grid'].items())
            for grid_id, (x, y) in sorted_grid:
                if not grid_id.startswith('occupied_'):
                    robot_x, robot_y = CoordinateConverter.to_robot_coordinates(
                        x, y,
                        self.workspace_bounds['x_fixed'],
                        self.workspace_bounds['y_fixed'],
                        self.workspace_bounds['box_width'],
                        self.workspace_bounds['box_height']
                    )
                    self.grid_list.addItem(f"{grid_id}: Cam({x:.3f}, {y:.3f}) → Real({robot_x:.3f}, {robot_y:.3f})")
            layout.addWidget(self.grid_list)
        
        self.select_btn = QPushButton("Select")
        self.select_btn.clicked.connect(self.accept)
        layout.addWidget(self.select_btn)
        
        self.setLayout(layout)

    def get_selected_positions(self):
        source = None
        destination = None
        
        if hasattr(self, 'objects_list') and self.objects_list.currentItem():
            text = self.objects_list.currentItem().text()
            current_idx = self.objects_list.currentRow()
            
            # Determine if pickable or non-pickable based on prefix
            if text.startswith('NP-'):
                source = ('non_pickable', current_idx - self.non_pickable_start_idx)
            else:  # Pickable object
                source = ('pickable', current_idx)
            
        if hasattr(self, 'grid_list') and self.grid_list.currentItem():
            text = self.grid_list.currentItem().text()
            grid_id = text.split(':')[0].strip()
            destination = ('grid', grid_id)
            
        return source, destination

class myclass(Ui_MainWindow, GUIInitializer, JogControls, MoveLControls):
    def __init__(self):
        super().setupUi(MainWindow)
        self.gnc()
        
        # Initialize video thread
        self.thread = VideoThread(
            use_realsense=False,
            use_calibration=False,
            use_raw_coordinates=False,
            use_interpolation=True
        )
        
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
            'pickable_objects': [],
            'non_pickable_objects': [],
            'grid': {}
        }
        self.thread.available_positions_signal.connect(self.update_available_positions)

        #grid initialization
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

        self.is_auto_mode2 = False
        self.pnp2_ops = None

    def is_position_safe(self, x, y, existing_objects):
        """Check if a position is safe considering all existing objects"""
        min_distance = 30  # minimum distance in millimeters
        for obj in existing_objects:
            if isinstance(obj, tuple) and len(obj) == 2:
                obj_x, obj_y = obj
                dist = np.sqrt((x - obj_x)**2 + (y - obj_y)**2)
                if dist < min_distance/1000.0:  # Convert mm to meters
                    return False
        return True

    def find_safe_placement_position(self, start_x, start_y):
        """Find next safe position for object placement considering existing objects"""
        current_y = start_y
        
        # Consider both initial top objects and newly placed objects
        all_top_objects = []
        if hasattr(self, 'initial_top_objects'):
            all_top_objects.extend([
                self.coordinate_converter.to_robot_coordinates(
                    x, y,
                    self.workspace_bounds['x_fixed'],
                    self.workspace_bounds['y_fixed'],
                    self.workspace_bounds['box_width'],
                    self.workspace_bounds['box_height']
                ) for x, y in self.initial_top_objects
            ])
        all_top_objects.extend(self.top_rectangle_objects)
        
        while current_y < start_y + 0.05:  # Limit vertical search to 50mm
            if self.is_position_safe(start_x, current_y, all_top_objects):
                return start_x, current_y
            current_y += self.object_spacing/1000.0
        return None

    def count_objects_in_basket(self, basket_center):
        """Count objects placed in a specific basket"""
        count = 0
        if basket_center in self.remembered_positions['basket_objects']:
            count = len(self.remembered_positions['basket_objects'][basket_center])
        return count

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
    def target_xy(self, target_x, target_y, z=190):
        """Handle target coordinates from video thread"""
        # Use the target coordinates for movement
        self.tar_x = target_x
        self.tar_y = target_y
        self.object_queue.put((target_x, target_y))
            
    def is_operation_running(self):
        return self.is_move_running or self.is_auto_pnp_running

    def update_positions(self, x, y, z):
        # Update sliders with current positions
        self.xpos_current_slider.setValue(int(x))
        self.ypos_current_slider.setValue(int(y))
        self.zpos_current_slider.setValue(int(z))

    def start_move_thread(self, mode, dest_x=0.0, dest_y=0.0, dest_id=None):
        print(f"move mode: {mode}")
        self.move_mode = mode
        if not self.is_move_running:
            print(f"Starting {mode} with X={self.tar_x}, Y={self.tar_y}, Z={self.tar_z}")
            if mode == "pnp" and dest_id:
                # Get grid coordinates from available_positions
                grid_x, grid_y = self.available_positions['grid'][dest_id]
                # Convert to robot coordinates
                dest_x, dest_y = self.coordinate_converter.to_robot_coordinates(
                    grid_x, grid_y,
                    self.workspace_bounds['x_fixed'],
                    self.workspace_bounds['y_fixed'],
                    self.workspace_bounds['box_width'],
                    self.workspace_bounds['box_height']
                )
            
            self.move_thread = MoveRobotThread(self.tar_x, self.tar_y, self.tar_z,
                                             self.move_mode, dest_x=dest_x, dest_y=dest_y)
            self.move_thread.movement_status.connect(self.handle_movement_status)
            self.move_thread.positions_update.connect(self.update_positions)
            self.move_thread.finished.connect(self.on_move_finished)
            self.is_move_running = True
            self.move_thread.start()
        else:
            print("An operation is already running, please wait for it to finish.")

    def handle_movement_status(self, status):
        print(f"Robot Status: {status}")
        # update a status label in the GUI here
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
        # Add check for auto2 mode completion
        if self.is_auto_mode2 and self.pnp2_ops:
            if not self.pnp2_ops.remembered_positions['objects'] and not self.pnp2_ops.remembered_positions['placed_top']:
                self.start_move_thread("home")
                self.is_auto_mode2 = False
                self.auto_bt_2.setText("Auto 2")
            else:
                QtCore.QTimer.singleShot(1000, self.process_next_auto2_step)

    def handle_grid_position(self, x, y):
        """Handle grid position signal from video thread"""
        self.tar_x = x
        self.tar_y = y
        self.start_move_thread("move")  # Start movement to target position
        
    def move_to_grid_position(self):
        """Called when auto_bt is clicked"""
        if not self.is_auto_mode:
            # Start auto mode
            self.is_auto_mode = True
            self.auto_bt.setText("Stop Auto")
            self.auto_check_timer.start()
            self.auto_pick_and_place()  # Start auto pick and place
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
        box_height = self.workspace_bounds['box_height']
        
        # Start X from left side (1/4 of the width)
        start_x = x_fixed + (box_width * 0.25)  # 25% from left edge
        
        # Calculate bottom half middle of top rectangle
        start_y = min(max(
            y_fixed - 10 - (box_height/2) + (box_height/4), 
            0), 200)
                
        return start_x, start_y

    def auto_pick_and_place(self):
        """Auto pick and place logic"""
        if not self.is_auto_mode or self.is_operation_running():
            return

        separation_y = self.workspace_bounds['y_fixed'] - 10  # Account for gap between rectangles

        # Check for pickable objects in top area
        top_objects = [(i, pos) for i, pos in enumerate(self.available_positions['pickable_objects']) 
                    if pos[1] < separation_y]  

        # Check for pickable objects in bottom area  
        bottom_objects = [(i, pos) for i, pos in enumerate(self.available_positions['pickable_objects']) 
                        if pos[1] >= separation_y]


        if top_objects:
            # Pick top object and place in available right cell
            obj_idx, (x, y) = top_objects[0]
            # Convert to robot coordinates
            robot_x, robot_y = CoordinateConverter.to_robot_coordinates(
                x, y,
                self.workspace_bounds['x_fixed'],
                self.workspace_bounds['y_fixed'],
                self.workspace_bounds['box_width'],
                self.workspace_bounds['box_height']
            )
            
            # Get first available right cell
            available_cell = self.get_first_available_cell()
            if available_cell:
                self.tar_x = robot_x
                self.tar_y = robot_y
                self.start_move_thread("pnp", dest_id=available_cell)
                
        elif bottom_objects:
            # Pick bottom object and place in middle of top area
            obj_idx, (x, y) = bottom_objects[0]
            # Convert to robot coordinates
            robot_x, robot_y = CoordinateConverter.to_robot_coordinates(
                x, y,
                self.workspace_bounds['x_fixed'],
                self.workspace_bounds['y_fixed'],
                self.workspace_bounds['box_width'],
                self.workspace_bounds['box_height']
            )
            
            middle_coords = self.calculate_middle_placement()
            if middle_coords:
                self.tar_x = robot_x
                self.tar_y = robot_y
                
                dest_x, dest_y = CoordinateConverter.to_robot_coordinates(
                    middle_coords[0], middle_coords[1],
                    self.workspace_bounds['x_fixed'],
                    self.workspace_bounds['y_fixed'],
                    self.workspace_bounds['box_width'],
                    self.workspace_bounds['box_height']
                )
                self.start_move_thread("pnp", dest_x=dest_x, dest_y=dest_y)
        else:
            # No objects to pick, stop auto mode
            self.is_auto_mode = False
            self.auto_bt.setText("Auto")
            self.auto_check_timer.stop()

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
        dialog = PositionSelectorDialog(self.available_positions, self.workspace_bounds, MainWindow)
        if dialog.exec_() == QDialog.Accepted:
            source, destination = dialog.get_selected_positions()
            if source:
                # Get coordinates based on source type
                if source[0] == 'pickable':
                    x, y = self.available_positions['pickable_objects'][source[1]]
                else:  # non_pickable
                    x, y = self.available_positions['non_pickable_objects'][source[1]]
                
                # Convert to robot coordinates
                robot_x, robot_y = self.coordinate_converter.to_robot_coordinates(
                    x, y,
                    self.workspace_bounds['x_fixed'],
                    self.workspace_bounds['y_fixed'],
                    self.workspace_bounds['box_width'],
                    self.workspace_bounds['box_height']
                )
                self.tar_x = robot_x
                self.tar_y = robot_y
                
                if destination:  # If grid destination selected
                    self.grid_target = destination[1]
                    self.start_move_thread("pnp", dest_id=destination[1])
                else:  # only object selected
                    self.start_move_thread("move")
            elif destination:  # Only grid selected
                self.grid_target = destination[1]
                self.thread.set_grid_target(destination[1])

    def on_auto_bt_2_clicked(self):
        if not self.is_auto_mode2:
            # Start auto mode 2
            self.is_auto_mode2 = True
            self.auto_bt_2.setText("Stop Auto 2")
            
            # Initialize PnP2 operations with fixed basket position
            self.pnp2_ops = PnP2Operations(
                self.thread, 
                self.workspace_bounds,
                fixed_basket_position=True  # Use fixed basket position mode or find avaliable space mode
            )
            if not self.pnp2_ops.start_operation():
                print("No baskets detected")
                self.is_auto_mode2 = False
                self.auto_bt_2.setText("Auto 2")
                return
                
            self.process_next_auto2_step()
        else:
            # Stop auto mode 2
            self.is_auto_mode2 = False
            self.auto_bt_2.setText("Auto 2")
            self.pnp2_ops = None

    def process_next_auto2_step(self):
        """Process next step in auto2 mode"""
        if not self.is_auto_mode2 or self.is_operation_running():
            return

        next_op = self.pnp2_ops.get_next_operation()
        if next_op:
            source_x, source_y, dest_x, dest_y = next_op
            self.tar_x = source_x
            self.tar_y = source_y
            self.tar_z = 199.99  # Set z-height for picking basket
            # Use pick_basket mode instead of pnp2 for basket
            if self.pnp2_ops.stage_sequence[self.pnp2_ops.current_stage_index] == 'move_baskets':
                self.start_move_thread("pick_basket", dest_x=dest_x, dest_y=dest_y)
            else:
                self.start_move_thread("pnp2", dest_x=dest_x, dest_y=dest_y)
        else:
            current_stage = self.pnp2_ops.stage_sequence[self.pnp2_ops.current_stage_index]
            if current_stage == 'check_left_cells':
                # Move to home position before checking left cells
                self.start_move_thread("home")
            elif current_stage == 'complete':
                self.start_move_thread("home")
                self.is_auto_mode2 = False
                self.auto_bt_2.setText("Auto 2")
            else:
                # Continue to next stage
                QtCore.QTimer.singleShot(1000, self.process_next_auto2_step)
                
if __name__ == "__main__":
    myobj = myclass()
    MainWindow.show()
    sys.exit(app.exec_())
