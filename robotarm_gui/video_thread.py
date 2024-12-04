from PyQt5.QtCore import QThread, pyqtSignal
import cv2
import numpy as np
from aruco_markers_detect import ArucoMarkerPosition
import pyrealsense2 as rs
from color_detection import ColorDetector    # Add this import

class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)
    target_signal = pyqtSignal(float, float)
    # Add new signal for grid start position
    grid_start_signal = pyqtSignal(float, float)

    def __init__(self, use_realsense=True):    # Add camera type parameter
        super().__init__()
        self.detect_mode = "black"
        self.use_realsense = use_realsense
        self.cap = None
        self.grid_start_found = False
        
        if self.use_realsense:
            # Initialize RealSense pipeline
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.pipeline.start(config)
            # Initialize align object to align depth frames to color frames
            self.align = rs.align(rs.stream.color)
        else:
            # Original webcam initialization code
            for index in range(2):
                print(f"Attempting to open camera index {index}")
                self.cap = cv2.VideoCapture(index)
                if self.cap.isOpened():
                    print(f"Successfully opened camera at index {index}")
                    break
            
            if not self.cap or not self.cap.isOpened():
                raise RuntimeError("Error: Could not open any webcam. Please check connections.")

            # Set camera properties for Logitech C922 Pro HD
            self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            self.cap.set(cv2.CAP_PROP_FOCUS, 0)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Initialize detector with existing camera
        self.aruco_detector = ArucoMarkerPosition(display_output=False, existing_cap=self.cap)

    def run(self):
        while True:
            if self.use_realsense:
                # Get frameset of color and depth
                frames = self.pipeline.wait_for_frames()
                # Align frames
                aligned_frames = self.align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue
                # Convert images to numpy arrays
                cv_img = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
                ret = True
            else:
                ret, cv_img = self.cap.read()
                depth_image = None

            if not ret:
                print("Error: Failed to grab frame.")
                break

            cv_img = cv2.resize(cv_img, (640, 480))
            
            # Fix marker detection
            corners, ids, _ = self.aruco_detector.detector.detectMarkers(cv_img)
            valid_positions = {}
            temp_positions = {}  # Store both current and estimated positions
            
            if ids is not None:
                # Draw detected markers
                cv2.aruco.drawDetectedMarkers(cv_img, corners, ids)
                
                for i in range(len(ids)):
                    marker_id = ids[i][0]
                    if 0 <= marker_id <= 3:
                        marker_corners = corners[i][0]
                        # Remove int() to keep float precision
                        center_x = np.mean(marker_corners[:, 0])  
                        center_y = np.mean(marker_corners[:, 1])
                        valid_positions[marker_id] = (center_x, center_y)
                        temp_positions[marker_id] = (center_x, center_y)
                        # For display only, convert to int
                        display_x = int(center_x)
                        display_y = int(center_y)
                        cv2.circle(cv_img, (display_x, display_y), 5, (0, 0, 255), -1)
                        cv2.putText(cv_img, str(marker_id), (display_x + 10, display_y + 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Update stored positions in aruco detector
            self.aruco_detector.update_marker_positions(valid_positions)
            remembered_positions = self.aruco_detector.get_valid_positions()

            # Draw remembered positions as yellow dots
            for marker_id, pos in remembered_positions.items():
                if marker_id not in valid_positions:
                    # Convert float coordinates to integers for drawing
                    display_pos = (int(pos[0]), int(pos[1]))
                    cv2.circle(cv_img, display_pos, 5, (0, 255, 255), -1)
                    cv2.putText(cv_img, str(marker_id), 
                                (display_pos[0] + 10, display_pos[1] + 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                    temp_positions[marker_id] = pos  # Keep original float values for calculations

            # If we have 3 markers, estimate the fourth
            if len(temp_positions) >= 3:
                missing_ids = set([0, 1, 2, 3]) - set(temp_positions.keys())
                for missing_id in missing_ids:
                    estimated_center = self.aruco_detector.estimate_missing_corner(temp_positions, missing_id)
                    if estimated_center:
                        temp_positions[missing_id] = estimated_center
                        # Draw blue dot and ID for estimated markers
                        cv2.circle(cv_img, estimated_center, 5, (255, 0, 0), -1)
                        cv2.putText(cv_img, str(missing_id), (estimated_center[0] + 10, estimated_center[1] + 10),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # Draw box if we have all 4 corners
            if len(temp_positions) == 4:
                for i in range(4):
                    pt1 = temp_positions[i]
                    pt2 = temp_positions[(i + 1) % 4]
                    # Convert to integers for drawing
                    pt1_int = (int(pt1[0]), int(pt1[1]))
                    pt2_int = (int(pt2[0]), int(pt2[1]))
                    cv2.line(cv_img, pt1_int, pt2_int, (0, 255, 0), 2)

                # Update box boundaries for object detection
                x_coords = [x for x, y in temp_positions.values()]
                y_coords = [y for x, y in temp_positions.values()]
                x_fixed = min(x_coords)
                y_fixed = min(y_coords)
                box_width = max(x_coords) - x_fixed
                box_height = max(y_coords) - y_fixed

                # Check for marker ID 4 inside the box boundaries
                if ids is not None:
                    for i in range(len(ids)):
                        marker_id = ids[i][0]
                        if marker_id == 4:
                            marker_corners = corners[i][0]
                            center_x = np.mean(marker_corners[:, 0])
                            center_y = np.mean(marker_corners[:, 1])
                            
                            # Check if marker 4 is inside the box
                            if (x_fixed <= center_x <= x_fixed + box_width and 
                                y_fixed <= center_y <= y_fixed + box_height):
                                # Convert to ROS coordinates like target objects
                                y_relative = (center_y - y_fixed) / box_height
                                x_relative = (center_x - x_fixed) / box_width
                                grid_x = (135 - (y_relative * 135)) - 4
                                grid_y = (145 - (x_relative * 140)) - 0
                                
                                if not self.grid_start_found:
                                    self.grid_start_signal.emit(grid_x * 0.001, grid_y * 0.001)
                                    cv2.circle(cv_img, (int(center_x), int(center_y)), 5, (255, 255, 0), -1)
                                    cv2.putText(cv_img, "Grid Start", (int(center_x) + 10, int(center_y) + 10),
                                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

                # Color object detection using new ColorDetector
                blurred = cv2.GaussianBlur(cv_img, (5, 5), 0)
                hsv_img = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
                
                # Get and process color mask
                color_mask = ColorDetector.get_color_mask(hsv_img, self.detect_mode)
                color_mask = ColorDetector.process_mask(color_mask)
                
                # Rest of the existing detection code
                contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                detected_objects = []

                for contour in contours:
                    area = cv2.contourArea(contour)
                    if 202 <= area <= 900:  # Updated area thresholds
                        # Rest of the contour processing
                        perimeter = cv2.arcLength(contour, True)
                        if perimeter > 0:
                            circularity = 4 * np.pi * area / (perimeter * perimeter)
                            
                            # Updated circularity threshold to 0.6
                            if circularity > 0.70:
                                M = cv2.moments(contour)
                                if M['m00'] > 0:
                                    target_x = int(M['m10'] / M['m00'])
                                    target_y = int(M['m01'] / M['m00'])
                                    
                                    # Only add if within the fixed box
                                    if (x_fixed <= target_x <= x_fixed + box_width) and (y_fixed <= target_y <= y_fixed + box_height):
                                        detected_objects.append(("Circle", target_x, target_y))

                detected_objects.sort(key=lambda obj: obj[1])

                if detected_objects:
                    highest_priority = detected_objects[0]
                    target_x, target_y = highest_priority[1], highest_priority[2]
                    
                    # Add depth measurement for RealSense
                    if self.use_realsense and depth_frame:
                        depth_value = depth_frame.get_distance(target_x, target_y)
                        # Convert depth to mm
                        depth_mm = int(depth_value * 1000)
                    else:
                        depth_mm = None

                    y_relative = (target_y - y_fixed) / box_height
                    x_relative = (target_x - x_fixed) / box_width

                    tar_x = (135 - (y_relative * 135))-(4)
                    tar_y = (145 - (x_relative * 140))-(0)
                    if tar_x < 0:
                        tar_x = 0
                    self.target_signal.emit(tar_x, tar_y)

                # Draw detected objects with converted coordinates and depth
                for priority, (shape_type, target_x, target_y) in enumerate(detected_objects, start=1):
                    cv2.circle(cv_img, (target_x, target_y), 5, (0, 0, 255), -1)
                    y_rel = (target_y - y_fixed) / box_height
                    x_rel = (target_x - x_fixed) / box_width
                    conv_x = 135 - (y_rel * 135)
                    conv_y = 145 - (x_rel * 140)
                    
                    # Get depth for this object if using RealSense
                    if self.use_realsense and depth_frame:
                        depth_value = depth_frame.get_distance(target_x, target_y)
                        depth_mm = int(depth_value * 1000)
                        depth_text = f"D:{depth_mm}mm"
                    else:
                        depth_text = ""

                    # Draw coordinates and depth
                    cv2.putText(
                        cv_img,
                        f"P{priority}:X{target_x},Y{target_y}",
                        (target_x - 10, target_y - 15),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4,
                        (0, 255, 0),
                        1,
                    )
                    cv2.putText(
                        cv_img,
                        f"Ros:X{conv_x:.2f},Y{conv_y:.2f} {depth_text}",
                        (target_x - 10, target_y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4,
                        (0, 255, 0),
                        1,
                    )

            self.change_pixmap_signal.emit(cv_img)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        self.cap.release()
        cv2.destroyAllWindows()

    def __del__(self):
        if self.use_realsense:
            self.pipeline.stop()
        else:
            self.cap.release()
