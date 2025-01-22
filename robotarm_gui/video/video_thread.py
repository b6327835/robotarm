from PyQt5.QtCore import QThread, pyqtSignal
import cv2
import numpy as np
from aruco_markers_detect import ArucoMarkerPosition
import pyrealsense2 as rs
from color_detection import ColorDetector
import pickle
import os
from video.detect_basket import BasketDetector
from utils.coordinate_converter import CoordinateConverter

class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)
    target_signal = pyqtSignal(float, float)
    grid_position_signal = pyqtSignal(float, float)
    available_positions_signal = pyqtSignal(dict)
    workspace_bounds_signal = pyqtSignal(float, float, float, float)

    def __init__(self, use_realsense=True, use_calibration=False, use_raw_coordinates=False, use_interpolation=False):
        super().__init__()
        self.detect_mode = "black"
        self.use_realsense = use_realsense
        self.use_calibration = use_calibration
        self.use_raw_coordinates = use_raw_coordinates
        self.use_interpolation = use_interpolation
        self.cap = None
        self.current_target_id = None
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
            #webcam initialization
            for index in range(2):
                print(f"Attempting to open camera index {index}")
                self.cap = cv2.VideoCapture(index)
                if self.cap.isOpened():
                    print(f"Successfully opened camera at index {index}")
                    break
            
            if not self.cap or not self.cap.isOpened():
                raise RuntimeError("Error: Could not open any webcam. Please check connections.")

            # Set camera properties
            self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            self.cap.set(cv2.CAP_PROP_FOCUS, 0)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Initialize detector with existing camera
        self.aruco_detector = ArucoMarkerPosition(display_output=False, existing_cap=self.cap)

        # Load camera calibration data if use_calibration is True
        self.camera_matrix = None
        self.dist_coeffs = None
        if self.use_calibration:  # Only load calibration if flag is True
            calibration_file = '../../../calibration/realsense_calibration.pkl' if use_realsense else '../../../calibration/camera_calibration.pkl'
            
            if os.path.exists(calibration_file):
                with open(calibration_file, 'rb') as f:
                    calibration_data = pickle.load(f)
                    self.camera_matrix = calibration_data['camera_matrix']
                    self.dist_coeffs = calibration_data['dist_coeffs']
                    print(f"Camera calibration loaded successfully from {calibration_file}")

        self.basket_detector = BasketDetector()

        # Add frame buffering
        self.frame_buffer_size = 60
        self.position_buffer = []
        self.marker_position_buffer = []
        self.object_position_buffer = []
        self.grid_position_buffer = []
        self.basket_info_buffer = []

        # Add smoothing parameters
        self.frame_buffer_size = 60  # Increased from previous value
        self.min_detection_confidence = 0.7  # Minimum confidence for object detection
        self.position_threshold = 5  # Pixel threshold for position changes
        self.depth_threshold = 50  # mm threshold for depth changes
        
        # Add tracking buffers
        self.position_buffer = []
        self.last_valid_position = None
        self.last_valid_depth = None
        self.stable_count = 0
        self.min_stable_frames = 10  # Number of frames position must be stable

    def set_grid_target(self, target_id):
        """Set the target grid position (e.g., 'L3' or 'R5')"""
        self.current_target_id = target_id

    def average_positions(self, positions_list):
        if not positions_list:
            return None
        avg_positions = {}
        # Get all unique keys across all dictionaries
        all_keys = set().union(*positions_list)
        
        for key in all_keys:
            # Get all values for this key that exist
            values = [pos[key] for pos in positions_list if key in pos]
            if values:
                # Average the x,y coordinates separately
                avg_x = sum(v[0] for v in values) / len(values)
                avg_y = sum(v[1] for v in values) / len(values)
                avg_positions[key] = (avg_x, avg_y)
        return avg_positions

    def average_basket_info(self, basket_info_list):
        if not basket_info_list:
            return None
        
        avg_info = []
        # Group basket infos by their relative position
        for i in range(len(basket_info_list[0])):  # Assume all lists have same number of baskets
            basket_group = [info[i] for info in basket_info_list if i < len(info)]
            if basket_group:
                avg_basket = {
                    'box': np.mean([b['box'] for b in basket_group], axis=0),
                    'center': np.mean([b['center'] for b in basket_group], axis=0),
                    'dimensions': np.mean([b['dimensions'] for b in basket_group], axis=0),
                    'angle': np.mean([b['angle'] for b in basket_group]),
                    'grid_params': basket_group[0]['grid_params']  # These don't need averaging
                }
                avg_info.append(avg_basket)
        return avg_info

    def average_point(self, points_list):
        """Average a list of points"""
        if not points_list:
            return None
        x_avg = sum(p[0] for p in points_list) / len(points_list)
        y_avg = sum(p[1] for p in points_list) / len(points_list)
        return (x_avg, y_avg)

    def is_position_stable(self, new_pos, last_pos, threshold=5):
        """Check if position has changed significantly"""
        if last_pos is None or new_pos is None:
            return False
        return abs(new_pos[0] - last_pos[0]) < threshold and abs(new_pos[1] - last_pos[1]) < threshold

    def apply_moving_average(self, positions, window_size=5):
        """Apply moving average to smooth position data"""
        if len(positions) < window_size:
            return positions[-1] if positions else None
        
        recent_positions = positions[-window_size:]
        avg_x = sum(p[0] for p in recent_positions) / window_size
        avg_y = sum(p[1] for p in recent_positions) / window_size
        return (avg_x, avg_y)

    def filter_outliers(self, positions, threshold=2.0):
        """Remove statistical outliers from position data"""
        if not positions:
            return []
        
        x_values = [p[0] for p in positions]
        y_values = [p[1] for p in positions]
        
        x_mean = np.mean(x_values)
        y_mean = np.mean(y_values)
        x_std = np.std(x_values)
        y_std = np.std(y_values)
        
        filtered = [(x, y) for x, y in positions if (
            abs(x - x_mean) < threshold * x_std and 
            abs(y - y_mean) < threshold * y_std
        )]
        
        return filtered

    def run(self):
        # Initialize buffers for various measurements
        target_buffer = []
        depth_buffer = []
        corners_buffer = []
        
        while True:
            # Initialize target variables at start of loop
            target_x = None
            target_y = None
            depth_value = None
            
            # Initialize lists at the start of each loop
            detected_objects = []
            pickable_objects_bottom = []
            pickable_objects_top = []
            basket_infos = None
            detected_object_positions = []

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

            # Apply undistortion only if calibration is enabled and data is available
            if self.use_calibration and self.camera_matrix is not None and self.dist_coeffs is not None:
                cv_img = cv2.undistort(cv_img, self.camera_matrix, self.dist_coeffs)

            cv_img = cv2.resize(cv_img, (640, 480))
            # cv_img = cv2.imread("test/workspace_test_01.png")
            
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

                # Store marker positions in buffer
                self.marker_position_buffer.append(valid_positions)
                if len(self.marker_position_buffer) > self.frame_buffer_size:
                    self.marker_position_buffer.pop(0)
                
                # Calculate average marker positions
                avg_positions = self.average_positions(self.marker_position_buffer)
                if avg_positions:
                    valid_positions = avg_positions
                    self.aruco_detector.update_marker_positions(valid_positions)

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
                    temp_positions[marker_id] = pos

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
                # Get corners in order (0,1,2,3)
                corners_ordered = [temp_positions[i] for i in range(4)]
                
                # Draw original rectangle sides
                for i in range(4):
                    pt1 = corners_ordered[i]
                    pt2 = corners_ordered[(i + 1) % 4]
                    pt1_int = (int(pt1[0]), int(pt1[1]))
                    pt2_int = (int(pt2[0]), int(pt2[1]))
                    cv2.line(cv_img, pt1_int, pt2_int, (0, 255, 0), 2)

                # Draw extended rectangle
                # Calculate width and height of original rectangle
                rect_width = max(corners_ordered[1][0], corners_ordered[2][0]) - min(corners_ordered[0][0], corners_ordered[3][0])
                rect_height = max(corners_ordered[2][1], corners_ordered[3][1]) - min(corners_ordered[0][1], corners_ordered[1][1])
                
                # Calculate extension points with actual gap
                gap = 10  # 50-pixel gap
                # First draw the original bottom rectangle
                bottom_corners = [
                    corners_ordered[0],  # top-left
                    corners_ordered[1],  # top-right
                    corners_ordered[2],  # bottom-right
                    corners_ordered[3]   # Original bottom-left
                ]
                
                # draw the top rectangle with gap
                top_corners = [
                    (corners_ordered[0][0], corners_ordered[0][1] - gap),  # Top-left start
                    (corners_ordered[1][0], corners_ordered[1][1] - gap),  # Top-right start
                    (corners_ordered[1][0], corners_ordered[1][1] - gap - rect_height),  # Top-right end
                    (corners_ordered[0][0], corners_ordered[0][1] - gap - rect_height)   # Top-left end
                ]
                
                # Draw bottom rectangle
                for i in range(4):
                    pt1 = bottom_corners[i]
                    pt2 = bottom_corners[(i + 1) % 4]
                    pt1_int = (int(pt1[0]), int(pt1[1]))
                    pt2_int = (int(pt2[0]), int(pt2[1]))
                    cv2.line(cv_img, pt1_int, pt2_int, (0, 255, 0), 2)

                # Draw top rectangle
                for i in range(4):
                    pt1 = top_corners[i]
                    pt2 = top_corners[(i + 1) % 4]
                    pt1_int = (int(pt1[0]), int(pt1[1]))
                    pt2_int = (int(pt2[0]), int(pt2[1]))
                    cv2.line(cv_img, pt1_int, pt2_int, (0, 255, 0), 2)

                # Add horizontal split line for top rectangle
                left_mid_x = (top_corners[0][0] + top_corners[3][0]) / 2
                left_mid_y = (top_corners[0][1] + top_corners[3][1]) / 2
                right_mid_x = (top_corners[1][0] + top_corners[2][0]) / 2
                right_mid_y = (top_corners[1][1] + top_corners[2][1]) / 2
                
                # Draw yellow horizontal split line
                cv2.line(cv_img,
                        (int(left_mid_x), int(left_mid_y)),
                        (int(right_mid_x), int(right_mid_y)),
                        (0, 255, 255), 2)  # Yellow line (BGR format)

                # Calculate midpoints of the top and bottom edges
                top_mid_x = (corners_ordered[0][0] + corners_ordered[1][0]) / 2
                top_mid_y = (corners_ordered[0][1] + corners_ordered[1][1]) / 2
                bottom_mid_x = (corners_ordered[2][0] + corners_ordered[3][0]) / 2
                bottom_mid_y = (corners_ordered[2][1] + corners_ordered[3][1]) / 2

                # Draw split line through midpoints
                cv2.line(cv_img,
                        (int(top_mid_x), int(top_mid_y)),
                        (int(bottom_mid_x), int(bottom_mid_y)),
                        (255, 0, 0), 2)  # Blue line

                # Update box boundaries
                x_coords = [x for x, y in temp_positions.values()]
                y_coords = [y for x, y in temp_positions.values()]
                x_fixed = min(x_coords)
                y_fixed = min(y_coords)
                box_width = max(x_coords) - x_fixed
                box_height = max(y_coords) - y_fixed

                # Store and emit workspace bounds
                self.workspace_bounds = {
                    'x_fixed': x_fixed,
                    'y_fixed': y_fixed,
                    'box_width': box_width,
                    'box_height': box_height
                }
                self.workspace_bounds_signal.emit(x_fixed, y_fixed, box_width, box_height)

                # check if a point is on the left side of the split line
                def is_left_side(px, py):
                    # Vector from top mid to bottom mid
                    dx = bottom_mid_x - top_mid_x
                    dy = bottom_mid_y - top_mid_y
                    # Vector from top mid to point
                    pdx = px - top_mid_x
                    pdy = py - top_mid_y
                    # Cross product
                    return (dx * pdy - dy * pdx) > 0

                # check if point is in top half of upper rectangle
                def is_top_half(px, py, left_mid_x, left_mid_y, right_mid_x, right_mid_y):
                    # Vector from left to right of split line
                    dx = right_mid_x - left_mid_x
                    dy = right_mid_y - left_mid_y
                    # Vector from left mid to point
                    pdx = px - left_mid_x
                    pdy = py - left_mid_y
                    # Cross product (removed negative sign to swap sides)
                    return (dx * pdy - dy * pdx) > 0

                # Color object detection using ColorDetector
                blurred = cv2.GaussianBlur(cv_img, (5, 5), 0)
                hsv_img = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
                
                # Get and process color mask
                color_mask = ColorDetector.get_color_mask(hsv_img, self.detect_mode)
                color_mask = ColorDetector.process_mask(color_mask)
                
                contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                detected_objects = []
                pickable_objects_bottom = []  # For bottom rectangle
                pickable_objects_top = []     # For top rectangle
                previous_centers = []  # List to store centers of detected objects

                def check_min_distance(center, previous_centers, min_distance=1):
                    for prev_center in previous_centers:
                        dist = np.sqrt((center[0] - prev_center[0])**2 + (center[1] - prev_center[1])**2)
                        if dist < min_distance:
                            return False
                    return True

                for contour in contours:
                    area = cv2.contourArea(contour)
                    if 10 <= area <= 150:  # modified area thresholds
                        perimeter = cv2.arcLength(contour, True)
                        if perimeter > 0:
                            circularity = 4 * np.pi * area / (perimeter * perimeter)
                            
                            # Increased circularity threshold for more strict detection
                            if circularity > self.min_detection_confidence:
                                M = cv2.moments(contour)
                                if M['m00'] > 0:
                                    # center coordinates
                                    center_x = M['m10'] / M['m00']
                                    center_y = M['m01'] / M['m00']
                                    
                                    # Check minimum distance from previously detected objects
                                    if not check_min_distance((center_x, center_y), previous_centers):
                                        continue
                                    
                                    previous_centers.append((center_x, center_y))
                                    # Check if object is in bottom rectangle
                                    in_bottom_rect = (x_fixed <= center_x <= x_fixed + box_width) and (y_fixed <= center_y <= y_fixed + box_height)
                                    
                                    # Check if object is in top rectangle
                                    top_rect_y_start = corners_ordered[0][1] - gap - rect_height
                                    top_rect_y_end = corners_ordered[0][1] - gap
                                    in_top_rect = (x_fixed <= center_x <= x_fixed + box_width) and (top_rect_y_start <= center_y <= top_rect_y_end)

                                    if in_bottom_rect:
                                        # logic for bottom rectangle
                                        is_pickable = is_left_side(center_x, center_y)
                                        object_info = ("Circle", center_x, center_y, is_pickable, "bottom")
                                        detected_objects.append(object_info)
                                        if is_pickable:
                                            pickable_objects_bottom.append(object_info)
                                    elif in_top_rect:
                                        # logic for top rectangle
                                        is_pickable = is_top_half(center_x, center_y, left_mid_x, left_mid_y, right_mid_x, right_mid_y)
                                        object_info = ("Circle", center_x, center_y, is_pickable, "top")
                                        detected_objects.append(object_info)
                                        if is_pickable:
                                            pickable_objects_top.append(object_info)

                                    # Store position in buffer
                                    current_pos = (center_x, center_y)
                                    self.position_buffer.append(current_pos)
                                    if len(self.position_buffer) > self.frame_buffer_size:
                                        self.position_buffer.pop(0)
                                    
                                    # Filter outliers and apply moving average
                                    filtered_positions = self.filter_outliers(self.position_buffer)
                                    if filtered_positions:
                                        smoothed_pos = self.apply_moving_average(filtered_positions)
                                        
                                        # Check position stability
                                        if self.is_position_stable(smoothed_pos, self.last_valid_position):
                                            self.stable_count += 1
                                        else:
                                            self.stable_count = 0
                                        
                                        # Only update position if stable for minimum frames
                                        if self.stable_count >= self.min_stable_frames:
                                            self.last_valid_position = smoothed_pos
                                            center_x, center_y = smoothed_pos

                # baskets detection
                detected_object_positions = [(x, y) for _, x, y, _, _ in detected_objects]
                basket_infos = self.basket_detector.detect_basket(hsv_img, x_fixed, box_width, box_height, y_fixed)
                if basket_infos:  # If any baskets were detected
                    grid_positions = BasketDetector.draw_basket_grid(cv_img, basket_infos, detected_object_positions)
                    if grid_positions:
                        self.available_positions['grid'] = grid_positions

                # Sort objects by x coordinate separately for each rectangle
                pickable_objects_bottom.sort(key=lambda obj: obj[1])
                pickable_objects_top.sort(key=lambda obj: obj[1])
                detected_objects.sort(key=lambda obj: obj[1])

                # After object detection, store target in buffer
                if target_x is not None and target_y is not None:
                    target_buffer.append((target_x, target_y))
                    if len(target_buffer) > self.frame_buffer_size:
                        target_buffer.pop(0)

                    # Only emit target signal when we have enough samples
                    if len(target_buffer) >= self.frame_buffer_size:
                        avg_target = self.average_point(target_buffer)
                        if avg_target:
                            tar_x, tar_y = CoordinateConverter.to_robot_coordinates(
                                avg_target[0], avg_target[1],
                                self.workspace_bounds['x_fixed'],
                                self.workspace_bounds['y_fixed'],
                                self.workspace_bounds['box_width'],
                                self.workspace_bounds['box_height']
                            )
                            self.target_signal.emit(tar_x, tar_y)

                # For depth measurements (RealSense only), add check for None
                if self.use_realsense and depth_frame and depth_value is not None:
                    # Apply temporal filtering to depth values
                    depth_buffer.append(depth_value)
                    if len(depth_buffer) > self.frame_buffer_size:
                        depth_buffer.pop(0)
                    
                    # Calculate median depth instead of mean
                    valid_depths = [d for d in depth_buffer if d is not None]
                    if valid_depths:
                        current_depth = np.median(valid_depths)
                        
                        # Only update depth if change is significant
                        if (self.last_valid_depth is None or 
                            abs(current_depth - self.last_valid_depth) > self.depth_threshold):
                            self.last_valid_depth = current_depth
                            depth_mm = int(current_depth * 1000)
                        else:
                            depth_mm = int(self.last_valid_depth * 1000)

                # Only emit signal for the first pickable object
                if pickable_objects_bottom:
                    highest_priority = pickable_objects_bottom[0]
                    target_x, target_y = highest_priority[1], highest_priority[2]
                    
                    if self.use_realsense and depth_frame:
                        try:
                            depth_value = depth_frame.get_distance(int(target_x), int(target_y))
                            if depth_value > 0:  # Validate depth reading
                                depth_mm = int(depth_value * 1000)
                            else:
                                depth_value = None
                                depth_mm = None
                        except:
                            depth_value = None
                            depth_mm = None

                    tar_x, tar_y = CoordinateConverter.to_robot_coordinates(
                        target_x, target_y, x_fixed, y_fixed, box_width, box_height,
                        use_raw=self.use_raw_coordinates,
                        use_interpolation=self.use_interpolation
                    )
                    self.target_signal.emit(tar_x, tar_y)

                elif pickable_objects_top:
                    highest_priority = pickable_objects_top[0]
                    target_x, target_y = highest_priority[1], highest_priority[2]
                    
                    if self.use_realsense and depth_frame:
                        depth_value = depth_frame.get_distance(int(target_x), int(target_y))
                        depth_mm = float(depth_value * 1000)
                    else:
                        depth_mm = None

                    tar_x, tar_y = CoordinateConverter.to_robot_coordinates(
                        target_x, target_y, x_fixed, y_fixed, box_width, box_height,
                        use_raw=self.use_raw_coordinates,
                        use_interpolation=self.use_interpolation
                    )
                    self.target_signal.emit(tar_x, tar_y)

                # only show details for highest priority object
                first_priority_bottom = None
                first_priority_top = None
                
                # First pass to find highest priority objects
                for obj in detected_objects:
                    shape_type, target_x, target_y, is_pickable, rect_location = obj
                    if is_pickable:
                        if rect_location == "bottom" and first_priority_bottom is None:
                            first_priority_bottom = obj
                        elif rect_location == "top" and first_priority_top is None:
                            first_priority_top = obj

                # Draw all detected objects
                for obj in detected_objects:
                    shape_type, target_x, target_y, is_pickable, rect_location = obj
                    color = (0, 255, 0) if is_pickable else (0, 0, 255)
                    
                    # Convert to integer coordinates for drawing
                    draw_x = int(target_x)
                    draw_y = int(target_y)
                    
                    # Draw basic circle for all objects
                    cv2.circle(cv_img, (draw_x, draw_y), 3, color, -1)
                    
                    # Only draw detailed information for first priority objects
                    if obj == first_priority_bottom or obj == first_priority_top:
                        # Convert to robot coordinates
                        tar_x, tar_y = CoordinateConverter.to_robot_coordinates(
                            target_x, target_y, x_fixed, y_fixed, box_width, box_height
                        )

                        status_text = f"{'B' if rect_location == 'bottom' else 'T'}1:Pickable"

                        # Get depth for priority object if using RealSense
                        if self.use_realsense and depth_frame:
                            depth_value = depth_frame.get_distance(draw_x, draw_y)
                            depth_mm = int(depth_value * 1000)
                            depth_text = f"D:{depth_mm}mm"
                        else:
                            depth_text = ""

                        # Draw detailed information only for priority object
                        cv2.putText(
                            cv_img,
                            status_text,
                            (draw_x - 10, draw_y - 15),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.4,
                            color,
                            1,
                        )
                        cv2.putText(
                            cv_img,
                            f"Real:X{tar_x:.2f},Y{tar_y:.2f} {depth_text}",
                            (draw_x - 10, draw_y),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.4,
                            color,
                            1,
                        )

            # After basket detection and drawing, add:
            if self.current_target_id and basket_infos:
                grid_pos = self.basket_detector.get_grid_position_by_id(cv_img, basket_infos, self.current_target_id)
                if grid_pos:
                    tar_x, tar_y = CoordinateConverter.to_robot_coordinates(
                        grid_pos[0], grid_pos[1], x_fixed, y_fixed, box_width, box_height
                    )
                    self.grid_position_signal.emit(tar_x, tar_y)
                    self.current_target_id = None  # Reset after emitting

            # After detecting objects, update available positions
            # Store positions in buffer for averaging
            if detected_objects:
                self.object_position_buffer.append({
                    'pickable': [(obj[1], obj[2]) for obj in detected_objects if obj[3]],
                    'non_pickable': [(obj[1], obj[2]) for obj in detected_objects if not obj[3]]
                })
                if len(self.object_position_buffer) > self.frame_buffer_size:
                    self.object_position_buffer.pop(0)

            # Calculate stable averaged positions
            self.available_positions['pickable_objects'] = []
            self.available_positions['non_pickable_objects'] = []

            if len(self.object_position_buffer) >= 3:  # Use minimum 3 frames for stability
                # Group positions by proximity
                pickable_groups = self.group_positions([pos for frame in self.object_position_buffer 
                                                     for pos in frame['pickable']], threshold=10)
                non_pickable_groups = self.group_positions([pos for frame in self.object_position_buffer 
                                                          for pos in frame['non_pickable']], threshold=10)
                
                # Average each group
                for group in pickable_groups:
                    if len(group) >= 3:  # Only use groups that appear in at least 3 frames
                        avg_x = sum(x for x, _ in group) / len(group)
                        avg_y = sum(y for _, y in group) / len(group)
                        self.available_positions['pickable_objects'].append((avg_x, avg_y))
                
                for group in non_pickable_groups:
                    if len(group) >= 3:
                        avg_x = sum(x for x, _ in group) / len(group)
                        avg_y = sum(y for _, y in group) / len(group)
                        self.available_positions['non_pickable_objects'].append((avg_x, avg_y))

            # Emit the averaged positions
            self.available_positions_signal.emit(self.available_positions)

            # After basket detection but before emitting available_positions
            if basket_infos:
                self.basket_info_buffer.append(basket_infos)
                if len(self.basket_info_buffer) > self.frame_buffer_size:
                    self.basket_info_buffer.pop(0)
                
                avg_basket_infos = self.average_basket_info(self.basket_info_buffer)
                if avg_basket_infos:
                    grid_positions = BasketDetector.draw_basket_grid(cv_img, avg_basket_infos, detected_object_positions)
                    if grid_positions:
                        self.available_positions['grid'] = grid_positions

            # Remove the position averaging for object list
            # We'll keep all detected objects instead of averaging them
            self.available_positions_signal.emit(self.available_positions)

            # Only average the target position for movement
            if len(self.position_buffer) >= self.frame_buffer_size and self.available_positions['pickable_objects']:
                first_pickable = self.available_positions['pickable_objects'][0]
                tar_x, tar_y = CoordinateConverter.to_robot_coordinates(
                    first_pickable[0], first_pickable[1],
                    self.workspace_bounds['x_fixed'],
                    self.workspace_bounds['y_fixed'],
                    self.workspace_bounds['box_width'],
                    self.workspace_bounds['box_height']
                )
                self.target_signal.emit(tar_x, tar_y)
                
            self.change_pixmap_signal.emit(cv_img)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        self.cap.release()
        cv2.destroyAllWindows()

    def group_positions(self, positions, threshold=10):
        """Group positions that are close to each other"""
        if not positions:
            return []
        
        groups = []
        used = set()
        
        for i, pos1 in enumerate(positions):
            if i in used:
                continue
                
            current_group = [pos1]
            used.add(i)
            
            for j, pos2 in enumerate(positions):
                if j in used:
                    continue
                    
                # Calculate distance between positions
                dist = np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
                if dist < threshold:
                    current_group.append(pos2)
                    used.add(j)
            
            groups.append(current_group)
        
        return groups

    def __del__(self):
        if self.use_realsense:
            self.pipeline.stop()
        else:
            self.cap.release()
