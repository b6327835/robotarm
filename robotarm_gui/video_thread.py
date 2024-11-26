from PyQt5.QtCore import QThread, pyqtSignal
import cv2
import numpy as np
from aruco_markers_detect import ArucoMarkerPosition

class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)
    target_signal = pyqtSignal(float, float)

    def __init__(self):
        super().__init__()
        self.detect_mode = "red"  # Default mode for round white objects
        self.cap = cv2.VideoCapture(0)
        
        if not self.cap.isOpened():
            print("Error: Could not open webcam.")
            exit()

        # Set camera properties for Logitech C922 Pro HD
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.cap.set(cv2.CAP_PROP_FOCUS, 0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Initialize detector with existing camera
        self.aruco_detector = ArucoMarkerPosition(display_output=False, existing_cap=self.cap)

    def run(self):
        while True:
            ret, cv_img = self.cap.read()
            if not ret:
                print("Error: Failed to grab cv_img.")
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

                # Color object detection based on mode
                blurred = cv2.GaussianBlur(cv_img, (5, 5), 0)
                hsv_img = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

                if self.detect_mode == "white":
                    # Define white color range with tighter thresholds
                    lower_color = np.array([0, 0, 167])
                    upper_color = np.array([95, 40, 255])
                elif self.detect_mode == "red":
                    # Define red color range (red wraps around in HSV)
                    lower_color1 = np.array([0, 120, 70])
                    upper_color1 = np.array([10, 255, 255])
                    lower_color2 = np.array([170, 120, 70])
                    upper_color2 = np.array([180, 255, 255])
                    
                    # Create mask for red color (combining both ranges)
                    mask1 = cv2.inRange(hsv_img, lower_color1, upper_color1)
                    mask2 = cv2.inRange(hsv_img, lower_color2, upper_color2)
                    color_mask = cv2.bitwise_or(mask1, mask2)
                else:
                    color_mask = np.zeros_like(hsv_img[:,:,0])

                # Create mask for selected color
                if self.detect_mode == "white":
                    color_mask = cv2.inRange(hsv_img, lower_color, upper_color)
                
                # Add morphological operations to clean up the mask
                kernel = np.ones((7,7), np.uint8)
                color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, kernel)
                color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, kernel)
                
                # Find contours in the mask
                contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                detected_objects = []

                for contour in contours:
                    area = cv2.contourArea(contour)
                    if 20 <= area <= 400:  # Updated area thresholds
                        # Rest of the contour processing
                        perimeter = cv2.arcLength(contour, True)
                        if perimeter > 0:
                            circularity = 4 * np.pi * area / (perimeter * perimeter)
                            
                            # Only process if the shape is roughly circular (circularity > 0.6)
                            if circularity > 0.6:
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
                    
                    y_relative = (target_y - y_fixed) / box_height
                    x_relative = (target_x - x_fixed) / box_width

                    tar_x = (135 - (y_relative * 135))-(4)# Maps from 135 to 0
                    tar_y = (145 - (x_relative * 140))-(0) # Maps from 145 to 5
                    if tar_x < 0:
                        tar_x = 0
                    self.target_signal.emit(tar_x, tar_y)

                # Draw detected objects with converted coordinates
                for priority, (shape_type, target_x, target_y) in enumerate(detected_objects, start=1):
                    cv2.circle(cv_img, (target_x, target_y), 5, (0, 0, 255), -1)
                    # Calculate converted coordinates for each object
                    y_rel = (target_y - y_fixed) / box_height
                    x_rel = (target_x - x_fixed) / box_width
                    conv_x = 135 - (y_rel * 135)
                    conv_y = 145 - (x_rel * 140)
                    
                    # Draw first line (pixel coordinates)
                    cv2.putText(
                        cv_img,
                        f"P{priority}:X{target_x},Y{target_y}",
                        (target_x - 10, target_y - 15),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4,
                        (0, 255, 0),
                        1,
                    )
                    # Draw second line (ROS coordinates)
                    cv2.putText(
                        cv_img,
                        f"Ros:X{conv_x:.2f},Y{conv_y:.2f}",
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
        self.cap.release()
