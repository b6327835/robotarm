
from PyQt5.QtCore import QThread, pyqtSignal
import cv2
import numpy as np

class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)
    target_signal = pyqtSignal(float, float)

    def run(self):
        # Open the webcam
        cap = cv2.VideoCapture(0)

        # Check if the webcam was opened successfully
        if not cap.isOpened():
            print("Error: Could not open webcam.")
            exit()

        x_limit = 640
        y_limit = 480
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, x_limit)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, y_limit)

        # Fixed box coordinates
        x_fixed, y_fixed = 120, 145
        box_width, box_height = 235, 195

        while True:
            ret, cv_img = cap.read()
            if not ret:
                print("Error: Failed to grab cv_img.")
                break

            # Resize and convert to HSV color space for better color detection
            cv_img = cv2.resize(cv_img, (x_limit, y_limit))
            hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

            # Define red color range for detection
            lower_red1 = np.array([0, 100, 100])  # Lower bound for red
            upper_red1 = np.array([10, 255, 255]) # Upper bound for red
            lower_red2 = np.array([160, 100, 100]) # Second range for red
            upper_red2 = np.array([180, 255, 255]) # Second upper bound for red

            # Create masks for red color
            mask1 = cv2.inRange(hsv_img, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv_img, lower_red2, upper_red2)
            red_mask = mask1 | mask2

            # Find contours in the red mask
            contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            detected_objects = []  # Store detected objects with their details

            # Draw a fixed box
            cv2.rectangle(cv_img, (x_fixed, y_fixed), (x_fixed + box_width, y_fixed + box_height), (255, 0, 0), 2)

            for contour in contours:
                if cv2.contourArea(contour) > 100:  # Filter out small areas
                    # Get the center of the contour
                    M = cv2.moments(contour)
                    if M['m00'] > 0:
                        target_x = int(M['m10'] / M['m00'])
                        target_y = int(M['m01'] / M['m00'])

                        # Determine shape type based on contour approximation
                        epsilon = 0.02 * cv2.arcLength(contour, True)
                        approx = cv2.approxPolyDP(contour, epsilon, True)

                        # Check if the detected shape is a circle or a square
                        shape_type = "Unknown"
                        if len(approx) == 3:
                            shape_type = "Triangle"
                        elif len(approx) == 4:
                            aspect_ratio = float(cv2.boundingRect(approx)[2]) / cv2.boundingRect(approx)[3]
                            shape_type = "Square" if 0.9 <= aspect_ratio <= 1.1 else "Rectangle"
                        elif len(approx) > 4:
                            shape_type = "Circle"

                        # Check if the center is within the fixed box
                        if (x_fixed <= target_x <= x_fixed + box_width) and (y_fixed <= target_y <= y_fixed + box_height):
                            detected_objects.append((shape_type, target_x, target_y))

            # Sort objects by their x coordinate (from left to right)
            detected_objects.sort(key=lambda obj: obj[1])  # Sort by target_x

            # Emit the coordinates of the most prioritized object (the first in the sorted list)
            if detected_objects:
                highest_priority = detected_objects[0]
                target_x, target_y = highest_priority[1], highest_priority[2]

                # Convert target_x, target_y to tar_x, tar_y
                # Mapping target_x from 100 (left) to 345 (right)
                y_relative = (target_y - y_fixed) / box_height  # Relative to height
                x_relative = (target_x - x_fixed) / box_width   # Relative to width

                # Correct tar_x and tar_y mapping
                tar_x = 135 - (y_relative * (135 - 10))  # tar_x decreases as y_relative increases (top to bottom)
                tar_y = 150 - (x_relative * 150)         # tar_y decreases as x_relative increases (left to right)

                # Emit the corrected coordinates
                self.target_signal.emit(tar_x, tar_y)

            # Loop through the sorted objects to draw them
            for priority, (shape_type, target_x, target_y) in enumerate(detected_objects, start=1):
                # Draw a dot at the center of the object
                cv2.circle(cv_img, (target_x, target_y), 5, (0, 0, 255), -1)
                cv2.putText(
                    cv_img,
                    f"P:{priority}, X:{target_x}, Y:{target_y}",
                    (target_x - 10, target_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )

            # Show the cv_img with detected objects, center dots, lines, and text
            display_img = cv2.resize(cv_img, (x_limit, y_limit))
            self.change_pixmap_signal.emit(display_img)

            # Press 'q' to exit
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        cap.release()
        cv2.destroyAllWindows()