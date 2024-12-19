import cv2
import numpy as np

class ColorDetector:
    @staticmethod
    def get_color_mask(hsv_img, detect_mode):
        if detect_mode == "white":
            lower_color = np.array([0, 0, 185])
            upper_color = np.array([95, 255, 255])
            return cv2.inRange(hsv_img, lower_color, upper_color)
        
        elif detect_mode == "red":
            lower_color1 = np.array([0, 120, 70])
            upper_color1 = np.array([10, 255, 255])
            lower_color2 = np.array([170, 120, 70])
            upper_color2 = np.array([180, 255, 255])
            
            mask1 = cv2.inRange(hsv_img, lower_color1, upper_color1)
            mask2 = cv2.inRange(hsv_img, lower_color2, upper_color2)
            return cv2.bitwise_or(mask1, mask2)
        
        elif detect_mode == "black":
            lower_color = np.array([0, 0, 0])
            upper_color = np.array([180, 100, 78])  # Modified S and V max values
            return cv2.inRange(hsv_img, lower_color, upper_color)
        
        return np.zeros_like(hsv_img[:,:,0])

    @staticmethod
    def process_mask(color_mask):
        kernel = np.ones((2,2), np.uint8)  # Changed kernel size to 2x2
        # Apply erosion 3 times
        for _ in range(3):
            color_mask = cv2.erode(color_mask, kernel)
        # Apply dilation 3 times
        for _ in range(3):
            color_mask = cv2.dilate(color_mask, kernel)
        return color_mask