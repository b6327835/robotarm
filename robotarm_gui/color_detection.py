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
            upper_color = np.array([180, 255, 161])  # Modified V max to 125, S max to 255
            return cv2.inRange(hsv_img, lower_color, upper_color)
        
        return np.zeros_like(hsv_img[:,:,0])

    @staticmethod
    def process_mask(color_mask, kernel_size=3, erode_count=1, dilate_count=0):
        """
        Process the color mask with configurable kernel size and erosion/dilation counts
        Args:
            color_mask: Input binary mask
            kernel_size: Size of the kernel (default: 3)
            erode_count: Number of erosion iterations (default: 1)
            dilate_count: Number of dilation iterations (default: 0)
        """
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        
        # Apply erosion
        if erode_count > 0:
            color_mask = cv2.erode(color_mask, kernel, iterations=erode_count)
        
        # Apply dilation
        if dilate_count > 0:
            color_mask = cv2.dilate(color_mask, kernel, iterations=dilate_count)
            
        return color_mask