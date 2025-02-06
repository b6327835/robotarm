import cv2
import numpy as np

class RedBasketDetector:
    def __init__(self):
        cv2.namedWindow('Red Basket Settings')
        
        def nothing(x): pass
        
        # Simplified parameters matching red_b.py
        cv2.createTrackbar('Low H1', 'Red Basket Settings', 0, 180, nothing)
        cv2.createTrackbar('High H1', 'Red Basket Settings', 10, 180, nothing)
        cv2.createTrackbar('Low H2', 'Red Basket Settings', 160, 180, nothing)
        cv2.createTrackbar('High H2', 'Red Basket Settings', 180, 180, nothing)
        cv2.createTrackbar('Low S', 'Red Basket Settings', 100, 255, nothing)
        cv2.createTrackbar('High S', 'Red Basket Settings', 255, 255, nothing)
        cv2.createTrackbar('Low V', 'Red Basket Settings', 100, 255, nothing)
        cv2.createTrackbar('High V', 'Red Basket Settings', 255, 255, nothing)
        cv2.createTrackbar('Min Area', 'Red Basket Settings', 500, 20000, nothing)
        cv2.createTrackbar('Kernel Size', 'Red Basket Settings', 5, 20, nothing)
        cv2.createTrackbar('Erosion Iter', 'Red Basket Settings', 1, 5, nothing)
        cv2.createTrackbar('Dilation Iter', 'Red Basket Settings', 1, 5, nothing)

    def detect(self, frame, depth_image=None, use_realsense=False):
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Get parameters from trackbars
        lower_red1 = np.array([
            cv2.getTrackbarPos('Low H1', 'Red Basket Settings'),
            cv2.getTrackbarPos('Low S', 'Red Basket Settings'),
            cv2.getTrackbarPos('Low V', 'Red Basket Settings')
        ])
        upper_red1 = np.array([
            cv2.getTrackbarPos('High H1', 'Red Basket Settings'),
            cv2.getTrackbarPos('High S', 'Red Basket Settings'),
            cv2.getTrackbarPos('High V', 'Red Basket Settings')
        ])
        lower_red2 = np.array([
            cv2.getTrackbarPos('Low H2', 'Red Basket Settings'),
            cv2.getTrackbarPos('Low S', 'Red Basket Settings'),
            cv2.getTrackbarPos('Low V', 'Red Basket Settings')
        ])
        upper_red2 = np.array([
            cv2.getTrackbarPos('High H2', 'Red Basket Settings'),
            cv2.getTrackbarPos('High S', 'Red Basket Settings'),
            cv2.getTrackbarPos('High V', 'Red Basket Settings')
        ])
        
        # Create mask
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)
        
        # Apply morphological operations
        kernel_size = cv2.getTrackbarPos('Kernel Size', 'Red Basket Settings')
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        red_mask = cv2.erode(red_mask, kernel, iterations=cv2.getTrackbarPos('Erosion Iter', 'Red Basket Settings'))
        red_mask = cv2.dilate(red_mask, kernel, iterations=cv2.getTrackbarPos('Dilation Iter', 'Red Basket Settings'))
        
        # Find and process contours
        red_result = frame.copy()
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected_objects = []
        
        min_area = cv2.getTrackbarPos('Min Area', 'Red Basket Settings')
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > min_area:
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int64(box)
                
                # Calculate center
                cx = int(rect[0][0])
                cy = int(rect[0][1])
                
                detected_objects.append({
                    'center': (cx, cy),
                    'area': area,
                    'box': box
                })
                
                # Draw detection
                cv2.drawContours(red_result, [box], 0, (0, 255, 0), 2)
        
        return red_result, red_mask, detected_objects
