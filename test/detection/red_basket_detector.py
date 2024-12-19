import cv2
import numpy as np

class RedBasketDetector:
    def __init__(self):
        cv2.namedWindow('Red Basket Settings')
        
        def nothing(x): pass
        
        # HSV range for red color (two ranges for red hue)
        cv2.createTrackbar('Low H1', 'Red Basket Settings', 0, 10, nothing)
        cv2.createTrackbar('High H1', 'Red Basket Settings', 10, 10, nothing)
        cv2.createTrackbar('Low H2', 'Red Basket Settings', 160, 180, nothing)
        cv2.createTrackbar('High H2', 'Red Basket Settings', 180, 180, nothing)
        cv2.createTrackbar('Low S', 'Red Basket Settings', 100, 255, nothing)
        cv2.createTrackbar('High S', 'Red Basket Settings', 255, 255, nothing)
        cv2.createTrackbar('Low V', 'Red Basket Settings', 100, 255, nothing)
        cv2.createTrackbar('High V', 'Red Basket Settings', 255, 255, nothing)
        
        # Shape parameters
        cv2.createTrackbar('Min Area', 'Red Basket Settings', 500, 5000, nothing)
        cv2.createTrackbar('Max Area', 'Red Basket Settings', 2000, 5000, nothing)
        cv2.createTrackbar('Min Ratio', 'Red Basket Settings', 2, 10, nothing)  # x10 for precision
        cv2.createTrackbar('Max Ratio', 'Red Basket Settings', 8, 10, nothing)  # x10 for precision
        
        # Processing parameters
        cv2.createTrackbar('Kernel Size', 'Red Basket Settings', 3, 20, nothing)
        cv2.createTrackbar('Erosion Iter', 'Red Basket Settings', 1, 5, nothing)
        cv2.createTrackbar('Dilation Iter', 'Red Basket Settings', 1, 5, nothing)

    def get_parameters(self):
        return {
            'low_h1': cv2.getTrackbarPos('Low H1', 'Red Basket Settings'),
            'high_h1': cv2.getTrackbarPos('High H1', 'Red Basket Settings'),
            'low_h2': cv2.getTrackbarPos('Low H2', 'Red Basket Settings'),
            'high_h2': cv2.getTrackbarPos('High H2', 'Red Basket Settings'),
            'low_s': cv2.getTrackbarPos('Low S', 'Red Basket Settings'),
            'high_s': cv2.getTrackbarPos('High S', 'Red Basket Settings'),
            'low_v': cv2.getTrackbarPos('Low V', 'Red Basket Settings'),
            'high_v': cv2.getTrackbarPos('High V', 'Red Basket Settings'),
            'min_area': cv2.getTrackbarPos('Min Area', 'Red Basket Settings'),
            'max_area': cv2.getTrackbarPos('Max Area', 'Red Basket Settings'),
            'min_ratio': cv2.getTrackbarPos('Min Ratio', 'Red Basket Settings') / 10.0,
            'max_ratio': cv2.getTrackbarPos('Max Ratio', 'Red Basket Settings') / 10.0,
            'kernel_size': cv2.getTrackbarPos('Kernel Size', 'Red Basket Settings'),
            'erosion_iter': cv2.getTrackbarPos('Erosion Iter', 'Red Basket Settings'),
            'dilation_iter': cv2.getTrackbarPos('Dilation Iter', 'Red Basket Settings')
        }

    def detect(self, frame, depth_image=None, use_realsense=False):
        params = self.get_parameters()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create mask for red color (combining two ranges)
        lower_red1 = np.array([params['low_h1'], params['low_s'], params['low_v']])
        upper_red1 = np.array([params['high_h1'], params['high_s'], params['high_v']])
        lower_red2 = np.array([params['low_h2'], params['low_s'], params['low_v']])
        upper_red2 = np.array([params['high_h2'], params['high_s'], params['high_v']])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # Apply morphological operations
        kernel_size = params['kernel_size'] + (1 if params['kernel_size'] % 2 == 0 else 0)
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        red_mask = cv2.erode(red_mask, kernel, iterations=params['erosion_iter'])
        red_mask = cv2.dilate(red_mask, kernel, iterations=params['dilation_iter'])

        # Find contours
        red_result = frame.copy()
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detected_objects = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if params['min_area'] < area < params['max_area']:
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w)/h if h != 0 else 0
                
                # Check if the shape is rectangular and thin
                if params['min_ratio'] < aspect_ratio < params['max_ratio']:
                    rect = cv2.minAreaRect(contour)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    
                    # Calculate center
                    M = cv2.moments(contour)
                    if M['m00'] > 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        
                        object_info = {
                            'center': (cx, cy),
                            'area': area,
                            'contour': contour,
                            'box': box,
                            'angle': rect[-1]
                        }
                        
                        if use_realsense and depth_image is not None:
                            object_info['depth'] = depth_image[cy, cx]
                        
                        detected_objects.append(object_info)
                        
                        # Draw detection
                        cv2.drawContours(red_result, [box], 0, (0, 255, 0), 2)
                        cv2.circle(red_result, (cx, cy), 3, (0, 255, 0), -1)
                        info_text = f"A:{area:.0f} R:{aspect_ratio:.1f}"
                        if use_realsense and depth_image is not None:
                            info_text += f" D:{depth_image[cy, cx]}mm"
                        cv2.putText(red_result, info_text, (cx-20, cy-10),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        return red_result, red_mask, detected_objects
