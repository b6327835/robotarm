import cv2
import numpy as np

class RedObjectDetector:
    def __init__(self):
        # Create window for red object settings
        cv2.namedWindow('Red Object Settings')
        
        # Setup trackbars
        def nothing(x): pass
        
        cv2.createTrackbar('Red H min 1', 'Red Object Settings', 0, 180, nothing)
        cv2.createTrackbar('Red H max 1', 'Red Object Settings', 10, 180, nothing)
        cv2.createTrackbar('Red H min 2', 'Red Object Settings', 170, 180, nothing)
        cv2.createTrackbar('Red H max 2', 'Red Object Settings', 180, 180, nothing)
        cv2.createTrackbar('Red S min', 'Red Object Settings', 120, 255, nothing)
        cv2.createTrackbar('Red V min', 'Red Object Settings', 70, 255, nothing)
        cv2.createTrackbar('Red Object Min Area', 'Red Object Settings', 20, 1000, nothing)
        cv2.createTrackbar('Red Object Max Area', 'Red Object Settings', 400, 1000, nothing)

    def get_parameters(self):
        return {
            'h_min1': cv2.getTrackbarPos('Red H min 1', 'Red Object Settings'),
            'h_max1': cv2.getTrackbarPos('Red H max 1', 'Red Object Settings'),
            'h_min2': cv2.getTrackbarPos('Red H min 2', 'Red Object Settings'),
            'h_max2': cv2.getTrackbarPos('Red H max 2', 'Red Object Settings'),
            's_min': cv2.getTrackbarPos('Red S min', 'Red Object Settings'),
            'v_min': cv2.getTrackbarPos('Red V min', 'Red Object Settings'),
            'min_area': cv2.getTrackbarPos('Red Object Min Area', 'Red Object Settings'),
            'max_area': cv2.getTrackbarPos('Red Object Max Area', 'Red Object Settings')
        }

    def detect(self, frame, depth_image=None, use_realsense=False):
        params = self.get_parameters()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create red marker masks
        lower_red1 = np.array([params['h_min1'], params['s_min'], params['v_min']])
        upper_red1 = np.array([params['h_max1'], 255, 255])
        lower_red2 = np.array([params['h_min2'], params['s_min'], params['v_min']])
        upper_red2 = np.array([params['h_max2'], 255, 255])
        
        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        marker_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # Process marker mask
        marker_kernel = np.ones((3, 3), np.uint8)
        marker_mask = cv2.morphologyEx(marker_mask, cv2.MORPH_OPEN, marker_kernel)
        marker_mask = cv2.morphologyEx(marker_mask, cv2.MORPH_CLOSE, marker_kernel)

        # Find marker contours
        result = frame.copy()
        marker_contours, _ = cv2.findContours(marker_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detected_objects = []
        for contour in marker_contours:
            area = cv2.contourArea(contour)
            if params['min_area'] < area < params['max_area']:
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w)/h
                if 0.7 < aspect_ratio < 1.3:
                    peri = cv2.arcLength(contour, True)
                    approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
                    if len(approx) >= 4 and len(approx) <= 6:
                        M = cv2.moments(contour)
                        if M['m00'] > 0:
                            cx = int(M['m10'] / M['m00'])
                            cy = int(M['m01'] / M['m00'])
                            
                            object_info = {
                                'center': (cx, cy),
                                'area': area,
                                'contour': contour
                            }
                            
                            if use_realsense and depth_image is not None:
                                object_info['depth'] = depth_image[cy, cx]
                            
                            detected_objects.append(object_info)
                            
                            # Draw detection
                            cv2.circle(result, (cx, cy), 3, (0, 0, 255), -1)
                            info_text = f"A:{area:.0f}"
                            if use_realsense and depth_image is not None:
                                info_text += f" D:{depth_image[cy, cx]}mm"
                            cv2.putText(result, info_text, (cx-20, cy-10),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        return result, marker_mask, detected_objects
