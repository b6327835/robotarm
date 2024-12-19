import cv2
import numpy as np

class BlackObjectDetector:
    def __init__(self):
        # Create window for black object settings
        cv2.namedWindow('Black Object Settings')
        
        # Setup trackbars
        def nothing(x): pass
        
        cv2.createTrackbar('Black V max', 'Black Object Settings', 50, 255, nothing)
        cv2.createTrackbar('Black S max', 'Black Object Settings', 50, 255, nothing)
        cv2.createTrackbar('Black Min Area', 'Black Object Settings', 100, 1000, nothing)
        cv2.createTrackbar('Black Max Area', 'Black Object Settings', 400, 1000, nothing)
        cv2.createTrackbar('Black Kernel Size', 'Black Object Settings', 5, 20, nothing)
        cv2.createTrackbar('Black Erosion Iterations', 'Black Object Settings', 1, 5, nothing)
        cv2.createTrackbar('Black Dilation Iterations', 'Black Object Settings', 1, 5, nothing)
        cv2.createTrackbar('Black Min Distance', 'Black Object Settings', 20, 100, nothing)
        cv2.createTrackbar('Black Contour Method', 'Black Object Settings', 0, 2, nothing)

    def get_parameters(self):
        return {
            'v_max': cv2.getTrackbarPos('Black V max', 'Black Object Settings'),
            's_max': cv2.getTrackbarPos('Black S max', 'Black Object Settings'),
            'min_area': cv2.getTrackbarPos('Black Min Area', 'Black Object Settings'),
            'max_area': cv2.getTrackbarPos('Black Max Area', 'Black Object Settings'),
            'kernel_size': cv2.getTrackbarPos('Black Kernel Size', 'Black Object Settings'),
            'erosion_iter': cv2.getTrackbarPos('Black Erosion Iterations', 'Black Object Settings'),
            'dilation_iter': cv2.getTrackbarPos('Black Dilation Iterations', 'Black Object Settings'),
            'min_distance': cv2.getTrackbarPos('Black Min Distance', 'Black Object Settings'),
            'contour_method': cv2.getTrackbarPos('Black Contour Method', 'Black Object Settings')
        }

    def detect(self, frame, depth_image=None, use_realsense=False):
        params = self.get_parameters()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create black object mask
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, params['s_max'], params['v_max']])
        black_mask = cv2.inRange(hsv, lower_black, upper_black)

        # Process black mask with morphological parameters
        kernel_size = params['kernel_size'] + (1 if params['kernel_size'] % 2 == 0 else 0)
        black_kernel = np.ones((kernel_size, kernel_size), np.uint8)
        black_mask = cv2.erode(black_mask, black_kernel, iterations=params['erosion_iter'])
        black_mask = cv2.dilate(black_mask, black_kernel, iterations=params['dilation_iter'])

        # Find contours
        contour_methods = [cv2.CHAIN_APPROX_SIMPLE, cv2.CHAIN_APPROX_NONE, cv2.CHAIN_APPROX_TC89_L1]
        black_result = frame.copy()
        black_contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, 
                                           contour_methods[params['contour_method']])
        
        filtered_contours = []
        centers = []
        detected_objects = []
        
        for contour in black_contours:
            area = cv2.contourArea(contour)
            if params['min_area'] < area < params['max_area']:
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w)/h
                if 0.7 < aspect_ratio < 1.3:
                    M = cv2.moments(contour)
                    if M['m00'] > 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        
                        # Check distance from existing centers
                        too_close = False
                        for existing_center in centers:
                            dist = np.sqrt((cx - existing_center[0])**2 + 
                                         (cy - existing_center[1])**2)
                            if dist < params['min_distance']:
                                too_close = True
                                break
                        
                        if not too_close:
                            filtered_contours.append(contour)
                            centers.append((cx, cy))
                            
                            object_info = {
                                'center': (cx, cy),
                                'area': area,
                                'contour': contour
                            }
                            
                            if use_realsense and depth_image is not None:
                                object_info['depth'] = depth_image[cy, cx]
                            
                            detected_objects.append(object_info)
                            
                            # Draw detection
                            cv2.circle(black_result, (cx, cy), 3, (0, 255, 0), -1)
                            info_text = f"A:{area:.0f}"
                            if use_realsense and depth_image is not None:
                                info_text += f" D:{depth_image[cy, cx]}mm"
                            cv2.putText(black_result, info_text, (cx-20, cy-10),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        return black_result, black_mask, detected_objects
