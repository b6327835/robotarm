import cv2
import numpy as np

class WhiteObjectDetector:
    def __init__(self):
        cv2.namedWindow('White Object Settings')
        
        def nothing(x): pass
        
        # Color detection trackbars (HSV)
        cv2.createTrackbar('H min', 'White Object Settings', 0, 180, nothing)
        cv2.createTrackbar('H max', 'White Object Settings', 180, 180, nothing)
        cv2.createTrackbar('S min', 'White Object Settings', 0, 255, nothing)
        cv2.createTrackbar('S max', 'White Object Settings', 30, 255, nothing)
        cv2.createTrackbar('V min', 'White Object Settings', 200, 255, nothing)
        cv2.createTrackbar('V max', 'White Object Settings', 255, 255, nothing)
        
        # Object detection parameters
        cv2.createTrackbar('Min Area', 'White Object Settings', 100, 1000, nothing)
        cv2.createTrackbar('Max Area', 'White Object Settings', 1000, 5000, nothing)
        cv2.createTrackbar('Circularity', 'White Object Settings', 70, 100, nothing)
        
        # Morphology trackbars
        cv2.createTrackbar('Kernel Size', 'White Object Settings', 5, 20, nothing)
        cv2.createTrackbar('Erosion Iterations', 'White Object Settings', 1, 5, nothing)
        cv2.createTrackbar('Dilation Iterations', 'White Object Settings', 1, 5, nothing)
        cv2.createTrackbar('Min Distance', 'White Object Settings', 20, 100, nothing)
        cv2.createTrackbar('Contour Method', 'White Object Settings', 0, 2, nothing)

    def get_parameters(self):
        return {
            'h_min': cv2.getTrackbarPos('H min', 'White Object Settings'),
            'h_max': cv2.getTrackbarPos('H max', 'White Object Settings'),
            's_min': cv2.getTrackbarPos('S min', 'White Object Settings'),
            's_max': cv2.getTrackbarPos('S max', 'White Object Settings'),
            'v_min': cv2.getTrackbarPos('V min', 'White Object Settings'),
            'v_max': cv2.getTrackbarPos('V max', 'White Object Settings'),
            'min_area': cv2.getTrackbarPos('Min Area', 'White Object Settings'),
            'max_area': cv2.getTrackbarPos('Max Area', 'White Object Settings'),
            'circularity': cv2.getTrackbarPos('Circularity', 'White Object Settings') / 100,
            'kernel_size': cv2.getTrackbarPos('Kernel Size', 'White Object Settings'),
            'erosion_iter': cv2.getTrackbarPos('Erosion Iterations', 'White Object Settings'),
            'dilation_iter': cv2.getTrackbarPos('Dilation Iterations', 'White Object Settings'),
            'min_distance': cv2.getTrackbarPos('Min Distance', 'White Object Settings'),
            'contour_method': cv2.getTrackbarPos('Contour Method', 'White Object Settings')
        }

    def detect(self, frame, depth_image=None, use_realsense=False):
        params = self.get_parameters()
        
        # Process image
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # Create mask
        lower = np.array([params['h_min'], params['s_min'], params['v_min']])
        upper = np.array([params['h_max'], params['s_max'], params['v_max']])
        mask = cv2.inRange(hsv, lower, upper)

        # Apply morphology
        kernel_size = params['kernel_size'] + (1 if params['kernel_size'] % 2 == 0 else 0)
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=params['erosion_iter'])
        mask = cv2.dilate(mask, kernel, iterations=params['dilation_iter'])

        # Find contours
        contour_methods = [cv2.CHAIN_APPROX_SIMPLE, cv2.CHAIN_APPROX_NONE, cv2.CHAIN_APPROX_TC89_L1]
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                     contour_methods[params['contour_method']])

        filtered_contours = []
        centers = []
        detected_objects = []
        
        # Process contours
        result = frame.copy()
        for contour in contours:
            area = cv2.contourArea(contour)
            if params['min_area'] < area < params['max_area']:
                perimeter = cv2.arcLength(contour, True)
                if perimeter > 0:
                    circularity = 4 * np.pi * area / (perimeter * perimeter)
                    if circularity > params['circularity']:
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
                                    'circularity': circularity,
                                    'contour': contour
                                }
                                
                                if use_realsense and depth_image is not None:
                                    object_info['depth'] = depth_image[cy, cx]
                                
                                detected_objects.append(object_info)
                                
                                # Draw detection
                                cv2.drawContours(result, [contour], -1, (0, 255, 0), 2)
                                cv2.circle(result, (cx, cy), 5, (0, 0, 255), -1)
                                
                                info_text = f'Area: {area:.0f}'
                                if use_realsense and depth_image is not None:
                                    info_text += f" D:{depth_image[cy, cx]}mm"
                                cv2.putText(result, info_text, (cx - 20, cy - 20),
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return result, mask, detected_objects
