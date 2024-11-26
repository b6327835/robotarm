import cv2
import numpy as np
from cv2 import aruco

def nothing(x):
    pass

def main():
    # Create windows and trackbars
    cv2.namedWindow('Settings')
    cv2.namedWindow('Red Object Settings')
    
    # Original trackbars for object detection
    cv2.createTrackbar('Focus', 'Settings', 0, 255, nothing)
    cv2.createTrackbar('Min Area', 'Settings', 100, 1000, nothing)
    cv2.createTrackbar('Circularity', 'Settings', 70, 100, nothing)
    
    # Color detection trackbars (HSV)
    cv2.createTrackbar('H min', 'Settings', 0, 180, nothing)
    cv2.createTrackbar('H max', 'Settings', 180, 180, nothing)
    cv2.createTrackbar('S min', 'Settings', 0, 255, nothing)
    cv2.createTrackbar('S max', 'Settings', 30, 255, nothing)
    cv2.createTrackbar('V min', 'Settings', 200, 255, nothing)
    cv2.createTrackbar('V max', 'Settings', 255, 255, nothing)
    
    # Morphology trackbars
    cv2.createTrackbar('Kernel Size', 'Settings', 5, 20, nothing)
    
    # Add red object detection trackbars (Red HSV ranges)
    cv2.createTrackbar('Red H min 1', 'Red Object Settings', 0, 180, nothing)
    cv2.createTrackbar('Red H max 1', 'Red Object Settings', 10, 180, nothing)
    cv2.createTrackbar('Red H min 2', 'Red Object Settings', 170, 180, nothing)
    cv2.createTrackbar('Red H max 2', 'Red Object Settings', 180, 180, nothing)
    cv2.createTrackbar('Red S min', 'Red Object Settings', 120, 255, nothing)
    cv2.createTrackbar('Red V min', 'Red Object Settings', 70, 255, nothing)
    cv2.createTrackbar('Red Object Min Area', 'Red Object Settings', 20, 1000, nothing)
    cv2.createTrackbar('Red Object Max Area', 'Red Object Settings', 400, 1000, nothing)

    # Setup ArUco
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    aruco_params = aruco.DetectorParameters()
    aruco_detector = aruco.ArucoDetector(aruco_dict, aruco_params)

    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    # Initial camera setup
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Get current trackbar values
        focus = cv2.getTrackbarPos('Focus', 'Settings')
        min_area = cv2.getTrackbarPos('Min Area', 'Settings')
        circularity_threshold = cv2.getTrackbarPos('Circularity', 'Settings') / 100
        kernel_size = cv2.getTrackbarPos('Kernel Size', 'Settings')
        if kernel_size % 2 == 0:
            kernel_size += 1  # Ensure odd number for kernel

        # Update camera focus
        cap.set(cv2.CAP_PROP_FOCUS, focus)

        # Get color thresholds
        h_min = cv2.getTrackbarPos('H min', 'Settings')
        h_max = cv2.getTrackbarPos('H max', 'Settings')
        s_min = cv2.getTrackbarPos('S min', 'Settings')
        s_max = cv2.getTrackbarPos('S max', 'Settings')
        v_min = cv2.getTrackbarPos('V min', 'Settings')
        v_max = cv2.getTrackbarPos('V max', 'Settings')

        # Get red object detection values
        red_h_min1 = cv2.getTrackbarPos('Red H min 1', 'Red Object Settings')
        red_h_max1 = cv2.getTrackbarPos('Red H max 1', 'Red Object Settings')
        red_h_min2 = cv2.getTrackbarPos('Red H min 2', 'Red Object Settings')
        red_h_max2 = cv2.getTrackbarPos('Red H max 2', 'Red Object Settings')
        red_s_min = cv2.getTrackbarPos('Red S min', 'Red Object Settings')
        red_v_min = cv2.getTrackbarPos('Red V min', 'Red Object Settings')
        marker_min_area = cv2.getTrackbarPos('Red Object Min Area', 'Red Object Settings')
        marker_max_area = cv2.getTrackbarPos('Red Object Max Area', 'Red Object Settings')

        # Process image
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # Create mask with current values
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, upper)

        # Create red marker masks
        lower_red1 = np.array([red_h_min1, red_s_min, red_v_min])
        upper_red1 = np.array([red_h_max1, 255, 255])
        lower_red2 = np.array([red_h_min2, red_s_min, red_v_min])
        upper_red2 = np.array([red_h_max2, 255, 255])
        
        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        marker_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # Apply morphology
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Process marker mask
        marker_kernel = np.ones((3, 3), np.uint8)
        marker_mask = cv2.morphologyEx(marker_mask, cv2.MORPH_OPEN, marker_kernel)
        marker_mask = cv2.morphologyEx(marker_mask, cv2.MORPH_CLOSE, marker_kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Find marker contours
        marker_result = frame.copy()
        marker_contours, _ = cv2.findContours(marker_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in marker_contours:
            area = cv2.contourArea(contour)
            if marker_min_area < area < marker_max_area:
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
                            cv2.circle(marker_result, (cx, cy), 3, (0, 0, 255), -1)
                            cv2.putText(marker_result, f"A:{area:.0f}", (cx-20, cy-10),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # Draw contours that match criteria
        result = frame.copy()
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > min_area:
                perimeter = cv2.arcLength(contour, True)
                if perimeter > 0:
                    circularity = 4 * np.pi * area / (perimeter * perimeter)
                    if circularity > circularity_threshold:
                        cv2.drawContours(result, [contour], -1, (0, 255, 0), 2)
                        M = cv2.moments(contour)
                        if M['m00'] > 0:
                            cx = int(M['m10'] / M['m00'])
                            cy = int(M['m01'] / M['m00'])
                            cv2.circle(result, (cx, cy), 5, (0, 0, 255), -1)
                            cv2.putText(result, f'Area: {area:.0f}', (cx - 20, cy - 20),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Detect ArUco markers
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco_detector.detectMarkers(gray)
        
        # Draw detected markers
        aruco_result = frame.copy()
        if ids is not None:
            aruco.drawDetectedMarkers(aruco_result, corners, ids)
            
            # Calculate and display marker centers
            for i in range(len(ids)):
                c = corners[i][0]
                center = np.mean(c, axis=0).astype(int)
                cv2.circle(aruco_result, tuple(center), 5, (0, 255, 255), -1)
                cv2.putText(aruco_result, f'ID: {ids[i][0]}', 
                          (center[0] - 20, center[1] - 20),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        # Show results
        cv2.imshow('Original', frame)
        cv2.imshow('Red Object Mask', marker_mask)
        cv2.imshow('Red Object Result', marker_result)
        cv2.imshow('Object Mask', mask)
        cv2.imshow('Object Result', result)
        cv2.imshow('ArUco Detection', aruco_result)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()