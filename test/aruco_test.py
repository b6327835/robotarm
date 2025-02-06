import cv2
import numpy as np
import time

# Initialize camera
cap = cv2.VideoCapture(0)

# Set up ArUco detector with new API
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

# Remove POSITION_TIMEOUT and simplify to just store positions
last_known_positions = {}

def get_marker_center(corners, marker_id):
    marker_corners = corners[marker_id][0]
    center_x = int(np.mean(marker_corners[:, 0]))
    center_y = int(np.mean(marker_corners[:, 1]))
    return (center_x, center_y)

def estimate_missing_corner(centers, missing_id):
    """Estimate missing corner position based on other 3 corners"""
    # Verify we have exactly 3 corners before estimating
    if len(centers) != 3 or not all(i in centers for i in set([0,1,2,3]) - {missing_id}):
        return None
        
    # Convert positions to numpy arrays for calculations
    positions = {k: np.array(v) for k, v in centers.items()}
    
    if missing_id == 0:  # top-left missing
        v1 = positions[1] - positions[2]  # vector from bottom-right to top-right
        v2 = positions[3] - positions[2]  # vector from bottom-right to bottom-left
        estimated = positions[2] + v1 + v2
    elif missing_id == 1:  # top-right missing
        v1 = positions[0] - positions[3]  # vector from bottom-left to top-left
        v2 = positions[2] - positions[3]  # vector from bottom-left to bottom-right
        estimated = positions[3] + v1 + v2
    elif missing_id == 2:  # bottom-right missing
        v1 = positions[1] - positions[0]  # vector from top-left to top-right
        v2 = positions[3] - positions[0]  # vector from top-left to bottom-left
        estimated = positions[0] + v1 + v2
    else:  # bottom-left missing (id 3)
        v1 = positions[0] - positions[1]  # vector from top-right to top-left
        v2 = positions[2] - positions[1]  # vector from top-right to bottom-right
        estimated = positions[1] + v1 + v2
    
    return tuple(map(int, estimated))

def update_marker_positions(centers):
    # Only update positions for detected markers, don't clear old ones
    valid_centers = {k: v for k, v in centers.items() if 0 <= k <= 3}
    last_known_positions.update(valid_centers)

def get_valid_positions():
    return last_known_positions.copy()

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    corners, ids, rejected = detector.detectMarkers(frame)
    centers = {}
    
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        
        for i in range(len(ids)):
            marker_id = ids[i][0]
            if 0 <= marker_id <= 3:  # Only process valid marker IDs
                center = get_marker_center(corners, i)
                centers[marker_id] = center
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
    
        update_marker_positions(centers)
    
    valid_positions = get_valid_positions()
    temp_positions = valid_positions.copy()
    
    # Draw yellow dots for remembered positions
    for marker_id, pos in valid_positions.items():
        if marker_id not in centers:
            cv2.circle(frame, pos, 5, (0, 255, 255), -1)
    
    # Estimate missing corners but don't store them
    if len(valid_positions) >= 3:
        missing_ids = set([0, 1, 2, 3]) - set(valid_positions.keys())
        for missing_id in missing_ids:
            estimated_center = estimate_missing_corner(valid_positions, missing_id)
            if estimated_center:
                temp_positions[missing_id] = estimated_center  # Only store in temporary dictionary
                cv2.circle(frame, estimated_center, 5, (255, 0, 0), -1)
    
    # Draw rectangle using temporary positions
    if all(k in temp_positions for k in [0, 1, 2, 3]):
        cv2.line(frame, temp_positions[0], temp_positions[1], (0, 255, 0), 2)  # Top line
        cv2.line(frame, temp_positions[1], temp_positions[2], (0, 255, 0), 2)  # Right line
        cv2.line(frame, temp_positions[2], temp_positions[3], (0, 255, 0), 2)  # Bottom line
        cv2.line(frame, temp_positions[3], temp_positions[0], (0, 255, 0), 2)  # Left line
    
    # Display result
    cv2.imshow('ArUco Detection', frame)
    
    # Exit on 'q' press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
