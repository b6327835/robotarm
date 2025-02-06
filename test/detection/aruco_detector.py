import cv2
import numpy as np
from cv2 import aruco

class ArUcoDetector:
    def __init__(self, dictionary_type=aruco.DICT_4X4_250):
        self.aruco_dict = aruco.getPredefinedDictionary(dictionary_type)
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

    def detect(self, frame, depth_image=None, use_realsense=False):
        result = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
        
        detected_markers = []
        
        if ids is not None:
            aruco.drawDetectedMarkers(result, corners, ids)
            
            # Calculate and display marker centers
            for i in range(len(ids)):
                c = corners[i][0]
                center = np.mean(c, axis=0).astype(int)
                cv2.circle(result, tuple(center), 5, (0, 255, 255), -1)
                
                marker_info = {
                    'id': ids[i][0],
                    'center': tuple(center),
                    'corners': c
                }
                
                if use_realsense and depth_image is not None:
                    marker_info['depth'] = depth_image[int(center[1]), int(center[0])]
                
                detected_markers.append(marker_info)
                
                # Draw marker information
                info_text = f'ID: {ids[i][0]}'
                if use_realsense and depth_image is not None:
                    info_text += f" D:{marker_info['depth']}mm"
                cv2.putText(result, info_text,
                          (int(center[0]) - 20, int(center[1]) - 20),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        return result, detected_markers
