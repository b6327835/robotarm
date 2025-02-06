import cv2
import numpy as np
from typing import Dict, Tuple, Optional

class ArucoMarkerPosition:
    def __init__(self, camera_id: int = 0, display_output: bool = True, existing_cap=None):
        self.display_output = display_output
        self.cap = existing_cap if existing_cap is not None else cv2.VideoCapture(camera_id)
        self.external_cap = existing_cap is not None  # Flag to track if camera is external
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        self.last_known_positions = {}
        self.is_running = False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.cleanup()

    def cleanup(self):
        """Release resources"""
        self.is_running = False
        if self.cap.isOpened() and not self.external_cap:  # Only release if we own the camera
            self.cap.release()
        cv2.destroyAllWindows()

    def get_marker_center(self, corners, marker_id):
        marker_corners = corners[marker_id][0]
        center_x = int(np.mean(marker_corners[:, 0]))
        center_y = int(np.mean(marker_corners[:, 1]))
        return (center_x, center_y)

    def estimate_missing_corner(self, centers, missing_id):
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

    def update_marker_positions(self, centers: Dict[int, Tuple[int, int]]):
        valid_centers = {k: v for k, v in centers.items() if 0 <= k <= 3}
        self.last_known_positions.update(valid_centers)

    def get_valid_positions(self):
        return self.last_known_positions.copy()

    def process_frame(self) -> Tuple[Optional[np.ndarray], Dict[int, Tuple[int, int]]]:
        """Process a single frame and return the processed frame and positions"""
        ret, frame = self.cap.read()
        if not ret:
            return None, {}

        corners, ids, _ = self.detector.detectMarkers(frame)
        centers = {}
        
        if ids is not None:
            if self.display_output:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            for i in range(len(ids)):
                marker_id = ids[i][0]
                if 0 <= marker_id <= 3:
                    center = self.get_marker_center(corners, i)
                    centers[marker_id] = center
                    if self.display_output:
                        cv2.circle(frame, center, 5, (0, 0, 255), -1)

        self.update_marker_positions(centers)
        valid_positions = self.get_valid_positions()
        temp_positions = valid_positions.copy()

        if self.display_output:
            # Draw remembered positions and estimates
            for marker_id, pos in valid_positions.items():
                if marker_id not in centers:
                    cv2.circle(frame, pos, 5, (0, 255, 255), -1)

            if len(valid_positions) >= 3:
                missing_ids = set([0, 1, 2, 3]) - set(valid_positions.keys())
                for missing_id in missing_ids:
                    estimated_center = self.estimate_missing_corner(valid_positions, missing_id)
                    if estimated_center:
                        temp_positions[missing_id] = estimated_center
                        cv2.circle(frame, estimated_center, 5, (255, 0, 0), -1)

            # Draw rectangle
            if all(k in temp_positions for k in [0, 1, 2, 3]):
                for i in range(4):
                    pt1 = temp_positions[i]
                    pt2 = temp_positions[(i + 1) % 4]
                    cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

            cv2.imshow('ArUco Detection', frame)

        return frame, temp_positions

    def run(self):
        """Main detection loop"""
        self.is_running = True
        while self.is_running:
            frame, positions = self.process_frame()
            if frame is None:
                break
                
            if self.display_output and cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cleanup()

def main():
    """Example usage"""
    with ArucoMarkerPosition() as detector:
        detector.run()

if __name__ == "__main__":
    main()
