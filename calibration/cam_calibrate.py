import numpy as np
import cv2
import glob
import pickle
import pyrealsense2 as rs

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:  # Changed to left button click
        param['capture'] = True

def calibrate_camera():
    # Chessboard dimensions
    CHESSBOARD_SIZE = (9, 6)
    SQUARE_SIZE = 20  # mm

    # Initialize RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    # Termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points
    objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
    objp[:,:2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1,2)
    objp = objp * SQUARE_SIZE

    # Arrays to store object points and image points
    objpoints = []
    imgpoints = []

    num_samples = 0
    required_samples = 20

    print("Click left mouse button to capture a frame for calibration (need 20 different angles)")
    print("Press 'q' to quit")

    # Create window and set mouse callback
    cv2.namedWindow('RealSense Calibration')
    callback_params = {'capture': False}
    cv2.setMouseCallback('RealSense Calibration', mouse_callback, callback_params)

    try:
        while num_samples < required_samples:
            # Get frameset of color
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # Convert images to numpy arrays
            frame = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

            if ret:
                corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
                cv2.drawChessboardCorners(frame, CHESSBOARD_SIZE, corners2, ret)

            cv2.imshow('RealSense Calibration', frame)
            key = cv2.waitKey(1) & 0xFF

            if callback_params['capture'] and ret:
                objpoints.append(objp)
                imgpoints.append(corners2)
                num_samples += 1
                print(f"Captured {num_samples}/{required_samples} samples")
                callback_params['capture'] = False  # Reset capture flag
            elif key == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    if num_samples > 0:
        print("Calculating camera calibration...")
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

        # Save calibration data
        calibration_data = {
            'camera_matrix': mtx,
            'dist_coeffs': dist
        }
        
        with open('realsense_calibration.pkl', 'wb') as f:
            pickle.dump(calibration_data, f)
        
        print("Calibration complete and saved to 'realsense_calibration.pkl'")
        return True
    
    return False

if __name__ == "__main__":
    calibrate_camera()
