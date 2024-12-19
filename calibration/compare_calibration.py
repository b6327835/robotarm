import numpy as np
import cv2
import pickle
import pyrealsense2 as rs

def load_calibration():
    try:
        with open('realsense_calibration.pkl', 'rb') as f:
            data = pickle.load(f)
            return data['camera_matrix'], data['dist_coeffs']
    except:
        print("No calibration file found. Run cam_calibrate.py first.")
        return None, None

def main():
    # Initialize RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    # Load calibration data
    camera_matrix, dist_coeffs = load_calibration()
    if camera_matrix is None:
        pipeline.stop()
        return

    print("Press 'q' to quit")
    print("Press 's' to save current frame pair")
    
    frame_count = 0

    try:
        while True:
            # Get frameset of color
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # Convert images to numpy arrays
            original = np.asanyarray(color_frame.get_data())
            
            # Apply calibration
            calibrated = cv2.undistort(original, camera_matrix, dist_coeffs)
            
            # Create side-by-side display
            display = np.hstack((original, calibrated))
            
            # Add labels
            cv2.putText(display, "Original", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(display, "Calibrated", (original.shape[1] + 10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Show the image
            cv2.imshow('Original vs Calibrated', display)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                filename = f'calibration_compare_{frame_count}.png'
                cv2.imwrite(filename, display)
                print(f"Saved comparison image as {filename}")
                frame_count += 1

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
