import cv2
import numpy as np
import time

def main():
    # Initialize ArUco detector with the dictionary and parameters
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    # Start the video capture with retries
    cap = None
    max_retries = 5
    retry_count = 0
    
    while cap is None or not cap.isOpened():
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            retry_count += 1
            if retry_count >= max_retries:
                print("Failed to open camera after", max_retries, "attempts")
                return
            print("Retrying camera initialization...")
            time.sleep(1)

    # Disable autofocus and set focus to 0
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # Disable autofocus
    cap.set(cv2.CAP_PROP_FOCUS, 0)      # Set focus to 0 (closest)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect the markers in the image using new API
        corners, ids, rejectedImgPoints = detector.detectMarkers(gray)

        # Draw the detected markers
        if ids is not None:
            frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Display the resulting frame
        cv2.imshow('frame', frame)

        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()