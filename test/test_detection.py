import cv2
import numpy as np
from cv2 import aruco
import pyrealsense2 as rs
from detection.red_detector import RedObjectDetector
from detection.black_detector import BlackObjectDetector
from detection.aruco_detector import ArUcoDetector
from detection.white_detector import WhiteObjectDetector
from detection.red_basket_detector import RedBasketDetector

def nothing(x):
    pass

def main():
    # Camera type flag
    use_realsense = False  # Set to False for normal webcam

    # Initialize camera
    if use_realsense:
        # Configure RealSense pipeline
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        pipeline.start(config)
        align = rs.align(rs.stream.color)
    else:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Error: Could not open webcam.")
            return
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Create only the main settings window for camera control
    cv2.namedWindow('Settings')
    
    # Keep only camera focus control
    cv2.createTrackbar('Focus', 'Settings', 0, 255, nothing)

    # Initialize detectors
    red_detector = RedObjectDetector()
    black_detector = BlackObjectDetector()
    aruco_detector = ArUcoDetector()
    white_detector = WhiteObjectDetector()
    red_basket_detector = RedBasketDetector()

    while True:
        if use_realsense:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            frame = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
        else:
            ret, frame = cap.read()
            if not ret:
                break

        # Get and update camera focus
        focus = cv2.getTrackbarPos('Focus', 'Settings')
        if not use_realsense:
            cap.set(cv2.CAP_PROP_FOCUS, focus)
        # frame = cv2.imread('test/workspace_test_01.png')
        # Process objects with detectors
        red_result, red_mask, red_objects = red_detector.detect(frame, depth_image if use_realsense else None, use_realsense)
        black_result, black_mask, black_objects = black_detector.detect(frame, depth_image if use_realsense else None, use_realsense)
        aruco_result, detected_markers = aruco_detector.detect(frame, depth_image if use_realsense else None, use_realsense)
        white_result, white_mask, white_objects = white_detector.detect(frame, depth_image if use_realsense else None, use_realsense)
        basket_result, basket_mask, basket_objects = red_basket_detector.detect(frame, depth_image if use_realsense else None, use_realsense)

        # Show results
        cv2.imshow('Original', frame)
        cv2.imshow('Red Object Mask', red_mask)
        cv2.imshow('Red Object Result', red_result)
        cv2.imshow('Black Object Mask', black_mask)
        cv2.imshow('Black Object Result', black_result)
        cv2.imshow('White Object Mask', white_mask)
        cv2.imshow('White Object Result', white_result)
        cv2.imshow('ArUco Detection', aruco_result)
        cv2.imshow('Red Basket Mask', basket_mask)
        cv2.imshow('Red Basket Result', basket_result)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    if use_realsense:
        pipeline.stop()
    else:
        cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()