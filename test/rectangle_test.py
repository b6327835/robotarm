import cv2
import numpy as np
from detection.red_basket_detector import RedBasketDetector

def original_detect_red_basket(frame):
    # Implementation from red_b.py
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=1)
    
    result = frame.copy()
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        if cv2.contourArea(cnt) > 500:
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int64(box)
            cv2.drawContours(result, [box], 0, (0, 255, 0), 2)
    
    return result, mask

def main():
    # Load test image
    image_path = 'test/workspace_test_01.png'
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"Error: Could not read image {image_path}")
        return

    # Test original implementation
    original_result, original_mask = original_detect_red_basket(frame)
    
    # Test detector class implementation
    detector = RedBasketDetector()
    # Set initial parameters to match red_b.py
    cv2.setTrackbarPos('Low H1', 'Red Basket Settings', 0)
    cv2.setTrackbarPos('High H1', 'Red Basket Settings', 10)
    cv2.setTrackbarPos('Low H2', 'Red Basket Settings', 160)
    cv2.setTrackbarPos('High H2', 'Red Basket Settings', 180)
    cv2.setTrackbarPos('Low S', 'Red Basket Settings', 100)
    cv2.setTrackbarPos('High S', 'Red Basket Settings', 255)
    cv2.setTrackbarPos('Low V', 'Red Basket Settings', 100)
    cv2.setTrackbarPos('High V', 'Red Basket Settings', 255)
    cv2.setTrackbarPos('Min Area', 'Red Basket Settings', 500)
    cv2.setTrackbarPos('Kernel Size', 'Red Basket Settings', 5)
    cv2.setTrackbarPos('Erosion Iter', 'Red Basket Settings', 1)
    cv2.setTrackbarPos('Dilation Iter', 'Red Basket Settings', 1)

    while True:
        # Get results from detector class
        detector_result, detector_mask, detected_objects = detector.detect(frame.copy())
        
        # Display results side by side
        cv2.imshow('Original Implementation', original_result)
        cv2.imshow('Original Mask', original_mask)
        cv2.imshow('Detector Implementation', detector_result)
        cv2.imshow('Detector Mask', detector_mask)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
