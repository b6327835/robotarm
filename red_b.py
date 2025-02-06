import cv2
import numpy as np

def detect_red_basket(frame):
    # Convert to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define red color range in HSV
    # Red color wraps around in HSV, so we need two ranges
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    
    # Create masks for red color
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    
    # Apply morphological operations to remove noise
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=1)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Process each contour
    for cnt in contours:
        # Filter small contours
        if cv2.contourArea(cnt) > 500:  # Adjust this threshold as needed
            # Find rotated rectangle
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int64(box)
            
            # Draw rotated rectangle
            cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)
    
    return frame

if __name__ == "__main__":
    # Read the image
    image_path = 'test/workspace_test_01.png'
    frame = cv2.imread(image_path)
    
    if frame is None:
        print(f"Error: Could not read image {image_path}")
        exit()
    
    # Process frame
    processed_frame = detect_red_basket(frame)
    
    # Show result
    cv2.imshow('Red Basket Detection', processed_frame)
    cv2.waitKey(0)  # Wait for any key press
    cv2.destroyAllWindows()
