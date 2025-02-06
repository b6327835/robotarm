import cv2
import numpy as np
from ultralytics import YOLO

# Load YOLOv8 model
model = YOLO('yolov8n.pt')

# Function to detect objects using YOLOv8
def detect_objects(frame, conf):
    results = model(frame, conf=conf)
    return results

# Camera settings
fov = 78  # FOV of C922 camera
image_width = 661  # Image width from camera
image_height = 361  # Image height from camera
focal_length = image_width / (2 * np.tan(np.radians(fov / 2)))

camera_matrix = np.array([[focal_length, 0, image_width / 2],
                          [0, focal_length, image_height / 2],
                          [0, 0, 1]])
dist_coeffs = np.zeros(5)  # Assume no lens distortion

# Set confidence threshold
conf_threshold = 0.5  # Adjust confidence threshold here

# Open camera
cap = cv2.VideoCapture(1)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    # Detect objects in the frame with adjusted confidence threshold
    results = detect_objects(frame, conf=conf_threshold)
    
    for result in results:
        for bbox in result.boxes.xyxy.cpu().numpy():
            x1, y1, x2, y2 = bbox
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)
            
            # Invert Y to align (0,0) at the bottom-left corner
            inverted_y = image_height - center_y
            
            # Draw bounding box and show XY coordinates on the image
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame, f"X: {center_x} Y: {inverted_y}", 
                        (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Draw center point of the detected object
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
    
    # Display the frame
    cv2.imshow('YOLOv8 XY Detection', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
