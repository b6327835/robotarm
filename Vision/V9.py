import cv2
from ultralytics import YOLO
import numpy as np

# Load YOLOv8n (smallest model)
try:
    model = YOLO('yolov8n.pt')
except Exception as e:
    print(f"Error loading YOLO model: {e}")
    exit()

# Open the webcam
cap = cv2.VideoCapture(0)  # Change 0 to your webcam index if needed

# Check if the webcam was opened successfully
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# Adjustable speed for the dot following the detected object
speed = 10  # Speed in pixels per frame

# Define the resolution for both the camera and window
x_limit = 300
y_limit = 200
cap.set(cv2.CAP_PROP_FRAME_WIDTH, x_limit)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, y_limit)

# Initialize dot position
current_x, current_y = x_limit - 100, y_limit - 100  # Initial position near the bottom-right corner

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to grab frame.")
        break

    # Resize frame to 300x200 in case it's not the same
    frame = cv2.resize(frame, (x_limit, y_limit))

    # Perform YOLOv8 inference on the frame
    try:
        results = model(frame)
    except Exception as e:
        print(f"Error during inference: {e}")
        break

    detected = False

    for result in results:
        # Loop through detected objects
        for obj_id, box in enumerate(result.boxes):
            if obj_id == 0:  # Only track object with ID 0
                detected = True
                # Get confidence score
                conf = box.conf[0].item() * 100  # Convert to percentage

                # Only process objects with confidence > 60%
                if conf > 60:
                    # Get bounding box coordinates (x1, y1, x2, y2)
                    x1, y1, x2, y2 = box.xyxy[0].int().tolist()

                    # Calculate the center of the bounding box
                    target_x = (x1 + x2) // 2
                    target_y = (y1 + y2) // 2

                    # Apply the boundary limits for target_x and target_y
                    target_x = min(target_x, x_limit)
                    target_y = min(target_y, y_limit)

                    # Draw the bounding box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    # Draw a dot at the center of the object
                    cv2.circle(frame, (target_x, target_y), 5, (0, 255, 0), -1)

                    # Draw horizontal and vertical lines across the object center
                    cv2.line(frame, (target_x, y1), (target_x, y2), (0, 255, 255), 1)  # Vertical line
                    cv2.line(frame, (x1, target_y), (x2, target_y), (0, 255, 255), 1)  # Horizontal line

                    # Print the x, y coordinates and object ID
                    print(f"Object {obj_id} center: x={target_x}, y={target_y}, conf={conf:.1f}%")

                    # Calculate direction and move the dot
                    dx = target_x - current_x
                    dy = target_y - current_y
                    distance = np.sqrt(dx**2 + dy**2)
                    if distance > 0:
                        # Normalize direction vector
                        dx /= distance
                        dy /= distance

                        # Move dot by fixed speed
                        current_x += int(dx * speed)
                        current_y += int(dy * speed)

                        # Ensure the dot is within the frame boundaries
                        current_x = max(0, min(x_limit - 1, current_x))
                        current_y = max(0, min(y_limit - 1, current_y))

    if not detected:
        # No object detected, so keep the dot in the last position
        pass

    # Draw the dot with vertical and horizontal lines
    cv2.circle(frame, (int(current_x), int(current_y)), 5, (0, 0, 255), -1)
    cv2.line(frame, (int(current_x), 0), (int(current_x), y_limit), (255, 255, 0), 1)  # Vertical line
    cv2.line(frame, (0, int(current_y)), (x_limit, int(current_y)), (255, 255, 0), 1)  # Horizontal line

    # Display the dot's coordinates above it
    cv2.putText(frame, f"x: {int(current_x)}, y: {int(current_y)}", (int(current_x) + 10, int(current_y) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # Show the frame with detected objects, center dots, lines, and text
    cv2.imshow('YOLOv8 Detection with Center Dots, Lines, and IDs', frame)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
