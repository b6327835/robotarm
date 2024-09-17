import cv2
from ultralytics import YOLO

# Load YOLOv8n (smallest model)
model = YOLO('yolov8n.pt')

# Open the webcam
cap = cv2.VideoCapture(1)  # Change 0 to your webcam index if needed

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to grab frame.")
        break

    # Perform YOLOv8 inference on the frame
    results = model(frame)

    for result in results:
        # Loop through detected objects
        for obj_id, box in enumerate(result.boxes):
            # Get confidence score
            conf = box.conf[0].item() * 100  # Convert to percentage

            # Only process objects with confidence > 70%
            if conf > 60:
                # Get bounding box coordinates (x1, y1, x2, y2)
                x1, y1, x2, y2 = box.xyxy[0].int().tolist()

                # Draw the bounding box
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

                # Calculate the center of the bounding box
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2

                # Draw a dot at the center of the object
                cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)

                # Draw horizontal and vertical lines across the object center
                cv2.line(frame, (center_x, y1), (center_x, y2), (0, 255, 255), 1)  # Vertical line
                cv2.line(frame, (x1, center_y), (x2, center_y), (0, 255, 255), 1)  # Horizontal line

                # Display object ID and the center coordinates as text over the dot
                text = f"ID: {obj_id}, x: {center_x}, y: {center_y}, conf: {conf:.1f}%"
                cv2.putText(frame, text, (center_x, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Print the x, y coordinates and object ID
                print(f"Object {obj_id} center: x={center_x}, y={center_y}, conf={conf:.1f}%")

    # Show the frame with detected objects, center dots, lines, and text
    cv2.imshow('YOLOv8 Detection with Center Dots, Lines, and IDs', frame)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
