import cv2
import numpy as np
from ultralytics import YOLO

# โหลดโมเดล YOLOv8
model = YOLO('yolov8n.pt')

# ฟังก์ชันสำหรับการตรวจจับวัตถุโดยใช้ YOLOv8
def detect_objects(frame, conf=0.4):
    results = model(frame, conf=conf)
    return results

# ตั้งค่ากล้อง C922
fov = 78  # FOV ของกล้อง C922
image_width = 640  # ความกว้างของภาพที่ได้จากกล้อง
image_height = 480  # ความสูงของภาพที่ได้จากกล้อง
focal_length = image_width / (2 * np.tan(np.radians(fov / 2)))

camera_matrix = np.array([[focal_length, 0, image_width / 2],
                          [0, focal_length, image_height / 2],
                          [0, 0, 1]])
dist_coeffs = np.zeros(5)  # ค่าสมมุติให้กล้องไม่มีการบิดเบือน

# เปิดกล้อง
cap = cv2.VideoCapture(1)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    # ตรวจจับวัตถุในภาพโดยปรับค่า confidence threshold
    results = detect_objects(frame, conf=0.4)  # ปรับค่า conf ได้ที่นี่
    
    for result in results:
        for bbox in result.boxes.xyxy.cpu().numpy():
            x1, y1, x2, y2 = bbox
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)
            
            # คำนวณ Y โดยให้จุดเริ่มต้น 0,0 อยู่ที่มุมซ้ายล่าง
            inverted_y = image_height - center_y
            
            # วาดจุดตรงกลางของวัตถุ
            cv2.circle(frame, (center_x, inverted_y), 5, (0, 0, 255), -1)
            
            # แสดงพิกัด XY ของจุดตรงกลางบนภาพ
            cv2.putText(frame, f"X: {center_x} Y: {inverted_y}", 
                        (center_x, inverted_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    
    # แสดงภาพ
    cv2.imshow('YOLOv8 Center XY Detection', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
