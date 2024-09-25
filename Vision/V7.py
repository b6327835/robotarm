import cv2
import numpy as np
from ultralytics import YOLO

# โหลดโมเดล YOLOv8
model = YOLO('yolov8n.pt')

# ฟังก์ชันสำหรับการตรวจจับวัตถุโดยใช้ YOLOv8
def detect_objects(frame, conf):
    results = model(frame, conf=conf)
    return results

# ตั้งค่ากล้อง C922
fov = 78  # FOV ของกล้อง C922
image_width = 300  # ความกว้างของภาพที่ได้จากกล้อง
image_height = 200  # ความสูงของภาพที่ได้จากกล้อง

# กำหนดค่าการปรับ confidence threshold
conf_threshold = 0.2  # ปรับค่า conf ได้ที่นี่

# เปิดกล้อง
cap = cv2.VideoCapture(1)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    # ตรวจจับวัตถุในภาพโดยปรับค่า confidence threshold
    results = detect_objects(frame, conf=conf_threshold)
    
    for result in results:
        for bbox in result.boxes.xyxy.cpu().numpy():
            x1, y1, x2, y2 = bbox
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)
            
            # คำนวณ Y โดยให้จุดเริ่มต้น 0,0 อยู่ที่มุมซ้ายล่าง
            inverted_y = image_height - center_y
            
            # วาดกรอบและแสดงพิกัด XY บนภาพ
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame, f"X: {(center_x)/3} Y: {(inverted_y)/3}",  (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # วาดจุดตรงกลางของวัตถุ
            cv2.circle(frame, (center_x,center_y), 5, (0, 0, 255), -1)
    
    # แสดงภาพ
    cv2.imshow('YOLOv8 XY Detection', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
