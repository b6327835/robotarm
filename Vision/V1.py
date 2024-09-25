import cv2
from ultralytics import YOLO

# โหลดโมเดล YOLOv8
model = YOLO('yolov8s.pt')

# กำหนดค่า confidence threshold
confidence_threshold = 0.7

# เปิดกล้องเว็บแคม
cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("ไม่สามารถเปิดกล้องได้")
    exit()

while True:
    # อ่านเฟรมจากกล้อง
    ret, frame = cap.read()
    if not ret:
        print("ไม่สามารถอ่านเฟรมจากกล้องได้")
        break

    # พลิกภาพตามแนวนอน (เพื่อให้เห็นภาพจากกล้องด้านข้าง)
    frame_flipped = cv2.flip(frame, 1)  # 1 หมายถึงพลิกตามแนวนอน

    # ทำการตรวจจับวัตถุในเฟรมที่พลิก
    results = model(frame_flipped)
    
    # วาดกล่องกรอบและป้ายชื่อบนภาพ
    for result in results:
        boxes = result.boxes.xyxy.cpu().numpy()  # รับ bounding boxes เป็น numpy array
        confs = result.boxes.conf.cpu().numpy()  # รับ confidence scores
        cls_ids = result.boxes.cls.cpu().numpy()  # รับ class IDs
        
        for box, conf, cls_id in zip(boxes, confs, cls_ids):
            if conf < confidence_threshold:  # กรองตามค่า confidence threshold
                continue
            
            if len(box) < 4:  # ตรวจสอบความยาวของ box
                continue
            
            x1, y1, x2, y2 = map(int, box[:4])
            cls_id = int(cls_id)
            label = f"{model.names[cls_id]} {conf:.2f}"
            
            # วาดกล่องกรอบ
            cv2.rectangle(frame_flipped, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame_flipped, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # แสดงเฟรมที่พลิก
    cv2.imshow("YOLOv8 Real-Time Detection", frame_flipped)
    
    # กด 'q' เพื่อออกจากโปรแกรม
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ปิดกล้องและหน้าต่างแสดงผล
cap.release()
cv2.destroyAllWindows()
