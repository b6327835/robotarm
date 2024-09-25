import cv2
from ultralytics import YOLO

# โหลดโมเดล YOLOv8
model = YOLO('yolov8s.pt')

# กำหนดค่า confidence threshold
confidence_threshold = 0.6

# ข้อมูล intrinsic parameters ของกล้อง (เปลี่ยนตามกล้องของคุณ)
fx = 1000  # ค่า focal length ในแนวนอน
fy = 1000  # ค่า focal length ในแนวตั้ง
cx = 640   # จุดศูนย์กลางในแนวนอน
cy = 360   # จุดศูนย์กลางในแนวตั้ง

# เปิดกล้องเว็บแคม
cap = cv2.VideoCapture(1)  # ใช้กล้องหลัก (เปลี่ยนเป็น 1, 2 ถ้าต้องการกล้องอื่น)

if not cap.isOpened():
    print("ไม่สามารถเปิดกล้องได้")
    exit()

while True:
    # อ่านเฟรมจากกล้อง
    ret, frame = cap.read()
    if not ret:
        print("ไม่สามารถอ่านเฟรมจากกล้องได้")
        break

    # ทำการตรวจจับวัตถุในเฟรม
    results = model(frame)
    
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
            
            # คำนวณตำแหน่งของวัตถุในพิกัด 2D
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            
            # สมมุติค่าความลึก (depth) ของวัตถุ
            depth = 1.0  # ตัวอย่าง: ระยะทางจากกล้องถึงวัตถุ (หน่วยเป็นเมตร)
            
            # คำนวณพิกัด 3D
            X = (center_x - cx) * depth / fx
            Y = (center_y - cy) * depth / fy
            Z = depth
            
            print(f"Object {model.names[cls_id]}: X={X:.2f}, Y={Y:.2f}, Z={Z:.2f}")
            
            # วาดกล่องกรอบ
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # แสดงเฟรมที่มีการตรวจจับ
    cv2.imshow("YOLOv8 Real-Time Detection", frame)
    
    # กด 'q' เพื่อออกจากโปรแกรม
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ปิดกล้องและหน้าต่างแสดงผล
cap.release()
cv2.destroyAllWindows()
