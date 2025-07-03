from ultralytics import YOLO
import cv2

model = YOLO("yolov8n.pt")
cap = cv2.VideoCapture(0)

frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_center = frame_width // 2
tolerance = 50

def get_direction(x_center):
    if x_center < frame_center - tolerance:
        return "KIRI"
    elif x_center > frame_center + tolerance:
        return "KANAN"
    else:
        return "LURUS"

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model.predict(frame, imgsz=640, conf=0.5)
    annotated_frame = results[0].plot()

    boxes = results[0].boxes
    if boxes is not None and len(boxes) > 0:
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            x_center = int((x1 + x2) / 2)
            y_center = int((y1 + y2) / 2)

            direction = get_direction(x_center)
            cv2.line(annotated_frame, (x_center, 0), (x_center, frame.shape[0]), (0, 255, 0), 2)
            cv2.putText(annotated_frame, f"Arah: {direction}", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 50, 50), 2)
            break

    cv2.line(annotated_frame, (frame_center, 0), (frame_center, frame.shape[0]), (255, 255, 255), 1)
    cv2.imshow("YOLOv8 Realtime Direction Tracker", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
