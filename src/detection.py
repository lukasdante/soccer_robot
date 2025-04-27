import cv2
from ultralytics import YOLO
import time

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)

model = YOLO("models/y11ndetect_ncnn_model")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    start = time.time()
    results = model(frame, conf=0.9)[0]

    annotated = results.plot()

    labels = [model.names[int(cls)] for cls in results.boxes.cls]
    for box in results.boxes:
        bbox =box.xyxy[0].tolist() 
        print(bbox)
        label = model.names[int(box.cls)]
        x1, y1, x2, y2 = bbox
        ldx = abs(x1 - 320)
        rdx = abs(x2 - 320)
        area = max(abs(y2 - y1), abs(x2 - x1)) ** 2

        print(f"{label} DIF: {(ldx-rdx):.2f}    LDX: {ldx:.2f}  RDX: {rdx:.2f} area: {area}")
    print(labels)

    end = time.time()
    fps = 1 / (end - start)

    cv2.putText(annotated, f'FPS: {fps:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow("Sample", annotated)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()