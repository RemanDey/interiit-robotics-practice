import cv2
import numpy as np
from ultralytics import YOLO

model = YOLO("yolov8n-seg.pt")

cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame, verbose=True)

    annotated_frame = results[0].plot()

    mask_overlay = np.zeros_like(frame)
    if results[0].masks is not None:
        for mask in results[0].masks.data:
            mask_np = mask.cpu().numpy()
            mask_resized = cv2.resize(mask_np, (frame.shape[1], frame.shape[0]))
            mask_overlay[mask_resized > 0.5] = frame[mask_resized > 0.5]

    cv2.imshow("Original + Segmentation", annotated_frame)
    cv2.imshow("Segmented Mask", mask_overlay)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()