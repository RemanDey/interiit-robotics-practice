import sys
import cv2
import numpy as np
from ultralytics import YOLO

model = YOLO("yolov8n-seg.pt")

def process_frame(frame):
    results = model(frame, verbose=False)

    annotated_frame = results[0].plot()

    mask_overlay = np.zeros_like(frame)
    if results[0].masks is not None:
        for mask in results[0].masks.data:
            mask_np = mask.cpu().numpy()
            mask_resized = cv2.resize(mask_np, (frame.shape[1], frame.shape[0]))
            mask_overlay[mask_resized > 0.5] = frame[mask_resized > 0.5]

    cv2.imshow("Original + Segmentation", annotated_frame)
    cv2.imshow("Segmented Mask", mask_overlay)

if len(sys.argv) > 1:
    image_path = sys.argv[1]
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"Error: Could not load image '{image_path}'")
        sys.exit(1)
    process_frame(frame)
    cv2.waitKey(0)
else:
    cap = cv2.VideoCapture(1, cv2.CAP_V4L2)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        process_frame(frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()

cv2.destroyAllWindows()