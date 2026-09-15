import sys
import os
import cv2
import numpy as np
import psutil
from ultralytics import YOLO

def get_process_memory_mb():
    process = psutil.Process(os.getpid())
    return process.memory_info().rss / (1024 * 1024)

mem_baseline = get_process_memory_mb()
print(f"[RAM] Baseline Process RAM: {mem_baseline:.2f} MB")

model = YOLO("yolo11n-seg-ade20k.pt")


mem_post_model = get_process_memory_mb()
print(f"[RAM] RAM after loading model: {mem_post_model:.2f} MB (Model overhead: {mem_post_model - mem_baseline:.2f} MB)")

def process_frame(frame):
 
    mem_before_infer = get_process_memory_mb()
    results = model(frame, verbose=False)
    annotated_frame = results[0].plot()
    mask_overlay = np.zeros_like(frame)
    if results[0].masks is not None:
        for mask in results[0].masks.data:
            mask_np = mask.cpu().numpy()
            mask_resized = cv2.resize(mask_np, (frame.shape[1], frame.shape[0]))
            mask_overlay[mask_resized > 0.5] = frame[mask_resized > 0.5]

   
    mem_after_infer = get_process_memory_mb()

    ram_text = f"RAM Usage: {mem_after_infer:.1f} MB (Delta: {mem_after_infer - mem_before_infer:+.2f} MB)"
    cv2.putText(annotated_frame, ram_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow("Original + Segmentation", annotated_frame)
    cv2.imshow("Segmented Mask", mask_overlay)

if len(sys.argv) > 1:
    image_path = sys.argv[1]
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"Error: Could not load image '{image_path}'")
        sys.exit(1)
    process_frame(frame)
    print(f"[RAM] Final Image Processing RAM: {get_process_memory_mb():.2f} MB")
    cv2.waitKey(0)
else:
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        process_frame(frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()

cv2.destroyAllWindows()
