# Semantic Mapping

Real-time semantic segmentation using YOLOv8 for robotics applications.

## Demo

[![Semantic Mapping Demo](https://img.youtube.com/vi/XxDwwBQxVR0/0.jpg)](https://youtu.be/XxDwwBQxVR0)

Click the image above or [watch the demo on YouTube](https://youtu.be/XxDwwBQxVR0).

## Files

- `experimentation.py` - Live webcam segmentation with masked overlay
- `main.py` - Entry point (to be implemented)
- `yolov8n-seg.pt` - Pre-trained YOLOv8 nano segmentation model (ignored by git)

## Usage

```bash
python experimentation.py
```

Press `q` to quit. Shows two windows:
1. Original feed with segmentation overlay
2. Segmented mask only (background removed)

## Requirements

- Python 3.8+
- OpenCV (`cv2`)
- NumPy
- Ultralytics (`pip install ultralytics`)