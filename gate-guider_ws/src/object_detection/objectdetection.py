import cv2
import random
import numpy as np
import time
import argparse
import os
import torch
import yolov5
from yolov5.utils.torch_utils import select_device
from yolov5.models.experimental import attempt_load
from yolov5.utils.general import non_max_suppression, scale_segments, xyxy2xywh
from yolov5.utils.augmentations import letterbox



def plot_one_box(xyxy, img, color=None, label=None, line_thickness=None):
    # Plots one bounding box on image img
    tl = line_thickness or round(0.002 * max(img.shape[0:2])) + 1  # line thickness
    color = color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3]))
    cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
    if label:
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
        cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)



# Initialize the Intel RealSense D435f
import pyrealsense2 as rs
pipe = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipe.start(config)

# Set up YOLOv5 model
device = select_device('')
model = attempt_load('best.pt', device= device)

# Set up OpenCV window
cv2.namedWindow("YOLOv5 Object Detection", cv2.WINDOW_NORMAL)

# Define random colors for each class
random.seed(0)
colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(model.names))]

while True:
    # Wait for a coherent color frame
    frames = pipe.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        continue

    # Convert the frame to a numpy array
    color_image = np.asanyarray(color_frame.get_data())

    # Prepare the image for YOLOv5 model
    img0 = color_image.copy()
    img = letterbox(img0, new_shape=640)[0]
    img = img[:, :, ::-1].transpose(2, 0, 1)
    img = np.ascontiguousarray(img)

    # Run the YOLOv5 model
    img = torch.from_numpy(img).to(device)
    img = img.float() / 255.0
    img = img.unsqueeze(0)
    pred = model(img)[0]
    pred = non_max_suppression(pred, 0.4, 0.5, agnostic=False)

    # Process the YOLOv5 output
    for i, det in enumerate(pred):
        if det is not None and len(det):
            det[:, :4] = scale_segments(img.shape[2:], det[:, :4], img0.shape).round()
            for *xyxy, conf, cls in reversed(det):
                label = f'{model.names[int(cls)]} {conf:.2f}'
                plot_one_box(xyxy, img0, label=label, color=colors[int(cls)], line_thickness=3)

    # Show the image with detected objects
    cv2.imshow("YOLOv5 Object Detection", img0)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up
pipe.stop()
cv2.destroyAllWindows()

