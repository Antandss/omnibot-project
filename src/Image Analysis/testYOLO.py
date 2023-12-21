import time
from ultralytics import YOLO
import torch

# Load a pre-trained model
model = YOLO("yolov8n.pt")

# Train the model
# Note: Make sure to specify the correct path in 'data' and adjust 'epochs' as needed
model.train(data="src/Image Analysis/config.yaml", epochs=100)

