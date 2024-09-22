import torch
from ultralytics import YOLO
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
import imageio

# Load the ONNX model
model = YOLO("______________________________________________________________", task='detect')

# Manually preprocess the frame using Pillow instead of OpenCV
def preprocess_frame(frame, input_size=(640, 640)):
    img = Image.fromarray(frame).convert("RGB")  # Convert frame to RGB
    img = img.resize(input_size)
    img = np.array(img).astype(np.float32)
    img = img / 255.0  # Normalize to [0, 1]
    img = np.transpose(img, (2, 0, 1))  # Convert to CHW format
    img = np.expand_dims(img, axis=0)   # Add batch dimension
    return torch.tensor(img)

# Process video stream frame by frame
def process_video(video_path):
    reader = imageio.get_reader(video_path)  # Open video file or stream

    for frame in reader:
        # Preprocess each frame
        input_tensor = preprocess_frame(frame)

        # Run inference on the frame
        results = model(input_tensor)

        # Display the results
        plt.imshow(results[0].plot())  # results[0] gets the first detection result
        plt.axis('off')  # Hide axes
        plt.pause(0.001)  # Brief pause to allow for frame updating
        plt.clf()  # Clear the previous frame's plot for the next frame

# Example usage
video_path = "__________________________________________________"  # Replace with video file path or camera stream URL
process_video(video_path)
