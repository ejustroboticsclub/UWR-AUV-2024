import torch
from ultralytics import YOLO
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt

# Load the ONNX model
model = YOLO("___________________________________________________________________", task='detect')

# Manually load and preprocess the image using Pillow instead of OpenCV
def preprocess_image(image_path, input_size=(640, 640)):
    img = Image.open(image_path).convert("RGB")
    img = img.resize(input_size)
    img = np.array(img).astype(np.float32)
    img = img / 255.0  # Normalize to [0, 1]
    img = np.transpose(img, (2, 0, 1))  # Convert to CHW format
    img = np.expand_dims(img, axis=0)   # Add batch dimension
    return torch.tensor(img)

# Path to the image
image_path = "_______________________________________________________________________________"

# Preprocess the image manually
input_tensor = preprocess_image(image_path)

# Run inference using the pre-processed tensor
results = model(input_tensor)

# Print raw results for debugging
print(results)

# Post-process and visualize results using Matplotlib
plt.imshow(results[0].plot())  # results[0] gets the first image in the list
plt.axis('off')  # Hide axes
plt.show()  # Display the image with detections
