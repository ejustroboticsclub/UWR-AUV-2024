import cv2
import threading
from ultralytics import YOLO

# Load the YOLO model
model = YOLO("______________________________")

# Function to capture frames in a separate thread
def capture_frames(cap):
    global frame, ret
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

# Open the RTSP stream
IP = "rtsp://admin:admin@192.168.100.60:1935"
pipeline = (
    f"rtspsrc location={IP} latency=0 ! "
    "rtph264depay ! avdec_h264 ! videoconvert ! appsink max-buffers=1 drop=True"
)
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

# Check if the stream is opened correctly
if not cap.isOpened():
    print("Error: Unable to open video stream.")
    exit()

# Start capturing frames in a separate thread
ret, frame = None, None
capture_thread = threading.Thread(target=capture_frames, args=(cap,))
capture_thread.start()

frame_counter = 0  # Initialize frame counter
frame_skip = 3  # Process every nth frame to reduce load

while True:
    if frame is None:
        continue
    
    frame_counter += 1
    if frame_counter % frame_skip != 0:
        continue  # Skip frames for performance
    
    # Resize frame to speed up processing
    resized_frame = cv2.resize(frame, (640, 480))

    # Perform inference
    results = model(resized_frame)

    # Parse results
    for result in results:
        boxes = result.boxes  # Bounding boxes
        for box in boxes:
            # Extract box coordinates
            x1, y1, x2, y2 = box.xyxy[0]  # Coordinates
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            # Extract confidence and class
            confidence = box.conf[0].item()
            cls_id = box.cls[0].item()
            class_name = model.names[int(cls_id)]

            # Draw bounding box with confidence
            cv2.rectangle(resized_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Put label with class name and confidence
            label = f"{class_name} {confidence:.2f}"
            (label_width, label_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(resized_frame, (x1, y1 - label_height - 4), 
                          (x1 + label_width, y1), (0, 255, 0), -1)
            cv2.putText(resized_frame, label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    # Display the resulting frame
    cv2.imshow('Faster_shapes_detection', resized_frame)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Gracefully close the stream and the thread
capture_thread.join()
cap.release()
cv2.destroyAllWindows()
