import cv2
from ultralytics import YOLO

# Load the model
model = YOLO("__________________________________________")

# open the stream
IP = "rtsp://admin:admin@192.168.100.60:1935"
pipeline = (
    f"rtspsrc location={IP} latency=0 ! "
    "rtph264depay ! avdec_h264 ! videoconvert ! appsink max-buffers=1 drop=True"
)
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Perform inference
    results = model(frame)

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

            # Draw bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Put label with class name and confidence
            label = f"{class_name} {confidence:.2f}"
            (label_width, label_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(frame, (x1, y1 - 20), (x1 + label_width, y1), (0, 255, 0), -1)
            cv2.putText(frame, label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    # Display the resulting frame
    cv2.imshow('Shapes_detection', frame)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()