from color_correct import correct
from ultralytics import YOLO
import numpy as np
import cv2

MODEL_PATH = "../../models/best-models/last.pt"
CONFIDENCE_THRESHOLD = 0.1
RATIO_COLOR_DETECTION_THRESHOLD = 0.3


class BucketDetection:
    def __init__(self):
        self.yolo_model = YOLO(MODEL_PATH)

    def detect(self, image: np.ndarray, color_correct=False):
        """
        Detect the bucket in the image.
        Args:
            image (np.ndarray) - BGR format: The image in which the bucket is to be detected.
            color_correct (bool) - Whether to apply color correction to the image before detection.
        Returns:
            results: The YOLOv8 results object containing bounding boxes and confidence scores.
        """
        image_copy = image.copy()
        if color_correct:
            image_copy = correct(image_copy)
        results = self.yolo_model(image_copy)
        return results

    def draw_bounding_boxes(self, image: np.ndarray, results, conf_threshold: float = CONFIDENCE_THRESHOLD, show_conf=True) -> np.ndarray:
        """
        Draw bounding boxes and confidence scores on the original image.
        Args:
            image (np.ndarray) - BGR format: The original image.
            results: The YOLOv8 results object containing bounding boxes and confidence scores.
            conf_threshold (float) - The confidence threshold to filter out weak detections.
            show_conf (bool) - Whether to show the confidence scores on the detected objects.
        Returns:
            np.ndarray - BGR format: The image with bounding boxes and confidence scores drawn.
        """
        image_copy = image.copy()

        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Filter out non-bucket detections
                if int(box.cls) != 0:
                    continue

                # Get the Bounding box coordinates (x1, y1, x2, y2)
                bounding_boxes = box.xyxy.cpu().numpy()

                # Get the confidence scores
                scores = box.conf.cpu().numpy()

                for bounding_box, score in zip(bounding_boxes, scores):
                    # Filter out weak detections
                    if score < conf_threshold:
                        continue

                    x1, y1, x2, y2 = map(int, bounding_box)
                    conf_text = f'{score:.2f}'

                    # Draw the bounding box
                    cv2.rectangle(image_copy, (x1, y1),
                                  (x2, y2), (0, 255, 0), 2)

                    # Put the confidence score on the image if required
                    if show_conf:
                        cv2.putText(image_copy, conf_text, (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return image_copy

    def get_bounding_boxes_and_conf_score(self, results) -> tuple[list[list[int]], list[float]]:
        """
        Extract bounding boxes and confidence scores from the YOLOv8 results.
        Args:
            results: The YOLOv8 results object containing bounding boxes and confidence scores.
        Returns:
            list[list[int]]: A list of bounding boxes in the format [x1, y1, x2, y2].
            list[float]: A list of confidence scores.
            None, None: If no bucket is detected in the image.
        """
        scores = []
        bboxes = []

        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Filter out non-bucket detections
                if int(box.cls) != 0:
                    continue

                # Get the Bounding box coordinates (x1, y1, x2, y2)
                bounding_boxes = box.xyxy.cpu().numpy()

                # Get the confidence scores
                score = box.conf.cpu().numpy().tolist()
                scores.append(score)

                for bounding_box in bounding_boxes:
                    x1, y1, x2, y2 = map(int, bounding_box)
                    bboxes.append([x1, y1, x2, y2])

        if len(bboxes) == 0:
            return None, None
        else:
            return bboxes, scores

    def get_direction_color(self, image: np.ndarray, bounding_boxes: list[list[int]], color: str, color_correct: bool = False) -> tuple[int, int]:
        """
        Get the direction of the bucket based on the color of the bucket.
        Args:
            image (np.ndarray) - BGR format: The original image containing the detected buckets.
            bounding_boxes (list[list[int]]) - A list of bounding boxes in the format [x1, y1, x2, y2].
            color (str) - The color of the bucket to be filtered. It can be 'red', 'green', or 'yellow'.
            color_correct (bool) - Whether to apply color correction to the image before color detection.
        Returns:
            tuple[int, int]: The direction of the bucket in the format (x, y).
            None, None: If no bucket is detected in the image.
        """
        x = None
        y = None

        # Check if there are no bounding boxes
        if bounding_boxes is None:
            return x, y

        for bounding_box in bounding_boxes:
            x1, y1, x2, y2 = bounding_box
            bucket_image = image[y1:y2, x1:x2]
            bucket_mask = self.detect_color(bucket_image, color, color_correct)
            bucket_pixels = cv2.countNonZero(bucket_mask)
            total_pixels = bucket_image.shape[0] * bucket_image.shape[1]
            bucket_ratio = bucket_pixels / total_pixels
            if bucket_ratio >= RATIO_COLOR_DETECTION_THRESHOLD:
                x = (x1 + x2) // 2
                y = y1
                break

        return x, y

    def get_direction_highest_confidence(self, bounding_boxes: list[list[int]], scores: list[float]) -> tuple[int, int]:
        """
        Get the direction of the bucket based on the highest confidence score.
        Args:
            bounding_boxes (list[list[int]]) - A list of bounding boxes in the format [x1, y1, x2, y2].
            scores (list[float]) - A list of confidence scores.
        Returns:
            tuple[int, int]: The direction of the bucket in the format (x, y).
            None, None: If no bucket is detected in the image.
        """
        x = None
        y = None

        # Check if there are no bounding boxes
        if bounding_boxes is None:
            return x, y

        max_score = max(scores)
        max_score_index = scores.index(max_score)

        x1, y1, x2, y2 = bounding_boxes[max_score_index]
        x = (x1 + x2) // 2
        y = y1

        return x, y

    def get_direction(self, image: np.ndarray, results, filter_criteria: str, color: str = None, color_correct: bool = False) -> tuple[int, int]:
        """
        Get the direction of the bucket based on the filter criteria.
        Args:
            image (np.ndarray) - BGR format: The original image containing the detected buckets.
            results: The YOLOv8 results object containing bounding boxes and confidence scores.
            filter_criteria (str) - The criteria to filter the buckets. It can be 'highest_confidence', or 'color'.
            color (str) - The color of the bucket to be filtered. It can be 'red', 'green', or 'yellow'.
            color_correct (bool) - Whether to apply color correction to the image before color detection.
        Returns:
            tuple[int, int]: The direction of the bucket in the format (x, y).
            None, None: If no bucket is detected in the image or no color is detected in the bucket.
        """
        bounding_boxes, scores = self.get_bounding_boxes_and_conf_score(
            results)

        if filter_criteria == "highest_confidence":
            x, y = self.get_direction_highest_confidence(
                bounding_boxes, scores)
            return x, y
        elif filter_criteria == "color":
            x, y = self.get_direction_color(
                image, bounding_boxes, color, color_correct)
            return x, y
        else:
            raise ValueError(
                "Invalid filter criteria specified. Please specify 'highest_confidence', or 'color'.")

    def recognize_bucket_color(self, image: np.ndarray, color_correct: bool = False) -> str:
        """
        Recognize the color of the bucket in the image by calculating the ratio of non-zero pixels.
        Args:
            image (np.ndarray) - BGR format: The cropped image of the bucket.
            color_correct (bool) - Whether to apply color correction to the image before color recognition.
        Returns:
            str: The color of the bucket detected in the image. It can be 'red', 'green', 'yellow'.
            None: If the color detection ratio is below the threshold, meaning the majority color is not red, green, or yellow.
        """
        # Detect the red, green, and yellow colors in the image
        red_mask = self.detect_color(image, "red", color_correct)
        green_mask = self.detect_color(image, "green", color_correct)
        yellow_mask = self.detect_color(image, "yellow", color_correct)

        # Calculate the number of non-zero pixels in each mask
        red_pixels = cv2.countNonZero(red_mask)
        green_pixels = cv2.countNonZero(green_mask)
        yellow_pixels = cv2.countNonZero(yellow_mask)

        print(
            f"Red pixels: {red_pixels}, Green pixels: {green_pixels}, Yellow pixels: {yellow_pixels}")

        # Total number of pixels in the image
        total_pixels = image.shape[0] * image.shape[1]

        # Calculate the ratio of non-zero pixels to total pixels for each color
        red_ratio = red_pixels / total_pixels
        green_ratio = green_pixels / total_pixels
        yellow_ratio = yellow_pixels / total_pixels

        print(
            f"Red ratio: {red_ratio:.2f}, Green ratio: {green_ratio:.2f}, Yellow ratio: {yellow_ratio:.2f}")

        # Store the color ratios in a dictionary
        color_ratio_dict = {"red": red_ratio,
                            "green": green_ratio, "yellow": yellow_ratio}

        # Find the color with the highest ratio
        bucket_color = max(color_ratio_dict, key=color_ratio_dict.get)
        highest_ratio = color_ratio_dict[bucket_color]

        # Return the color if the highest ratio is greater than or equal to the threshold, otherwise return None
        if highest_ratio >= RATIO_COLOR_DETECTION_THRESHOLD:
            return bucket_color
        else:
            return None

    def detect_color(self, image: np.ndarray, color: str, color_correct: bool = False) -> np.ndarray:
        """
        Detect the specified color in the image.
        Args:
            image (np.ndarray) - BGR format: The image in which the color is to be detected.
            color (str) - The color to be detected. It can be one of 'red', 'green', or 'yellow'.
            color_correct (bool) - Whether to apply color correction to the image before detection.
        Returns:
            np.ndarray - BGR format: The mask of the specified color in the image.
        """
        if color == "red":
            return self.detect_red_color(image, color_correct)
        elif color == "green":
            return self.detect_green_color(image, color_correct)
        elif color == "yellow":
            return self.detect_yellow_color(image, color_correct)
        else:
            raise ValueError(
                "Invalid color specified. Please specify 'red', 'green', or 'yellow'.")

    def detect_red_color(self, image: np.ndarray, color_correct: bool = False) -> np.ndarray:
        """
        Detect red color in the image.
        Args:
            image (np.ndarray) - BGR format: The image in which the red color is to be detected.
            color_correct (bool) - Whether to apply color correction to the image before detection.
        Returns:
            np.ndarray - BGR format: The mask of the red color in the image.
        """
        image_copy = image.copy()

        # Apply color correction to the image if required
        if color_correct:
            image_copy = correct(image_copy)

        # Convert the image to the HSV color space
        hsv_image = cv2.cvtColor(image_copy, cv2.COLOR_BGR2HSV)

        # Define lower and upper bounds for the red color in HSV
        lower_red1 = np.array([0, 75, 20])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([160, 75, 20])
        upper_red2 = np.array([180, 255, 255])

        # Create masks to detect the red color in both specified ranges
        lower_mask = cv2.inRange(hsv_image, lower_red1, upper_red1)
        upper_mask = cv2.inRange(hsv_image, lower_red2, upper_red2)

        # Combine the lower and upper masks to get a single mask
        combined_mask = lower_mask + upper_mask

        return combined_mask

    def detect_green_color(self, image: np.ndarray, color_correct: bool = False) -> np.ndarray:
        """
        Detect green color in the image.
        Args:
            image (np.ndarray) - BGR format: The image in which the green color is to be detected.
            color_correct (bool) - Whether to apply color correction to the image before detection.
        Returns:
            np.ndarray - BGR format: The mask of the green color in the image.
        """
        image_copy = image.copy()

        # Apply color correction to the image if required
        if color_correct:
            image_copy = correct(image_copy)

        # Convert the image to the HSV color space
        hsv_image = cv2.cvtColor(image_copy, cv2.COLOR_BGR2HSV)

        # Define lower and upper bounds for the green color in HSV
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])

        # Create mask to detect the green color
        mask = cv2.inRange(hsv_image, lower_green, upper_green)

        return mask

    def detect_yellow_color(self, image: np.ndarray, color_correct: bool = False) -> np.ndarray:
        """
        Detect yellow color in the image.
        Args:
            image (np.ndarray) - BGR format: The image in which the yellow color is to be detected.
            color_correct (bool) - Whether to apply color correction to the image before detection.
        Returns:
            np.ndarray - BGR format: The mask of the yellow color in the image.
        """
        image_copy = image.copy()

        # Apply color correction to the image if required
        if color_correct:
            image_copy = correct(image_copy)

        # Convert the image to the HSV color space
        hsv_image = cv2.cvtColor(image_copy, cv2.COLOR_BGR2HSV)

        # Define lower and upper bounds for the yellow color in HSV
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        # Create mask to detect the yellow color
        mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

        return mask

    def extract_buckets_from_image(self, image: np.ndarray, results) -> list[np.ndarray]:
        """
        Extract the detected buckets from the YOLOv8 results.
        Args:
            image (np.ndarray) - BGR format: The original image containing the detected buckets.
            results: The YOLOv8 results object containing bounding boxes and confidence scores.
        Returns:
            list[np.ndarray]: A list of cropped images of the detected buckets.
            None: If no bucket is detected in the image.
        """
        bucket_images = []
        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Filter out non-bucket detections
                if int(box.cls) != 0:
                    continue

                # Get the Bounding box coordinates (x1, y1, x2, y2)
                bounding_boxes = box.xyxy.cpu().numpy()

                for bounding_box in bounding_boxes:
                    x1, y1, x2, y2 = map(int, bounding_box)
                    bucket_image = image[y1:y2, x1:x2]
                    bucket_images.append(bucket_image)

        if len(bucket_images) == 0:
            return None
        else:
            return bucket_images


if __name__ == "__main__":
    # Load an image
    image_path = "../../images/image5.jpg"
    color_correct = True
    image = cv2.imread(image_path)

    # Initialize the BucketDetection class
    bucket_detector = BucketDetection()

    # Detect the bucket in the image
    results = bucket_detector.detect(image, color_correct)

    # Draw bounding boxes on the image
    image_with_boxes = bucket_detector.draw_bounding_boxes(image, results)

    # Extract the detected buckets from the image
    bucket_images = bucket_detector.extract_buckets_from_image(image, results)

    # Display the bucket images
    # if bucket_images is not None:
    #     for i, bucket_image in enumerate(bucket_images):
    #         cv2.imshow(f"Bucket {i + 1}", bucket_image)

    # Detect the red color of the buckets
    if bucket_images is not None:
        for i, bucket_image in enumerate(bucket_images):
            mask = bucket_detector.detect_color(
                bucket_image, "red", color_correct)
            cv2.imshow("mask"+str(i), mask)

    if bucket_images is not None:
        for i, bucket_image in enumerate(bucket_images):
            # Recognize the color of the bucket
            bucket_color = bucket_detector.recognize_bucket_color(
                bucket_image, color_correct)

            if bucket_color is not None:
                print(f"Bucket {i + 1} color: {bucket_color}")
            else:
                print(f"Bucket {i + 1} color: Unknown")
    else:
        print("No bucket detected in the image.")


    # Display the image with bounding boxes
    cv2.imshow("Detected Buckets", image_with_boxes)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
