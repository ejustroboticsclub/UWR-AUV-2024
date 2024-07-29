from segmentation_model import SegmentationModel
from rotatingcaliper import rotatingCaliper
import numpy as np
import cv2


class LengthEstimation(SegmentationModel):
    def __init__(self):
        super().__init__()

    def transform_total_length_to_cm(self, total_length: float) -> float:
        """
        Transform total length in pixels to cm.
        Args:
            total_length (float): total length to transform to cm.
        Returns:
            float: total length in cm.
        """
        return total_length

    def get_total_length_in_cm_and_endpoints(self, polygon_points: np.ndarray) -> list[tuple[float, np.ndarray, np.ndarray]]:
        """
        Get the total length in cm and the endpoints of the polygon that has the maximum length.
        The two points are the fish mouth and the end of its tail.
        Args:
            polygon_points (np.ndarray): polygon points of the detected objects in the image.
        Returns:
            list[tuple[float, np.ndarray, np.ndarray]]: list of tuples that contain the total length in cm of the detected object and the endpoints of the polygon as x, y coordinates.
        """
        data = []
        for p in polygon_points:
            total_length, pt_dict = rotatingCaliper(p)
            total_length = self.transform_total_length_to_cm(total_length)
            p1 = pt_dict["p1"]
            p2 = pt_dict["p2"]
            if p1 is None or p2 is None:
                continue
            data.append((total_length, p1, p2))
        return data
    
    def draw_total_length_and_endpoints(self, image: np.ndarray, data: list[tuple[float, np.ndarray, np.ndarray]]):
        """
        Draw the total length and the endpoints of the detected objects on the image.
        Args:
            image (np.ndarray - BGR format): image to draw the total length and the endpoints of the detected objects on.
            data (list[tuple[float, np.ndarray, np.ndarray]]): list of tuples that contain the total length in cm of the detected object and the endpoints of the polygon as x, y coordinates.
        Returns:
            np.ndarray (BGR format): image with the total length and the endpoints of the detected objects drawn on.
        """
        COLOR_LINE = (0, 255, 0)
        COLOR_TEXT = (0, 0, 255) 
        THICKNESS = 4
        RECTANGLE_COLOR = (255, 255, 255)
        FONT_SCALE = 1.5
        FONT_THICKNESS = 2


        image_copy = image.copy()
        for total_length, p1, p2 in data:
            # Draw line between endpoints
            cv2.line(image_copy, tuple(p1), tuple(p2), color=COLOR_LINE, thickness=THICKNESS)

            # Compute middle point for placing text
            middle_point = tuple((p1 + p2) // 2)

            # Set text properties
            text = f"{total_length:.2f} cm"
            font = cv2.FONT_HERSHEY_SIMPLEX
            (text_width, text_height), baseline = cv2.getTextSize(text, font, FONT_SCALE, FONT_THICKNESS)

            # Define rectangle position and size
            rectangle_top_left = (middle_point[0] - text_width // 2 - 10, middle_point[1] - text_height // 2 - 10)
            rectangle_bottom_right = (middle_point[0] + text_width // 2 + 10, middle_point[1] + text_height // 2 + 10)

            # Draw the rectangle with a white background
            cv2.rectangle(image_copy, rectangle_top_left, rectangle_bottom_right, RECTANGLE_COLOR, cv2.FILLED)

            # Put the text inside the rectangle
            cv2.putText(
                image_copy,
                text,
                (middle_point[0] - text_width // 2, middle_point[1] + text_height // 2),
                font,
                FONT_SCALE,
                COLOR_TEXT,
                FONT_THICKNESS,
                lineType=cv2.LINE_AA
            )

        return image_copy
    
if __name__ == "__main__":
    image = cv2.imread("1_06_21-B4.jpg")
    model = LengthEstimation()
    results = model.predict(image)
    polygon_points = model.get_object_polygon_points(results)
    data = model.get_total_length_in_cm_and_endpoints(polygon_points)
    image_with_length = model.draw_total_length_and_endpoints(image, data)
    cv2.imwrite("image_with_length.jpg", image_with_length)