from segmentation_model import SegmentationModel
from rotatingcaliper import rotatingCaliper
import numpy as np
import cv2


COLOR_CORRECT = False
LENGTH_IN_PIXELS_ARR_PATH = "../data/length_in_pixels.npy"
LENGTH_IN_CM_ARR_PATH = "../data/length_in_cm.npy"
k = 3


class LengthEstimation(SegmentationModel):
    def __init__(self):
        super().__init__()
        self.length_in_pixels = np.load(LENGTH_IN_PIXELS_ARR_PATH)
        self.length_in_cm = np.load(LENGTH_IN_CM_ARR_PATH)

    def knn(self, x: float, k: int) -> float:
        """
        Perform k-nearest neighbors regression to estimate the length in cm.
        Args:
            x (float): length in pixels.
            k (int): number of neighbors to consider.
        Returns:
            float: estimated length in cm.
        """
        # Calculate the absolute difference between x and each length in pixels
        differences = np.abs(self.length_in_pixels - x)

        nearest_neighbors = []

        # Iterate over the differences and their indices
        for i, diff in enumerate(differences):
            # If we have less than k neighbors, add the current one
            if len(nearest_neighbors) < k:
                nearest_neighbors.append((diff, self.length_in_cm[i]))
                nearest_neighbors.sort()  # Keep the list sorted
            else:
                # Check if the current neighbor is closer than the farthest in the list
                if diff < nearest_neighbors[-1][0]:
                    nearest_neighbors[-1] = (diff, self.length_in_cm[i])
                    nearest_neighbors.sort()  # Keep the list sorted

        # Extract the lengths in cm from the nearest neighbors
        nearest_lengths_cm = [length for _, length in nearest_neighbors]

        # Compute the average of these lengths in cm
        average_length_cm = np.mean(nearest_lengths_cm)

        return average_length_cm

    def transform_total_length_to_cm(self, total_length: float) -> float:
        """
        Transform total length in pixels to cm.
        Args:
            total_length (float): total length to transform to cm.
        Returns:
            float: total length in cm.
        """
        total_length = self.knn(total_length, k)
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
        FONT_SCALE = 2.5  # Increased font scale
        FONT_THICKNESS = 4  # Increased font thickness
        PADDING = 30  # Increased padding for the rectangle

        image_copy = image.copy()
        for total_length, p1, p2 in data:
            # Draw line between endpoints
            cv2.line(image_copy, tuple(p1), tuple(p2),
                     color=COLOR_LINE, thickness=THICKNESS)

            # Compute middle point for placing text
            middle_point = tuple((p1 + p2) // 2)

            # Set text properties
            text = f"{total_length:.2f} cm"
            font = cv2.FONT_HERSHEY_SIMPLEX
            (text_width, text_height), baseline = cv2.getTextSize(
                text, font, FONT_SCALE, FONT_THICKNESS)

            # Define rectangle position and size with increased padding
            rectangle_top_left = (
                middle_point[0] - text_width // 2 - PADDING, middle_point[1] - text_height // 2 - PADDING)
            rectangle_bottom_right = (
                middle_point[0] + text_width // 2 + PADDING, middle_point[1] + text_height // 2 + PADDING)

            # Draw the rectangle with a white background
            cv2.rectangle(image_copy, rectangle_top_left,
                          rectangle_bottom_right, RECTANGLE_COLOR, cv2.FILLED)

            # Put the text inside the rectangle
            cv2.putText(
                image_copy,
                text,
                (middle_point[0] - text_width // 2,
                 middle_point[1] + text_height // 2),
                font,
                FONT_SCALE,
                COLOR_TEXT,
                FONT_THICKNESS,
                lineType=cv2.LINE_AA
            )

        return image_copy


if __name__ == "__main__":
    image = cv2.imread("../data/external/all-pagrus-images/1_06_21-B4.jpg")
    if image.shape != (3024, 4032, 3): # comman shape in the data
        image = cv2.resize(image, (4032, 3024))
    # Define the scaling factors
    # scale_factor_x = 2  # Scaling factor for width
    # scale_factor_y = 2  # Scaling factor for height
    # # Resize the image
    # scaled_image = cv2.resize(image, None, fx=scale_factor_x, fy=scale_factor_y, interpolation=cv2.INTER_LINEAR)
    # image = scaled_image

    model = LengthEstimation()
    results = model.segment(image, COLOR_CORRECT)
    polygon_points = model.get_object_polygon_points(results)
    data = model.get_total_length_in_cm_and_endpoints(polygon_points)
    image_with_length = model.draw_total_length_and_endpoints(image, data)
    cv2.imwrite("image_with_length.jpg", image_with_length)
