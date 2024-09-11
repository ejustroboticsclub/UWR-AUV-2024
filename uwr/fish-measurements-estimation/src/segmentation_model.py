import ultralytics
import numpy as np
import cv2
from color_correct import correct

WEIGHTS_PATH = "../models/best-models/segmentation_model_last.pt"

class SegmentationModel:
    def __init__(self):
        self.model = ultralytics.YOLO(WEIGHTS_PATH)
    
    def segment(self, image: np.ndarray, color_correct: bool = False):
        """
        Predict the segmentation of the image.
        Args:
            image (np.ndarray - BGR format): image to predict the segmentation of.
            color_correct (bool): whether to color correct the image before predicting the segmentation.
        Returns:
            results of the segmentation model.
        """
        image_copy = image.copy()
        if color_correct:
            image_copy = correct(image_copy)
        results = self.model(image_copy)[0]
        return results

    def get_object_polygon_points(self, results) -> list[np.ndarray]:
        """
        Get the object polygon points as integers which are the masks of the detected objects in the image.
        They are x, y coordinates of the polygon points of the detected objects.
        Args:
            results: results of the segmentation model.
        Returns:
            list[np.ndarray]: list of polygons of detected objects.
        """
        masks = results.masks

        # Return an empty list if no objects are detected.
        if masks is None:
            return []
        
        # get the x, y coordinates of the polygon points of the detected objects.
        polygon_points = masks.xy

        # convert the x, y coordinates to integers.
        polygon_points_int = [array.astype(int) for array in polygon_points]

        return polygon_points_int
    
    def color_polygon_region(self, image, results, mask_color=(255, 0, 0)) -> np.ndarray:
        """
        Color the region of the detected object in the image.
        Args:
            image (np.ndarray - BGR format): image to color the detected object region in.
            results: results of the segmentation model.
            mask_color (tuple): color to use for coloring the detected object region.
        Returns:
            np.ndarray (BGR format): image with the detected object region colored.
        """
        # Get the polygon points of the detected objects.
        polygon_points = self.get_object_polygon_points(results)

        # Return the original image if no objects are detected.
        if len(polygon_points) == 0:
            return image
        
        # Fill the detected objects region with the mask color.
        image_copy = image.copy()
        for polygon in polygon_points:
            cv2.fillPoly(image_copy, [polygon], mask_color)
        
        return image_copy

if __name__ == "__main__":
    model = SegmentationModel()
    image = cv2.imread("../data/external/all-pagrus-images/1_06_21-B4.jpg")
    results = model.segment(image)
    colored_image = model.color_polygon_region(image, results)
    cv2.imwrite("colored_image.jpg", colored_image)
    # cv2.imshow("Colored Image", colored_image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()


        