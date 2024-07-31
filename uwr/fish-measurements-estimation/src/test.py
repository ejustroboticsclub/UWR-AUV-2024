"""
This is a test file for the project. It takes all the images in the 
`data/external/all-pagrus-images` and `data/external/pagrus-images-with-water-effect` 
directories and run the segmentation model on them, draw the 
a line between the endpoints as well as the total length of the detected fish in cm,
and finally save the images in the `results` directory.
"""

import cv2
import os
from length_estimation import LengthEstimation

def do_job(images_dir: str, results_dir: str) -> None:
    estimator = LengthEstimation()
    for image_file in os.listdir(images_dir):
        image_path = os.path.join(images_dir, image_file)
        image = cv2.imread(image_path)
        results = estimator.segment(image)
        polygon_points = estimator.get_object_polygon_points(results)
        if len(polygon_points) == 0:
            continue
        data = estimator.get_total_length_in_cm_and_endpoints(polygon_points)
        image_with_length = estimator.draw_total_length_and_endpoints(image, data)
        cv2.imwrite(os.path.join(results_dir, image_file), image_with_length)


def main():
    # Pargus images without water effect
    images_dir = "../data/external/all-pagrus-images"
    results_dir = "../results/without-water-effect"
    os.makedirs(results_dir, exist_ok=True)
    do_job(images_dir, results_dir)

    # Pargus images with water effect
    images_dir = "../data/external/pagrus-images-with-water-effect"
    results_dir = "../results/with-water-effect"
    os.makedirs(results_dir, exist_ok=True)
    do_job(images_dir, results_dir)

if __name__ == "__main__":
    main()