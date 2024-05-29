import cv2
import numpy as np

def calculate_normalized_cross_correlation(image_path, template_path):

    image = cv2.imread(image_path)
    template = cv2.imread(template_path)

    if image is None or template is None:
        raise ValueError(f"One or more images did not load. Please check the file paths.\n Image: {image_path}\n Template: {template_path}")

    # Convert images to grayscale
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)

    # Apply template matching
    result = cv2.matchTemplate(image_gray, template_gray, cv2.TM_CCORR_NORMED)

    # Find the maximum NCC value
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)

    return max_val

def main(image1_path, template_path):
    print(image1_path)
    print(template_path)
    ncc = calculate_normalized_cross_correlation(image1_path, template_path)
    return ncc
