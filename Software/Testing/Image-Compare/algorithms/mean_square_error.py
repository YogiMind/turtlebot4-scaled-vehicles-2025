import numpy as np
from skimage.metrics import structural_similarity as ssim
from skimage.io import imread
from skimage.color import rgb2gray
import cv2

def mse(imageA, imageB):
    """Compute the Mean Squared Error between the two images."""
    err = np.sum((imageA.astype("float") - imageB.astype("float")) ** 2)
    err /= float(imageA.shape[0] * imageA.shape[1])
    return err

def apply_mask(image, threshold=75):
    """Apply a binary threshold to create a mask that isolates lines."""
    _, mask = cv2.threshold(image, threshold, 255, cv2.THRESH_BINARY)
    return mask

def compare_images(imageA, imageB, use_mask=True):
    # Handle RGBA images by slicing off the alpha channel
    if imageA.shape[2] == 4:
        imageA = imageA[:, :, :3]
    if imageB.shape[2] == 4:
        imageB = imageB[:, :, :3]

    # Convert images to grayscale
    imageA_gray = rgb2gray(imageA)
    imageB_gray = rgb2gray(imageB)

    # Apply Gaussian Blur
    #imageA_gray = cv2.GaussianBlur(imageA_gray, (5, 5), 0)
    #imageB_gray = cv2.GaussianBlur(imageB_gray, (5, 5), 0)

    # Optional: Use a mask to focus on the lines
    if use_mask:
        mask = apply_mask((imageA_gray * 255).astype(np.uint8))  # Create a mask from the first image
        imageA_gray = np.where(mask == 255, imageA_gray, 0)
        imageB_gray = np.where(mask == 255, imageB_gray, 0)

    # Compute MSE and SSIM 
    m = mse(imageA_gray, imageB_gray)
    s = ssim(imageA_gray, imageB_gray, data_range=imageA_gray.max() - imageA_gray.min())
    return m, s

def main(imageA_path, imageB_path):
    # Load the images
    imageA = imread(imageA_path)
    imageB = imread(imageB_path)

    # Compare the images
    mse_result, ssim_result = compare_images(imageA, imageB)

    return mse_result