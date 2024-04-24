import cv2
import numpy as np
image = cv2.imread('halfrightcropped.jpg')

def average_regions(image, num_rows, num_cols):
    height, width, _ = image.shape
    region_height = height // num_rows
    region_width = width // num_cols

    averaged_image = np.zeros_like(image)

    for i in range(num_rows):
        for j in range(num_cols):
            start_x = j * region_width
            end_x = (j + 1) * region_width
            start_y = i * region_height
            end_y = (i + 1) * region_height

            region = image[start_y:end_y, start_x:end_x]
            average_color = np.mean(region, axis=(0, 1)).astype(np.uint8)

            averaged_image[start_y:end_y, start_x:end_x] = average_color

    return averaged_image

# Load the image
image = cv2.imread("input_image.jpg")

# Define the number of rows and columns to divide the image into
num_rows = 10
num_cols = 10

# Process the image
averaged_image = average_regions(image, num_rows, num_cols)

# Display the original and processed images
cv2.imshow("Original Image", image)
cv2.imshow("Averaged Image", averaged_image)
cv2.waitKey(0)
cv2.destroyAllWindows()


