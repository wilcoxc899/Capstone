import cv2
import numpy as np

# Load the image
image = cv2.imread('pillars.jpg')

# Convert the image from BGR to HSV color space
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Define the lower and upper bounds of the black color in HSV
lower_black = np.array([0, 0, 0])
upper_black = np.array([180, 255, 30])  # Adjust the upper bound for better detection

# Threshold the HSV image to get only black colors
mask = cv2.inRange(hsv_image, lower_black, upper_black)

# Find contours in the mask
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Initialize variables to store information about the largest circle
largest_radius = 0
largest_center = (0, 0)

# Iterate through the contours to find the largest circle
for contour in contours:
    # Get the center and radius of the contour
    (x, y), radius = cv2.minEnclosingCircle(contour)
    center = (int(x), int(y))
    radius = int(radius)

    # Update the largest circle if the current circle is larger
    if radius > largest_radius:
        largest_radius = radius
        largest_center = center

# Draw the largest circle in the middle of the detected black object
if largest_radius > 0:
    cv2.circle(image, largest_center, largest_radius, (0, 0, 255), 2)  # Red color, thickness=2

# Calculate angle from the center point
if largest_center != (0, 0):
    angle = np.arctan(largest_center[0] / largest_center[1]) * 180 / np.pi
    print("Angle:", angle)

# Show the original image with circles drawn on detected black objects
cv2.imwrite('Black Detection Result.jpg', image)
