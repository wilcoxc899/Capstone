import cv2
import numpy as np

# Load the image
image = cv2.imread('jimmy.jpg')

# Convert the image from BGR to HSV color space
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Define the lower and upper bounds of the orange color in HSV
lower_orange = np.array([0, 100, 100])
upper_orange = np.array([20, 255, 255])

# Threshold the HSV image to get only orange colors
mask = cv2.inRange(hsv_image, lower_orange, upper_orange)

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
	

# Draw the largest circle in the middle of the detected orange object
if largest_radius > 0:
    cv2.circle(image, largest_center, largest_radius, (0, 0, 255), 2)   
    # Draw the circle
    cv2.circle(image, center, radius, (0, 0, 255), 2)
    print (center)
  # Red color, thickness=2

# Show the original image with circles drawn on detected orange objects
cv2.imwrite('Orange Detection Result.jpg', image)
