import cv2
import numpy as np

# open camera
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)

# set dimensions
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 256)

# take frame
ret, frame = cap.read()
# write frame to file
cv2.imwrite('awesome.jpg', frame)
# release camera
cap.release()
# Load the image
img = cv2.imread('awesome.jpg')

# FLip the image
image = cv2.flip(img, -1)

# Convert the image to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Apply Gaussian blur to reduce noise
blurred = cv2.GaussianBlur(gray, (5, 5), 0)

# Perform edge detection using Canny
edges = cv2.Canny(blurred, 50, 150)

# Perform Hough line detection
lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=100, minLineLength=100, maxLineGap=10)

# Filter lines to keep only those likely to be sidewalk edges
filtered_lines = []
if lines is not None:
    for line in lines:
        x1, y1, x2, y2 = line[0]
        # take an angle to be used later
        angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
        # Filter out lines that are not in the left half
        if min(x1,x2)< 0.5*image.shape[1]:
            max_length=0
            for lines in lines:
                if len(line) > max_length:
                    longest_line = line
                    max_length = len(line)
                    filteredleft_lines.append(line)
        else:
            max_length=0
            for lines in lines:
                if len(line) > max_length:
                    longest_line = line
                    max_length = len(line)
                    filteredright_lines.append(line)

# Draw detected lines and endpoints on the original image
if lines is not None:
    for line in filteredleft_lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    for line in filteredright_lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(image, (x1, y1), (x2, y2), (255, 0, 0), 2)


# Display the result
cv2.imwrite('Sidewalk Endpoint Detection.jpg', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
