import cv2
import numpy as np

def findAngle(img, lower_hsv=(55, 40, 70), upper_hsv=(75, 50, 85)):
    try:
        img
    except NameError:
        return 360

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    mask_blur = cv2.blur(mask, (5, 5))
    thresh = cv2.threshold(mask_blur, 200, 255, cv2.THRESH_BINARY)[1]
    cv2.imwrite('thresh.jpg', thresh)

    # Check the thresholded image
    cv2.imshow('Thresholded Image', thresh)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    M = cv2.moments(thresh)
    if M["m00"] == 0:
        return 360

    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])

    # Check the centroid coordinates
    print("Centroid (X, Y):", cX, cY)

    thresh = cv2.circle(img, (cX, cY), 40, (0, 0, 255), 20)  # red circle
    cv2.imwrite('testimg.jpg', img)

    return np.arctan((cX - (img.shape[0]) / 2) / (img.shape[1] - cY)) * 180 / np.pi - 20

# Load the image
image_path = 'test_image1.jpg'
image = cv2.imread(image_path)

# Check if the image is loaded successfully
if image is not None:
    # Call the findAngle function with custom HSV values
    angle = findAngle(image, lower_hsv=(60, 50, 75), upper_hsv=(66, 255, 255))

    # Print the calculated angle
    print("Angle:", angle)

    # Save modified image with circle
    cv2.imwrite('image_with_circle.jpg', image)
else:
    print("Error loading the image.")
