import cv2

# Load the image
image_path = 'test_image.jpg'  # Replace 'path_to_your_image.jpg' with the actual path to your image
image = cv2.imread(image_path)

# Check if the image is loaded successfully
if image is not None:
    # Call the findAngle function and pass the image as an argument
    angle = findAngle(image)

    # Print the calculated angle
    print("Angle:", angle)

    # Display the modified image with the circle drawn
    cv2.imshow('Image with Circle', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("Error loading the image.")

def findAngle(img):
    try:
        img
    except NameError:
        return 360

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (11, 0, 0), (17, 255,255))
    mask_blur = cv2.blur(mask,(5,5))
    thresh = cv2.threshold(mask_blur, 200, 255, cv2.THRESH_BINARY)[1]
    cv2.imwrite('thresh.jpg', thresh)
    M = cv2.moments(thresh)
    if(M["m00"]==0):
        return 360
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    thresh = cv2.circle(img, (cX,cY), 40, (0,0,255), 20) # red circle
    cv2.imwrite('testimg.jpg', img)
    
    return np.arctan((cX-(img.shape[0])/2)/(img.shape[1]-cY))*180/np.pi-20
