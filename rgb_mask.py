import cv2
import numpy as np

def findAngle(img_path, lower_rgb, upper_rgb, circle_radius=40, circle_thickness=20):
    # Load the image
    img = cv2.imread(img_path)
    if img is None:
        print("Error loading the image.")
        return

    # Create a mask using RGB range
    mask = cv2.inRange(img, lower_rgb, upper_rgb)
    cv2.imwrite('mask.jpg', mask)  # Debugging output: Save the mask image
    
    # Blur the mask for smoothing
    mask_blur = cv2.blur(mask, (5, 5))
    cv2.imwrite('mask_blur.jpg', mask_blur)  # Debugging output: Save the blurred mask image
    
    # Apply threshold to create binary image
    _, thresh = cv2.threshold(mask_blur, 200, 255, cv2.THRESH_BINARY)
    cv2.imwrite('thresh.jpg', thresh)  # Debugging output: Save the thresholded image
   
    # Calculate moments of the binary image
    M = cv2.moments(thresh)

    # Check if moment is valid
    if M["m00"] == 0:
        print("Invalid moment. Returning default angle.")
        return 360

    # Calculate centroid coordinates
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])

    # Draw a red circle around the centroid
    img_with_circle = cv2.circle(img.copy(), (cX, cY), circle_radius, (0, 0, 255), circle_thickness)
    cv2.imwrite('image_with_circle.jpg', img_with_circle)  # Debugging output: Save the image with circle
    
    # Save the modified image with the circle drawn
    cv2.imwrite('image_with_circle.jpg', img_with_circle)

    # Calculate the angle using trigonometry
    angle_rad = np.arctan((cX - (img.shape[0]) / 2) / (img.shape[1] - cY))
    angle_deg = np.degrees(angle_rad) - 20  # Convert to degrees and adjust

    return angle_deg

# Specify the input image path
image_path = 'jimmy.jpg'

# Specify the RGB range for the mask
lower_rgb = (200, 100, 0)
upper_rgb = (255, 160, 30)

# Specify circle radius and thickness
radius = 40
thickness = 20

# Call the function to find the angle and save the modified image
angle = findAngle(image_path, lower_rgb, upper_rgb, radius, thickness)

# Print the calculated angle
print("Angle:", angle)

