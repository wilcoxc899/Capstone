import cv2
import numpy as np

# open camera
#cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)

# set dimensions
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 256)

# take frame
#ret, frame = cap.read()
# write frame to file
#cv2.imwrite('awesome.jpg', frame)
# release camera
#cap.release()
# Load the image
image = cv2.imread('straight_cropped2.jpg')

# Flip the image
#image = cv2.flip(img, -1)


# Convert the image to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Apply Gaussian blur to reduce noise
blurred = cv2.GaussianBlur(gray, (1, 1), 0)

# Perform edge detection using Canny
edges = cv2.Canny(blurred, 50, 150)

# Perform Hough line detection
lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=100, minLineLength=100, maxLineGap=50)

# Filter lines to keep only those likely to be sidewalk edges
filteredleft_lines= []
filteredright_lines= []
if lines is not None:
    max_length=0
    for line in lines:
        x1, y1, x2, y2 = line[0]
        # take an angle to be used later
        angle_rad=np.arctan2(y2-y1,x2-x1)
        angle=np.degrees(angle_rad)
        # Filter out lines that are not in the left half
        if min(x1,x2)> 0.5*image.shape[1] and max(y1,y2)>0.5*image.shape[0]:
            max_length=0
            if len(line) > 0:
                #filteredright_lines.pop(0)
                longest_line = line
                max_length = len(longest_line)
                filteredright_lines.append(longest_line)
        elif max(x1,x2)<0.5*image.shape[1] and max(y1,y2)>0.5*image.shape[0]:
            max_length=0
            if len(line) > 0:
                #filteredleft_lines.pop(0)
                longest_line = line
                max_length = len(longest_line)
                filteredleft_lines.append(longest_line)

# Draw detected lines and endpoints on the original image
if lines is not None:
    for line in filteredleft_lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        left_x1 += x1
        left_x2 += x2
        left_y1 += y1
        left_y2 += y2
    for line in filteredright_lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(image, (x1, y1), (x2, y2), (255, 0, 0), 2)
        right_x1 += x1
        right_x2 += x2
        right_y1 += y1
        right_y2 += y2
    
    #mid_x=(leftmax_x+rightmax_x)/2
    #mid_y=(leftmax_y+rightmax_y)/2
    #point=(int(mid_x),int(mid_y))
    #color=(0,0,255)
    #cv2.circle(image,point,20,color,-1)

# Display the result
cv2.imwrite('Sidewalk Endpoint Detection.jpg', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
