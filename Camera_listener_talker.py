#!/usr/bin/env python

#CAMERA TALKER

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import picamera
import picamera.array
import time

class CameraPublisher:
    def __init__(self):
        self.node_name = "picamera_publisher"
        self.topic_name = "/camera/image_raw"
        
        rospy.init_node(self.node_name, anonymous=True)
        self.image_pub = rospy.Publisher(self.topic_name, Image, queue_size=10)
        
        self.bridge = CvBridge()
        self.camera = picamera.PiCamera()
        self.stream = picamera.array.PiRGBArray(self.camera)
        
        # Set camera parameters
        self.camera.resolution = (640, 480)
        self.camera.framerate = 30
        
        # Allow the camera to warm up
        time.sleep(2)

    def publish_frame(self):
        while not rospy.is_shutdown():
            self.camera.capture(self.stream, 'bgr', use_video_port=True)
            image_msg = self.bridge.cv2_to_imgmsg(self.stream.array, 'bgr8')
            self.image_pub.publish(image_msg)
            
            # Clear the stream to prepare for the next frame
            self.stream.seek(0)
            self.stream.truncate()

    def run(self):
        try:
            self.publish_frame()
        except rospy.ROSInterruptException:
            pass
        finally:
            self.camera.close()

if __name__ == '__main__':
    camera_publisher = CameraPublisher()
    camera_publisher.run()

# CAMERA LISTENER

#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageListener:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Process the cv_image here
        # For example, display the image
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

def main():
    rospy.init_node('image_listener', anonymous=True)
    image_listener = ImageListener()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

# CROPPING THE IMAGE

import cv2

# Load the image from file
image = cv2.imread('path_to_your_image.jpg')

# Define the crop area [y1:y2, x1:x2]
crop_area = image[100:300, 100:300]

# Save the cropped image
cv2.imwrite('cropped_image.jpg', crop_area)


# APPLYING THE EDGE DETECTION (USE CADENS CODE THIS IS A PLACE HOLDER)
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            # Convert the ROS image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # Convert the image to grayscale
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Apply Canny edge detection
            edges = cv2.Canny(gray_image, 100, 200)
            
            # Display the result
            cv2.imshow("Edge Detected Image", edges)
            cv2.waitKey(3)
            
        except CvBridgeError as e:
            rospy.logerr(e)

def main():
    rospy.init_node('image_subscriber', anonymous=True)
    image_subscriber = ImageSubscriber()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
