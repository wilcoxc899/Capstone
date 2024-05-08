#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
#import picamera
#import picamera.array
import time
import cv2

def talker():
    pub = rospy.Publisher('camera', Image, queue_size = 10 )
    rospy.init_node('talk_cam', anonymous = True)
    rate = rospy.Rate(0.2)                               # take picture every 2 seconds
    bridge = CvBridge()

    cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)   # opens the camera

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)              # set dimensions
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 700)

    ret, frame = cap.read()                               # take frame 
    cap.release()

    while not rospy.is_shutdown():
        if ret:
            cv2.imwrite("ROS_PIC.jpg", frame)             # Save the frame to a file
            image_message = bridge.cv2_to_imgmsg(frame, "bgr8")       # Convert the frame to ROS Image message
            pub.publish(image_message)                   # Publish the image message
        else:
            rospy.logerr("Failed to capture frame.")
            break

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
