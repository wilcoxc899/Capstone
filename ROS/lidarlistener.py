#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

def lidar_callback(msg):
    # Process the received LaserScan message
    # For example, you can print out the ranges
    print("Received LaserScan message:")
    print("Header:", msg.header)
    print("Number of ranges:", len(msg.ranges))
    print("Minimum range:", min(msg.ranges))
    print("Maximum range:", max(msg.ranges))
    print("")

def listener():
    rospy.init_node('lidar_listener', anonymous=True)
    rospy.Subscriber('lidar_scan', LaserScan, lidar_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
