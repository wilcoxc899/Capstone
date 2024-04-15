#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from adafruit_rplidar import RPLidar

PORT_NAME = '/dev/ttyUSB0'

def run():
    rospy.init_node('lidar_talker', anonymous=True)
    lidar_pub = rospy.Publisher('lidar_scan', LaserScan, queue_size=10)
    
    lidar = RPLidar(None, PORT_NAME, timeout=3)
    
    try:
        print('Publishing measurements...')
        for scan in lidar.iter_scans():
            ranges = scan['distances']
            intensities = scan['intensities']
            
            # Create LaserScan message
            lidar_msg = LaserScan()
            lidar_msg.header.stamp = rospy.Time.now()
            lidar_msg.header.frame_id = 'lidar_frame'
            lidar_msg.angle_min = min(scan['angle'])
            lidar_msg.angle_max = max(scan['angle'])
            lidar_msg.angle_increment = lidar_msg.angle_max / len(ranges)
            lidar_msg.range_min = min(ranges)
            lidar_msg.range_max = max(ranges)
            lidar_msg.ranges = ranges
            lidar_msg.intensities = intensities
            
            lidar_pub.publish(lidar_msg)
            
    except rospy.ROSInterruptException:
        print('ROS node terminated.')
        
    finally:
        lidar.stop()
        lidar.disconnect()

if __name__ == '__main__':
    try:
        run()
    except KeyboardInterrupt:
        pass
