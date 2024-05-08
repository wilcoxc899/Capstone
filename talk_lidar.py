#!/usr/bin/env python3

#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from adafruit_rplidar import RPLidar

class LidarPublisher:
    def __init__(self):
        rospy.init_node('lidar_publisher', anonymous=True)
        try:
            self.lidar = RPLidar(None, '/dev/ttyUSB0', timeout=3)
        except Exception as e:
            rospy.logerr("Failed to initialize RPLidar: %s", str(e))
            rospy.signal_shutdown("Failed to initialize RPLidar")
            return

        self.scan_pub = rospy.Publisher('/lidar/scan', LaserScan, queue_size=10)
        rospy.on_shutdown(self.shutdown)

    def publish_scan(self):
        scan_msg = LaserScan()
        scan_msg.header.stamp = rospy.Time.now()
        scan_msg.header.frame_id = 'lidar_frame'

        for scan in self.lidar.iter_scans():
            ranges = []
            intensities = []
            for (_, angle, distance) in scan:
                # Check if the angle is within the specified range (100 to 250 degrees)
                if angle >= 100 and angle <= 250:
                    ranges.append(distance)
                    intensities.append(0)  # No intensity data available from RPLidar
            
            # Calculate angle increment and minimum/maximum angles based on the processed range
            num_ranges = len(ranges)
            scan_msg.angle_min = 100 * 3.14159 / 180  # Convert to radians
            scan_msg.angle_max = 250 * 3.14159 / 180  # Convert to radians
            scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / num_ranges
            scan_msg.time_increment = 0  # Time between measurements, not provided by RPLidar
            scan_msg.scan_time = 1 / 400  # Approximate scan time of RPLidar A1
            scan_msg.range_min = 0.15  # Minimum range of RPLidar A1
            scan_msg.range_max = 12.0  # Maximum range of RPLidar A1
            scan_msg.ranges = ranges
            scan_msg.intensities = intensities
            self.scan_pub.publish(scan_msg)

    def shutdown(self):
        rospy.loginfo("Shutting down LidarPublisher node.")
        self.lidar.stop()
        self.lidar.disconnect()

def main():
    lp = LidarPublisher()
    try:
        lp.publish_scan()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
