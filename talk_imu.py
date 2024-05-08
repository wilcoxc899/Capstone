#!/usr/bin/env python3
## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import time
import board
import adafruit_mpu6050
from std_msgs.msg import String

def talker():
    i2c = board.I2C()  # uses board.SCL and board.SDA
    mpu = adafruit_mpu6050.MPU6050(i2c)
    pub = rospy.Publisher('imu', String, queue_size=10)
    rospy.init_node('talk_imu', anonymous=True)
    rate = rospy.Rate(1) # every 2 seconds
    while not rospy.is_shutdown():
        gyro_x,  gyro_y,  gyro_z  = mpu.gyro
        accel_x, accel_y, accel_z = mpu.acceleration
        #outstring = "X:{0:10.2f}, Y:{1:10.2f}, Z:{2:10.2f}".format( accel_x, accel_y, accel_z)
        # just raw numbers
        outstring = "{0:10.2f} {1:10.2f} {2:10.2f}".format( gyro_x, gyro_y, gyro_z)
        rospy.loginfo(outstring)
        pub.publish(outstring)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
