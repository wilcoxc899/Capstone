#!/usr/bin/env python3
## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import time
import board
import adafruit_mmc56x3
from std_msgs.msg import String

def talker():
    i2c = board.I2C() # uses board.SCL and board.SDA
    # i2c = board.STEMMA_I2C() # For using the built-in STEMMA QT connector on a microcontroller
    sensor = adafruit_mmc56x3.MMC5603(i2c)

    pub = rospy.Publisher('mmc5603', String, queue_size=10)
    rospy.init_node('talk_mmc5603', anonymous=True)
    rate = rospy.Rate(0.5) # every 2 seconds
    while not rospy.is_shutdown():
        mag_x, mag_y, mag_z = sensor.magnetic
        temp = sensor.temperature
        # outstring = "X:{0:10.2f}, Y:{1:10.2f}, Z:{2:10.2f} uT\tTemp:{3:6.1f}*C".format( mag_x, mag_y, mag_z, temp)
        # just raw numbers
        outstring = "{0:10.2f} {1:10.2f} {2:10.2f} {3:6.1f}".format( mag_x, mag_y, mag_z, temp)
        rospy.loginfo(outstring)
        pub.publish(outstring)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
