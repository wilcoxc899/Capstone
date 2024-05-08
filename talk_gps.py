#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import time
import board
import busio
import adafruit_gps

import serial

def talker():
    uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=10)
    gps = adafruit_gps.GPS(uart, debug=False)                            # Use UART/pyserial
    gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")   # Turn on CGA and RMC
    gps.send_command(b"PMTK220,5000")                                    # Set update rate to once every 5 seconds (5hz)
    last_print  = time.monotonic()

    pub = rospy.Publisher('gps', String, queue_size = 10)
    rospy.init_node('talk_gps', anonymous = True)
    rate = rospy.Rate(0.2)                                           # Every 5 Seconds
    while not rospy.is_shutdown():
        lat = gps.latitude
        long = gps.longitude
#        outstring1 = "{0:0.6f}".format(lat)
#        outstring2 = "{0:0.6f}".format(long)
#        rospy.loginfo(outstring1)
#        rospy.loginfo(outstring2)
#        pub.publish(outstring1)
#        pub.publish(outstring2)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
