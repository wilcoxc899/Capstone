#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

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
