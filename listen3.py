#!/usr/bin/env pyth## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
import time
import math
from board import SCL,SDA
import busio
from adafruit_pca9685 import PCA9685
import adafruit_motor.servo
import curses
from board import SCL, SDA
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import adafruit_gps
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
# status is a global that I'll use to print
# 0 - imu   1 - 5603    2 - both    3 - GPS
# While I'm not a fan of globals, in this case, you can use 
# globals to pass information from one of the call functions to the other
status = 0

i2c =  busio.I2C(SCL,SDA)
pca = PCA9685(i2c)
pca.frequency = 100
channel_num = 14
servo7 = servo.Servo(pca.channels[channel_num])

#IMU motor control variables
straight_time=time.time() + 7
turn_time=straight_time + 2.5
end_time = turn_time + 7
array = []
dumb = 86
integ=0
prev_error=0

#def Servo_Motor_Initialization():
#   i2c_bus = busio.I2C(SCL,SDA)
#   pca = PCA9685(i2c_bus)
#   pca.frequency = 100
#   return pca

#def Motor_Start(pca):
#   x = input("Press and hold EZ button. Once the LED turns red, immediately relase the button. After the LED blink red once, press 'ENTER'on keyboard.")
#   Motor_Speed(pca, 1)
#   time.sleep(2)
#   y = input("If the LED just blinked TWICE, then press the 'ENTER'on keyboard.")
#   Motor_Speed(pca, -1)
#   time.sleep(2)
#   z = input("Now the LED should be in solid green, indicating the initialization is complete. Press 'ENTER' on keyboard to proceed")

def Motor_Speed(pca,percent):
   #converts a -1 to 1 value to 16-bit duty cycle
   speed = ((percent) * 3277) + 65535 * 0.15
   pca.channels[15].duty_cycle = math.floor(speed)
#   print(speed/65535)

#pca = Servo_Motor_Initialization()
#Motor_Start(pca)

#def call_key(data):
#    rospy.loginfo(rospy.get_caller_id() + 'Keyboard %s', data.data)
    #print("keyboard: " + str(data.data))
#    char = str(data.data)
#    if char == 'q':
#        servo7.angle=90
#    elif char == 'w':
#        print ("go")
#        Motor_Speed(pca,0.15)
#    elif char == 's':
#        print ("stop")
#        Motor_Speed(pca, 0)
#    elif char == 'd':
#        print ("right")
#        servo7.angle = 75
#    elif char == 'a':
#        print ("left")
#        servo7.angle=105
#    global status
#    if (isinstance(int(str(data.data)), int)):
#        status = int(str(data.data))
#        if (status > 4):
#            status = 0

def call_imu(data):
    # rospy.loginfo(rospy.get_caller_id() + 'imu %s', data.data)
    [gyro_x, gyro_y, gyro_z] = str(data.data).split()
    ax = float(gyro_x)  # destringify
    ay = float(gyro_y)
    az = float(gyro_z)
    #print(f'{az}')
    setpoint=0
    # PID Parameters
    p=1.0
    i=0.15
    d=0.02
    dummy = 86
    global status
    if (status == 0 or status == 2):
        with open('/home/marytazwell/catkin_ws/src/keyboard_test/scripts/data_IMU.txt', 'a') as file:
            timestamp = time.time()
            file.write(f'{timestamp}, {ax}, {ay}, {az}\n')
        #control loop
        if time.time()<straight_time:
            Motor_Speed(pca,0.35)
            error=0-az
            #setpoint = az
            print(f'z={az}')
#            print(f'{error}')
            array.append(error)
            if len(array)>10:
                array.pop(0)
            integ=0
            integ +=sum(array)
            print(f'integ={integ}')
            #der   = error-prev_error
            #prev_error=error
            output = (p*error)+(i*integ)
            #print(f'output:{output}')
#            print(f'i_value={integ}')
            #steering
            #if output > 5:
                #output = 5
            #elif output < -5:
                #output = -5
            servo7.angle = 86 + output
            dummy = 86 + output
            print(f'angle:{dummy}')
            #print(f'straight')
        elif straight_time <= time.time()<turn_time:
            for angle in range(86, 100):
                Motor_Speed(pca, 0.2)
                servo7.angle = angle
                time.sleep(0.01)
                print(f'turn')
        elif turn_time <= time.time() < end_time:
            Motor_Speed(pca, 0.3)
            error=0-az
            #setpoint = az
  #          print(f'z={az}')
#            print(f'{error}')
            array.append(error)
            if len(array)>10:
                array.pop(0)
            integ=0
            integ +=sum(array)
 #           print(f'integ={integ}')
            #der   = error-prev_error
            #prev_error=error
            output = (p*error)+(i*integ)
            #print(f'output:{output}')
#            print(f'i_value={integ}')
            #steering
            #if output > 5:
                #output = 5
            #elif output < -5:
                #output = -5
            servo7.angle = 85.5 + output
            dummy = 86 + output
#            print(f'angle:{dummy}')
            #print(f'straight')

        else: 
            servo7.angle = 86
            Motor_Speed(pca,0)


def call_mmc5603(data):
    # rospy.loginfo(rospy.get_caller_id() + 'mmc5603 %s', data.data)
    [mx, my, mz, temp] = str(data.data).split()
    mx = float(mx)
    my = float(my)
    mz = float(mz)
    temp = float(temp)
    global status
    if (status == 1 or status == 2):
        #print('mms5603: ', data.data)
        print(f'mx: {mx:.2f}\tmy: {my:.2f}\tmz: {mz:.2f}\ttemp: {temp:.1f}')
        with open('/home/marytazwell/catkin_ws/src/keyboard_test/scripts/data_MMC.txt', 'a') as file:
            timestamp = time.time()
            file.write(f'{timestamp}, {mx}, {my}, {mz}\n')

def call_gps(data):
    lat = str(data.data)
    long = str(data.data)
    lat = float(lat)
    long = float(long)
    global status
    if (status == 3):
        print(f'longitude: {long:0.6f}\t latitude: {lat:0.6f}')
        with open('/home/marytazwell/catkin_ws/src/keyboard_test/scripts/data_GPS.txt', 'a') as file:
            file.write(f'{lat}, {long}\n')

def callback(frame):
    bridge = CvBridge()
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(frame, "bgr8")

        # Process the image as needed
        picture = os.path.join('/home/marytazwell/catkin_ws/src/keyboard_test/scripts', 'ROS_PIC1.jpg')
        cv2.imwrite(picture, cv_image)
    except Exception as e:
        rospy.logerr(e)

def call_lidar(data):
    rospy.loginfo("Received Lidar scan data:")
    rospy.loginfo("Number of ranges: %d", len(data.ranges))
    global status
    if status == 0:
        angle = data.angle_min
        for distance in data.ranges:
            rospy.loginfo("Angle: %f degrees, Distance: %f meters", angle * 180 / 3.14159, distance)
            angle += data.angle_increment

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

#    rospy.Subscriber('keyboard', String, call_key)
    rospy.Subscriber('mmc5603',   String, call_mmc5603)
    rospy.Subscriber('imu',       String, call_imu)
    rospy.Subscriber('gps',       String, call_gps)
    rospy.Subscriber('camera',    Image,  callback)
    rospy.Subscriber('/lidar/scan', LaserScan, call_lidar)
    rospy.spin()

if __name__ == '__main__':
    listener()
