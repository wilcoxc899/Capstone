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

# status is a global that I'll use to print
# 0 - imu   1 - 5603    2 - both    3 - neither
# While I'm not a fan of globals, in this case, you can use 
# globals to pass information from one of the call functions to the other
status = 0

i2c =  busio.I2C(SCL,SDA)
pca = PCA9685(i2c)
pca.frequency = 100
channel_num = 14
servo7 = servo.Servo(pca.channels[channel_num])

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
   print(speed/65535)

#pca = Servo_Motor_Initialization()
#Motor_Start(pca)

# def call_key(data):
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
#    elif char =='x':
#        rospy.signal_shutdown('shut down')
#    global status
#    if (isinstance(int(str(data.data)), int)):
#        status = int(str(data.data))
#        if (status > 4):
#            status = 0

def call_imu(data):
    # rospy.loginfo(rospy.get_caller_id() + 'imu %s', data.data)
    [ax, ay, az] = str(data.data).split()
    ax = float(ax)  # destringify
    ay = float(ay)
    az = float(az)
    Motor_Speed(pca,0.15)
    #PID initiliazation variables
    setpoint=0
    integ=0
    prev_der=0
    # PID Parameters
    p=1.0
    i=0.1
    d=0.01
  
    global status
    if (status == 0 or status == 2):
        #print('imu: ', data.data)
        #print(f'ax: {ax:.2f}\tmy: {ay:.2f}\taz: {az:.2f}')
        with open('/home/marytazwell/catkin_ws/src/keyboard_test/data_IMU.txt', 'w') as file:
            timestamp = time.time()
            file.write(f'{timestamp}, {ax}, {ay}, {az}\n')
        #control loop
        error=setpoint-az
        integ +=error
        der= error-prev_der
        output = (p*error)+(i*integ)+(d*der)
        #steering
        servo7.angle = 90-output
        
        
        

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
        with open('/home/marytazwell/catkin_ws/src/keyboard_test/data_MMC.txt', 'w') as file:
            timestamp = time.time()
            file.write(f'{timestamp}, {mx}, {my}, {mz}\n')

def listener():
    while not rospy.is_shutdown():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
        rospy.init_node('listener', anonymous=True)
 #       rospy.Subscriber('keyboard', String, call_key)
        rospy.Subscriber('mmc5603',   String, call_mmc5603)
        rospy.Subscriber('imu',   String, call_imu)
        rospy.spin()

if __name__ == '__main__':
    listener()
