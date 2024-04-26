import os
import time
from math import cos, sin, pi, floor, atan
from adafruit_rplidar import RPLidar
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

# servo stuff
i2c = busio.I2C(SCL,SDA)
pca = PCA9685(i2c)
pca.frequency = 100
channel_num = 14
servo7 = servo.Servo(pca.channels[channel_num])

# Setup the RPLidar
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME, timeout=3)

# parameters
theta = 55
L = 100
kp = 1
ki = 0.5
kd = 0.1
pre_error = 0
integral = 0

def Motor_Speed(pca,percent):
    speed = ((percent)*3277) + 65535*0.15
    pca.channels[15].duty_cycle = floor(speed)


def all_sensors(data):
    for angle, distance in enumerate(data):
        b = scan_data[270] # distance at zero angle relative to the car's x-axis
        a = scan_data[270 + theta] # distance at theta
        alpha = atan((a*cos(theta))-b, a*sin(theta)) # offset of the car: should be zero if car is straight
        D = b*cos(theta) # current distance between the car and the right wall
        D_new = D + (L*sin(alpha)) # future projectd distance

        # Control algorithm
        error = 1000 - D_new
        integral += error
        derivative = error - pre_error
        pre_error = error
        control = (kp*error) + (ki*integral) + k(kd*derivative)

        servo7.angle = 86.5 + control
        car_angle = servo7.angle

        # changing the speed of the car to slow down as the turn angle increases  
        if (76.5 <= car_angle <= 86.5):
            Motor_Speed(pca, 0.2)
        elif (66.5 < car_angle < 76.5):
            Motor_Speed(pca, 0.18)
        elif (car_angle <= 66.5):
            Motor_Speed(pca,0.15)
        else:
            Motor_Speed(pca,0)

scan_data = [0]*360

try:
    print(lidar.info)
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            scan_data[min([359, floor(angle)])] = distance
        all_sensors(scan_data)

except KeyboardInterrupt:
    print('Stopping.')
lidar.stop()
lidar.disconnect()