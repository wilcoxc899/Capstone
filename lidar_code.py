import os
import time
from math import cos, sin, pi, floor
from adafruit_rplidar import RPLidar
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import math

# servo stuff
i2c = busio.I2C(SCL,SDA)
pca = PCA9685(i2c)
pca.frequency = 100
channel_num = 14
servo7 = servo.Servo(pca.channels[channel_num])

# Setup the RPLidar
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME, timeout=3)

max_distance = 0
stack = []
def Motor_Speed(pca,percent):
    speed = ((percent)*3277) + 65535*0.15
    pca.channels[15].duty_cycle = math.floor(speed)


def print_lidar_data(data):
    with open('lidarreadings.txt', 'w') as f:
        for angle, distance in enumerate(data):
            Motor_Speed(pca,0)
            if angle==270 and 1<distance<=600:
                servo7.angle = 95
                print(f'too close: {distance}')
                print('too close')
            elif angle==270 and 1200<distance<2000:
                servo7.angle = 75
                print(f'too far:{distance}')
                print('too far')
            elif angle==270 and 600<=distance<=1200:
                servo7.angle = 86.5
                print(f'just right:{distance}')
                print('just right')
            elif (angle== 270 and distance > 3000): # or (angle == 280 and distance <1000):
                for x in range (60, 87):
                    servo7.angle = x
                    time.sleep(0.1)
                    print(f'turn:{distance}')
                    print('turn')
      #      elif angle==190 and distance < 100:
      #          Motor_Speed(pca,0)
#                servo7.angle = 87
                #print('This is 90')
                #radians = angle * pi / 180.0
                #x = distance * cos(radians)
                #y = distance * sin(radians)
#                print(f'Angle: {angle}, Distance: {distance}, (x, y): ({x:.2f}, {y:.2f})')
                f.write(f'Angle: {angle}, Distance: {distance}\n')

scan_data = [0]*360

try:
    print(lidar.info)
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            scan_data[min([359, floor(angle)])] = distance
        print_lidar_data(scan_data)

except KeyboardInterrupt:
    print('Stopping.')
lidar.stop()
lidar.disconnect()
