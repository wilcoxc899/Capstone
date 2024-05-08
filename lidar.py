
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
lidar.clear_input()
reverse=True

#servo7.angle = 86.5

def Motor_Speed(pca,percent):
    speed = ((percent)*3277) + 65535*0.15
    pca.channels[15].duty_cycle = math.floor(speed)


def print_lidar_data(data):
    Motor_Speed(pca,0)
    global reverse
    start_time = time.time()
    endtime=start_time+1
    with open('lidarreadings.txt', 'w') as f:
#        n=0
#        array=[1,2,3,4,5,6,7]
        for angle, distance in enumerate(data):
#            n+=1
		#if angle==270:
                #print(f'angle={distance}') 
            if angle==270 and 1<distance<=600:
                servo7.angle = 93
                #print(f'too close: {distance}')
                print(f'too close: printing reverse {reverse}')
 #               array.append(distance)
                reverse=True
            elif angle==270 and 1200<distance<2000:
                servo7.angle = 80
                #print(f'too far:{distance}')
                print(f'too far: printing reverse {reverse}')
#                array.append(distance)
                reverse=True
            elif angle==270 and 600<distance<=1200:
                servo7.angle = 86.5
                #print(f'just right:{distance}')
                print(f'just right: printing reverse{reverse}')
#                array.append(distance)
#            elif n>10 and n<len(array):
                reverse=True
            elif angle == 270 and distance > 2500:
#                if array[n]==array[n-5]:
                if reverse == True:
                    endtime=time.time()+3
#                    Motor_Speed(pca, -0.15)
                    print('turn')
                    print(reverse)
                servo7.angle=40
#                Motor_Speed(pca,-0.15)
      #      elif angle==190 and distance < 100:
      #          Motor_Speed(pca,0)
#                servo7.angle = 87
                #print('This is 90')
                #radians = angle * pi / 180.0
                #x = distance * cos(radians)
                #y = distance * sin(radians)
#                print(f'Angle: {angle}, Distance: {distance}, (x, y): ({x:.2f}, {y:.2f})')
               # f.write(f'Angle: {angle}, Distance: {distance}\n')

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
