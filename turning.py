import math
from board import SCL,SDA
import busio
from adafruit_pca9685 import PCA9685
import time
import adafruit_motor.servo
import curses
from board import SCL, SDA
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c =  busio.I2C(SCL,SDA)

pca = PCA9685(i2c)

pca.frequency = 100

channel_num = 14

servo7 = servo.Servo(pca.channels[channel_num])
def Servo_Motor_Initialization():
   i2c_bus = busio.I2C(SCL,SDA)
   pca = PCA9685(i2c_bus)
   pca.frequency = 100
   return pca

def Motor_Start(pca):
   x = input("Press and hold EZ button. Once the LED turns red, immediately relase the button. After the LED blink red once, press 'ENTER'on keyboard.")
   Motor_Speed(pca, 1)
   time.sleep(2)
   y = input("If the LED just blinked TWICE, then press the 'ENTER'on keyboard.")
   Motor_Speed(pca, -1)
   time.sleep(2)
   z = input("Now the LED should be in solid green, indicating the initialization is complete. Press 'ENTER' on keyboard to proceed")
   

def Motor_Speed(pca,percent):
   #converts a -1 to 1 value to 16-bit duty cycle
   speed = ((percent) * 3277) + 65535 * 0.15
   pca.channels[15].duty_cycle = math.floor(speed)
   print(speed/65535)

#initialization
pca = Servo_Motor_Initialization()
#Motor_Start(pca)
# import curses

# Get the curses window, turn off echoing of keyboard to screen, turn on
# instant (no waiting) key response, and use special values for cursor keys
screen = curses.initscr()
curses.noecho() 
curses.cbreak()
screen.keypad(True)

try:
        while True:   
            char = screen.getch()
            if char == ord('q'):
                servo7.angle=90
            elif char == curses.KEY_UP:
                print ("up")
                Motor_Speed(pca,0.16)
            elif char == curses.KEY_DOWN:
                print ("down")
                Motor_Speed(pca, 0)
            elif char == curses.KEY_RIGHT:
                print ("right")
                servo7.angle = 75
            elif char == curses.KEY_LEFT:
                print ("left")
                servo7.angle=105
            elif char == 10:
                print ("stop")    
             
finally:
    #Close down curses properly, inc turn echo back on!
    curses.nocbreak(); screen.keypad(0); curses.echo()
    curses.endwin()
