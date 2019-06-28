#!/usr/bin/env python
import sys
from time import sleep
sys.path.append('/opt/nvidia/jetson-gpio/lib/python/')
sys.path.append('/opt/nvidia/jetson-gpio/lib/python/GPIO')
import Jetson.GPIO as gpio
import rospy
from funcase_msgs.msg import JoyCmd

gpio.setmode(gpio.BCM)
switch = 18
gpio.setup(switch, gpio.OUT, initial = gpio.LOW)
#gpio.setwarnings(False)
swit = False

def callback(data):
    global swit
    joy_cmd = data
    button = joy_cmd.joy[0].encode("hex")
    sum_ = int(button[0]) * 16 + int(button[1])*1
    #print(c)
    
    if sum_ == 4:
        swit = True
        gpio.output(switch,gpio.HIGH)
    if sum_ == 2:
        swit = False
        gpio.output(switch,gpio.LOW)
    print(swit)
     
def listener():
    rospy.init_node('switch_camera')
    rospy.Subscriber("/joy_commands", JoyCmd, callback, queue_size=1)

while not rospy.is_shutdown():
    listener()
    rospy.spin()

