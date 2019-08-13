#!/usr/bin/env python
import rospy
import os
#import RPi.GPIO as gpio

import sys
sys.path.append('/opt/nvidia/jetson-gpio/lib/python/')
sys.path.append('/opt/nvidia/jetson-gpio/lib/python/Jetson/GPIO')
import Jetson.GPIO as gpio

rospy.init_node("starto")
button = 4
stat = False

gpio.setmode(gpio.BCM)
gpio.setup(button, gpio.IN)
#pub = rospy.Publisher('button_status', Bool, queue_size = 1)
#rate = rospy.Rate(10)

while stat == False:
    status = gpio.input(button)
    print "button status :", status
    if status == 1:
        stat = True

os.system("rosrun funcase_test test_main_loop2")

