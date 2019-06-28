#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
#from std_msgs.msg import UInt8MultiArray
from funcase_msgs.msg import JoyCmd

def callback(data):
    axe = data.axes
    button = data.buttons
    #joy_cmddd = UInt8MultiArray()
    joy = axe + button
    seg_axes = []
    enc_axes = []
    for i in range(0,8):
        tmp = joy[i]
        if tmp > 0.2:
            tmp = 1
        elif tmp < 0.2 and tmp > -0.2:
            tmp = 0
        elif tmp < -0.2:
            tmp = -1
        seg_axes.append(tmp)
    
    if seg_axes[2] <= 1 and seg_axes[2] > -1:
        seg_axes[2] = 0
    elif seg_axes[2] == -1:
        seg_axes[2] = 1
    if seg_axes[5] <= 1 and seg_axes[5] > -1:
        seg_axes[5] = 0
    elif seg_axes[5] == -1:
        seg_axes[5] = 1

    for i in range(0,8):
        if seg_axes[i] == 0:
            tmp2 = 0
        elif seg_axes[i] == 1:
            tmp2 = 1
        elif seg_axes[i] == -1:
            tmp2 = 2
        enc_axes.append(tmp2)

    sum_axe_front = enc_axes[0]*64 + enc_axes[1]*16 + enc_axes[2]*4 + enc_axes[3]*1
    sum_axe_below = enc_axes[4]*64 + enc_axes[5]*16 + enc_axes[6]*4 + enc_axes[7]*1
    sum_button = joy[8]*64 + joy[9]*32 + joy[10]*16 + joy[11]*8 +joy[14]*4 + joy[15]*2 + joy[16]*1
    
    joy_cmddd = [sum_button, sum_axe_front, sum_axe_below]
    pub = rospy.Publisher('joy_commands', JoyCmd, queue_size = 10)
    pub.publish(joy_cmddd)

def listener():
    rospy.init_node('pub_and_enc_joystick')
    rospy.Subscriber("/joy", Joy, callback, queue_size=1)
    rospy.spin()

while not rospy.is_shutdown():
    listener()
    rospy.spin()

#if __name__ == '__main__':
    #try:
        #listener()
    #except rospy.ROSInterruptException:
        #pass
