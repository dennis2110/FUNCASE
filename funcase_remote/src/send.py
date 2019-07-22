#!/usr/bin/env python 
import rospy
import socket
from std_msgs.msg import UInt8

UDP_IP = "10.42.0.86"
UDP_PORT = 5230
MESSAGE1 = "omxplayer -o local warning1.mp3"
MESSAGE2 = "omxplayer -o local warning2.mp3"
MESSAGE3 = "omxplayer -o local warning3.mp3"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

#sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
def callback(data):
    cmd = data.data
    print cmd
    if cmd == 1:
        print "Play warning 1"
        sock.sendto(MESSAGE1, (UDP_IP, UDP_PORT))
    if cmd == 2:
        print "Play warning 2"
        sock.sendto(MESSAGE2, (UDP_IP, UDP_PORT))
    if cmd == 3:
        print "Play warning 3"
        sock.sendto(MESSAGE3, (UDP_IP, UDP_PORT))

def listener():
    print "listener"
    rospy.init_node('send')
    rospy.Subscriber("/BZ5", UInt8, callback, queue_size=1)

while not rospy.is_shutdown():
    listener()
    rospy.spin()
