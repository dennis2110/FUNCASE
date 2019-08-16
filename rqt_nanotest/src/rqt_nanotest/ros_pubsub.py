#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Version PYQT5

import os
import rospy
import rospkg
import math

from PyQt5 import QtCore
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget,QApplication
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *
from std_msgs.msg import Int16,Int16MultiArray,Int16,String,UInt8MultiArray,Float64MultiArray,Bool
from sensor_msgs.msg import LaserScan,Imu
import tf
import numpy as np
import cv2 as cv

class ROSdata(QWidget):

    updata_pic = QtCore.pyqtSignal()

    def __init__(self,context):
        super(ROSdata, self).__init__()
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_nanotest'), 'resource', 'MyPlugin.ui')
        loadUi(ui_file, self)
        self.cnt=0
        self.zerocnt=0
        self.backwarn=0
        self.warn=False
        self.point=0
        self.timer = QTimer(self)
        self.timer2 = QTimer(self)
        self.timer3 = QTimer(self)
        self.timer.timeout.connect(self.timeslot)
        self.timer2.timeout.connect(self.timeslot2)
        self.timer3.timeout.connect(self.timeslot3)

	self.gif = QMovie('/home/nano606a/funcase_ws/src/rqt_nanotest/smilegif2.gif')
        self.gif.setSpeed(300)
        self.gif.setScaledSize(QSize(431,300))
        self.label.setMovie(self.gif)
        self.gif.start()

        self.subscriber_group()
        #self.publisher_group()

        #self.pushButton.clicked.connect(self.startCount)

        #self.pushButton_2.clicked.connect(self.stopCount)
        self.updata_pic.connect(self.slotpic)

    #def publisher_group(self):


    def subscriber_group(self):
        rospy.Subscriber("/BZ5",Bool, self.callback)
        #self.timer.start(1000)



    def timeslot(self):
        self.label.setPixmap(QPixmap("/home/denny3/new_work/src/rqt_nano/LOGOtest4.png"))
        self.cnt=self.cnt+1
        self.lcdNumber.display(self.cnt)
        self.timer.stop()

    def timeslot2(self):
        self.label.setPixmap(QPixmap("/home/denny3/new_work/src/rqt_nanotest/scared.jpg"))
        self.timer2.stop()
    def timeslot3(self):
        if self.point ==1:
            self.label.setPixmap(QPixmap("/home/denny3/new_work/src/rqt_nano/LOGOtest5.png"))
            self.timer3.stop()
        elif self.point ==0:
            self.label.setPixmap(QPixmap("/home/denny3/new_work/src/rqt_nano/LOGOtest1.png"))
            self.timer3.stop()

        print("point:" + str(self.point))

    def callback(self,alarm):
        #self.gif = QMovie('/home/denny3/new_work/src/rqt_nano/LOGO_gif_final.gif')
        self.warn = alarm.data
        print("alarm:" + str(alarm.data))
        if alarm.data ==True:

            pass

            #self.label.setPixmap(QPixmap("/home/denny3/new_work/src/rqt_nano/LOGOtest5.png"))

         #self.label.setPixmap(QPixmap("/home/denny3/new_work/src/rqt_nano/LOGOtest4.png"))
        else :
            #self.label.setPixmap(QPixmap("/home/denny3/new_work/src/rqt_nano/LOGOtest1.png"))
            pass
        self.updata_pic.emit()
    def slotpic(self):
        #self.gif = QMovie('/home/nano606a/funcase_ws/src/rqt_nanotest/smilegif2.gif')
        

        if self.warn == False and self.backwarn == 0:
	    pass
            #self.gif.setSpeed(90000000)
            #self.gif.setScaledSize(QSize(431,300))
            #self.label.setMovie(self.gif)
            #self.gif.start()
            #self.backwarn = self.warn

            #self.point=1
            #self.timer3.start(100)

            #self.timer3.start(1)
        else:
            #pass
            #self.point=0
            #self.gif.stop()
            self.backwarn = 1
            print("self.zerocnt=" + str(self.zerocnt))
            self.label.setPixmap(QPixmap("/home/nano606a/funcase_ws/src/rqt_nanotest/scared2.png"))
            if self.backwarn ==1:
                self.zerocnt = self.zerocnt+1
            if self.zerocnt ==150:
		self.movie()
                self.zerocnt=0
                self.backwarn = 0


            #self.timer2.start(2000)
            #self.label.setPixmap(QPixmap("/home/denny3/new_work/src/rqt_nano/LOGOtest1.png"))

    def movie(self):
	self.gif = QMovie('/home/nano606a/funcase_ws/src/rqt_nanotest/smilegif2.gif')
        self.gif.setSpeed(300)
        self.gif.setScaledSize(QSize(431,300))
        self.label.setMovie(self.gif)
        self.gif.start()





