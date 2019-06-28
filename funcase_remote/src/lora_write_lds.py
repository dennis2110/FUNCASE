#!/usr/bin/env python
import rospy
#from std_msgs.msg import Int16MultiArray
from std_msgs.msg import String
#from funcase_client.msg import StringArray
import ifroglab_jetson
from time import time
import numpy as np

LoRa =  ifroglab_jetson.LoRa()
rospy.init_node('lora_lds_write')

# LoRa setup
ser=LoRa.FunLora_initByName("/dev/ttyUSB0")
LoRa.FunLora_0_GetChipID()
LoRa.FunLora_1_Init()
LoRa.FunLora_2_ReadSetup()
LoRa.FunLora_3_TX()
print("LoRa Ready!!!")

#coord_point = Int32MultiArray()
header = '*'

sep = ','

cnt = 0

def callback(data):
    tic = time()
    num = []
    num_list = []
    coord_point = data.data
    #print(len(coord_point))
    for i in coord_point:
        if i != ',':
            num.append(i)
        if i == ',':
            num_list.append("".join(num))
            num = []
    #tic2 = time()
    #print("time1 = ",tic2-tic1)
    
    #print(len(num_list))
    
    #for i in range(0,len(num_list)):
        #print(num_list[i])
    #print(len(coord_point))
    #for i in range(0,len(coor_point)):
        #coor_point.append(data)
    #print(coor_point)
    #print(num_list[63])
    LoRa.FunLora_5_write16bytesArrayString(str(header))
    #print(num_list)
    #for i in range(0,64):
    #tic3 = time()
    cnt = 0
    #comb = num_list[cnt] + sep
    '''
    while cnt <= 62:
        comb = num_list[cnt] + sep + num_list[cnt+1] + sep + num_list[cnt+2] + sep + num_list[cnt+3] + sep
        cnt = cnt+4
        print(comb)
        LoRa.FunLora_5_write16bytesArrayString(comb)
    '''    
    comb = num_list[cnt] + sep
    for cnt in range(0,len(num_list)-1): ##0~62
        if (len(comb) + len(num_list[cnt+1]) + 1) <= 16:
            cnt = cnt + 1
            comb = comb + num_list[cnt] + sep
            #print(comb)
            #print(cnt)
            if cnt == 63:
                print("comb=",comb)
                LoRa.FunLora_5_write16bytesArrayString(comb)
        #if (len(comb) + len(num_list[cnt+1]) + 1) > 16:
        else:
            print(comb)
            LoRa.FunLora_5_write16bytesArrayString(comb)
            comb = num_list[cnt+1] + sep 
            #if comb == num_list[63] + sep:
                #LoRa.FunLora_5_write16bytesArrayString(comb)
                #print("aaaa")
            #print("comb' = ",comb)
    #print("comb=",comb)
    #print(len(comb))
    #tic4 = time()
    #print("time2 = ",tic4-tic3)
    ### sent all data spend 0.8 sec
    num_list = []
    #tic = time.time()
    
    toc = time()
    print("processing time = ",toc-tic)
    
def reader():
    rospy.init_node('lora_lds_write')
    rospy.Subscriber("/pointsss", String, callback,queue_size = 1)
    rospy.spin()

while not rospy.is_shutdown():
    reader()
    rospy.spin()
