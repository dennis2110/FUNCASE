#!/usr/bin/env python
# www.ifroglab.com
# -*- coding: utf8 -*-
# coding=UTF-8
# * iFrogLab IL-LORA1272  www.ifroglab.com
# *
# * 功能,             USB to TTL , IFROGLAB LORA
# * 電源VDD,          3.3V       ,Pin 3
# * 接地GND,          GND        ,Pin 1
# * 接收反應Host_IRQ,  null       , Pin 2
# * UART,             RX         ,UART_RX  Pin 7
# * UART,             TX         ,UART_TX  Pin 8
import ifroglab_funcase
import time
import rospy
from std_msgs.msg  import Int16MultiArray

LoRa = ifroglab_funcase.LoRa()
rospy.init_node('lora_read_lds')


ser=LoRa.FunLora_initByName("/dev/ttyACM0")
LoRa.FunLora_0_GetChipID()
LoRa.FunLora_1_Init()
LoRa.FunLora_2_ReadSetup();
LoRa.FunLora_3_RX();
LoRa.debug=False
print("Lora ready!!")

lds = []
j = 0
tic = 0
lds_pub = Int16MultiArray()

while True:
  data=LoRa.FunLora_6_readPureData()

  if len(data) != 0 and data != data2:
    str_data = []
    for i in data:
      if i == 42:# and len(lds) == 64:
        
        #lds_pub.data = lds
        #pub = rospy.Publisher('linesss',Int16MultiArray,queue_size = 10)
        #pub.publish(lds_pub)
        #print(lds)
        if len(lds) == 64:
            lds_pub.data = lds
            pub = rospy.Publisher('linesss',Int16MultiArray,queue_size = 1)
            pub.publish(lds_pub)
            print(lds)
            #print(len(lds))
        lds = []
      if i >= 42 and i <= 57:
        #str_data.append(chr(i))
        if i != 44:
          str_data.append(chr(i))
          str_data2 = "".join(str_data)
          #print("str = ",str_data2)
        if i == 44:
          lds.append(int(str_data2))
          #print("str = ",str_data2)
          str_data = []
  data2 = data

# 關閉
LoRa.FunLora_close() 


