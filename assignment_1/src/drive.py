#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, math
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor

ultrasonicData = [0.0, 0.0, 0.0, 0.0, 0.0]
# 0 left 1 lf 2 f 3 rf 4 r

def callback(msg): 
    global ultrasonicData
    ultrasonicData = msg.data  

rospy.init_node('driver')
rospy.Subscriber('ultrasonic', Int32MultiArray, callback)
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

xycar_msg = xycar_motor()

count = 0

while not rospy.is_shutdown():

    
    if count == 0:
        xycar_msg.angle = 0
        xycar_msg.speed = 10
        count += 1
     
    elif ultrasonicData[1] < ultrasonicData[3]: # left front < right front
        xycar_msg.angle = 30
        xycar_msg.speed = 50
    elif ultrasonicData[3] < ultrasonicData[1]: # right front < left front
        xycar_msg.angle = -30
        xycar_msg.speed = 50
    else:
        xycar_msg.angle = 0
        xycar_msg.speed = 50
    motor_pub.publish(xycar_msg)

    print('speed:', xycar_msg.speed)
