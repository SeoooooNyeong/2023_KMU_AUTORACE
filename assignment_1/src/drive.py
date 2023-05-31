#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, math
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor

ultrasonicData = [0.0, 0.0, 0.0, 0.0, 0.0]
# 0 left 1 lf 2 f 3 rf 4 r
# 0 lf 1 f 2 rf 6 r 7 l
def callback(msg): 
    global ultrasonicData
    ultrasonicData = msg.data  

rospy.init_node('driver')
rospy.Subscriber('ultrasonic', Int32MultiArray, callback)
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

xycar_msg = xycar_motor()

while not rospy.is_shutdown():

    angle = 0
    speed = 10
    xycar_msg.angle = angle
    xycar_msg.speed = speed
    motor_pub.publish(xycar_msg)

    if ultrasonicData[1] < ultrasonicData[3]:
        xycar_msg.angle = (ultrasonicData[3] - ultrasonicData[1] - 10)
        xycar_msg.speed = 60
        motor_pub.publish(xycar_msg)
    elif ultrasonicData[3] < ultrasonicData[1]:
        xycar_msg.angle = (ultrasonicData[3] - ultrasonicData[1] - 10)
        xycar_msg.speed = 60
        motor_pub.publish(xycar_msg)
    else:
        xycar_msg.angle = 0
        xycar_msg.speed = 120
        motor_pub.publish(xycar_msg)
