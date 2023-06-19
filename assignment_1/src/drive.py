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

# while loop 실행 횟수를 세는 변수
count = 0

while not rospy.is_shutdown():

    #=========================================
    # 첫번째 실행인 경우
    # angle=0, speed=50으로 설정한다
    #=========================================
    if count == 0:
        xycar_msg.angle = 0
        xycar_msg.speed = 50
        count += 1
    #=========================================
    # 왼쪽으로 치우친 경우 (왼쪽 앞 센서 < 오른쪽 앞 센서)
    # 오른쪽으로 30도 회전한다
    #=========================================
    elif ultrasonicData[1] < ultrasonicData[3]:
        xycar_msg.angle = 30
        xycar_msg.speed = 50
    #=========================================
    # 오른쪽으로 치우친 경우 <왼쪽 앞 센서 > 오른쪽 앞 센서)
    # 왼쪽으로 30도 회전한다
    #=========================================
    elif ultrasonicData[3] < ultrasonicData[1]:
        xycar_msg.angle = -30
        xycar_msg.speed = 50
    #=========================================
    # 왼쪽 앞 센서와 오른쪽 앞 센서의 차이가 없는 경우
    # 정면으로 주행한다
    #=========================================
    else:
        xycar_msg.angle = 0
        xycar_msg.speed = 50
        
    #=========================================
    # 변경된 각도와 속도 publish
    #=========================================
    motor_pub.publish(xycar_msg)
