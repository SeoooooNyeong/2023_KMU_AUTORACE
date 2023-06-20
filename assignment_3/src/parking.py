#!/usr/bin/env python
#-- coding:utf-8 --

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import pygame
import numpy as np
import math
import rospy
from xycar_msgs.msg import xycar_motor

#=============================================
# 모터 토픽을 발행할 것임을 선언
#============================================= 
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
xycar_msg = xycar_motor()

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#============================================= 
rx = [0 for _ in range(100)]
ry = [0 for _ in range(100)]
rx[-1] = 1100
ry[-1] = 95
i = 0 # 글로벌 변수 i지정
#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
AR = (1142, 62) # AR 태그의 위치
P_ENTRY = (1036, 162) # 주차라인 진입 시점의 좌표
P_END = (1129, 69) # 주차라인 끝의 좌표

#=============================================
# 모터 토픽을 발행하는 함수
# 입력으로 받은 angle과 speed 값을
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):
    xycar_msg.angle = int(angle)
    xycar_msg.speed = int(speed)
    motor_pub.publish(xycar_msg)

#=============================================
# 경로를 생성하는 함수
# 차량의 시작위치 sx, sy, 시작각도 syaw
# 최대가속도 max_acceleration, 단위시간 dt 를 전달받고
# 경로를 리스트를 생성하여 반환한다.
#=============================================
def planning(sx, sy, syaw, max_acceleration, dt):
    global rx, ry, i
    i = 0 #플래닝 할때마다 i초기화
    print("Start Planning")
    for k in range(100): # 100개 점을 플래닝함
    	rx[k] = (rx[-1]-sx)/100*(k+1)+sx
    	ry[k] = (ry[-1]-sy)/100*(k+1)+sy
    return rx, ry

#=============================================
# 생성된 경로를 따라가는 함수
# 파이게임 screen, 현재위치 x,y 현재각도, yaw
# 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
# 각도와 속도를 결정하여 주행한다.
#=============================================
def tracking(screen, x, y, yaw, velocity, max_acceleration, dt):
    global rx, ry, i
    speed = 50
    if i >= 99: # 점의 끝까지 가면 스탑
        drive(0,0)
    elif y > ry[i]: # i번째 점과의 직선과의 각도차이 계산
        atan = math.atan((y-ry[i])/(rx[i]-x))*180/3.1415
        if atan < 0:
            atan += 180
        angle = yaw-atan
        drive(angle, speed)
        print(f"yaw: {yaw}", f"atan: {atan}")
    else:
        i+=1 # 점을 넘어갈때 i증가하여 다음 점 추적
