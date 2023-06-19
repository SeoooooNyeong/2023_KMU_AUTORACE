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
rx, ry = [0, 0, 0, 1100], [0, 0, 0, 95] # 끝점만 프리셋 길이는 자유지만 planning range값을 바꿔줘야

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
    global rx, ry
    print("Start Planning")
    for i in range(3): # 끝점을 제외한 사이값을 계산하여 rx, ry를 업데이트
    	rx[i] = (rx[3]-sx)/4*(i+1)+sx
    	ry[i] = (ry[3]-sy)/4*(i+1)+sy
    print(rx, ry)
    return rx, ry

#=============================================
# 생성된 경로를 따라가는 함수
# 파이게임 screen, 현재위치 x,y 현재각도, yaw
# 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
# 각도와 속도를 결정하여 주행한다.
#=============================================
def tracking(screen, x, y, yaw, velocity, max_acceleration, dt):
    global rx, ry
    speed = 50
    if x < rx[0]: #첫번째 구간일때 (x값을 기준으로 했음)
    	atan = math.atan((y-ry[0])/(rx[0]-x))*180/3.1415 (현재점과 현재목표점(1번점)사이의 직선을 그어 자표계의 360도법으로 변환)
    	angle = yaw-atan (현재 핸들 각에서 위값을 빼준값을 새로운 핸들값)
    	drive(angle, speed)
    	print(1, f"yaw: {yaw}", f"atan: {atan}") #디버깅용
    elif x < rx[1]:
    	atan = math.atan((y-ry[1])/(rx[1]-x))*180/3.1415 # 
    	angle = yaw-atan
    	drive(angle, speed)
    	print(2, f"yaw: {yaw}", f"atan: {atan}")
    elif x < rx[2]:
    	atan = math.atan((y-ry[2])/(rx[2]-x))*180/3.1415
    	angle = yaw-atan
    	drive(angle, speed)
    	print(3, f"yaw: {yaw}", f"atan: {atan}")
    elif x < rx[3]:
    	atan = math.atan((y-ry[3])/(rx[3]-x))*180/3.1415
    	angle = yaw-atan
    	drive(angle, speed)
    	print(4, f"yaw: {yaw}", f"atan: {atan}")
    else:
    	drive(0, 0)
   # x의 값이 목보다 작을때만 동작함(x값을 기준으로 했기때문에). 시작점이 목표값보다 뒤에 있다면 오류.
