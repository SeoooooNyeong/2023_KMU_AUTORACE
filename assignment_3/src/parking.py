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
rx, ry = [], []    # 이동 경로를 담는 리스트
i = 0              # 이동 경로 tracking을 위한 pointer
iMax = 0           # planning 경로 리스트 점의 개수(i 최댓값)
isTooClose = False # 시작점과 목적지의 거리가 가까운지 판단하는 boolean

#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
AR = (1142, 62)      # AR 태그의 위치
P_ENTRY = (1100, 95) # 주차라인 진입 시점의 좌표
P_END = (1129, 69)   # 주차라인 끝의 좌표

MAX_T = 100.0  # maximum time to the goal[s]
MIN_T = 10.0   # minimum time to the goal[s]

#=============================================
# 
#=============================================
class QuinticPolynomial:
    #xqp = QuinticPolynomial(sx, vxs, axs, gx, vxg, axg, T) #도착지 sx, vxs, axs / 목적지 gx, vxg , axg, T
    def __init__(self, xs, vxs, axs, xe, vxe, axe, time):
        # calc coefficient of quintic polynomial
        # See jupyter notebook document for derivation of this equation.
        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[time ** 3, time ** 4, time ** 5],
                      [3 * time ** 2, 4 * time ** 3, 5 * time ** 4],
                      [6 * time, 12 * time ** 2, 20 * time ** 3]])
        b = np.array([xe - self.a0 - self.a1 * time - self.a2 * time ** 2,
                      vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4 + self.a5 * t ** 5

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3 + 5 * self.a5 * t ** 4

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2 + 20 * self.a5 * t ** 3

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t ** 2

        return xt
    
#=============================================

#=============================================
def quintic_polynomials_planner(sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga, max_accel, max_jerk, dt):
    """
    quintic polynomial planner

    input
        s_x: start x position [m]
        s_y: start y position [m]
        s_yaw: start yaw angle [rad]
        sa: start accel [m/ss]
        gx: goal x position [m]
        gy: goal y position [m]
        gyaw: goal yaw angle [rad]
        ga: goal accel [m/ss]
        max_accel: maximum accel [m/ss]
        max_jerk: maximum jerk [m/sss]
        dt: time tick [s]

    return
        time: time result
        rx: x position result list
        ry: y position result list
        ryaw: yaw angle result list
        rv: velocity result list
        ra: accel result list

    """
    #속도 x축 y축 분해
    vxs = sv * math.cos(syaw)
    vys = sv * math.sin(syaw)
    vxg = gv * math.cos(gyaw)
    vyg = gv * math.sin(gyaw)
    
    #가속도 x축 y축 분해
    axs = sa * math.cos(syaw)
    ays = sa * math.sin(syaw)
    axg = ga * math.cos(gyaw)
    ayg = ga * math.sin(gyaw)

    time, rx, ry, ryaw, rv, ra, rj = [], [], [], [], [], [], []

    for T in np.arange(MIN_T, MAX_T, MIN_T):
        xqp = QuinticPolynomial(sx, vxs, axs, gx, vxg, axg, T) #도착지 sx, vxs, axs / 목적지 gx, vxg , axg, T
        
        yqp = QuinticPolynomial(sy, vys, ays, gy, vyg, ayg, T)


        time, rx, ry, ryaw, rv, ra, rj = [], [], [], [], [], [], []

        for t in np.arange(0.0, T + dt, dt):
            time.append(t)
            rx.append(xqp.calc_point(t))
            ry.append(yqp.calc_point(t))

            vx = xqp.calc_first_derivative(t)
            vy = yqp.calc_first_derivative(t)
            v = np.hypot(vx, vy)
            yaw = math.atan2(vy, vx)
            rv.append(v)
            ryaw.append(yaw)

            ax = xqp.calc_second_derivative(t)
            ay = yqp.calc_second_derivative(t)
            a = np.hypot(ax, ay)
            if len(rv) >= 2 and rv[-1] - rv[-2] < 0.0:
                a *= -1
            ra.append(a)

            jx = xqp.calc_third_derivative(t)
            jy = yqp.calc_third_derivative(t)
            j = np.hypot(jx, jy)
            if len(ra) >= 2 and ra[-1] - ra[-2] < 0.0:
                j *= -1
            rj.append(j)

        if max([abs(i) for i in ra]) <= max_accel and max([abs(i) for i in rj]) <= max_jerk:
            print("find path!!")
            break

    return time, rx, ry, ryaw, rv, ra, rj

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
# 차량의 시작위치 px, py, 시작각도 syaw
# 최대가속도 max_acceleration, 단위시간 dtt 를 전달받고
# 경로를 리스트를 생성하여 반환한다.
#=============================================
def planning(px, py, ssyaw, max_acceleration, dtt):
    global rx, ry, i, iMax, isTooClose, P_ENTRY
    global time, rx, ry, cyaw, v, a, j
    global sx, sy, syaw
    print("Start Planning")
    
    # 시작점과 목적지 사이의 거리가 300 미만이라면 isTooClose = True
    isTooClose = False
    if math.sqrt((px-P_ENTRY[0])**2 + (py-P_ENTRY[1])**2) < 300:
        isTooClose = True 
    
    sx = px  # start x position [m]
    sy = py  # start y position [m]
    syaw = ssyaw  # start yaw angle [rad]
    sv = 10.0  # start speed [m/s]
    sa = 2.0   # start accel [m/ss]
    
    gx = P_ENTRY[0]  # goal x position [m]
    gy = P_ENTRY[1]  # goal y position [m]
    gyaw = np.deg2rad(-45)  # goal yaw angle [rad]
    gv = 10.0  # goal speed [m/s]
    ga = 2.0   # goal accel [m/ss]
    
    max_accel = max_acceleration  # max accel [m/ss]
    max_jerk = 10  # max jerk [m/sss]
    dt = dtt       # time tick [s] 점 거리
    print(dtt)
    time, rx, ry, cyaw, v, a, j = quintic_polynomials_planner(
        sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga, max_accel, max_jerk, dt)
    
    i = 0
    iMax = len(ry)
    print(ry)
    return rx, ry

#=============================================
# 생성된 경로를 따라가는 함수
# 파이게임 screen, 현재위치 x,y 현재각도, yaw
# 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
# 각도와 속도를 결정하여 주행한다.
#=============================================
def tracking(screen, x, y, yaw, velocity, max_acceleration, dt):
    global rx, ry, i, iMax, P_END, isTooClose
    speed = 50
    
    # 현 위치와 목적지 사이의 거리가 300 넘을 때까지 후진
    # 현 위치와 목적지 사이의 거리가 300 초과이면 planning 수행
    if isTooClose==True and math.sqrt((x-P_END[0])**2 + (y-P_END[1])**2) > 300:
        isTooClose = False
        planning(x, y, yaw, max_acceleration, dt)
        return 0
    if isTooClose == True:
        drive(0,-50)
        return 0
        
        
    if i >= iMax-1:
        drive(0,0)
    elif y > ry[i]:
        atan = math.atan((ry[i]-ry[i+1])/(rx[i+1]-rx[i]))*180/3.1415
        if atan < 0:
            atan += 180
        #오차 계산하여 보정
        error = x - rx[i]
        if error > 50:
            error = 50
        elif error < -50:
            error = -50
        if 0<error<20:
            error = 20
        elif -20<error<0:
            error = -20
        error /= 5
        angle = yaw-atan-error
        drive(angle, speed)
        print(f"yaw: {yaw}", f"atan: {atan}")
    else:
        i+=10
