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
# 시간에 대한 위치를 나타내는 5차 다항식 객체
# Q(t) = a0 + (a1 * t) + (a2 * t**2) + (a3 * t**3) + (a4 * t**4) + (a5 * t**5)
#=============================================
class QuinticPolynomial:
    #xqp = QuinticPolynomial(sx, vxs, axs, gx, vxg, axg, T) #도착지 sx, vxs, axs / 목적지 gx, vxg , axg, T
    def __init__(self, xs, vxs, axs, xe, vxe, axe, time):
        # 5차 다항식 계수 계산
        self.a0 = xs        # 상수항: 초기 위치
        self.a1 = vxs       # 일차항 계수: 초기 속도
        self.a2 = axs / 2.0 # 이차항 계수: 초기 가속도

        # 시간에 대한 위치, 속도, 가속도를 근사한 표현
        A = np.array([[time ** 3, time ** 4, time ** 5],
                      [3 * time ** 2, 4 * time ** 3, 5 * time ** 4],
                      [6 * time, 12 * time ** 2, 20 * time ** 3]])
        b = np.array([xe - self.a0 - self.a1 * time - self.a2 * time ** 2,
                      vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])

        # 3차항, 4차항, 5차항 계수 계산
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    # 주어진 시간에 따른 위치 계산
    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4 + self.a5 * t ** 5

        return xt

    # 주어진 시간에 따른 속도 계산
    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3 + 5 * self.a5 * t ** 4

        return xt

    # 주어진 시간에 따른 가속도 계산
    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2 + 20 * self.a5 * t ** 3

        return xt

    # 주어진 시간에 따른 가가속도 계산
    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t ** 2

        return xt
    
#=============================================
# 5차 다항식을 이용하여 주어진 조건에 맞는 경로를 계획하는 함수
#=============================================
def quintic_polynomials_planner(sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga, max_accel, max_jerk, dt):
    """
    quintic polynomial planner

    input
        sx:   시작 x 좌표
        sy:   시작 y 좌표
        syaw: 시작 yaw 각도 [단위: rad]
        sv:   시작 속도
        sa:   시작 가속도
        gx:   목표 x 좌표
        gy:   목표 y 좌표
        gyaw: 목표점에서 yaw 각도 [단위: rad]
        gv:   목표점에서 속도
        ga:   목표점에서 가속도
        max_accel: 최대 가속도
        max_jerk:  최대 가가속도
        dt: time tick

    return
        time: time
        rx: x 좌표 list
        ry: y 좌표 list
        ryaw: yaw 각도 list
        rv: 속도 list
        ra: 가속도 list

    """
    # 시작 속도 x축 y축 분해
    vxs = sv * math.cos(syaw)
    vys = sv * math.sin(syaw)
    # 목표 속도 x축 y축 분해
    vxg = gv * math.cos(gyaw)
    vyg = gv * math.sin(gyaw)
    
    # 시작 가속도 x축 y축 분해
    axs = sa * math.cos(syaw)
    ays = sa * math.sin(syaw)
    # 목표 가속도 x축 y축 분해
    axg = ga * math.cos(gyaw)
    ayg = ga * math.sin(gyaw)

    # 다항식의 T 값(시간)을 조절하며 경로를 계산
    for T in np.arange(MIN_T, MAX_T, MIN_T):
        # 주어진 초기 조건과 목표 조건을 사용하여 x축 및 y축에 대한 5차 다항식 객체 생성
        # 도착지 sx, vxs, axs | 목적지 gx, vxg , axg, T
        xqp = QuinticPolynomial(sx, vxs, axs, gx, vxg, axg, T)
        yqp = QuinticPolynomial(sy, vys, ays, gy, vyg, ayg, T)

        # return 값 초기화
        time, rx, ry, ryaw, rv, ra, rj = [], [], [], [], [], [], []

        # 시간에 따른 위치, 속도, 가속도, 가가속도를 계산하는 반복문
        # 0부터 T까지 dt 간격으로 반복하여 각 시간에 대한 위치와 도함수 값을 계산
        for t in np.arange(0.0, T + dt, dt):

            time.append(t) # 시간 저장
            rx.append(xqp.calc_point(t)) # x 좌표 저장
            ry.append(yqp.calc_point(t)) # y 좌표 저장

            vx = xqp.calc_first_derivative(t) # x축 속도 계산
            vy = yqp.calc_first_derivative(t) # y축 속도 계산
            v = np.hypot(vx, vy)     # 속도의 크기 계산
            yaw = math.atan2(vy, vx) # 속도의 방향 계산
            rv.append(v)     # 속도 저장
            ryaw.append(yaw) # 방향 저장

            ax = xqp.calc_second_derivative(t) # x축 가속도 계산
            ay = yqp.calc_second_derivative(t) # y축 가속도 계산
            a = np.hypot(ax, ay) # 가속도의 크기 계산
            # 속도가 감소하는 지점에서는 가속도의 부호를 바꾼다
            if len(rv) >= 2 and rv[-1] - rv[-2] < 0.0:
                a *= -1
            ra.append(a) # 가속도 저장

            jx = xqp.calc_third_derivative(t) # x축 가가속도 계산
            jy = yqp.calc_third_derivative(t) # y축 가가속도 계산
            j = np.hypot(jx, jy) # 가가속도의 크기 계산
            # 가속도가 감소하는 지점에서 가가속도의 부호를 바꾼다
            if len(ra) >= 2 and ra[-1] - ra[-2] < 0.0:
                j *= -1
            rj.append(j) # 가가속도 저장

        # 가속도와 가가속도의 최댓값이 주어진 범위 내에 있는지 확인한다
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
    
    sx = px  # 시작 x 좌표
    sy = py  # 시작 y 좌표
    syaw = ssyaw  # 시작 yaw 각도 [단위: rad]
    sv = 10.0  # 시작 속도
    sa = 2.0   # 시작 가속도
    
    gx = P_ENTRY[0]  # 목표 x 좌표
    gy = P_ENTRY[1]  # 목표 y 좌표
    gyaw = np.deg2rad(-45)  # 목표점에서 yaw 각도 [rad]
    gv = 10.0  # 목표점에서 속도
    ga = 2.0   # 목표점에서 가속도
    
    max_accel = max_acceleration  # 최대 가속도
    max_jerk = 10  # 최대 가가속도
    dt = dtt       # time tick 점 거리
    print(dtt)

    # 경로 계획
    time, rx, ry, cyaw, v, a, j = quintic_polynomials_planner(
        sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga, max_accel, max_jerk, dt)
    
    i = 0          # 경로 리스트 원소를 가리키는 포인터 변수 초기화
    iMax = len(ry) # 포인터 변수 최댓값 설정
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
    
    # 현 위치와 목적지 사이의 거리가 300 초과이면 planning 수행
    if isTooClose==True and math.sqrt((x-P_END[0])**2 + (y-P_END[1])**2) > 300:
        # 매우 가까운 상태에서 후진하여 거리가 300을 넘으면 거리 상태 갱신
        isTooClose = False
        # 거리가 충분히 멀어졌으므로 planning 재수행
        planning(x, y, yaw, max_acceleration, dt)
        return 0
    # 시작 위치와 목적지 사이의 거리가 가깝다면
    # 현 위치와 목적지 사이의 거리가 300 넘을 때까지 후진
    if isTooClose == True:
        drive(0,-50)
        return 0
        
    # 플래닝 된 경로를 모두 tracking 하였다면 주행 종료
    if i >= iMax-1:
        drive(0,0)
    elif y > ry[i]:
        # planning 트랙을 다항함수로 간주하고 현재 차량이 목표로 하고있는 점에서의 미분값을 구해 360도법으로 변환
        atan = math.atan((ry[i]-ry[i+1])/(rx[i+1]-rx[i]))*180/3.1415
        if atan < 0:
            atan += 180
        # 차량 기울기가 목표점의 기울기와 평행해지면 오차가 없다고 판단하기 때문에 목표점과의 x 좌표 차이를 활용해서 오차를 계산하여 보정
        error = x - rx[i] # 차량과 목표점의 x좌표 차이 

        # 핸들 최대각이 -50~50 이기에 범위를 벗어나는 값은 50,-50으로 잘라줌 
        if error > 50:
            error = 50
        elif error < -50:
            error = -50

        # 오차가 작아질수록 오차반영이 작아져 영원히 만나지 않으므로 오차의 하한선을 정해줌
        if 0<error<20:
            error = 20
        elif -20<error<0:
            error = -20

        # 오차를 -10~10 도로 비례하게 줄여줌 
        error /= 5
        # 위 계산값에 오차를 반영하여 각도를 정해줌
        angle = yaw-atan-error

        drive(angle, speed)
        print(f"yaw: {yaw}", f"atan: {atan}")
    else:
        i+=10
