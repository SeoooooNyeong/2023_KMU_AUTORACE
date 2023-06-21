#!/usr/bin/env python
# -*- coding: utf-8 -*-

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import cv2
import rospy, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os


Width = 640
Height = 480
warp_Offset = 120

# 트랙 동영상 읽어 들이기
window_title = 'camera'
window_title2 = 'camera2'

warp_img_w = 320
warp_img_h = 240

# 슬라이딩 윈도우 개수
nwindows = 9
# 슬라이딩 윈도우 넓이
margin = 12
# 선을 그리기 위해 최소한 있어야 할 점의 개수
minpix = 3

lane_bin_th = 145

warp_src  = np.array([
    [-130, 350],  
    [-200, 400],
    [Width + 130, 350],
    [Width + 200, 400]
], dtype=np.float32)

warp_dist = np.array([
    [0,0],
    [0,warp_img_h],
    [warp_img_w,0],
    [warp_img_w, warp_img_h]
], dtype=np.float32)


# 변환전과 후의 4개 점 좌표를 전댈해서 이미지를 원근변환 처리하여 새로운 이미지로 만들기
def warp_image(img, src, dst, size):
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)
    warp_img = cv2.warpPerspective(img, M, size, flags=cv2.INTER_LINEAR)

    return warp_img, M, Minv

def warp_process_image(img):
    global nwindows
    global margin
    global minpix
    global lane_bin_th
    
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 0, 255])
    upper_red = np.array([0, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)
    kernel = np.ones((4, 4), np.uint8)
    lane = cv2.erode(mask, kernel)
    
    # 히스토그램이란 이미지를 구성하는 픽셀 분포에 대한 그래프
    # (1) x축: 픽셀의 x 좌표값
    # (2) y축: 특정 x 좌표값을 갖는 모든 흰색 픽셀의 개수
    histogram = np.sum(lane[lane.shape[0]//2:,:], axis=0)
    # x축(x좌표)을 반으로 나누어 왼쪽 차선과 오른쪽 차선을 구분하기      
    midpoint = np.int(histogram.shape[0]/2)
    # 왼쪽 절반 구역에서 흰색 픽셀의 개수가 가장 많은 위치를 슬라이딩 윈도우의 왼쪽 시작 위치로 잡기
    leftx_current = np.argmax(histogram[:midpoint])
    # 오른쪽 절반 구역에서 흰색 픽셀의 개수가 가장 많은 위치를 슬라이딩 윈도우의 오른쪽 시작 위치로 잡기
    rightx_current = np.argmax(histogram[midpoint:]) + midpoint

    window_height = np.int(lane.shape[0]/nwindows)
    nz = lane.nonzero()

    left_lane_inds = []
    right_lane_inds = []
    
    lx, ly, rx, ry = [], [], [], []

    out_img = np.dstack((lane, lane, lane))*255

    for window in range(nwindows):

        win_yl = lane.shape[0] - (window+1)*window_height
        win_yh = lane.shape[0] - window*window_height

        win_xll = leftx_current - margin
        win_xlh = leftx_current + margin
        win_xrl = rightx_current - margin
        win_xrh = rightx_current + margin

        cv2.rectangle(out_img,(win_xll,win_yl),(win_xlh,win_yh),(0,255,0), 2) 
        cv2.rectangle(out_img,(win_xrl,win_yl),(win_xrh,win_yh),(0,255,0), 2) 

        # 슬라이딩 윈도우 박스(녹색박스) 하나 안에 있는 흰색 픽셀의 x 좌표를 모두 모으기
        # 왼쪽과 오른쪽 슬라이딩 박스를 따로 작업하기
        good_left_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xll)&(nz[1] < win_xlh)).nonzero()[0]
        good_right_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xrl)&(nz[1] < win_xrh)).nonzero()[0]

        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

        # 위에서 구한 x좌표 리스트에서 흰색점이 5개 이상인 경우에 한해서 x좌표의 평균값을 구하기
        # 이 값을 위에 쌓을 슬라이딩 윈도우의 중심점으로 사용함(그리고 계쏙 for 9번 반복)
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nz[1][good_left_inds]))
        if len(good_right_inds) > minpix:        
            rightx_current = np.int(np.mean(nz[1][good_right_inds]))

        lx.append(leftx_current)
        ly.append((win_yl + win_yh)/2)

        rx.append(rightx_current)
        ry.append((win_yl + win_yh)/2)

    # 슬라이딩 윈도우의 중심점(x좌표)를 1x/1y, rx/ry에 담아두기(9개를 모두 모으기)
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    #left_fit = np.polyfit(nz[0][left_lane_inds], nz[1][left_lane_inds], 2)
    #right_fit = np.polyfit(nz[0][right_lane_inds] , nz[1][right_lane_inds], 2)
    
    # 슬라이딩 윈도우의 중심점(x좌표) 9개를 가지고 2차 함수를 만들어내기
    lfit = np.polyfit(np.array(ly),np.array(lx),2)
    rfit = np.polyfit(np.array(ry),np.array(rx),2)

    # 기존 하얀색 차선 픽셀을 왼쪽과 오른쪽 각각 파란색과 빨간색으로 색상 변경
    out_img[nz[0][left_lane_inds], nz[1][left_lane_inds]] = [255, 0, 0]
    out_img[nz[0][right_lane_inds] , nz[1][right_lane_inds]] = [0, 0, 255]
    cv2.imshow("viewer", out_img)
    
    #return left_fit, right_fit
    return lfit, rfit

def get_pos_from_fit(fit, y=warp_Offset, left=False, right=False):
    a, b, c = fit[0], fit[1], fit[2]

    if a==0 and b==0 and c==0:
        if left:
            pos = 0
        elif right:
            pos = Width
    else:
        pos = a*y*y + b*y + c
        
    return pos
    
def draw_lane(image, warp_img, Minv, left_fit, right_fit):
    global Width, Height
    yMax = warp_img.shape[0]
    ploty = np.linspace(0, yMax - 1, yMax)
    color_warp = np.zeros_like(warp_img).astype(np.uint8)
    
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    
    # 이차함수 이용해서 사다리꼴 이미지 외곽선 픽셀 좌표 계산해서
    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))]) 
    pts = np.hstack((pts_left, pts_right))
    
    # 사다리꼴 이미지를 칼라(녹색)로 그리고 거꾸로 원근 변환해서 원본 이미지와 오버레이하기
    color_warp = cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
    newwarp = cv2.warpPerspective(color_warp, Minv, (Width, Height))

    return cv2.addWeighted(image, 1, newwarp, 0.3, 0)

#=============================================
# 터미널에서 Ctrl-c 키입력이로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
#=============================================
def signal_handler(sig, frame):
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
image = np.empty(shape=[0]) # 카메라 이미지를 담을 변수
bridge = CvBridge()
motor = None # 모터 토픽을 담을 변수

#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
CAM_FPS = 30    # 카메라 FPS - 초당 30장의 사진을 보냄
WIDTH, HEIGHT = 640, 480    # 카메라 이미지 가로x세로 크기

#=============================================
# 콜백함수 - 카메라 토픽을 처리하는 콜백함수
# 카메라 이미지 토픽이 도착하면 자동으로 호출되는 함수
# 토픽에서 이미지 정보를 꺼내 image 변수에 옮겨 담음.
#=============================================
def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

#=============================================
# 모터 토픽을 발행하는 함수
# 입력으로 받은 angle과 speed 값을
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):

    global motor

    motor_msg = xycar_motor()
    motor_msg.angle = angle
    motor_msg.speed = speed

    motor.publish(motor_msg)

#=============================================
# 실질적인 메인 함수
# 카메라 토픽을 받아 각종 영상처리와 알고리즘을 통해
# 차선의 위치를 파악한 후에 조향각을 결정하고,
# 최종적으로 모터 토픽을 발행하는 일을 수행함.
#=============================================
def start():

    # 위에서 선언한 변수를 start() 안에서 사용하고자 함
    global motor, image

    #=========================================
    # ROS 노드를 생성하고 초기화 함.
    # 카메라 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
    #=========================================
    rospy.init_node('driving')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw/",Image, img_callback)

    print ("----- Xycar self driving -----")

    # 첫번째 카메라 토픽이 도착할 때까지 기다림.
    rospy.wait_for_message("/usb_cam/image_raw/", Image)

    #=========================================
    # 메인 루프
    # 카메라 토픽이 도착하는 주기에 맞춰 한번씩 루프를 돌면서
    # "이미지처리 + 차선위치찾기 + 조향각 결정 + 모터토픽 발행"
    # 작업을 반복적으로 수행함.
    #=========================================
    while not rospy.is_shutdown():

        # 이미지 처리를 위해 카메라 원본 이미지를 img에 복사 저장한다.
        img = image.copy()

        warp_img, M, Minv = warp_image(img, warp_src, warp_dist, (warp_img_w, warp_img_h))
        
        left_fit, right_fit = warp_process_image(warp_img)
        lane_img = draw_lane(image, warp_img, Minv, left_fit, right_fit)
        
        lpos = get_pos_from_fit(left_fit, y=warp_Offset, left=True, right=False)
        rpos = get_pos_from_fit(right_fit, y=warp_Offset, left=False, right=True)
        center = (lpos + rpos) / 2
        angle = warp_img_w/2 - center
        steer_angle = angle *0.3
        # steer_img = draw_steer(lane_img, steer_angle)
        
        # print(lpos, rpos)
        drive(steer_angle, 5)
        for r in warp_src:
            cv2.circle(img, (r[0], r[1]), 5, (255,0,0))
                
        cv2.imshow('image', img)
        cv2.imshow(window_title, warp_img)
        cv2.imshow(window_title2, lane_img)

        k = cv2.waitKey(1) & 0xff
        if k == 27 : break
        
        cv2.waitKey(1)


#=============================================
# 메인 함수
# 가장 먼저 호출되는 함수로 여기서 start() 함수를 호출함
# start() 함수가 실질적인 메인 함수임.
#=============================================
if __name__ == '__main__':
    start()

