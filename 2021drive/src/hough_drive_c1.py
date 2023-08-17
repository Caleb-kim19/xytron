#!/usr/bin/env python
# -*- coding: utf-8 -*-
####################################################################
# 프로그램명 : hough_drive_c1.py
# 작 성 자 : (주)자이트론
# 생 성 일 : 2020년 07월 23일
# 본 프로그램은 상업 라이센스에 의해 제공되므로 무단 배포 및 상업적 이용을 금합니다.
####################################################################

import rospy, rospkg, time
import numpy as np
import cv2, math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from mydrive.msg import rect_size
from math import *
import signal
import sys
import os

# 추가된 부분: 로그와 그래프를 위한 라이브러리
import time
import matplotlib.pyplot as plt

def signal_handler(sig, frame):
    import time
    time.sleep(3)
    
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

image = np.empty(shape=[0])
img = np.empty(shape=[0])
bridge = CvBridge()
motor = None
Width = 320
Height = 240
Offset = 175
Gap = 85
m = 0
prev_time = 0
#size = 0

'''
mode 0 = hough_drive
mode 1 = obstacle detected
mode 2 = stopline_detected
'''
drive_mode = 0
back_time = 0
cam = True
cam_debug = True
sub_f = 0
time_c = 0
stopline_count = 0
obstacle = (0,0,0,0)
'''
yellow_line(center_line) position
on the left = 0
on the right = 1
'''
yellow_line = 0
#fourcc = cv2.VideoWriter_fourcc(*'XVID')

#out1 = cv2.VideoWriter('../result.avi', fourcc, 15.0, (Width, Height))

cascade = cv2.CascadeClassifier('/home/pi/xycar_ws/src/oval_team1/2021drive/src/obs_cascade.xml')

def img_callback(data):
    global image   
    global sub_f 
    global time_c
    image = bridge.imgmsg_to_cv2(data, "bgr8")
        
def detect_obstacle(image):
    global obstacle, drive_mode, back_time
    
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    cascades = cascade.detectMultiScale(gray,1.01,50)

    if len(cascades) > 0:
        obstacle = cascades[0]
        print('obstacle detected')
        drive_mode = 1
    else:
        obstacle = (0,0,0,0)

# publish xycar_motor msg
def drive(Angle, Speed): 
    global motor

    motor_msg = xycar_motor()
    motor_msg.angle = Angle
    motor_msg.speed = Speed

    motor.publish(motor_msg)

# draw lines
def draw_lines(img, lines):
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0]
        img = cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), (0, 255, 0), 2)
    return img

def yellow_line_detect(image):
    global yellow_line
    x_sum = 0
    m_sum = 0
    hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
    yellow_lower = np.array([15, 50, 20])
    yellow_upper = np.array([40, 220, 220])
    yellow_mask = cv2.inRange(hls, yellow_lower, yellow_upper)
    masked = cv2.bitwise_and(image, image, mask = yellow_mask)
    
    #cv2.imshow('yellow',masked)
    
    gray = cv2.cvtColor(masked,cv2.COLOR_BGR2GRAY)
    # blur
    kernel_size = 3
    standard_deviation_x = 1.5     #Kernel standard deviation along X-axis
    blur_gray = cv2.GaussianBlur(gray, (kernel_size, kernel_size), standard_deviation_x)

    # canny edge
    low_threshold = 90
    high_threshold = 180
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold, kernel_size)

    lines = cv2.HoughLinesP(edge_img, 1, math.pi/180,30,30,5)
    if lines is not None:
        size = len(lines)
        if size != 0:
            for line in lines:
                x1, y1, x2, y2 = line[0]

                x_sum += x1 + x2
                if (x2-x1) != 0:
                    m_sum += float(y2 - y1) / float(x2 - x1)

            x_avg = x_sum / (size * 2)

            m = m_sum / size
            if (m < 0) and (x_avg < Width/2 + 45):
                print("Center_line : left")
                yellow_line = 0
            elif (m > 0) and (x_avg > Width/2 - 45):
                print("Center_line : right")
                yellow_line = 1

# draw rectangle
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2

    cv2.rectangle(img, (lpos - 2, 7 + offset),
                       (lpos + 2, 12 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (rpos - 2, 7 + offset),
                       (rpos + 2, 12 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (center-2, 7 + offset),
                       (center+2, 12 + offset),
                       (0, 255, 0), 2)    
    cv2.rectangle(img, (157, 7 + offset),
                       (162, 12 + offset),
                       (0, 0, 255), 2)
    return img

# left lines, right lines
def divide_left_right(lines):
    global Width

    low_slope_threshold = 0.1
    high_slope_threshold = 10

    # calculate slope & filtering with threshold
    slopes = []
    new_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]

        if x2 - x1 == 0:
            slope = 0
        else:
            slope = float(y2-y1) / float(x2-x1)
            #print(slope)
        
        if low_slope_threshold < abs(slope) < high_slope_threshold:
            slopes.append(slope)
            new_lines.append(line[0])

    # divide lines left to right
    left_lines = []
    right_lines = []
    th = 50

    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]

        x1, y1, x2, y2 = Line

        if (slope < 0) and (x2 < Width/2 + th):
            left_lines.append([Line.tolist()])
        elif (slope > 0) and (x1 > Width/2 - th):
            right_lines.append([Line.tolist()])

    return left_lines, right_lines

# get average m, b of line, sum of x, y, mget lpos, rpos
def get_line_pos(img, lines, left=False, right=False):
    global Width, Height
    global Offset, Gap, cam_debug
    global dis
    global m

    x_sum = 0.0
    y_sum = 0.0
    m_sum = 0.0
    dis = 0.0
    size = len(lines)
    
    m = 0
    b = 0

    if size != 0:
        for line in lines:
            x1, y1, x2, y2 = line[0]

            x_sum += x1 + x2
            y_sum += y1 + y2
            m_sum += float(y2 - y1) / float(x2 - x1)

        x_avg = x_sum / (size * 2)
        y_avg = y_sum / (size * 2)

        m = m_sum / size
        b = y_avg - m * x_avg
    

    if m == 0 and b == 0:
        if left:
            pos = -1
        elif right:
            pos = -1
    else:
        y = Gap / 2

        #pos = (y - b) / m
        pos = x_avg

        if cam_debug:
            #b += Offset
            xs = (Height - b) / float(m)
            xe = ((Height/2) - b) / float(m)

            cv2.line(img, (int(xs), Height), (int(xe), (Height/2)), (255, 0,0), 3)

    return img, int(pos)

def region_of_interest(img, vertices, color3=(255, 255, 255), color1=255):
    mask = np.zeros_like(img)
    #img와 같은 크기의 배열 생성 -> 값은 0
    if len(img.shape) > 2:
        color = color3
        #채널 수가 2보다 크면 흰색(RGB)
    else:
        color = color1
        #채널 수가 2보다 작거나 같으면 흰색(RGB가 아닌 다른 방식)
        #->결과적으로 흰색을 받아냄
    cv2.fillPoly(mask, vertices, color)
    #색이 채워진 다각형 그리기
    #'mask'라는 이미지에, vertices라는 좌표점들, 색상(흰색)의 다각형을 그림
    ROI_image = cv2.bitwise_and(img, mask)
    #img와 mask의 같은 위치의 색이 흰색이면 흰색, 아니라면 검정색

    return ROI_image

def detect_stopline(line_count):
    global stopline_count, drive_mode, prev_time
    # 정지선 인식
    #print('line count :'+str(line_count))
    if line_count >= 55:
        if stopline_count == 0 :
            stopline_count = stopline_count + 1
            prev_time = time.time()
            print('STOPLINE Detected ' + str(line_count) + 'LAP: ' + str(stopline_count))
                
        elif stopline_count == 1 and (time.time()-prev_time)> 8:
            stopline_count = stopline_count + 1
            prev_time = time.time()
            print('STOPLINE Detected ' + str(line_count) + 'LAP: ' + str(stopline_count))
            drive_mode=2


# show image and return lpos, rpos
def process_image(frame):
    global Width
    global Offset, Gap
    global cam, cam_debug, img

    # blur
    kernel_size = 3
    standard_deviation_x = 3     #Kernel standard deviation along X-axis
    blur = cv2.GaussianBlur(frame, (kernel_size, kernel_size), standard_deviation_x)
    
    vertices5 = np.array([[(0,175), (0,240), (320,240), (320,175), (300,175), (210,145), (110,145), (20,175)]], dtype=np.int32)
    
    
    roi = region_of_interest(blur, vertices5)
    roi = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
    #cv2.imshow('roi', roi)
    
    # gray
    gray = cv2.cvtColor(roi,cv2.COLOR_BGR2GRAY)
    
    ret, dest = cv2.threshold(gray, 130, 255, cv2.THRESH_BINARY )
    #cv2.imshow('gray', dest)

    # canny edge
    low_threshold = 170
    high_threshold = 200
    edge_img = cv2.Canny(np.uint8(dest), low_threshold, high_threshold, kernel_size)
    
    #cv2.imshow('Canny', edge_img)
    
    # HoughLinesP
    all_lines = cv2.HoughLinesP(edge_img, 1, math.pi/180,15,30,3) #라인 검출
    
    if cam:
        cv2.imshow('calibration', frame)
    # divide left, right lines

    # 선이 인식 안될 경우
    if all_lines is None:
        return (Width)/2, (Width)/2, False
    
    #정지선 검출
    line_count = len(all_lines)
    detect_stopline(line_count)
    left_lines, right_lines = divide_left_right(all_lines)
    
    if left_lines is not None: # 라인 정보를 받았으면
        for i in range(len(left_lines)):
            frame = cv2.line(frame, (left_lines[i][0][0],left_lines[i][0][1]), (left_lines[i][0][2], left_lines[i][0][3]),(255,0,0) , 3 )
    if right_lines is not None: # 라인 정보를 받았으면
        for i in range(len(right_lines)):
            frame = cv2.line(frame, (right_lines[i][0][0],right_lines[i][0][1]), (right_lines[i][0][2], right_lines[i][0][3]),(0,255,255) , 3 )
    #print(left_lines)

    # get center of lines
    frame, lpos = get_line_pos(frame, left_lines, left=True)
    frame, rpos = get_line_pos(frame, right_lines, right=True)
    #rospy.loginfo("rpos : " + str(rpos) + " lpos : " + str(lpos))

    if lpos < 0 and rpos < 0:
        return lpos, rpos, False
    
    if lpos >= 0 and rpos < 0:
        rpos = lpos + 200

    if rpos >= 0 and lpos < 0:
        lpos =  rpos - 200
        

    if cam_debug and lpos >= 0 and rpos <= Width:
        # draw lines
        frame = draw_lines(frame, left_lines)
        #frame = draw_lines(frame, right_lines)
        #frame = cv2.line(frame, (115, 117), (205, 117), (0,255,255), 2)

        # draw rectangle
        frame = draw_rectangle(frame, lpos, rpos, offset=Offset)
        frame = cv2.polylines(frame, [vertices5], True, (0,255,0),2)
        #frame = cv2.rectangle(frame, (0, Offset), (Width, Offset+Gap), (0, 255, 0), 2)

    img = frame

    return lpos, rpos, True

def draw_steer(steer_angle):
    global Width, Height, img, obstacle
    #img = cv_image

    arrow = cv2.imread('/home/pi/xycar_ws/src/auto_drive/src/steer_arrow.png')

    origin_Height = arrow.shape[0]
    origin_Width = arrow.shape[1]
    steer_wheel_center = origin_Height * 0.74
    arrow_Height = Height/2
    arrow_Width = (arrow_Height * 462)/728

    matrix = cv2.getRotationMatrix2D((origin_Width/2, steer_wheel_center), (-steer_angle) * 1.5, 0.7)    
    arrow = cv2.warpAffine(arrow, matrix, (origin_Width+60, origin_Height))
    arrow = cv2.resize(arrow, dsize=(arrow_Width, arrow_Height), interpolation=cv2.INTER_AREA)

    gray_arrow = cv2.cvtColor(arrow, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray_arrow, 1, 255, cv2.THRESH_BINARY_INV)

    arrow_roi = img[arrow_Height: Height, (Width/2 - arrow_Width/2) : (Width/2 + arrow_Width/2)]
    arrow_roi = cv2.add(arrow, arrow_roi, mask=mask)
    res = cv2.add(arrow_roi, arrow)
    img[(Height - arrow_Height): Height, (Width/2 - arrow_Width/2): (Width/2 + arrow_Width/2)] = res
     
    cv2.imshow('steer', img)
    

def start():
    global motor, drive_mode, obstacle, back_time, yellow_line
    global image, img
    global size
    global Width, Height
    global m
    
    rospy.init_node('auto_drive')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
    print ("---------- Xycar C1 HD v1.0 ----------")
    
    time.sleep(2)

    sq = rospy.Rate(30)

    t_check = time.time()
    obs_detected_time = 0

    uint8_image = np.uint8(image)
    yellow_line_detect(uint8_image)
    #numbering = 0

    #PID 제어 관련 변수
    dt = 0.1
    prev_pos = Width / 2
    curr_pos = Width / 2
    setpoint = Width / 2 # 기준이 되는 위치, 도로의 중앙으로 와야 함
    Kp = 0.65#- proportional gain
    Ki = 0.01#- integral gain
    Kd = 0.1#- derivative gain
    A0 = Kp + Ki*dt + Kd/dt
    A1 = -Kp - 2*Kd/dt
    A2 = Kd/dt
    error2 = 0 # e(t-2)
    error1 = 0 # e(t-1)
    error0 = 0 # e(t)
    output = 0 #output angle

    # 초기화
    setpoint = Width / 2
    error_integral = 0
    prev_error = 0
    time_prev = time.time()
    output_values = []
    position_values = []

    while not rospy.is_shutdown():
        
        while not image.size == (Width*Height*3):
            continue

        draw_img = image.copy()
        
        #장애물 인식
        #obstacle_img = image.copy()
        #detect_obstacle(obstacle_img)
        #obstacle_img = cv2.rectangle(obstacle_img,(obstacle[0],obstacle[1]),(obstacle[0]+obstacle[2],obstacle[1]+obstacle[3]),(255,0,0),2)
        #cv2.imshow('obstacle',obstacle_img)


        lpos, rpos, line_exists = process_image(draw_img)
        if line_exists:
            curr_pos = ((lpos + rpos) / 2)
            prev_pos = curr_pos
        else:
            curr_pos = prev_pos

        error2 = error1
        error1 = error0
        error0 = curr_pos - setpoint 
        output = output + (A0*error0) + (A1*error1) + (A2*error2)
        print('output' + str(output))
        angle_c = output # 조향각 보정 

        '''
        # 시간 경과 계산
        time_curr = time.time()
        dt = time_curr - time_prev
        time_prev = time_curr
        
        # 오차 계산
        error = setpoint - curr_pos
        
        # 누적 오차 계산
        error_integral += error * dt
        
        # 미분 오차 계산
        error_derivative = (error - prev_error) / dt
        
        # PID 제어 값 계산
        output = Kp * error + Ki * error_integral + Kd * error_derivative

        # 추가된 부분: 현재 위치와 제어 값 로깅
        position_values.append(curr_pos)
        output_values.append(output)

        # 그래프 그리기
        plt.clf()
        plt.plot(position_values, label='Position')
        plt.plot(output_values, label='Output')
        plt.xlabel('Time')
        plt.ylabel('Value')
        plt.legend()
        plt.pause(0.001)

        '''

        #애매하게 핸들꺽지말고 꺾을꺼면 확실하게 꺽기 위함
        if angle_c > 49 :
            angle_c = 50
        elif angle_c < -49 :
            angle_c = -50
        

        draw_steer(angle_c)
        #print(drive_mode)

        if drive_mode == 0:
            #drive(0, 0)
            #time.sleep(4)
            drive(angle_c, 22)

            #장애물 만나면 피하는 알고리즘, 장애물을 만나 차선을 바꾼 뒤 얼마동안 주행 후 다시 원래 차선으로 복귀 
            if time.time() - obs_detected_time > 0.7  and time.time() - obs_detected_time < 2.5:
                if yellow_line == 0:
                    if m > 2.2 :
                        continue
                    else :
                        drive(35,17)
                else :
                    if m > 2.2 :
                        continue
                    else :
                        drive(-35,17)
                        
        elif drive_mode == 1:
            if yellow_line == 0:
                if m > 2.5 :
                    drive(40,17)
                    time.sleep(0.2)
                    obs_detected_time = time.time()
                    drive_mode = 0
                else :
                    drive(-35,17)
            else:
                if m > 2.5 :
                    drive(-45,17)
                    time.sleep(0.4)
                    obs_detected_time = time.time()
                    drive_mode = 0
                else :
                    drive(45,17)
        
        elif drive_mode == 2:
            drive(0,10)
            time.sleep(0.2)
            drive(0,-30)
            drive(0,0)
            sys.exit(0)


        #f = open("../Log.txt",'a')
        #f.write(data)
        #f.close()
        cv2.waitKey(1)

    #out1.release()

if __name__ == '__main__':
    start()

