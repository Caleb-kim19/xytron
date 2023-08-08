#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, time
import numpy as np
import cv2, math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from std_msgs.msg import Int32MultiArray
import signal
import sys
import os


def signal_handler(sig, frame):#close exit kill switch
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

bridge = CvBridge()
sub_f = 0
time_c = time.time()

image = None
ultra_msg = None

Width = 320
Height = 240
Offset = 170 #sanghansun
Gap = 60 #hahansun 160+40
Cnt = 0


def img_callback(data):
    global image
    global time_c
    global sub_f

    sub_f += 1
    if time.time() - time_c > 1:
        #print("pub fps :", sub_f)
        time_c = time.time()
        sub_f = 0

    try:
        image = bridge.imgmsg_to_cv2(data, "bgr8")
    except Exception as e:
        print(e)

def ultra_callback(data):
    global ultra_msg
    ultra_msg = data.data

def drive(angle, speed):
    global motor_pub
    motor_msg = xycar_motor()
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor_pub.publish(motor_msg)

# 나누기 함수 추가
def divide_left_right(lines):
    left_lines = []
    right_lines = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        slope = (y2 - y1) / (x2 - x1 + 0.00001)
        if slope > 0:
            right_lines.append(line)
        else:
            left_lines.append(line)
    return left_lines, right_lines

# 기존 함수 수정
def get_line_pos(frame, lines, left=True):
    pos = []
    count = 0
    for line in lines:
        x1, y1, x2, y2 = line[0]
        if left:
            if x2 < 150:
                count += 1
                pos.extend([x1, y1, x2, y2])
        else:
            if x1 > 170:
                count += 1
                pos.extend([x1, y1, x2, y2])
    if count > 0:
        pos = np.array(pos).reshape(count, 2, 2)
        if len(pos) > 0:  # Add a check to ensure pos is not empty
            pos = np.mean(pos, axis=0, dtype=np.int32)
            x1, y1 = pos[0]
            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 5)
            return x1
    return 0  # Return 0 when no valid position is found


# 기존 함수 수정
def process_image(frame, Width, Offset, Gap, cam_debug):
    global Width
    global Offset, Gap
    global cam, cam_debug, img
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    Offset = 200
    Gap = 40

    roi = gray[Offset: Offset + Gap, 0: Width]

    kernel_size = 5
    standard_deviation_x = 3
    blur_gray = cv2.GaussianBlur(roi, (kernel_size, kernel_size), standard_deviation_x)

    low_threshold = 170
    high_threshold = 200
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold, kernel_size)

    all_lines = cv2.HoughLinesP(edge_img, 0.7, math.pi / 180, 8, 28, 2)

    if cam_debug:
        cv2.imshow('calibration', frame)

    left_lines, right_lines = divide_left_right(all_lines)

    if len(left_lines) == 0:
        left_lines = []
    if len(right_lines) == 0:
        right_lines = []

    lpos = get_line_pos(frame, left_lines, left=True)
    rpos = get_line_pos(frame, right_lines, left=False)

    if cam_debug:
        frame = draw_lines(frame, left_lines)
        frame = draw_lines(frame, right_lines)

    return lpos, rpos, len(all_lines)


# 기존 함수 수정
def draw_lines(frame, lines):
    for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1 + 0.00001)
            if slope > 0:
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Green color for right lanes
            else:
                cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Blue color for left lanes
    return frame

# 기존 함수 수정
def draw_steer(steer_angle, img, Width, Height):
    global Width, Height, img
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

# 기존 함수 수정
def pid_angle(ITerm, error, b_angle, b_error, Cnt):
    angle = 0
    Kp = 0.5 #0.5 good
    Ki = 0.0001 #0.0001 good #0.0002
    Kd = 1.0 #1.0 good #2.0
    dt = 1

    PTerm = Kp * error
    ITerm += Ki * error * dt
    derror = error - b_error
    DTerm = Kd * (derror / dt)
    angle = PTerm + ITerm + DTerm

    return angle, ITerm

def stop(all_lines, flag, line_count):
    line_len=all_lines
    print("all_lines",line_len)
    
    if (line_count == 1) and (line_len > 49): #출발후 1바퀴 돌고-> 1번째 바퀴 완주전 2번-> 2번째 바퀴 인식하고 정지
        flag=0
        line_count = 2
        return line_count, flag
    
    if (line_len > 49): #출발후  1바퀴 돌고
        flag = 1
        print("flag up")
        print("flag up")
        
    if(line_len < 43) and (flag==1): #출발후 1바퀴 돌고-> 1번째 바퀴 완주전 2번
        line_count += 1
        flag = 0
        print("count up")

    return line_count, flag

def start():
    global motor_pub
    rospy.init_node('auto_drive')
    motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, ultra_callback)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)

    Width = 320
    Height = 240
    Offset = 170
    Gap = 60
    cam_debug = True

    t_check = time.time()
    f_n = 0
    p_angle = 0
    flag = 0
    line_count = 0
    b_angle = 0
    b_error = 0
    ITerm = 0
    avoid_time = time.time() + 3.8
    Cnt = 0
    turn_right = time.time()
    stop_time = time.time() + 10.5

    while not rospy.is_shutdown():
        rospy.Rate(100)
        while image is None:
            continue
        
        f_n += 1
        if (time.time() - t_check) > 1:
            t_check = time.time()
            f_n = 0

        draw_img = image.copy()
        
        lpos, rpos, len_all_lines = process_image(draw_img, Width, Offset, Gap, cam_debug)

        if time.time() > stop_time:
            line_count, flag = stop(len_all_lines, flag, line_count)

        if (line_count == 2):
            drive(0, -16)
            line_count = 0

        center = ((lpos + rpos) / 2)
        original_angle = -((Width / 2 - center))
        error = (center - Width / 2)
        angle, ITerm = pid_angle(ITerm, error, b_angle, b_error, Cnt)

        if lpos == 0 and rpos == 320:
            angle = 70
            drive(angle, 20)
        else:
            if time.time() > avoid_time and Cnt == 0:
                Cnt = 1

            if (ultra_msg[2] < 75 or ultra_msg[3] < 60) and Cnt == 1:
                Cnt = 2

                # 여기서 피해야 할 동작을 구현 (경로 변경 등)
                # 구현 완료 후에 Cnt를 증가시키면 됨
                # 예) Cnt = 3

            if ultra_msg[3] > 100 and Cnt == 2 and time.time() > turn_right:
                # 돌아서 다시 라인에 들어가는 동작 구현
                Cnt = 3

            if Cnt == 3:
                # 피해야 할 동작이 끝난 후 원래의 주행 동작을 수행
                ang = angle * 0.8
                drive(angle, 20)
            else:
                drive(angle, 20)

        steer_angle = angle * 0.4
        draw_steer(steer_angle, draw_img, Width, Height)

        cv2.waitKey(1)
        b_angle = angle
        b_error = error

if __name__ == '__main__':
    start()
