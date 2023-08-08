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

image = np.empty(shape=[0])
bridge = CvBridge()
stopline_count = -1
Width = 320
Height = 240
#stop_cascade = cv2.CascadeClassifier('/home/pi/xycar_ws/src/auto_drive/src/stop_cascade.xml')


def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    

def region_of_interest(img, vertices, color3=(255, 255, 255), color1=255):
    cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
    mask = np.zeros_like(img)
    if len(img.shape) > 2:
        color = color3
    else:
        color = color1
    cv2.fillPoly(mask, vertices, color)
    ROI_image = cv2.bitwise_and(img, mask)
    
    return ROI_image

def detect_stopline(x):
    global prev_time
    global size_r
    global recte
    global stopline_count
    frame5 = x.copy()
    img5 = frame5.copy()
    kernel_size = 5
    blur_frame5 = cv2.GaussianBlur(frame5, (kernel_size, kernel_size), 0)

    vertices5 = np.array([[
        (40, frame5.shape[0]),
        (frame5.shape[1]-250, frame5.shape[0]-80),
        (frame5.shape[1]-80, frame5.shape[0] - 80),
        (frame5.shape[1]-40, frame5.shape[0])
    ]], dtype=np.int32)

    roi5 = region_of_interest(blur_frame5, vertices5)

    roi5 = cv2.cvtColor(roi5, cv2.COLOR_BGR2RGB)
    
    #cv2.imshow('ROI5',roi5)

    gray5 = cv2.cvtColor(roi5, cv2.COLOR_BGR2GRAY)
    
    stop_gray = cv2.cvtColor(img5, cv2.COLOR_BGR2GRAY)
    #cv2.imshow('gray5',gray5)

    ret5, dest5 = cv2.threshold(gray5, 95, 255, cv2.THRESH_BINARY)

    low_threshold, high_threshold = 80,190
    edge_img5 = cv2.Canny(np.uint8(dest5), low_threshold, high_threshold)

    _, contours5, hierarchy5 = cv2.findContours(edge_img5, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(edge_img5, contours5, -1, (255,0,0))
    
    if contours5:
        stopline_info5 = [0, 0, 0, 0]
        for contour in contours5:
            epsilon5 = 0.01 * cv2.arcLength(contour, True)
            approx5 = cv2.approxPolyDP(contour, epsilon5, True)

            x, y, w, h = cv2.boundingRect(contour)
            
            #cv2.imshow('edge_img5', edge_img5)
            rect5 = cv2.minAreaRect(approx5)
            box5 = cv2.boxPoints(rect5)
            box5 = np.int0(box5)
            result5 = cv2.drawContours(frame5, [box5], 0, (0, 255, 0),1)
            

            rect_w = ((box5[0][0] - box5[1][0]) ** 2 + (box5[0][1] - box5[1][1]) ** 2) ** 0.5
            rect_h = ((box5[1][0] - box5[2][0]) ** 2 + (box5[1][1] - box5[2][1]) ** 2) ** 0.5

            recte = rect_w*rect_h

            if recte >= 1600.0:
                #print('STOPLINE Detected ' + str(recte) + ' ' + str(stopline_count))
                    
                if stopline_count == -1 :
                    stopline_count = stopline_count + 1
                    prev_time = time.time()
                
                elif stopline_count == 0 and (time.time()-prev_time)> 8:
                    stopline_count = stopline_count + 1
                    prev_time = time.time()
               
                elif stopline_count == 1 and (time.time()-prev_time) > 8:
                    size_r = rect_size()
                    size_r.size = recte
                    size.publish(size_r)
                    
                '''
                
                '''
            #print(stopline_count)

def start():
    global motor
    global image
    global size
    global Width, Height
    
    #print('running')
    rospy.init_node('stopline_detect')
    size = rospy.Publisher('stopline_detect', rect_size, queue_size = 1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)

    sq = rospy.Rate(30)

    while not rospy.is_shutdown():

        while not image.size == (Width*Height*3):
            continue
        draw_img = image.copy()
        detect_stopline(draw_img)
        sq.sleep()

if __name__ == '__main__':
    start()