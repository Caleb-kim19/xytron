#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg, time
import numpy as np
import cv2, math

import signal
import sys
import os

from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion

def signal_handler(sig, frame):#close exit kill switch
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

rospy.init_node('best_driver')
rospy.Subscriber("/usb_cam/image_raw/", Image, cam_callback)
rospy.Subscriber('xycar_ultrasonic', Int32MultiArray, ultra_callback)
motor_publisher = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

image = np.empty(shape=[0])
bridge = CvBridge() 
def cam_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

ultra_msg = [0,0,0,0,0,0,0,0]
def ultra_callback(data):
    global ultra_msg
    ultra_msg = data.data

motor_msg = xycar_motor()
motor_publisher = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
def car_drive(angle, speed):
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor_publisher.publish(motor_msg)

def start():
    

if __name__ == '__main__':
    start()

