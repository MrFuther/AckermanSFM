#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
import numpy as np
from time import sleep

from std_msgs.msg import Int16
from std_msgs.msg import  Float64
from std_msgs.msg import  UInt16

def joystick_sensor_callback(msg):
    axes = msg.axes
    buttons = msg.buttons

    def _map(x, in_min, in_max, out_min, out_max):
        return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
    getLeftYAxis = _map(axes[1], -1.0, 1.0, 20.0, 160.0) # pwm
    getBackward = _map(getLeftYAxis, 255, 0, 90,20)
    getForward = _map(getLeftYAxis, 255, 510, 90, 170)


    setPoint = 2050
    servoDegree = 250
     
    getRightXAxis = _map(axes[3], -1.0, 1.0, 0.0, 510.0) #velocity sudut
    getRight = _map(getRightXAxis, 255, 0, setPoint, setPoint+servoDegree)
    getLeft = _map(getRightXAxis, 255, 510, setPoint, setPoint-servoDegree)
    
    pub1.publish(getLeftYAxis)
   # pub2.publish(getBackward)    

    pub3.publish(getRight)
    pub4.publish(getLeft)

    print("Y:", getLeftYAxis)
#    print("forward:", getForward)
 #   print("backward:", getBackward)
    print("left ", getLeft)
    # print("PWM2:", PWM2)
    # print("Buttons:", buttons)

if __name__ == '__main__': 
    rospy.init_node('joystick_sensor_subscriber', anonymous = True)
    rospy.Subscriber('/joy', Joy, joystick_sensor_callback)
    pub1 = rospy.Publisher('v1', UInt16, queue_size=5)
   # pub2 = rospy.Publisher('v2', Float64, queue_size=5)
    
    pub3 = rospy.Publisher('velocity1', Float64, queue_size=5)
    pub4 = rospy.Publisher('velocity2', Float64, queue_size=5)
    
    while not rospy.is_shutdown(): 
        pass