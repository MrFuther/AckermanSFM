#!/usr/bin/env python3

import rospy
import time
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt16, Float64
from geometry_msgs.msg import Point

v0 = 1.5
m = 2.6
dt = 0.1

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

def _map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def increment_positions(start, end, step):
    if start < end:
        step = abs(step)
    else:
        step = -abs(step)

    positions = []
    for pos in range(start, end + step, step):
        if (step < 0 and pos < end) or (step > 0 and pos > end):
            break
        positions.append(pos)

    return positions

def joystick_sensor_callback(msg):
    axes = msg.axes
    buttons = msg.buttons

    getLeftYAxis = _map(axes[1], -1.0, 1.0, 50.0, 130.0)  # pwm
    getBackward = _map(getLeftYAxis, 255, 0, 90, 20)
    getForward = _map(getLeftYAxis, 255, 510, 90, 170)

    setPoint = 2040
    servoDegree = 250

    getRightXAxis = _map(axes[3], -1.0, 1.0, 0.0, 510.0)  # velocity sudut
    getRight = _map(getRightXAxis, 255, 0, setPoint, setPoint + servoDegree)
    getLeft = _map(getRightXAxis, 255, 510, setPoint, setPoint - servoDegree)
    
    pub1.publish(getLeftYAxis)
    pub3.publish(getRight)
    pub4.publish(getLeft)

def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)
# Kp semakin besar semakin overreactive
# fine 1 = 1.0 0.02 0.1


# metode Zurich
# Kp=0.6, Ki=1.2, Kd=0.075


# sukses 
# Kp=0.6, Ki=0.01, Kd=0.075
# step = 100

# 0.5 ms
# 1 ms
# 1.5 ms 

# Kp=1.5, Ki=0.02, Kd=0.075
# 150




pid = PIDController(Kp=1.5, Ki=0.02, Kd=0.075)  # Tune Ki and Kd as needed

current_turn = 2050  # Initial turn value

def steering_callback(msg):
    global current_turn
    setPoint = 2050
    angle = msg.x
    step_size = 150
    # step mengecil, Kp naik
    
    # priset 1 : step 70 K = 1.0 / 1.2
    # presets 2 : step 100 K = 0.5

    if angle < -180 or angle > 180:
        rospy.logwarn("Angle out of range: %f", angle)
        return

    if angle >= -180 and angle <= -150:
        # feedback = pid.compute(-165, angle)  # Use PID to reduce overreactiveness
        # target_turn = clamp(setPoint - feedback, 1800, 2050)
        feedback = _map(angle, -160, -180, 250.0, 0)
        target_turn = setPoint - feedback
    elif angle >= -160 and angle <= -90:
        target_turn = 1800  # Full left
    elif angle > 150 and angle <= 180:
        # feedback = pid.compute(165, angle)  # Use PID to reduce overreactiveness
        # target_turn = clamp(setPoint + feedback, 2050, 2300)
        feedback = _map(angle, 160, 180, 250.0, 0)
        target_turn = setPoint + feedback
    elif angle > 90 and angle <= 160:
        target_turn = 2300  # Full right
    else:
        target_turn = setPoint
        
    # Ensure target_turn is within limits
    target_turn = clamp(target_turn, 1800, 2300)

    # Smooth transition to target_turn
    if current_turn < target_turn:
        current_turn = min(current_turn + step_size, target_turn)
    elif current_turn > target_turn:
        current_turn = max(current_turn - step_size, target_turn)

    print("angle:", angle, "turn:", current_turn)
    pub4.publish(current_turn)

def _velmap(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def distance_callback(msg):
    
    fnav = msg.x
    velocity = v0 + (fnav/m) * dt
    if velocity >= 3:
        velocity = 3
    velocity_now = _velmap(velocity, 0.3, 3, 100, 114)
    pub1.publish(velocity_now)
    # print("fnav:", fnav)
    # print("v:", velocity)
    print("v_now:", velocity_now)

if __name__ == '__main__':
    rospy.init_node('joystick_sensor_subscriber', anonymous=True)
    rospy.Subscriber('/joy', Joy, joystick_sensor_callback)
    rospy.Subscriber('/rho', Point, steering_callback)
    rospy.Subscriber('/fnav', Point, distance_callback)

    pub1 = rospy.Publisher('v1', UInt16, queue_size=100)
    pub3 = rospy.Publisher('velocity1', Float64, queue_size=100)
    pub4 = rospy.Publisher('velocity2', Float64, queue_size=100)
    
    rospy.spin()
