#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point

import math
import numpy as np
import threading


# Social force
Fsos = 0.0
Fphy = 0.0
k = 1.0 #gain
# r = 0.0 #radius max jangkauan lidar
d = 0.0 
psi = 1.0 #jarak reaktif in m
front_degree = 180
a = 2.0
b = 1.0

v0 = 2
m = 2.6 #kg
dt = 0.1



steering_pub = rospy.Publisher('/rho', Point, queue_size=100)
magnitude_pub = rospy.Publisher('/fnav', Point, queue_size=100)

def calculate_coordinates(degree):
    # Convert degree to radians
    angle = np.deg2rad(degree)
    
    # Calculate r for the given degree using the ellipse formula
    r = (a * b) / np.sqrt((b * np.cos(angle))**2 + (a * np.sin(angle))**2)
    
    # Convert polar coordinates (r, theta) to Cartesian coordinates (x, y)
    x = r * np.cos(angle)
    y = r * np.sin(angle)
    
    # Calculate distance from the origin
    distance = np.sqrt(x**2 + y**2)    
    return distance



def force_social(distances, degree):
    r_values = calculate_coordinates(degree)
    Fsos = k * np.exp((r_values - distances) / psi)
    return Fsos

def force_physical(distances, degree):
    r_values = calculate_coordinates(degree)
    Fphy = k * (r_values - distances)
    return Fphy

def calculate_resultant_vector(angles, distances):
    angles_rad = np.radians(angles)
    x_components = distances * np.cos(angles_rad)
    y_components = distances * np.sin(angles_rad)
    
    resultant_x = np.sum(x_components)
    resultant_y = np.sum(y_components)

    resultant_magnitude = np.hypot(resultant_x, resultant_y)
    resultant_direction = np.degrees(np.arctan2(resultant_y, resultant_x))

    return resultant_x, resultant_y, resultant_magnitude, resultant_direction

def laser_callback(msg):
    ranges = msg.ranges
    angle_increment = msg.angle_increment
    angle_min = msg.angle_min

    distances_90_to_180 = []
    angles_90_to_180 = []

    distances_minus180_to_minus90 = []
    angles_minus180_to_minus90 = []

    for i, distance in enumerate(ranges):
        if distance > 3.0:
            continue
        
        angle = angle_min + i * angle_increment
        angle_degrees = math.degrees(angle)
        
        # if angle_degrees <= 0:
        #     angle_degrees += 180
        # elif angle_degrees >= 0:
        #     angle_degrees -= 180

        if 90 <= angle_degrees <= 180:
            distances_90_to_180.append((distance, angle_degrees))
        elif -180 <= angle_degrees <= -90:
            distances_minus180_to_minus90.append((distance, angle_degrees))

    nearest_90_to_180 = min(distances_90_to_180, key=lambda x: x[0], default=None)
    nearest_minus180_to_minus90 = min(distances_minus180_to_minus90, key=lambda x: x[0], default=None)

    if nearest_90_to_180:
        nearest_distance_90_to_180, nearest_angle_90_to_180 = nearest_90_to_180
        sosial = force_social(nearest_distance_90_to_180, nearest_angle_90_to_180)
        physical = force_physical(nearest_distance_90_to_180, nearest_angle_90_to_180)
        # r = calculate_coordinates(nearest_angle_90_to_180)
        # force_static = sosial + physical
        
        if(calculate_coordinates(nearest_angle_90_to_180) <= nearest_distance_90_to_180):
            force_static = 0.0
            # r_kanan = 0
        else:
            force_static = sosial + physical
            # r_kanan = calculate_coordinates(nearest_angle_90_to_180)
            
        angle_reverse=nearest_angle_90_to_180 * -1
        # print(f"r : {r:.2f} , angle: {nearest_angle_90_to_180:.2f} ")
        # print(f"kanan : {nearest_angle_90_to_180:.2f}, force: {nearest_distance_90_to_180:.2f} N")
        # print(f"r_kanan : {angle_reverse:.2f}, force: {force_static:.2f} N")
        
    
    if nearest_minus180_to_minus90:
        nearest_distance_minus180_to_minus90, nearest_angle_minus180_to_minus90 = nearest_minus180_to_minus90
        rsosial = force_social(nearest_distance_minus180_to_minus90, nearest_angle_minus180_to_minus90)
        rphysical = force_physical(nearest_distance_minus180_to_minus90, nearest_angle_minus180_to_minus90)
        # r_force_static = rsosial + rphysical
        if(calculate_coordinates(nearest_angle_minus180_to_minus90) <= nearest_distance_minus180_to_minus90):
            r_force_static = 0.0
        else:
            r_force_static = rsosial + rphysical

        r_angle_reverse = nearest_angle_minus180_to_minus90 * -1
        # print(f"angle : {nearest_angle_minus180_to_minus90:.2f} , r_angle: {r_angle_reverse:.2f} ")
        # print(f"kiri : {nearest_distance_minus180_to_minus90:.2f}, force: {r_force_static:.2f} N")
    
    distances = [2]
    angles = [180]

    if nearest_90_to_180:
        distances.append(force_static)
        angles.append(angle_reverse)
    if nearest_minus180_to_minus90:
        distances.append(r_force_static)
        angles.append(r_angle_reverse)
        
    # print("Dis :", distances)
    # print("Ang :", angles)

    resultant_x, resultant_y, magnitude, direction = calculate_resultant_vector(angles, distances)
    velocity = v0 + (magnitude/m)* dt
    # print("Resultant X :", resultant_x)
    # print("Resultant Y :", resultant_y)
    # print("Vel :", velocity)
    # print("Fnav :", magnitude)
    # print("rho :", direction)
    # print("====================================================")
    # print("")
    
    threading.Thread(target=publish_results, args=(resultant_x, resultant_y, magnitude, direction)).start()

def publish_results(resultant_x, resultant_y, velocity, direction):
    steering_msg = Point()
    steering_msg.x = direction
    steering_pub.publish(steering_msg)
    
    magnitude_msg = Point()
    magnitude_msg.x = velocity
    magnitude_pub.publish(magnitude_msg)
    
def laser_subscriber():
    rospy.init_node('laser_subscriber', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, laser_callback)
    

    rospy.spin()

if __name__ == '__main__':
    try:
        laser_subscriber()
    except rospy.ROSInterruptException:
        pass
