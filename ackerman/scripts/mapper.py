# Import necessary libraries
import rospy
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np

# Callback function to process lidar data
def lidar_callback(data):
    # Convert LaserScan message to numpy array
    ranges = np.array(data.ranges)
    
    # Example processing (you might need to adjust this based on your lidar model and scan characteristics)
    # Assuming ranges are distances from the lidar sensor
    # Convert polar coordinates to Cartesian coordinates
    # Example: Convert lidar data to 2D occupancy grid map
    resolution = 0.05  # adjust based on your map resolution
    map_size = 400     # adjust based on your map size
    map_img = np.zeros((map_size, map_size), dtype=np.uint8)

    for i in range(len(ranges)):
        angle = data.angle_min + i * data.angle_increment
        x = int(ranges[i] / resolution * np.cos(angle))
        y = int(ranges[i] / resolution * np.sin(angle))
        if 0 <= x < map_size and 0 <= y < map_size:
            map_img[x, y] = 255

    # Display or further process the map_img using OpenCV
    cv2.imshow("2D Map", map_img)
    cv2.waitKey(1)  # Adjust the wait key as needed

# Initialize the ROS node
rospy.init_node('lidar_mapper', anonymous=True)

# Subscribe to lidar data
rospy.Subscriber("/hokuyo", LaserScan, lidar_callback)

# Spin until node is shutdown
rospy.spin()
