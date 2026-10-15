#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import cv2
import numpy as np

class GPSVisualizer:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('gps_visualizer', anonymous=True)
        
        # Subscribe to the GPS topics
        self.sub_x = rospy.Subscriber('/gps/x', Float32, self.callback_x)
        self.sub_y = rospy.Subscriber('/gps/y', Float32, self.callback_y)
        
        # Initialize variables to store GPS coordinates
        self.gps_x = 0.0
        self.gps_y = 0.0
        
        # Create a blank image for the 2D map
        self.map_size = 500  # Size of the 2D map
        self.map_img = np.zeros((self.map_size, self.map_size, 3), dtype=np.uint8)
        
        # Set the scale and origin for the map
        self.scale = 10.0  # Scale factor to fit coordinates within the map
        self.origin = (self.map_size // 2, self.map_size // 2)  # Center of the map

        # Start OpenCV window
        cv2.namedWindow("GPS 2D Map")

        # Keep the node running
        rospy.spin()

    def callback_x(self, data):
        self.gps_x = data.data
        self.update_map()

    def callback_y(self, data):
        self.gps_y = data.data
        self.update_map()

    def update_map(self):
        # Convert GPS coordinates to map coordinates
        map_x = int(self.origin[0] + self.gps_x * self.scale)
        map_y = int(self.origin[1] - self.gps_y * self.scale)  # Invert y-axis for correct map orientation
        
        # Draw a circle at the current GPS coordinates
        cv2.circle(self.map_img, (map_x, map_y), 2, (0, 255, 0), -1)
        
        # Display the updated map
        cv2.imshow("GPS 2D Map", self.map_img)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        GPSVisualizer()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
