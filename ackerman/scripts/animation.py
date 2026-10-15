#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import threading

class LaserVisualizer:
    def __init__(self):
        """
        Initializes the ROS node, data storage, and the Matplotlib figure for visualization.
        Optimized to perform calculations in callbacks and use blitting for smooth animation.
        """
        rospy.init_node('laser_visualizer', anonymous=True)

        # --- Thread-safe Data Storage ---
        self.data_lock = threading.Lock()
        
        # Store raw distances and angles from subscribers
        self.all_distances = []
        self.all_angles = []
        
        self.nearest_obstacle_xy = {'front': (None, None)}
        
        # Data for the resultant vector
        self.resultant_vector_xy = (0, 0)
        self.resultant_magnitude = 0.0
        self.resultant_angle = 0.0
        
        # Data for the moving ellipse
        self.ellipse_center_xy = (0, 0)
        self.ellipse_semi_axes = (1.0, 0.5) # (a, b)

        # --- Plot Setup ---
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-4.1, 4.1)
        self.ax.set_ylim(-4.1, 4.1)
        self.ax.set_title('Visualization') # UPDATED Title
        self.ax.grid(True)

        # --- Plot Artists (Elements) ---
        # Set animated=True for all artists that will be updated for blitting
        self.scan_points, = self.ax.plot([], [], 'k.', markersize=2, label='Scan Points', animated=True)
        self.lidar_center, = self.ax.plot(0, 0, 'ko', label='LiDAR Center')
        
        # Artists for nearest obstacle
        self.nearest_obstacle_point, = self.ax.plot([], [], 'yo', markersize=8, label='Nearest Obstacle', animated=True)
        self.line_to_obstacle, = self.ax.plot([], [], 'y-', animated=True)
        self.obstacle_annotation = self.ax.text(0, 0, '', fontsize=9, animated=True)

        # Artists for navigation vector
        self.resultant_vector_line, = self.ax.plot([], [], 'g-', linewidth=2, label='Fnav Vector', animated=True)
        
        # Artists for text information
        self.info_text = self.ax.text(0.02, 0.98, '', fontsize=10, ha='left', va='top', 
                                      transform=self.ax.transAxes, color='green', animated=True)

        # Artists for ellipses
        self.static_ellipse, = self.ax.plot([], [], 'r--', label='Robot Proxemics') # UPDATED Label
        self.moving_ellipse, = self.ax.plot([], [], 'b--', label='Object Proxemics', animated=True) # UPDATED Label
        
        self.setup_static_elements()
        self.ax.legend(loc='upper right')

        # --- Subscribers ---
        rospy.Subscriber("/all_distances", Float32MultiArray, self.all_distances_callback)
        rospy.Subscriber("/all_angles", Float32MultiArray, self.all_angles_callback)
        rospy.Subscriber("/distances_minus150_to_150", Float32MultiArray, self.nearest_obstacle_callback)
        rospy.Subscriber('/fnav', Point, self.fnav_callback)
        rospy.Subscriber("/resultant_vector_x", Float32, self.vector_x_callback)
        rospy.Subscriber("/resultant_vector_y", Float32, self.vector_y_callback)

        # --- Animation ---
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=33, blit=True, cache_frame_data=False)

        # --- Shutdown Hooks ---
        self.fig.canvas.mpl_connect('close_event', self.on_close)
        rospy.on_shutdown(self.shutdown)

    def setup_static_elements(self):
        """Calculates and draws elements that do not change, like the static ellipse."""
        h, k = 0, 0
        a, b = self.ellipse_semi_axes
        angles = np.linspace(0, 2 * np.pi, 100)
        x = h + a * np.cos(angles)
        y = k + b * np.sin(angles)
        self.static_ellipse.set_data(x, y)

    # --- Optimized Callbacks ---
    def all_distances_callback(self, msg):
        """Callback to store the list of all measured distances."""
        with self.data_lock:
            self.all_distances = msg.data

    def all_angles_callback(self, msg):
        """Callback to store the list of all measured angles."""
        with self.data_lock:
            self.all_angles = msg.data

    def nearest_obstacle_callback(self, msg):
        """Receives front-facing obstacles, finds the nearest, and stores its coordinates."""
        if not msg.data:
            with self.data_lock:
                self.nearest_obstacle_xy['front'] = (None, None)
                # Also reset the ellipse center when no obstacles are present
                self.ellipse_center_xy = (None, None) 
            return

        try:
            data = np.array(msg.data).reshape(-1, 2)
            min_dist_idx = np.argmin(data[:, 0])
            nearest_dist, nearest_angle_deg = data[min_dist_idx]
            
            angle_rad = np.deg2rad(nearest_angle_deg)
            x = nearest_dist * np.cos(angle_rad)
            y = nearest_dist * np.sin(angle_rad)
            
            with self.data_lock:
                self.nearest_obstacle_xy['front'] = (x, y)
                self.ellipse_center_xy = (x, y)
        except (ValueError, IndexError):
            rospy.logwarn_throttle(5, "Received malformed /distances_minus150_to_150 data.")

    def fnav_callback(self, msg):
        """Callback for the FNAV magnitude and direction topic."""
        with self.data_lock:
            self.resultant_magnitude = msg.x
            self.resultant_angle = msg.y

    def vector_x_callback(self, data):
        with self.data_lock:
            self.resultant_vector_xy = (data.data, self.resultant_vector_xy[1])

    def vector_y_callback(self, data):
        with self.data_lock:
            self.resultant_vector_xy = (self.resultant_vector_xy[0], data.data)

    def update_plot(self, frame):
        """
        This function is called by FuncAnimation to update the plot.
        It now only reads pre-calculated data and updates the artists.
        """
        with self.data_lock:
            # Copy data to local variables to avoid holding lock during plotting
            distances_local = self.all_distances
            angles_local = self.all_angles
            nearest_xy_local = self.nearest_obstacle_xy['front']
            vector_xy_local = self.resultant_vector_xy
            mag_local = self.resultant_magnitude
            angle_local = self.resultant_angle
            ellipse_center_local = self.ellipse_center_xy

        # Update all scan points
        if distances_local and angles_local and len(distances_local) == len(angles_local):
            angles_rad = np.deg2rad(angles_local)
            x_all = distances_local * np.cos(angles_rad)
            y_all = distances_local * np.sin(angles_rad)
            self.scan_points.set_data(x_all, y_all)
        else:
            self.scan_points.set_data([], [])

        # Update nearest obstacle point, annotation, AND moving ellipse
        if nearest_xy_local[0] is not None:
            x, y = nearest_xy_local
            self.nearest_obstacle_point.set_data(x, y)
            self.line_to_obstacle.set_data([0, x], [0, y])
            
            dist = np.sqrt(x**2 + y**2)
            self.obstacle_annotation.set_position((x + 0.1, y + 0.1))
            self.obstacle_annotation.set_text(f'Dist: {dist:.2f}m')

            # --- FIXED: Only draw moving ellipse when an obstacle is present ---
            h, k = ellipse_center_local
            a, b = self.ellipse_semi_axes
            angles = np.linspace(0, 2 * np.pi, 100)
            ellipse_x = h + a * np.cos(angles)
            ellipse_y = k + b * np.sin(angles)
            self.moving_ellipse.set_data(ellipse_x, ellipse_y)

        else:
            # --- FIXED: Clear all obstacle-related artists when none is detected ---
            self.nearest_obstacle_point.set_data([], [])
            self.line_to_obstacle.set_data([], [])
            self.obstacle_annotation.set_text('')
            self.moving_ellipse.set_data([], []) # This line removes the blue ring

        # Update resultant vector line
        self.resultant_vector_line.set_data([0, vector_xy_local[0]], [0, vector_xy_local[1]])

        # Update info text
        info_str = f'Fnav: {mag_local:.2f} N\nRho: {angle_local:.2f}°'
        self.info_text.set_text(info_str)
        
        # Return all artists that were modified for blitting
        return (self.scan_points, self.nearest_obstacle_point, self.line_to_obstacle, 
                self.obstacle_annotation, self.resultant_vector_line, self.info_text,
                self.moving_ellipse)

    def on_close(self, event):
        """Shuts down the ROS node when the plot window is closed."""
        rospy.signal_shutdown('Window closed')

    def shutdown(self):
        """Gracefully closes the plot figure on ROS shutdown."""
        rospy.loginfo("Shutting down visualizer...")
        plt.close(self.fig)

    def run(self):
        """Starts the Matplotlib event loop. rospy.spin() is not needed."""
        plt.show()

if __name__ == '__main__':
    try:
        visualizer = LaserVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass