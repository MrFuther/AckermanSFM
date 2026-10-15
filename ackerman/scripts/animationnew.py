#!/usr/bin/env python3

# import rospy
# from std_msgs.msg import Float32, Float32MultiArray, Int32
# from geometry_msgs.msg import Point
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
# import numpy as np
# import threading

# class LaserVisualizer:
#     def __init__(self):
#         """
#         Initializes the ROS node, data storage, and the Matplotlib figure for visualization.
#         Now includes adaptive ellipse sizing based on detected object type.
#         """
#         rospy.init_node('laser_visualizer', anonymous=True)

#         # --- Thread-safe Data Storage ---
#         self.data_lock = threading.Lock()
        
#         # Store raw distances and angles from subscribers
#         self.all_distances = []
#         self.all_angles = []
        
#         self.nearest_obstacle_xy = {'front': (None, None)}
        
#         # Data for the resultant vector
#         self.resultant_vector_xy = (0, 0)
#         self.resultant_magnitude = 0.0
#         self.resultant_angle = 0.0
        
#         # Data for the moving ellipse
#         self.ellipse_center_xy = (0, 0)
        
#         # ADAPTIVE ELLIPSE PARAMETERS
#         # Index: 0=Bus, 1=Car, 2=Motorcycle, 3=Person, 4=Truck, -1=Default
#         self.object_params = {
#             0: {'a': 1.2, 'b': 0.5, 'name': 'Bus', 'color': 'red'},
#             1: {'a': 1.1, 'b': 0.5, 'name': 'Car', 'color': 'orange'},
#             2: {'a': 0.8, 'b': 0.5, 'name': 'Motorcycle', 'color': 'yellow'},
#             3: {'a': 0.9, 'b': 0.5, 'name': 'Person', 'color': 'green'},
#             4: {'a': 1.2, 'b': 0.5, 'name': 'Truck', 'color': 'darkred'},
#             -1: {'a': 1.0, 'b': 0.5, 'name': 'Default', 'color': 'red'}
#         }
        
#         self.current_active_object = -1  # -1 = no object
#         self.ellipse_semi_axes = (1.0, 0.5)  # Default (a, b)
#         self.moving_ellipse_color = 'blue'

#         # --- Plot Setup ---
#         self.fig, self.ax = plt.subplots(figsize=(8, 8))
#         self.ax.set_aspect('equal')
#         self.ax.set_xlim(-4.1, 4.1)
#         self.ax.set_ylim(-4.1, 4.1)
#         self.ax.set_title('Adaptive SFM Visualization')
#         self.ax.grid(True)

#         # --- Plot Artists (Elements) ---
#         self.scan_points, = self.ax.plot([], [], 'k.', markersize=2, label='Scan Points', animated=True)
#         self.lidar_center, = self.ax.plot(0, 0, 'ko', label='LiDAR Center')
        
#         # Artists for nearest obstacle
#         self.nearest_obstacle_point, = self.ax.plot([], [], 'yo', markersize=8, label='Nearest Obstacle', animated=True)
#         self.line_to_obstacle, = self.ax.plot([], [], 'y-', animated=True)
#         self.obstacle_annotation = self.ax.text(0, 0, '', fontsize=9, animated=True)

#         # Artists for navigation vector
#         self.resultant_vector_line, = self.ax.plot([], [], 'g-', linewidth=2, label='Fnav Vector', animated=True)
        
#         # Artists for text information
#         self.info_text = self.ax.text(0.02, 0.98, '', fontsize=10, ha='left', va='top', 
#                                       transform=self.ax.transAxes, color='green', animated=True)

#         # Object type display
#         self.object_text = self.ax.text(0.02, 0.88, '', fontsize=10, ha='left', va='top',
#                                        transform=self.ax.transAxes, color='blue', animated=True)

#         # Artists for ellipses
#         self.static_ellipse, = self.ax.plot([], [], 'r--', label='Robot Proxemics')
#         self.moving_ellipse, = self.ax.plot([], [], 'b--', linewidth=2, label='Object Proxemics', animated=True)
        
#         self.setup_static_elements()
#         self.ax.legend(loc='upper right')

#         # --- Subscribers ---
#         rospy.Subscriber("/all_distances", Float32MultiArray, self.all_distances_callback)
#         rospy.Subscriber("/all_angles", Float32MultiArray, self.all_angles_callback)
#         rospy.Subscriber("/distances_minus150_to_150", Float32MultiArray, self.nearest_obstacle_callback)
#         rospy.Subscriber('/fnav', Point, self.fnav_callback)
#         rospy.Subscriber("/resultant_vector_x", Float32, self.vector_x_callback)
#         rospy.Subscriber("/resultant_vector_y", Float32, self.vector_y_callback)
#         rospy.Subscriber("/active_object_type", Int32, self.active_object_callback)

#         # --- Animation ---
#         self.ani = FuncAnimation(self.fig, self.update_plot, interval=33, blit=True, cache_frame_data=False)

#         # --- Shutdown Hooks ---
#         self.fig.canvas.mpl_connect('close_event', self.on_close)
#         rospy.on_shutdown(self.shutdown)

#     def setup_static_elements(self):
#         """Calculates and draws elements that do not change, like the static ellipse."""
#         h, k = 0, 0
#         a, b = 1.0, 0.5  # Robot ellipse (fixed)
#         angles = np.linspace(0, 2 * np.pi, 100)
#         x = h + a * np.cos(angles)
#         y = k + b * np.sin(angles)
#         self.static_ellipse.set_data(x, y)

#     # --- Callbacks ---
#     def all_distances_callback(self, msg):
#         """Callback to store the list of all measured distances."""
#         with self.data_lock:
#             self.all_distances = msg.data

#     def all_angles_callback(self, msg):
#         """Callback to store the list of all measured angles."""
#         with self.data_lock:
#             self.all_angles = msg.data

#     def nearest_obstacle_callback(self, msg):
#         """Receives front-facing obstacles, finds the nearest, and stores its coordinates."""
#         if not msg.data:
#             with self.data_lock:
#                 self.nearest_obstacle_xy['front'] = (None, None)
#                 self.ellipse_center_xy = (None, None) 
#             return

#         try:
#             data = np.array(msg.data).reshape(-1, 2)
#             min_dist_idx = np.argmin(data[:, 0])
#             nearest_dist, nearest_angle_deg = data[min_dist_idx]
            
#             angle_rad = np.deg2rad(nearest_angle_deg)
#             x = nearest_dist * np.cos(angle_rad)
#             y = nearest_dist * np.sin(angle_rad)
            
#             with self.data_lock:
#                 self.nearest_obstacle_xy['front'] = (x, y)
#                 self.ellipse_center_xy = (x, y)
#         except (ValueError, IndexError):
#             rospy.logwarn_throttle(5, "Received malformed /distances_minus150_to_150 data.")

#     def fnav_callback(self, msg):
#         """Callback for the FNAV magnitude and direction topic."""
#         with self.data_lock:
#             self.resultant_magnitude = msg.x
#             self.resultant_angle = msg.y

#     def vector_x_callback(self, data):
#         with self.data_lock:
#             self.resultant_vector_xy = (data.data, self.resultant_vector_xy[1])

#     def vector_y_callback(self, data):
#         with self.data_lock:
#             self.resultant_vector_xy = (self.resultant_vector_xy[0], data.data)

#     def active_object_callback(self, msg):
#         """
#         Callback untuk menerima tipe objek aktif dari SFM.
#         Mengubah ukuran ellipse sesuai tipe objek.
#         """
#         object_id = msg.data
        
#         with self.data_lock:
#             self.current_active_object = object_id
            
#             if object_id in self.object_params:
#                 params = self.object_params[object_id]
#                 self.ellipse_semi_axes = (params['a'], params['b'])
#                 self.moving_ellipse_color = params['color']
                
#                 if object_id >= 0:
#                     rospy.loginfo_throttle(2, f"Animation: Active object = {params['name']} "
#                                           f"(a={params['a']}, b={params['b']})")
#             else:
#                 # Default parameters
#                 params = self.object_params[-1]
#                 self.ellipse_semi_axes = (params['a'], params['b'])
#                 self.moving_ellipse_color = params['color']

#     def update_plot(self, frame):
#         """
#         This function is called by FuncAnimation to update the plot.
#         Now includes adaptive ellipse visualization.
#         """
#         with self.data_lock:
#             # Copy data to local variables
#             distances_local = self.all_distances
#             angles_local = self.all_angles
#             nearest_xy_local = self.nearest_obstacle_xy['front']
#             vector_xy_local = self.resultant_vector_xy
#             mag_local = self.resultant_magnitude
#             angle_local = self.resultant_angle
#             ellipse_center_local = self.ellipse_center_xy
#             ellipse_axes_local = self.ellipse_semi_axes
#             ellipse_color_local = self.moving_ellipse_color
#             active_object_local = self.current_active_object

#         # Update all scan points
#         if distances_local and angles_local and len(distances_local) == len(angles_local):
#             angles_rad = np.deg2rad(angles_local)
#             x_all = distances_local * np.cos(angles_rad)
#             y_all = distances_local * np.sin(angles_rad)
#             self.scan_points.set_data(x_all, y_all)
#         else:
#             self.scan_points.set_data([], [])

#         # Update nearest obstacle point, annotation, AND adaptive moving ellipse
#         if nearest_xy_local[0] is not None:
#             x, y = nearest_xy_local
#             self.nearest_obstacle_point.set_data(x, y)
#             self.line_to_obstacle.set_data([0, x], [0, y])
            
#             dist = np.sqrt(x**2 + y**2)
#             self.obstacle_annotation.set_position((x + 0.1, y + 0.1))
#             self.obstacle_annotation.set_text(f'Dist: {dist:.2f}m')

#             # Draw ADAPTIVE moving ellipse with updated size
#             h, k = ellipse_center_local
#             a, b = ellipse_axes_local
#             angles = np.linspace(0, 2 * np.pi, 100)
#             ellipse_x = h + a * np.cos(angles)
#             ellipse_y = k + b * np.sin(angles)
#             self.moving_ellipse.set_data(ellipse_x, ellipse_y)
#             self.moving_ellipse.set_color(ellipse_color_local)

#         else:
#             # Clear all obstacle-related artists when none is detected
#             self.nearest_obstacle_point.set_data([], [])
#             self.line_to_obstacle.set_data([], [])
#             self.obstacle_annotation.set_text('')
#             self.moving_ellipse.set_data([], [])

#         # Update resultant vector line
#         self.resultant_vector_line.set_data([0, vector_xy_local[0]], [0, vector_xy_local[1]])

#         # Update info text
#         info_str = f'Fnav: {mag_local:.2f} N\nRho: {angle_local:.2f}°'
#         self.info_text.set_text(info_str)

#         # Update object type text
#         if active_object_local >= 0 and active_object_local in self.object_params:
#             obj_name = self.object_params[active_object_local]['name']
#             obj_str = f'Active: {obj_name}\na={ellipse_axes_local[0]:.1f}, b={ellipse_axes_local[1]:.1f}'
#             self.object_text.set_text(obj_str)
#             self.object_text.set_color(ellipse_color_local)
#         else:
#             self.object_text.set_text('No Active Object')
#             self.object_text.set_color('gray')
        
#         # Return all artists that were modified for blitting
#         return (self.scan_points, self.nearest_obstacle_point, self.line_to_obstacle, 
#                 self.obstacle_annotation, self.resultant_vector_line, self.info_text,
#                 self.moving_ellipse, self.object_text)

#     def on_close(self, event):
#         """Shuts down the ROS node when the plot window is closed."""
#         rospy.signal_shutdown('Window closed')

#     def shutdown(self):
#         """Gracefully closes the plot figure on ROS shutdown."""
#         rospy.loginfo("Shutting down visualizer...")
#         plt.close(self.fig)

#     def run(self):
#         """Starts the Matplotlib event loop."""
#         plt.show()

# if __name__ == '__main__':
#     try:
#         visualizer = LaserVisualizer()
#         visualizer.run()
#     except rospy.ROSInterruptException:
#         pass

#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Float32MultiArray, Int32, Float64
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import threading

class LaserVisualizer:
    def __init__(self):
        rospy.init_node('laser_visualizer', anonymous=True)

        self.data_lock = threading.Lock()
        
        self.all_distances = []
        self.all_angles = []
        self.nearest_obstacle_xy = {'front': (None, None)}
        
        # Data untuk Vektor Gaya SFM
        self.f_goal_x = 0.0
        self.f_lane_y = 0.0
        self.f_obs_x = 0.0
        self.f_obs_y = 0.0
        self.FORCE_SCALE = 0.06  # Skala visual: Gaya 50 N akan digambar sepanjang 3.0 di layar
        
        self.resultant_magnitude = 0.0
        self.resultant_angle = 0.0
        self.ellipse_center_xy = (0, 0)
        
        # Parameter Ellipse Adaptif
        self.object_params = {
            0: {'a': 1.2, 'b': 0.5, 'name': 'Bus', 'color': 'red'},
            1: {'a': 1.1, 'b': 0.5, 'name': 'Car', 'color': 'orange'},
            2: {'a': 0.8, 'b': 0.5, 'name': 'Motorcycle', 'color': 'yellow'},
            3: {'a': 0.9, 'b': 0.5, 'name': 'Person', 'color': 'green'},
            4: {'a': 1.2, 'b': 0.5, 'name': 'Truck', 'color': 'darkred'},
            -1: {'a': 1.0, 'b': 0.5, 'name': 'Default', 'color': 'red'}
        }
        
        self.current_active_object = -1
        self.ellipse_semi_axes = (1.0, 0.5)
        self.moving_ellipse_color = 'blue'
        self.current_v0 = 1.0

        # Setup Plot
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-4.1, 4.1)
        self.ax.set_ylim(-4.1, 4.1)
        self.ax.set_title('Coupled SFM Vector Visualization')
        self.ax.grid(True)

        # Plot Artists
        self.scan_points, = self.ax.plot([], [], 'k.', markersize=2, label='Scan Points', animated=True)
        self.lidar_center, = self.ax.plot(0, 0, 'ko', label='Robot/LiDAR')
        
        self.nearest_obstacle_point, = self.ax.plot([], [], 'yo', markersize=8, label='Nearest Obstacle', animated=True)
        self.line_to_obstacle, = self.ax.plot([], [], 'y-', animated=True)
        self.obstacle_annotation = self.ax.text(0, 0, '', fontsize=9, animated=True)

        # ARTISTS VEKTOR GAYA BARU
        self.force_goal_line, = self.ax.plot([], [], 'c--', linewidth=2, label='F_goal (Maju)', animated=True)
        self.force_lane_line, = self.ax.plot([], [], 'm--', linewidth=2, label='F_lane (Setir)', animated=True)
        self.force_obs_line,  = self.ax.plot([], [], 'r--', linewidth=2, label='F_obs (Tolak)', animated=True)
        self.force_total_line,= self.ax.plot([], [], 'g-',  linewidth=3, label='F_total (Resultan)', animated=True)
        
        # Teks Info
        self.info_text = self.ax.text(0.02, 0.98, '', fontsize=10, ha='left', va='top', 
                                      transform=self.ax.transAxes, color='green', animated=True)
        self.object_text = self.ax.text(0.02, 0.88, '', fontsize=10, ha='left', va='top',
                                       transform=self.ax.transAxes, color='blue', animated=True)

        # Ellipse
        self.static_ellipse, = self.ax.plot([], [], 'r--', alpha=0.5, label='Robot Proxemics')
        self.moving_ellipse, = self.ax.plot([], [], 'b--', linewidth=2, label='Object Proxemics', animated=True)
        
        self.setup_static_elements()
        self.ax.legend(loc='upper right')

        # Subscribers
        rospy.Subscriber("/all_distances", Float32MultiArray, self.all_distances_callback)
        rospy.Subscriber("/all_angles", Float32MultiArray, self.all_angles_callback)
        rospy.Subscriber("/distances_minus150_to_150", Float32MultiArray, self.nearest_obstacle_callback)
        rospy.Subscriber('/fnav', Point, self.fnav_callback)
        rospy.Subscriber('/rho', Point, self.rho_callback)
        rospy.Subscriber("/active_object_type", Int32, self.active_object_callback)
        
        # Subscriber Baru untuk Gaya SFM
        rospy.Subscriber("/force_components", Float32MultiArray, self.forces_callback)
        rospy.Subscriber("/v0_target", Float64, self.v0_callback)

        self.ani = FuncAnimation(self.fig, self.update_plot, interval=33, blit=True, cache_frame_data=False)
        self.fig.canvas.mpl_connect('close_event', self.on_close)
        rospy.on_shutdown(self.shutdown)

    def setup_static_elements(self):
        h, k = 0, 0
        a, b = 1.0, 0.5
        angles = np.linspace(0, 2 * np.pi, 100)
        x = h + a * np.cos(angles)
        y = k + b * np.sin(angles)
        self.static_ellipse.set_data(x, y)

    def forces_callback(self, msg):
        if len(msg.data) >= 4:
            with self.data_lock:
                self.f_goal_x = msg.data[0]
                self.f_lane_y = msg.data[1]
                self.f_obs_x = msg.data[2]
                self.f_obs_y = msg.data[3]

    def v0_callback(self, msg):
        with self.data_lock:
            self.current_v0 = msg.data

    def all_distances_callback(self, msg):
        with self.data_lock:
            self.all_distances = msg.data

    def all_angles_callback(self, msg):
        with self.data_lock:
            self.all_angles = msg.data

    def nearest_obstacle_callback(self, msg):
        if not msg.data:
            with self.data_lock:
                self.nearest_obstacle_xy['front'] = (None, None)
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
            pass

    def fnav_callback(self, msg):
        with self.data_lock:
            self.resultant_magnitude = msg.x

    def rho_callback(self, msg):
        with self.data_lock:
            self.resultant_angle = msg.x

    def active_object_callback(self, msg):
        object_id = msg.data
        with self.data_lock:
            self.current_active_object = object_id
            if object_id in self.object_params:
                params = self.object_params[object_id]
                self.ellipse_semi_axes = (params['a'], params['b'])
                self.moving_ellipse_color = params['color']
            else:
                params = self.object_params[-1]
                self.ellipse_semi_axes = (params['a'], params['b'])
                self.moving_ellipse_color = params['color']

    def update_plot(self, frame):
        with self.data_lock:
            distances_local = self.all_distances
            angles_local = self.all_angles
            nearest_xy_local = self.nearest_obstacle_xy['front']
            mag_local = self.resultant_magnitude
            angle_local = self.resultant_angle
            ellipse_center_local = self.ellipse_center_xy
            ellipse_axes_local = self.ellipse_semi_axes
            ellipse_color_local = self.moving_ellipse_color
            active_object_local = self.current_active_object
            v0_local = self.current_v0
            
            f_gx, f_ly, f_ox, f_oy = self.f_goal_x, self.f_lane_y, self.f_obs_x, self.f_obs_y

        if distances_local and angles_local and len(distances_local) == len(angles_local):
            angles_rad = np.deg2rad(angles_local)
            x_all = distances_local * np.cos(angles_rad)
            y_all = distances_local * np.sin(angles_rad)
            self.scan_points.set_data(x_all, y_all)
        else:
            self.scan_points.set_data([], [])

        if nearest_xy_local[0] is not None:
            x, y = nearest_xy_local
            self.nearest_obstacle_point.set_data(x, y)
            self.line_to_obstacle.set_data([0, x], [0, y])
            
            dist = np.sqrt(x**2 + y**2)
            self.obstacle_annotation.set_position((x + 0.1, y + 0.1))
            self.obstacle_annotation.set_text(f'Dist: {dist:.2f}m')

            h, k = ellipse_center_local
            a, b = ellipse_axes_local
            angles = np.linspace(0, 2 * np.pi, 100)
            ellipse_x = h + a * np.cos(angles)
            ellipse_y = k + b * np.sin(angles)
            self.moving_ellipse.set_data(ellipse_x, ellipse_y)
            self.moving_ellipse.set_color(ellipse_color_local)
        else:
            self.nearest_obstacle_point.set_data([], [])
            self.line_to_obstacle.set_data([], [])
            self.obstacle_annotation.set_text('')
            self.moving_ellipse.set_data([], [])

        # Update dynamic robot proxemics based on v0
        v0_clamped = max(0.3, min(1.0, v0_local))
        robot_a = 0.5 + (v0_clamped - 0.3) * (1.0 - 0.5) / (1.0 - 0.3)
        robot_b = 0.3 + (v0_clamped - 0.3) * (0.5 - 0.3) / (1.0 - 0.3)
        
        angles = np.linspace(0, 2 * np.pi, 100)
        robot_x = robot_a * np.cos(angles)
        robot_y = robot_b * np.sin(angles)
        self.static_ellipse.set_data(robot_x, robot_y)

        # ==========================================
        # MENGGAMBAR VEKTOR GAYA (Dikalikan dengan skala visual)
        # ==========================================
        scale = self.FORCE_SCALE
        
        # 1. Gaya Maju (Cyan, menunjuk lurus ke atas/Sumbu X)
        self.force_goal_line.set_data([0, f_gx * scale], [0, 0])
        
        # 2. Gaya LKA (Magenta, menunjuk ke Kiri/Kanan Sumbu Y)
        self.force_lane_line.set_data([0, 0], [0, f_ly * scale])
        
        # 3. Gaya Tolak Objek (Merah, menunjuk berlawanan dari posisi objek)
        self.force_obs_line.set_data([0, f_ox * scale], [0, f_oy * scale])
        
        # 4. Gaya Resultan Total (Hijau, superposisi X dan Y)
        f_tx = f_gx + f_ox
        f_ty = f_ly + f_oy
        self.force_total_line.set_data([0, f_tx * scale], [0, f_ty * scale])

        # Teks Info
        info_str = f'Output Motor => Rem (Fnav): {mag_local:.1f} | Setir (Rho): {angle_local:.1f}°'
        self.info_text.set_text(info_str)

        if active_object_local >= 0 and active_object_local in self.object_params:
            obj_name = self.object_params[active_object_local]['name']
            obj_str = f'Active: {obj_name}\na={ellipse_axes_local[0]:.1f}, b={ellipse_axes_local[1]:.1f}'
            self.object_text.set_text(obj_str)
            self.object_text.set_color(ellipse_color_local)
        else:
            self.object_text.set_text('No Active Object')
            self.object_text.set_color('gray')
        
        return (self.scan_points, self.nearest_obstacle_point, self.line_to_obstacle, 
                self.obstacle_annotation, self.force_goal_line, self.force_lane_line, 
                self.force_obs_line, self.force_total_line, self.info_text,
                self.moving_ellipse, self.object_text, self.static_ellipse)

    def on_close(self, event):
        rospy.signal_shutdown('Window closed')

    def shutdown(self):
        plt.close(self.fig)

    def run(self):
        plt.show()

if __name__ == '__main__':
    try:
        visualizer = LaserVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass