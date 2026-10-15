#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Float32
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
from collections import deque

class MultiTopicPlotter:
    def __init__(self):
        """
        Initializes the ROS node, data storage, and the Matplotlib figure with three subplots.
        """
        rospy.init_node('multi_topic_plotter', anonymous=True)

        self.data_lock = threading.Lock()
        
        # --- Data Storage ---
        # Use deque for efficient appends and pops from both ends.
        self.start_time = rospy.Time.now().to_sec()
        self.time_window = 10.0  # Display window in seconds

        # Data for PID plot (Setpoint vs Actual)
        self.setpoint_times = deque()
        self.setpoint_values = deque()
        self.actual_times = deque()
        self.actual_values = deque()

        # Data for Nearest Distance plot
        self.distance_times = deque()
        self.distance_values = deque()

        # Data for Fuzzy Logic plot
        self.fuzzy_times = deque()
        self.fuzzy_values = deque()

        # Data for FNAV Magnitude plot
        self.magnitude_times = deque()
        self.magnitude_values = deque()

        # --- Plot Setup for 3 Subplots ---
        # Create a figure with 3 rows, 1 column of subplots. `sharex=True` links the x-axes.
        self.fig, self.axes = plt.subplots(4, 1, sharex=True, figsize=(10, 8))
        self.fig.tight_layout(pad=3.0) # Add padding between subplots

        # Plot 1: PID Response
        self.setpoint_line, = self.axes[0].plot([], [], 'r-', label='Setpoint', animated=True)
        self.actual_line, = self.axes[0].plot([], [], 'b-', label='Actual Speed', animated=True)
        self.axes[0].set_title('PID Response')
        self.axes[0].set_ylabel('Speed (m/s)')
        self.axes[0].set_xlabel('Time (s)') # X-label only on the bottom plot
        self.axes[0].legend()
        self.axes[0].grid(True)
        self.axes[0].set_ylim(-1, 2) # Set a fixed y-axis range for this plot

        # Plot 2: Nearest Distance
        self.distance_line, = self.axes[1].plot([], [], 'g-', label='Nearest Distance', animated=True)
        self.axes[1].set_title('Distance')
        self.axes[1].set_ylabel('Distance (m)')
        self.axes[1].set_xlabel('Time (s)') # X-label only on the bottom plot
        self.axes[1].legend()
        self.axes[1].grid(True)
        self.axes[1].set_ylim(0, 10) # Example y-axis range, adjust as needed

        # Plot 3: FNAV Magnitude
        self.magnitude_line, = self.axes[2].plot([], [], 'm-', label='Resultant Magnitude', animated=True)
        self.axes[2].set_title('Fnav (Repulsive Force)')
        self.axes[2].set_ylabel('Magnitude')
        self.axes[2].set_xlabel('Time (s)') # X-label only on the bottom plot
        self.axes[2].legend()
        self.axes[2].grid(True)
        self.axes[2].set_ylim(0, 100) # Example y-axis range, adjust as needed

        # Plot 4: Gain
        self.fuzzy_line, = self.axes[3].plot([], [], 'y-', label='Gain', animated=True)
        self.axes[3].set_title('Fuzzy Logic Out')
        self.axes[3].set_ylabel('Gain')
        self.axes[3].set_xlabel('Time (s)') # X-label only on the bottom plot
        self.axes[3].legend()
        self.axes[3].grid(True)
        self.axes[3].set_ylim(0, 1) # Example y-axis range, adjust as needed

        # --- Subscribers ---
        rospy.Subscriber("/velopub", Float64, self.setpoint_callback)
        rospy.Subscriber("/vehicle_speed", Float64, self.actual_callback)
        rospy.Subscriber("/near_distance", Float32, self.distance_callback)
        rospy.Subscriber("/fuzzy", Float32, self.fuzzy_callback)
        rospy.Subscriber("/fnav", Point, self.magnitude_callback)

        # --- Animation ---
        # Use blitting for high-performance animation updates.
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=30, blit=True, cache_frame_data=False)
        self.fig.canvas.mpl_connect('close_event', self.on_close)
        rospy.on_shutdown(self.shutdown)

    def setpoint_callback(self, msg):
        """Callback for the setpoint topic."""
        current_time = rospy.Time.now().to_sec() - self.start_time
        with self.data_lock:
            self.setpoint_times.append(current_time)
            self.setpoint_values.append(msg.data)

    def actual_callback(self, msg):
        """Callback for the actual vehicle speed topic."""
        current_time = rospy.Time.now().to_sec() - self.start_time
        with self.data_lock:
            self.actual_times.append(current_time)
            self.actual_values.append(msg.data)

    def fuzzy_callback(self, msg):
        """Callback for the nearest distance topic."""
        current_time = rospy.Time.now().to_sec() - self.start_time
        with self.data_lock:
            self.fuzzy_times.append(current_time)
            self.fuzzy_values.append(msg.data)

    def distance_callback(self, msg):
        """Callback for the nearest distance topic."""
        current_time = rospy.Time.now().to_sec() - self.start_time
        with self.data_lock:
            self.distance_times.append(current_time)
            self.distance_values.append(msg.data)

    def magnitude_callback(self, msg):
        """Callback for the FNAV magnitude topic."""
        current_time = rospy.Time.now().to_sec() - self.start_time
        with self.data_lock:
            self.magnitude_times.append(current_time)
            self.magnitude_values.append(msg.x) # As per your request, using msg.x

    def update_plot(self, frame):
        """
        This function is called by FuncAnimation to update the plot data.
        It trims old data and updates the line objects for all three subplots.
        """
        with self.data_lock:
            current_time = rospy.Time.now().to_sec() - self.start_time
            cutoff = current_time - self.time_window

            # Trim old data for all deques
            while self.setpoint_times and self.setpoint_times[0] < cutoff:
                self.setpoint_times.popleft()
                self.setpoint_values.popleft()

            while self.actual_times and self.actual_times[0] < cutoff:
                self.actual_times.popleft()
                self.actual_values.popleft()

            while self.distance_times and self.distance_times[0] < cutoff:
                self.distance_times.popleft()
                self.distance_values.popleft()

            while self.magnitude_times and self.magnitude_times[0] < cutoff:
                self.magnitude_times.popleft()
                self.magnitude_values.popleft()

            while self.fuzzy_times and self.fuzzy_times[0] < cutoff:
                self.fuzzy_times.popleft()
                self.fuzzy_values.popleft()

            # Set the new data for each line
            self.setpoint_line.set_data(self.setpoint_times, self.setpoint_values)
            self.actual_line.set_data(self.actual_times, self.actual_values)
            self.distance_line.set_data(self.distance_times, self.distance_values)
            self.magnitude_line.set_data(self.magnitude_times, self.magnitude_values)
            self.fuzzy_line.set_data(self.fuzzy_times, self.fuzzy_values)

        # Update the x-axis limits for the scrolling effect.
        # This applies to all subplots because sharex=True.
        self.axes[0].set_xlim(max(0, current_time - self.time_window), current_time)
        
        # Dynamically adjust y-limits for plots 2 and 3 if needed, or keep them fixed.
        # Example of auto-scaling for the distance plot:
        # if self.distance_values:
        #     min_val = min(self.distance_values)
        #     max_val = max(self.distance_values)
        #     self.axes[1].set_ylim(min_val - 1, max_val + 1)
        
        # Return all the artists that have been updated for blitting
        return self.setpoint_line, self.actual_line, self.distance_line, self.magnitude_line, self.fuzzy_line

    def on_close(self, event):
        """Shuts down the ROS node when the plot window is closed."""
        rospy.signal_shutdown("Plot window closed")

    def shutdown(self):
        """Gracefully closes the plot figure on ROS shutdown."""
        rospy.loginfo("Shutting down plotter node...")
        if self.fig.canvas.manager is not None:
            plt.close(self.fig)

    def run(self):
        """Starts the Matplotlib event loop."""
        plt.show()

if __name__ == '__main__':
    try:
        plotter = MultiTopicPlotter()
        plotter.run()
    except rospy.ROSInterruptException:
        pass
