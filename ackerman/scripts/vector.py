import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import matplotlib.pyplot as plt

class SFMObstacleAvoidance:
    def __init__(self):
        rospy.init_node('sfm_obstacle_avoidance')
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # SFM parameters
        self.repulsive_force_gain = 1.0
        self.attractive_force_gain = 1.0
        self.goal_x = 0.0
        self.goal_y = 0.0
        
        # Initialize plot
        self.fig, self.ax = plt.subplots()
        self.robot_position, = self.ax.plot([], [], 'bo', label='Robot')
        self.obstacles, = self.ax.plot([], [], 'ro', label='Obstacles')
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.legend()

        self.robot_pos_x = []
        self.robot_pos_y = []
        self.obstacle_pos_x = []
        self.obstacle_pos_y = []

    def scan_callback(self, scan):
        # Preprocess the scan data
        angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
        distances = np.array(scan.ranges)
        xs = distances * np.cos(angles)
        ys = distances * np.sin(angles)
        
        # Plot robot position
        self.robot_pos_x.append(0)
        self.robot_pos_y.append(0)
        self.robot_position.set_data(self.robot_pos_x, self.robot_pos_y)
        
        # Plot obstacles
        self.obstacle_pos_x = xs
        self.obstacle_pos_y = ys
        self.obstacles.set_data(self.obstacle_pos_x, self.obstacle_pos_y)
        
        # Calculate social forces
        repulsive_forces = self.calculate_repulsive_forces(xs, ys)
        attractive_force = self.calculate_attractive_force(xs[-1], ys[-1])  # Assume last point is the goal
        
        # Combine forces
        total_force_x = np.sum(repulsive_forces[:, 0]) + attractive_force[0]
        total_force_y = np.sum(repulsive_forces[:, 1]) + attractive_force[1]
        
        # Convert forces to velocity commands
        cmd_vel = Twist()
        cmd_vel.linear.x = total_force_x
        cmd_vel.linear.y = total_force_y
        self.cmd_vel_publisher.publish(cmd_vel)
        
        # Update plot
        plt.pause(0.001)
    
    def calculate_repulsive_forces(self, xs, ys):
        # Calculate repulsive forces from obstacles
        # For simplicity, let's assume a simple linear repulsive force based on distance
        distances = np.sqrt(xs**2 + ys**2)
        repulsive_forces_x = -self.repulsive_force_gain * xs / distances
        repulsive_forces_y = -self.repulsive_force_gain * ys / distances
        repulsive_forces = np.column_stack((repulsive_forces_x, repulsive_forces_y))
        return repulsive_forces
    
    def calculate_attractive_force(self, x, y):
        # Calculate attractive force towards the goal
        attractive_force_x = self.attractive_force_gain * (self.goal_x - x)
        attractive_force_y = self.attractive_force_gain * (self.goal_y - y)
        attractive_force = (attractive_force_x, attractive_force_y)
        return attractive_force
    
    def run(self):
        plt.show()
        rospy.spin()

if __name__ == '__main__':
    try:
        sfm_obstacle_avoidance = SFMObstacleAvoidance()
        sfm_obstacle_avoidance.run()
    except rospy.ROSInterruptException:
        pass
