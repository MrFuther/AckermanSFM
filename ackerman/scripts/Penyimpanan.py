#Program cek speed
#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt16

class V1Publisher:
    def __init__(self):
        # Initialize the node
        rospy.init_node('v1_publisher', anonymous=True)
        
        # Publisher for v1
        self.pub_v1 = rospy.Publisher('v1', UInt16, queue_size=10)
        
        # Set the rate of publishing (e.g., 10 Hz)
        self.rate = rospy.Rate(10)
        
        # Initial v1 value
        self.v1_value = 85
        
        # Step size for incrementing/decrementing
        self.step_size = 1
        
        # Direction flag: True for incrementing, False for decrementing
        self.increment = True

    def publish_v1(self):
        while not rospy.is_shutdown():
            # Create a UInt16 message
            msg = UInt16()
            msg.data = self.v1_value
            
            # Publish the message
            self.pub_v1.publish(msg)
            
            # Log the published value (optional)
            rospy.loginfo(f"Published to v1: {msg.data}")
            
            # Update the v1 value
            if self.increment:
                self.v1_value += self.step_size
                if self.v1_value >= 105:
                    self.v1_value = 105
                    self.increment = False  # Switch to decrementing
            else:
                self.v1_value -= self.step_size
                if self.v1_value <= 85:
                    self.v1_value = 85
                    self.increment = True  # Switch to incrementing

            print(self.v1_value)
            
            # Sleep to maintain the desired publishing rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        # Create an instance of the V1Publisher class
        v1_publisher = V1Publisher()
        
        # Start publishing to v1
        v1_publisher.publish_v1()
    except rospy.ROSInterruptException:
        pass


#Penyimpanan program test steer
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

class VelocityPublisher:
    def __init__(self):
        # Initialize the node
        rospy.init_node('velocity2_publisher', anonymous=True)
        
        # Publisher for velocity2
        self.pub_velocity2 = rospy.Publisher('velocity2', Float64, queue_size=10)
        
        # Set the rate of publishing (e.g., 10 Hz)
        self.rate = rospy.Rate(10)
        
        # Initial velocity value
        self.velocity2_value = 1800.0
        
        # Step size for incrementing/decrementing
        self.step_size = 10
        
        # Direction flag: True for incrementing, False for decrementing
        self.increment = True

    def publish_velocity2(self):
        while not rospy.is_shutdown():
            # Create a Float64 message
            msg = Float64()
            msg.data = self.velocity2_value
            
            # Publish the message
            self.pub_velocity2.publish(msg)
            
            # Update the velocity value
            if self.increment:
                self.velocity2_value += self.step_size
                if self.velocity2_value >= 2300:
                    self.velocity2_value = 2300
                    self.increment = False  # Switch to decrementing
            else:
                self.velocity2_value -= self.step_size
                if self.velocity2_value <= 1800:
                    self.velocity2_value = 1800
                    self.increment = True  # Switch to incrementing
            #print(self.velocity2_value)
            # Sleep to maintain the desired publishing rate
            self.rate.sleep()                                                                                                                    

if __name__ == '__main__':
    try:
        # Create an instance of the VelocityPublisher class
        velocity_publisher = VelocityPublisher()
        
        # Start publishing to velocity2
        velocity_publisher.publish_velocity2()
    except rospy.ROSInterruptException:
        pass