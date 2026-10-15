#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt16

def publish_data():
    rospy.init_node('single_publisher_node', anonymous=True)
    pub = rospy.Publisher('v1', UInt16, queue_size=5)
    rospy.loginfo("Publishing data...")
    
    # Your data to be sent
    data = UInt16()
    data.data = 90  # Example data, change it as needed
    
    # Publish the data
    pub.publish(data)
    
    rospy.loginfo("robot stopped:", data)
    
if __name__ == '__main__':
    try:
        publish_data()
    except rospy.ROSInterruptException:
        pass