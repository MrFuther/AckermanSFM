#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class ImageSubscriber:
    def __init__(self, topic):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic, Image, self.image_callback)
        self.last_time = time.time()
        self.frame_count = 0

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        # Display the image
        cv2.imshow("Image", cv_image)
        
        # Calculate FPS
        self.frame_count += 1
        current_time = time.time()
        elapsed_time = current_time - self.last_time
        if elapsed_time >= 1.0:  # Update FPS every 1 second
            fps = self.frame_count / elapsed_time
            print(f"FPS: {fps:.2f}")
            self.last_time = current_time
            self.frame_count = 0

        # Check for the 'q' key to exit the program
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("User pressed 'q' key")

if __name__ == '__main__':
    rospy.init_node('image_subscriber_node', anonymous=True)
    topic_name = '/webcam/image_raw'  # Replace with your image topic
    image_subscriber = ImageSubscriber(topic_name)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

