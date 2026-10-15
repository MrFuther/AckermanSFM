#!/usr/bin/env python3
import cv2
import rospy
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def set_resolution(cap, width, height):
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

def main():
    rospy.init_node('webcam_publisher', anonymous=True)
    image_pub = rospy.Publisher('/webcam/image_raw', Image, queue_size=10)
    rate = rospy.Rate(30) 
    
    cap = cv2.VideoCapture(2)

    set_resolution(cap, 320, 240)
    bridge = CvBridge()
    start_time = time.time()
    frame_count = 0
    
    while True:
        ret, frame = cap.read()

        if not ret:
            print("Failed to grab frame")
            break

        frame_count += 1
        elapsed_time = time.time() - start_time
        fps = frame_count / elapsed_time
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow('Webcam', frame)

        ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
        image_pub.publish(ros_image)
        rate.sleep()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()