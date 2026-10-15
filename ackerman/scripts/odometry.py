#!/usr/bin/env python3
import serial
import rospy
import time

# Define serial port settings
port = '/dev/blackpill'  # Change this to your serial port
baudrate = 115200  # Change this to match your baud rate

# Open serial port
ser = serial.Serial(port, baudrate)

def main():
    # Initialize the ROS node
    rospy.init_node('serial_reader_node', anonymous=True)

    try:
        while not rospy.is_shutdown():
            # Read data from serial port
            dummy = "1"
            ser.write(dummy.encode())
            data = ser.readline()
            
            # Display received data
            data = data.split(b';')
            
            pos = float(data[0])
            rpm = float(data[1])
            
            # Get the current ROS time
            current_time = rospy.Time.now()
            time_str = "{}.{}".format(current_time.secs, current_time.nsecs)
            
            # Print the data with ROS time
            rospy.loginfo("Time: %s, pos: %d, rpm: %d", time_str, int(pos), int(rpm))

    except KeyboardInterrupt:
        # Close the serial port on keyboard interrupt
        ser.close()
        rospy.loginfo("Serial port closed.")
        
if __name__ == '__main__':
    main()
