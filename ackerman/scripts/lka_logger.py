#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import csv
import time
import os
from collections import deque

class LKALogger:
    def __init__(self):
        rospy.init_node('lka_fps_logger', anonymous=True)
        
        # Buffer untuk menghitung rata-rata FPS (sliding window 10 sampel)
        self.timestamps = deque(maxlen=10)
        self.last_recv_time = None
        
        # Resolusi dinamis lokasi penyimpanan file log ke root workspace
        script_dir = os.path.dirname(os.path.abspath(__file__))
        workspace_dir = os.path.dirname(os.path.dirname(script_dir))
        
        self.filename = "lka_performance_log_{}.csv".format(time.strftime("%H_%M_%S_%d-%m-%Y"))
        self.filepath = os.path.join(workspace_dir, self.filename)
        
        try:
            self.csv_file = open(self.filepath, 'w')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['Timestamp', 'Relative_Time_Sec', 'Lane_Error', 'Instantaneous_FPS', 'Average_FPS_10_Frames'])
            rospy.loginfo("LKA Performance Logger aktif. Menyimpan data ke: %s", self.filepath)
        except Exception as e:
            rospy.logerr("Gagal membuat file log CSV di workspace: %s. Mengalihkan ke folder tmp...", str(e))
            self.filepath = os.path.join('/tmp', self.filename)
            self.csv_file = open(self.filepath, 'w')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['Timestamp', 'Relative_Time_Sec', 'Lane_Error', 'Instantaneous_FPS', 'Average_FPS_10_Frames'])
            rospy.loginfo("Fallback: Menyimpan data ke: %s", self.filepath)
            
        self.start_time = rospy.get_time()
        self.sub = rospy.Subscriber("/lane_error", Float32, self.callback)
        
    def callback(self, msg):
        current_time = rospy.get_time()
        rel_time = current_time - self.start_time
        lane_error = msg.data
        
        # Hitung FPS instan
        instant_fps = 0.0
        if self.last_recv_time is not None:
            dt = current_time - self.last_recv_time
            if dt > 0:
                instant_fps = 1.0 / dt
                
        self.last_recv_time = current_time
        self.timestamps.append(current_time)
        
        # Hitung FPS rata-rata dari 10 sampel terakhir
        avg_fps = 0.0
        if len(self.timestamps) >= 2:
            total_dt = self.timestamps[-1] - self.timestamps[0]
            if total_dt > 0:
                avg_fps = (len(self.timestamps) - 1) / total_dt
                
        timestamp_str = time.strftime("%H:%M:%S") + ".{:03d}".format(int((current_time % 1) * 1000))
        
        # Tulis baris CSV
        self.csv_writer.writerow([
            timestamp_str, 
            "{:.3f}".format(rel_time), 
            "{:.3f}".format(lane_error), 
            "{:.2f}".format(instant_fps), 
            "{:.2f}".format(avg_fps)
        ])
        self.csv_file.flush() # Paksa tulis ke disk agar tidak hilang jika di-force close
        
        # Cetak info secara berkala ke terminal (setiap 1 detik agar tidak spam)
        rospy.loginfo_throttle(1.0, "[LKA Log] Error: %6.2f | FPS: %5.2f Hz", lane_error, avg_fps)

    def cleanup(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
            rospy.loginfo("File log berhasil disimpan di: %s", self.filepath)

if __name__ == '__main__':
    try:
        logger = LKALogger()
        rospy.on_shutdown(logger.cleanup)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
