#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>
#include <cmath>
#include <algorithm>

// Jarak maksimum antar titik agar dianggap dalam satu cluster (meter)
const double CLUSTER_DISTANCE_THRESHOLD = 0.25; 
// Ukuran minimum sebuah cluster agar dianggap sebagai objek valid
const int MIN_CLUSTER_SIZE = 3; 
// Sudut area depan untuk seleksi target (derajat)
const double TARGET_CONE_ANGLE = 30.0; 
// Jarak maksimum pemrosesan LiDAR (meter)
const double MAX_LIDAR_DISTANCE = 5.0; 

// Publisher global
ros::Publisher target_pub;
ros::Publisher obstacles_pub;

// Struktur untuk menyimpan informasi titik
struct PointInfo {
    double x;
    double y;
    double range;
    double angle; // dalam radian
};

// Fungsi untuk menghitung jarak antara dua titik
double euclidean_distance(const PointInfo& a, const PointInfo& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    std::vector<PointInfo> points;
    
    // Langkah 1: Konversi data LaserScan dari Polar ke Kartesius (X, Y)
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float range = msg->ranges[i];
        if (std::isinf(range) || std::isnan(range) || range > MAX_LIDAR_DISTANCE) {
            continue;
        }

        float angle = msg->angle_min + i * msg->angle_increment;
        
        PointInfo p;
        p.x = range * std::cos(angle);
        p.y = range * std::sin(angle);
        p.range = range;
        p.angle = angle;
        points.push_back(p);
    }

    // Langkah 2: Clustering (Pengelompokan Objek)
    std::vector<std::vector<PointInfo>> clusters;
    std::vector<bool> processed(points.size(), false);

    for (size_t i = 0; i < points.size(); ++i) {
        if (processed[i]) {
            continue;
        }

        std::vector<PointInfo> current_cluster;
        std::vector<size_t> queue;
        queue.push_back(i);
        processed[i] = true;

        int head = 0;
        while(head < queue.size()){
            size_t current_point_idx = queue[head++];
            current_cluster.push_back(points[current_point_idx]);

            for (size_t j = 0; j < points.size(); ++j) {
                if (!processed[j] && euclidean_distance(points[current_point_idx], points[j]) < CLUSTER_DISTANCE_THRESHOLD) {
                    processed[j] = true;
                    queue.push_back(j);
                }
            }
        }
        
        if (current_cluster.size() >= MIN_CLUSTER_SIZE) {
            clusters.push_back(current_cluster);
        }
    }

    // Langkah 3: Hitung Titik Pusat Cluster dan Pilih Target
    int target_idx = -1;
    double min_target_dist = 1e9;
    std::vector<PointInfo> object_centroids;

    for (const auto& cluster : clusters) {
        double sum_x = 0, sum_y = 0;
        for (const auto& point : cluster) {
            sum_x += point.x;
            sum_y += point.y;
        }
        PointInfo centroid;
        centroid.x = sum_x / cluster.size();
        centroid.y = sum_y / cluster.size();
        centroid.range = std::sqrt(std::pow(centroid.x, 2) + std::pow(centroid.y, 2));
        centroid.angle = std::atan2(centroid.y, centroid.x);
        object_centroids.push_back(centroid);
    }

    // Logika seleksi target: cari objek terdekat di dalam kerucut depan
    for (size_t i = 0; i < object_centroids.size(); ++i) {
        double angle_deg = object_centroids[i].angle * 180.0 / M_PI;
        if (std::abs(angle_deg) < TARGET_CONE_ANGLE) {
            if (object_centroids[i].range < min_target_dist) {
                min_target_dist = object_centroids[i].range;
                target_idx = i;
            }
        }
    }

    // Langkah 4: Publikasikan Target dan Rintangan Lainnya
    std_msgs::Float32MultiArray obstacles_msg;

    for (size_t i = 0; i < object_centroids.size(); ++i) {
        if (i == target_idx) {
            // Ini adalah target, publikasikan ke topik /target_object
            geometry_msgs::Point target_msg;
            target_msg.x = object_centroids[i].range; // Jarak
            target_msg.y = object_centroids[i].angle * 180.0 / M_PI; // Sudut dalam derajat
            target_msg.z = 0;
            target_pub.publish(target_msg);
        } else {
            // Ini adalah rintangan lain, tambahkan ke pesan /other_obstacles
            obstacles_msg.data.push_back(object_centroids[i].range);
            obstacles_msg.data.push_back(object_centroids[i].angle * 180.0 / M_PI);
        }
    }
    obstacles_pub.publish(obstacles_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_tracker_node");
    ros::NodeHandle nh;

    // Definisikan publisher
    target_pub = nh.advertise<geometry_msgs::Point>("/target_object", 10);
    obstacles_pub = nh.advertise<std_msgs::Float32MultiArray>("/other_obstacles", 10);

    // Definisikan subscriber
    ros::Subscriber scan_sub = nh.subscribe("/scan", 10, scanCallback);

    ROS_INFO("Object Tracker Node has started.");
    ros::spin();

    return 0;
}