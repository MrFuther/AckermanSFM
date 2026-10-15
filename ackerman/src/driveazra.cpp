#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <algorithm>

/*IF YOU WANT TO TUNE THE PID JUST UNCOMMENT THE DEFINE BELOW DAWG*/
// #define PID_TUNE
/* ^^^^^^^^^^^^^^^^ TS*/

// Publishers
ros::Publisher pub1, pub4, velopub, pid, v0_pub;

// global variables
double current_turn = 2000;
double v0 = 0.3;
double current_speed = 0.0;  // Actual speed from encoder
double prev_velocity = 0;
bool command_received = false;
bool prev_command_received = false;
bool steer_command_received = false;
double manual_steer = 2000.0; //terbaik 2040 - 2048
int manual_speed = 95;   
float up_down_value;
float left_right_value;
double accel_max = 0.4; //awal 0.2
//double decel_max = 1;   
ros::Time prev_vel_time;

// PID Parameters
double Kp = 0.00750;
double Ki = 0.05550;
double Kd = 0.00070;
double output_min = -1.0;
double output_max = 1.0;

// PID State variables
double integral = 0.0;
double derivative = 0.0;
double prev_error = 0.0;
ros::Time last_time; // initialized on first use
bool initialized = false;

double steer_integral = 0.0;
double steer_prev_error = 0.0;
bool steer_prev_auto_mode = false;

void speed_encoder_callback(const std_msgs::Float64::ConstPtr& msg);
void command_callback(const std_msgs::Bool::ConstPtr& msg);
double map(double x, double in_min, double in_max, double out_min, double out_max);
double clamp(double value, double min_value, double max_value);
double pid_update(double current_value, double setpoint);

void controlCallback(const std_msgs::Float32MultiArray::ConstPtr& array_msg) {
    #ifdef PID_TUNE
    if (array_msg->data.size() != 7) {
        ROS_WARN("Received array with unexpected size: %lu", array_msg->data.size());
        return;
    }
    #else
    if (array_msg->data.size() != 5) {
        ROS_WARN("Received array with unexpected size: %lu", array_msg->data.size());
        return;
    }
    #endif

    manual_speed = static_cast<int16_t>(array_msg->data[0]);
    manual_steer = static_cast<double>(array_msg->data[1]);
    command_received = (static_cast<int>(array_msg->data[2]) != 0);       // Tombol T (ACC)
    steer_command_received = (static_cast<int>(array_msg->data[3]) != 0); // Tombol Y (LKA)
    v0 = static_cast<double>(array_msg->data[4]);

    // Publikasikan nilai v0
    std_msgs::Float64 v0_msg;
    v0_msg.data = v0;
    v0_pub.publish(v0_msg);
    
    #ifdef PID_TUNE
    up_down_value = array_msg->data[5];
    left_right_value = array_msg->data[6];

    if(up_down_value != 0){
        if(left_right_value == 1){
            Kp = Kp + (up_down_value * 0.001);

            double current_time = ros::Time::now().toSec();
            double pid_output = pid_update(current_speed, v0) * 10;
            double pwm_output = map(pid_output, -1.0, 1.0, 67, 128);
            pwm_output = clamp(pwm_output, 90, 128);
            int velocity_now = static_cast<int>(pwm_output);

            ROS_INFO("PID Output = %f", pid_output);
            ROS_INFO("PWM Output = %d", velocity_now);
            ROS_INFO("Kp = %.5f, Ki = %.5f, Kd = %.5f\n",Kp ,Ki ,Kd);

            prev_velocity = pwm_output;
            initialized = false;
        }
        else if(left_right_value == 2){
            Ki = Ki + (up_down_value * 0.001);

            double current_time = ros::Time::now().toSec();
            double pid_output = pid_update(current_speed, v0) * 10;
            double pwm_output = map(pid_output, -1.0, 1.0, 67, 128);
            pwm_output = clamp(pwm_output, 90, 128);
            int velocity_now = static_cast<int>(pwm_output);

            ROS_INFO("PID Output = %f", pid_output);
            ROS_INFO("PWM Output = %d", velocity_now);
            ROS_INFO("Kp = %.5f, Ki = %.5f, Kd = %.5f\n",Kp ,Ki ,Kd);

            prev_velocity = pwm_output;
            initialized = false;
        }
        else if(left_right_value == 3){
            Kd = Kd + (up_down_value * 0.001);

            double current_time = ros::Time::now().toSec();
            double pid_output = pid_update(current_speed, v0) * 10;
            double pwm_output = map(pid_output, -1.0, 1.0, 67, 128);
            pwm_output = clamp(pwm_output, 90, 128);
            int velocity_now = static_cast<int>(pwm_output);

            ROS_INFO("PID Output = %f", pid_output);
            ROS_INFO("PWM Output = %d", velocity_now);
            ROS_INFO("Kp = %.5f, Ki = %.5f, Kd = %.5f\n",Kp ,Ki ,Kd);

            prev_velocity = pwm_output;
            initialized = false;
        }
    }
    #endif
}

double pid_update(double current_value, double setpoint) {
    ros::Time current_time = ros::Time::now();

    // Initialize dt on first call
    double dt = 0.02;
    if (initialized) {
        dt = (current_time - last_time).toSec();
    }
    last_time = current_time;

    double error = setpoint - current_value;
    if (!initialized) {
        integral = 0;
    }
    integral += error * dt;

    if (!initialized) {
        derivative = (error - error) / dt;
    }
    else{
        derivative = (error - prev_error) / dt;
    }
    prev_error = error;

    double output = Kp * error + Ki * integral + Kd * derivative;

    if (output > output_max) output = output_max;
    if (output < output_min) output = output_min;

    initialized = true;
    return output;
}

double map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double clamp(double value, double min_value, double max_value) {
    return std::max(std::min(value, max_value), min_value);
}

void speed_encoder_callback(const std_msgs::Float64::ConstPtr& msg) {
    current_speed = msg->data;
}

void steering_callback(const geometry_msgs::Point::ConstPtr& msg) {
    if (steer_command_received) {

        if (!steer_prev_auto_mode) {
            steer_prev_error = 0.0;
        }

        double setPoint = 2050.0; 
        double angle = msg->x;    

        // TIPS TUNING: Turunkan Kp sedikit agar tidak terlalu agresif di awal
        double Kp = 4.0;  // Sebelumnya 8.0
        double Kd = 1.0;

        double error = angle - 180.0;

        while (error > 180.0) error -= 360.0;
        while (error <= -180.0) error += 360.0;

        error = -error;
        
        double derivative = error - steer_prev_error;
        double output_pid = (Kp * error) + (Kd * derivative);
        steer_prev_error = error; 

        // 1. Ini adalah TARGET AKHIR kemudi (bisa lompat-lompat/bergetar)
        double target_turn = setPoint + output_pid;

        if (target_turn > 2250.0) target_turn = 2250.0;
        if (target_turn < 1850.0) target_turn = 1850.0;

        // ========================================================
        // 2. SLEW RATE LIMITER (Mencegah banting setir seketika)
        // ========================================================
        // step_size menentukan kecepatan putar servo. Semakin kecil = semakin lambat/halus.
        // Asumsi program berjalan 30 FPS. Jika step_size = 15, maka butuh 10 frame (0.3 detik)
        // untuk bergeser 150 poin PWM. 
        double step_size = 15.0; 

        if (current_turn < target_turn) {
            // Jika target ada di kanan, geser pelan-pelan ke kanan
            current_turn = std::min(current_turn + step_size, target_turn);
        } else if (current_turn > target_turn) {
            // Jika target ada di kiri, geser pelan-pelan ke kiri
            current_turn = std::max(current_turn - step_size, target_turn);
        }

        // 3. Publish nilai yang sudah dihaluskan
        std_msgs::Float64 turn_msg;
        turn_msg.data = current_turn; 
        pub4.publish(turn_msg);
        steer_prev_auto_mode = true;
    } else {
        steer_prev_error = 0.0;
        std_msgs::Float64 turn_msg;
        turn_msg.data = manual_steer; // Gunakan A & D dari keyboard
        pub4.publish(turn_msg);
        steer_prev_auto_mode = false;
    }
}

// void steering_callback(const geometry_msgs::Point::ConstPtr& msg) {
//     if (command_received) {

//         if (!steer_prev_auto_mode) {
//             steer_prev_error = 0.0;
//         }

//         double setPoint = 2000.0; 
//         double angle = msg->x;    

//         double Kp = 5.0;  
//         double Kd = 2.0;

//         double error = angle - 180.0;

//         while (error > 180.0) error -= 360.0;
//         while (error <= -180.0) error += 360.0;
        
//         //error = -error;

//         double derivative = error - steer_prev_error;
//         double output_pid = (Kp * error) + (Kd * derivative);
//         steer_prev_error = error; 
//         double target_turn = setPoint + output_pid;

//         if (target_turn > 2300.0) target_turn = 2300.0;
//         if (target_turn < 1700.0) target_turn = 1700.0;

//         std_msgs::Float64 turn_msg;
//         turn_msg.data = target_turn;
//         pub4.publish(turn_msg);
//         steer_prev_auto_mode = true;
//     } else {
//         steer_prev_error = 0.0;

//         std_msgs::Float64 turn_msg;
//         turn_msg.data = manual_steer;
//         pub4.publish(turn_msg);
//         steer_prev_auto_mode = false;
//     }
// }

void distance_callback(const geometry_msgs::Point::ConstPtr& msg) {

    ros::Time now = ros::Time::now();
    if (command_received != prev_command_received) {
        prev_vel_time = now;
        prev_velocity = current_speed;  
     }
    prev_command_received = command_received;
    double dt_vel = (prev_vel_time.isZero()) ? 0.033 : (now - prev_vel_time).toSec();
    if (dt_vel <= 0.0 || dt_vel > 1.0) dt_vel = 0.033;
    prev_vel_time = now;
	

    if (command_received) {
        double m = 2.6, tau = 0.1;

        double fnav = msg->x;
        double velocity = v0 - (fnav / m) * tau;
        velocity = clamp(velocity, -0.1, v0);

	  // Rate limiter acc dcc
        //double max_step_up   =  accel_max * dt_vel;
        //double max_step_down = -decel_max * dt_vel;
        //double delta = clamp(velocity - prev_velocity, max_step_down, max_step_up);
        //velocity = prev_velocity + delta;
        //prev_velocity = velocity;
    
        //limit acceleration
	double max_step_up = accel_max * dt_vel;
        double delta_raw = velocity - prev_velocity;
        double delta = (delta_raw > max_step_up) ? max_step_up : delta_raw;
        velocity = prev_velocity + delta;
        prev_velocity = velocity;


        double current_time = ros::Time::now().toSec();
        double pid_output = pid_update(current_speed, velocity) * 10;
        double pwm_output = map(pid_output, -1.0, 1.0, 60, 120);
        pwm_output = clamp(pwm_output, 88, 128);
        int velocity_now = static_cast<int>(pwm_output);

        // if(pwm_output != prev_velocity){
            // ROS_INFO("Velocity = %.2f", velocity);
        //     ROS_INFO("PID Output = %f", pid_output);
        //     // ROS_INFO("Current Speed = %.2f", current_speed);
            // ROS_INFO("PWM Output = %d", velocity_now);
        //     ROS_INFO("Kp = %.5f, Ki = %.5f, Kd = %.5f\n",Kp ,Ki ,Kd);
        //     prev_velocity = pwm_output;
        // }

        std_msgs::Float64 velopub_msg;
        velopub_msg.data = velocity; 
        velopub.publish(velopub_msg);

        std_msgs::UInt16 vtt_msg;

        vtt_msg.data = velocity_now;
        pub1.publish(vtt_msg);
    } else {

        std_msgs::Float64 velopub_msg;
        velopub_msg.data = v0; 
        velopub.publish(velopub_msg);
        std_msgs::UInt16 vtt_msg;
        vtt_msg.data = manual_speed;
        initialized = false;
        pub1.publish(vtt_msg);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "joystick_sensor_subscriber");
    ros::NodeHandle nh;

    // Publishers
    pub1 = nh.advertise<std_msgs::UInt16>("v1", 10);
    pub4 = nh.advertise<std_msgs::Float64>("velocity2", 10);
    velopub = nh.advertise<std_msgs::Float64>("velopub", 10);
    pid = nh.advertise<std_msgs::Float64>("pid", 10);
    v0_pub = nh.advertise<std_msgs::Float64>("/v0_target", 10);

    // Subscribers
    ros::Subscriber rho_sub = nh.subscribe("/rho", 10, steering_callback);
    ros::Subscriber fnav_sub = nh.subscribe("/fnav", 10,  distance_callback);
    ros::Subscriber sub = nh.subscribe("/control_array", 10, controlCallback);
    ros::Subscriber speed_encoder_sub = nh.subscribe("/vehicle_speed", 10, speed_encoder_callback);

    ros::Rate rate(30);
    while (ros::ok()) {
        ros::spinOnce(); 
        rate.sleep();
    }

    return 0;
}