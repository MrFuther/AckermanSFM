#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <SDL2/SDL.h>
#include <iostream>

/*IF YOU WANT TO TUNE THE PID JUST UNCOMMENT THE DEFINE BELOW DAWG*/
// #define PID_TUNE
/* ^^^^^^^^^^^^^^^^ THIS ONE*/


int main(int argc, char **argv) {
    ros::init(argc, argv, "keyboard_control_node");
    ros::NodeHandle nh;

    ros::Publisher control_pub = nh.advertise<std_msgs::Float32MultiArray>("/control_array", 10);

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cerr << "SDL_Init error: " << SDL_GetError() << std::endl;
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow("Keyboard Control", 
                                          SDL_WINDOWPOS_CENTERED, 
                                          SDL_WINDOWPOS_CENTERED, 
                                          320, 240, SDL_WINDOW_SHOWN);
    if (!window) {
        std::cerr << "SDL_CreateWindow error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return 1;
    }

    ROS_INFO("Keyboard control node started. Use W, A, S, D to control the robot.");
    ROS_INFO("Press T and release it to toggle the command flag.");
    ROS_INFO("Press X on the keyboard or close the window to exit.");
    ROS_INFO("Use Shift + '.' (>) and Shift + ',' (<) to adjust the parameter value by 0.1 (on key release).");
    ROS_INFO("Make sure to click on the window so it has focus!");

    ros::Rate loop_rate(30);

    int current_speed = 95;   
    double center_steering = 2000.0; //terbaik 2040 - 2048
    double current_steering = center_steering;
    bool t_pressed = false;      
    bool command_toggle = false; // Untuk ACC / Speed
    bool y_pressed = false;
    bool steer_toggle = false;   // Untuk LKA / Steering
    double parameter_value = 1.0;

    bool greater_pressed_prev = false;
    bool less_pressed_prev = false;

    #ifdef PID_TUNE
    float up_down_value = 0.0f;
    float left_right_value = 1.0f;
    bool up_pressed_prev = false;
    bool down_pressed_prev = false;
    bool left_pressed_prev = false;
    bool right_pressed_prev = false;
    #endif

    bool running = true;
    while (ros::ok() && running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            }
            if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_x) {
                    running = false;
                }
            }
        }

        const Uint8* keystate = SDL_GetKeyboardState(NULL);
        bool w_pressed = keystate[SDL_SCANCODE_W];
        bool s_pressed = keystate[SDL_SCANCODE_S];
        bool a_pressed = keystate[SDL_SCANCODE_A];
        bool d_pressed = keystate[SDL_SCANCODE_D];
        bool t_currently_pressed = keystate[SDL_SCANCODE_T];
        bool y_currently_pressed = keystate[SDL_SCANCODE_Y];

        #ifdef PID_TUNE
        bool up_current = keystate[SDL_SCANCODE_UP];
        bool down_current = keystate[SDL_SCANCODE_DOWN];
        bool left_current = keystate[SDL_SCANCODE_LEFT];
        bool right_current = keystate[SDL_SCANCODE_RIGHT];
        #endif

        bool current_greater_pressed = keystate[SDL_SCANCODE_PERIOD] && 
                                       (SDL_GetModState() & KMOD_SHIFT);
        bool current_less_pressed = keystate[SDL_SCANCODE_COMMA] && 
                                    (SDL_GetModState() & KMOD_SHIFT);

        if (!current_greater_pressed && greater_pressed_prev) {
            parameter_value += 0.1;
            ROS_INFO("v0 = %f", parameter_value);
            // ROS_INFO("Parameter increased to %.1f", parameter_value);
        }

        if (!current_less_pressed && less_pressed_prev) {
            parameter_value -= 0.1;
            ROS_INFO("v0 = %f", parameter_value);
            // ROS_INFO("Parameter decreased to %.1f", parameter_value);
        }

        greater_pressed_prev = current_greater_pressed;
        less_pressed_prev = current_less_pressed;

        if (!t_currently_pressed && t_pressed) {
            command_toggle = !command_toggle;
            ROS_INFO("Command flag toggled to %s (on 'T' release).", 
                     command_toggle ? "true" : "false");
        }
        t_pressed = t_currently_pressed;

        if (!y_currently_pressed && y_pressed) {
            steer_toggle = !steer_toggle;
            ROS_INFO("LKA/Steering flag toggled to %s (on 'Y' release).", steer_toggle ? "true" : "false");
        }
        y_pressed = y_currently_pressed;

        bool forward_or_reverse = (w_pressed ^ s_pressed);
        if (forward_or_reverse) {
            if (w_pressed && current_speed < 103) {
                if (current_speed == 0) current_speed = 95;
                current_speed += 1;
            } else if (s_pressed && (current_speed > 80 || current_speed == 0)) {
                if (current_speed == 0) current_speed = 95;
                current_speed -= 1;
            }
        } else {
            current_speed = 0;
        }

        // bool left_or_right = (a_pressed ^ d_pressed);
        // if (left_or_right) {
        //     if (a_pressed && current_steering > 1800.0) {
        //         current_steering -= 100.0;
        //     } else if (d_pressed && current_steering < 2300.0) {
        //         current_steering += 100.0;
        //     }
        // } else {
        //     if (current_steering > center_steering)
        //         current_steering -= 100.0;
        //     else if (current_steering < center_steering)
        //         current_steering += 100.0;
        // }

        bool left_or_right = (a_pressed ^ d_pressed);
        if (left_or_right) {
            if (a_pressed && current_steering > 1800.0) {
                current_steering -= 100.0;
            } else if (d_pressed && current_steering < 2300.0) {
                current_steering += 100.0;
            }
        } else {
            if (current_steering > 2000.0)
                current_steering -= 100.0;
            else if (current_steering < 2000.0)
                current_steering += 100.0;
        }

        /*PID TUNER PART START*/
        #ifdef PID_TUNE
        if (!up_current && up_pressed_prev) {
            up_down_value = (SDL_GetModState() & KMOD_SHIFT) ? 1 : 0.01f;
            // ROS_INFO("Up pressed (released): value = %.2f", up_down_value);
        }

        if (!down_current && down_pressed_prev) {
            up_down_value = (SDL_GetModState() & KMOD_SHIFT) ? -1 : -0.01f;
            // ROS_INFO("Down pressed (released): value = %.2f", up_down_value);
        }

        if (!left_current && left_pressed_prev) {
            left_right_value -= 1.0f;
            if (left_right_value < 1.0f) left_right_value = 3.0f;
            // ROS_INFO("Left pressed (released): value = %.0f", left_right_value);
        }

        if (!right_current && right_pressed_prev) {
            left_right_value += 1.0f;
            if (left_right_value > 3.0f) left_right_value = 1.0f;
            // ROS_INFO("Right pressed (released): value = %.0f", left_right_value);
        }
        up_pressed_prev = up_current;
        down_pressed_prev = down_current;
        left_pressed_prev = left_current;
        right_pressed_prev = right_current;
        #endif
        /*PID TUNER PART END*/

        std_msgs::Float32MultiArray array_msg;

        // ROS_INFO("spit %d", current_speed);

        array_msg.data.clear();
        array_msg.data.push_back(static_cast<float>(current_speed));
        array_msg.data.push_back(static_cast<float>(current_steering));
        array_msg.data.push_back(command_toggle ? 1.0f : 0.0f);
        array_msg.data.push_back(steer_toggle ? 1.0f : 0.0f);
        array_msg.data.push_back(static_cast<float>(parameter_value));

        #ifdef PID_TUNE
        array_msg.data.push_back(up_down_value);              // index 4
        array_msg.data.push_back(left_right_value);           // index 5
        up_down_value = 0;
        #endif

        control_pub.publish(array_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}

// #include <ros/ros.h>
// #include <std_msgs/Int16.h>
// #include <std_msgs/Float64.h>
// #include <std_msgs/Float32MultiArray.h>
// #include <std_msgs/Bool.h>
// #include <SDL2/SDL.h>
// #include <iostream>

// /*IF YOU WANT TO TUNE THE PID JUST UNCOMMENT THE DEFINE BELOW DAWG*/
// // #define PID_TUNE
// /* ^^^^^^^^^^^^^^^^ THIS ONE*/


// int main(int argc, char **argv) {
//     ros::init(argc, argv, "keyboard_control_node");
//     ros::NodeHandle nh;

//     ros::Publisher control_pub = nh.advertise<std_msgs::Float32MultiArray>("/control_array", 10);

//     if (SDL_Init(SDL_INIT_VIDEO) != 0) {
//         std::cerr << "SDL_Init error: " << SDL_GetError() << std::endl;
//         return 1;
//     }

//     SDL_Window* window = SDL_CreateWindow("Keyboard Control", 
//                                           SDL_WINDOWPOS_CENTERED, 
//                                           SDL_WINDOWPOS_CENTERED, 
//                                           320, 240, SDL_WINDOW_SHOWN);
//     if (!window) {
//         std::cerr << "SDL_CreateWindow error: " << SDL_GetError() << std::endl;
//         SDL_Quit();
//         return 1;
//     }

//     ROS_INFO("Keyboard control node started. Use W, A, S, D to control the robot.");
//     ROS_INFO("Press T and release it to toggle the command flag.");
//     ROS_INFO("Press X on the keyboard or close the window to exit.");
//     ROS_INFO("Use Shift + '.' (>) and Shift + ',' (<) to adjust the parameter value by 0.1 (on key release).");
//     ROS_INFO("Make sure to click on the window so it has focus!");

//     ros::Rate loop_rate(30);

//     int current_speed = 95;   
//     double current_steering = 2000.0;
//     bool t_pressed = false;      
//     bool command_toggle = false; 
//     double parameter_value = 1.0;

//     bool greater_pressed_prev = false;
//     bool less_pressed_prev = false;

//     #ifdef PID_TUNE
//     float up_down_value = 0.0f;
//     float left_right_value = 1.0f;
//     bool up_pressed_prev = false;
//     bool down_pressed_prev = false;
//     bool left_pressed_prev = false;
//     bool right_pressed_prev = false;
//     #endif

//     bool running = true;
//     while (ros::ok() && running) {
//         SDL_Event event;
//         while (SDL_PollEvent(&event)) {
//             if (event.type == SDL_QUIT) {
//                 running = false;
//             }
//             if (event.type == SDL_KEYDOWN) {
//                 if (event.key.keysym.sym == SDLK_x) {
//                     running = false;
//                 }
//             }
//         }

//         const Uint8* keystate = SDL_GetKeyboardState(NULL);
//         bool w_pressed = keystate[SDL_SCANCODE_W];
//         bool s_pressed = keystate[SDL_SCANCODE_S];
//         bool a_pressed = keystate[SDL_SCANCODE_A];
//         bool d_pressed = keystate[SDL_SCANCODE_D];
//         bool t_currently_pressed = keystate[SDL_SCANCODE_T];

//         #ifdef PID_TUNE
//         bool up_current = keystate[SDL_SCANCODE_UP];
//         bool down_current = keystate[SDL_SCANCODE_DOWN];
//         bool left_current = keystate[SDL_SCANCODE_LEFT];
//         bool right_current = keystate[SDL_SCANCODE_RIGHT];
//         #endif

//         bool current_greater_pressed = keystate[SDL_SCANCODE_PERIOD] && 
//                                        (SDL_GetModState() & KMOD_SHIFT);
//         bool current_less_pressed = keystate[SDL_SCANCODE_COMMA] && 
//                                     (SDL_GetModState() & KMOD_SHIFT);

//         if (!current_greater_pressed && greater_pressed_prev) {
//             parameter_value += 0.1;
//             ROS_INFO("v0 = %f", parameter_value);
//             // ROS_INFO("Parameter increased to %.1f", parameter_value);
//         }

//         if (!current_less_pressed && less_pressed_prev) {
//             parameter_value -= 0.1;
//             ROS_INFO("v0 = %f", parameter_value);
//             // ROS_INFO("Parameter decreased to %.1f", parameter_value);
//         }

//         greater_pressed_prev = current_greater_pressed;
//         less_pressed_prev = current_less_pressed;

//         if (!t_currently_pressed && t_pressed) {
//             command_toggle = !command_toggle;
//             ROS_INFO("Command flag toggled to %s (on 'T' release).", 
//                      command_toggle ? "true" : "false");
//         }
//         t_pressed = t_currently_pressed;

//         bool forward_or_reverse = (w_pressed ^ s_pressed);
//         if (forward_or_reverse) {
//             if (w_pressed && current_speed < 103) {
//                 if (current_speed == 0) current_speed = 95;
//                 current_speed += 1;
//             } else if (s_pressed && (current_speed > 80 || current_speed == 0)) {
//                 if (current_speed == 0) current_speed = 95;
//                 current_speed -= 1;
//             }
//         } else {
//             current_speed = 0;
//         }

//         bool left_or_right = (a_pressed ^ d_pressed);
//         if (left_or_right) {
//             if (a_pressed && current_steering > 1800.0) {
//                 current_steering -= 100.0;
//             } else if (d_pressed && current_steering < 2300.0) {
//                 current_steering += 100.0;
//             }
//         } else {
//             if (current_steering > 2000.0)
//                 current_steering -= 100.0;
//             else if (current_steering < 2000.0)
//                 current_steering += 100.0;
//         }

//         /*PID TUNER PART START*/
//         #ifdef PID_TUNE
//         if (!up_current && up_pressed_prev) {
//             up_down_value = (SDL_GetModState() & KMOD_SHIFT) ? 1 : 0.01f;
//             // ROS_INFO("Up pressed (released): value = %.2f", up_down_value);
//         }

//         if (!down_current && down_pressed_prev) {
//             up_down_value = (SDL_GetModState() & KMOD_SHIFT) ? -1 : -0.01f;
//             // ROS_INFO("Down pressed (released): value = %.2f", up_down_value);
//         }

//         if (!left_current && left_pressed_prev) {
//             left_right_value -= 1.0f;
//             if (left_right_value < 1.0f) left_right_value = 3.0f;
//             // ROS_INFO("Left pressed (released): value = %.0f", left_right_value);
//         }

//         if (!right_current && right_pressed_prev) {
//             left_right_value += 1.0f;
//             if (left_right_value > 3.0f) left_right_value = 1.0f;
//             // ROS_INFO("Right pressed (released): value = %.0f", left_right_value);
//         }
//         up_pressed_prev = up_current;
//         down_pressed_prev = down_current;
//         left_pressed_prev = left_current;
//         right_pressed_prev = right_current;
//         #endif
//         /*PID TUNER PART END*/

//         std_msgs::Float32MultiArray array_msg;

//         // ROS_INFO("spit %d", current_speed);

//         array_msg.data.clear();
//         array_msg.data.push_back(static_cast<float>(current_speed));
//         array_msg.data.push_back(static_cast<float>(current_steering));
//         array_msg.data.push_back(command_toggle ? 1.0f : 0.0f);
//         array_msg.data.push_back(static_cast<float>(parameter_value));

//         #ifdef PID_TUNE
//         array_msg.data.push_back(up_down_value);              // index 4
//         array_msg.data.push_back(left_right_value);           // index 5
//         up_down_value = 0;
//         #endif

//         control_pub.publish(array_msg);

//         ros::spinOnce();
//         loop_rate.sleep();
//     }

//     SDL_DestroyWindow(window);
//     SDL_Quit();
//     return 0;
// }