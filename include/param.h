//
// Created by jaguar on 25/12/2020.
//

#ifndef TOPO_VTR_PARAM_H
#define TOPO_VTR_PARAM_H

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>

#include <opencv/cv.h>

#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>

using namespace std;

class Param {

public:
    ros::NodeHandle nh;

    /* General Parameters */
    string map_folder;
    float max_forward_speed;
    float max_angular_speed;
    float max_forward_acceleration;

    int linear_axis;
    int angular_axis;
    int stop_button;
    int pause_button;

    /* ROS Topics / Server */
    string IMAGE_TOPIC;
    string ODOM_TOPIC;
    string JOY_TOPIC;

    string SET_DIST_TOPIC = "/topo_vtr/set_distance";
    string DIST_TOPIC = "/topo_vtr/distance";

    typedef enum
    {
        PAUSE,
        NAVIGATING,
        COMPLETED
    } NavigationState;

    Param() {
        /* Navigation Parameters */
        // joystick axis
        nh.param<int>("axis_linear", linear_axis, 1);
        nh.param<int>("axis_angular", angular_axis, 0);
        nh.param<int>("stop_button", stop_button, 2);
        nh.param<int>("pause_button", pause_button, 0);

        // robot speed limits
        nh.param<float>("max_angular_speed", max_angular_speed, 0.5);
        nh.param<float>("max_forward_speed", max_forward_speed, 2.0);
        nh.param<float>("max_forward_acceleration", max_forward_acceleration, 0.05);

        nh.param<std::string>("topo_vtr/imageTopic", IMAGE_TOPIC, "image");
        nh.param<std::string>("topo_vtr/odomTopic", ODOM_TOPIC, "odom");
        nh.param<std::string>("topo_vtr/joyTopic", JOY_TOPIC, "joy");
    }
};


#endif //TOPO_VTR_PARAM_H
