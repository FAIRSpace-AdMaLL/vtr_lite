//
// Created by jaguar on 25/12/2020.
//

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

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

class ParamSever
{

public:
    ros::NodeHandle nh;

    /* General Parameters */
    string FOLDER;
    float TOPO_INTERVAL;
    float MAX_LINEAR_VEL;
    float MIN_LINEAR_VEL;
    float MAX_ANGULAR_VEL;
    float MAX_LINEAR_ACC;

    int LINEAR_AXIS;
    int ANGULAR_AXIS;
    int STOP_BUTTON;
    int PAUSE_BUTTON;
    int ROT_ACC_BUTTON; // in the acceleration mode,  the robot will apply the 2x max angular velocity
    int ROT_MAX_BUTTON; // in the acceleration Max mode, the robot will ignore the linear velocity (for rotation within origin)

    /* ROS Topics / Server */
    string IMAGE_TOPIC;
    string ODOM_TOPIC;
    string JOY_TOPIC;

    string SET_DIST_SERVER = "/vtr_lite/set_distance";
    string DIST_TOPIC = "/vtr_lite/distance";
    string VEL_CMD_TOPIC = "vel_cmd";
    string MAP_TOPIC = "vtr_lite/map_image";
    string MATCHER_VIS_TOPIC = "vtr_lite/matching_visualisation";

    ParamSever()
    {
        /* Navigation Parameters */
        // joystick axis
        nh.param<int>("axis_linear", LINEAR_AXIS, 1);
        nh.param<int>("axis_angular", ANGULAR_AXIS, 0);
        nh.param<int>("stop_button", STOP_BUTTON, 2);
        nh.param<int>("pause_button", PAUSE_BUTTON, 0);
        nh.param<int>("rotation_acceleration_button", ROT_ACC_BUTTON, 6);
        nh.param<int>("rotation_max_button", ROT_MAX_BUTTON, 7);

        // map parameters
        nh.param<float>("topological_map_interval", TOPO_INTERVAL, 0.5);
        nh.param<string>("map_folder", FOLDER, "/home/jaguar/maps");

        // robot speed limits
        nh.param<float>("max_angular_velocity", MAX_ANGULAR_VEL, 2.0);
        nh.param<float>("min_linear_velocity", MIN_LINEAR_VEL, 0.05);
        nh.param<float>("max_linear_velocity", MAX_LINEAR_VEL, 3.0);
        nh.param<float>("max_linear_acceleration", MAX_LINEAR_ACC, 0.1);

        nh.param<std::string>("topo_vtr/image_topic", IMAGE_TOPIC, "/zed2/zed_node/rgb/image_rect_color");
        nh.param<std::string>("topo_vtr/odom_topic", ODOM_TOPIC, "/encoder_odom");
        nh.param<std::string>("topo_vtr/joy_topic", JOY_TOPIC, "/joy");
    }
};
