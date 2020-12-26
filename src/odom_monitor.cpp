//
// Created by jaguar on 25/12/2020.
//

#include "param.h"
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <topo_vtr/SetDistance.h>

class OdomMonitor : public Param {
private:
    ros::Subscriber dist_sub;
    ros::Publisher dist_pub;
    ros::Subscriber odom_sub;

    float total_dist = 0;
    float last_x = FLT_MAX;
    float last_y = FLT_MAX;
    float current_x;
    float current_y;
    std_msgs::Float32 dist;

public:

    OdomMonitor () {
        ros::NodeHandle nh;
        /* initiate service */
        dist_sub     = nh.subscribe<std_msgs::Float32>(SET_DIST_TOPIC, 1, boost::bind(&OdomMonitor::setDistance, this, _1));
        odom_sub     = nh.subscribe<nav_msgs::Odometry>(ODOM_TOPIC, 10, boost::bind(&OdomMonitor::odomCallBack, this, _1));
        dist_pub     = nh.advertise<std_msgs::Float32>(DIST_TOPIC, 1);
    }

    void setDistance(const std_msgs::Float32::ConstPtr &dist_msg);
    void odomCallBack(const nav_msgs::Odometry::ConstPtr &odom_msg);
};

/* service for set/reset the distance */

void OdomMonitor::setDistance(const std_msgs::Float32::ConstPtr &dist_msg) {
    total_dist = dist_msg->data;
    last_x = current_x;
    last_y = current_y;
    dist.data = total_dist;
    dist_pub.publish(dist);
}

void OdomMonitor::odomCallBack(const nav_msgs::Odometry::ConstPtr &odom_msg) {
    /*if last_x and last_y are not initialized*/
    if (last_x == FLT_MAX && last_y == FLT_MAX){
        last_x = odom_msg->pose.pose.position.x;
        last_y = odom_msg->pose.pose.position.y;
    }

    /*calculate translational distance from the last odometry position*/
    current_x = odom_msg->pose.pose.position.x;
    current_y = odom_msg->pose.pose.position.y;

    total_dist += sqrt(pow(current_x - last_x, 2) + pow(current_y - last_y, 2));

    ROS_INFO("Travel distance now: %f", total_dist);

    last_x = current_x;
    last_y = current_y;

    // Publish the distance message
    dist.data = total_dist;
    dist_pub.publish(dist);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_monitor");

    OdomMonitor om;

    ROS_INFO("Odometry Monitor Started.");

    ros::spin();

    return 0;
}

