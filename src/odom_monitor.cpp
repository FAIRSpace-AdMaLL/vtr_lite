//
// Created by jaguar on 25/12/2020.
//

#include "param.h"
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <vtr_lite/SetDistance.h>

#include "eigen_conversions/eigen_msg.h"
#include <Eigen/Dense>

class OdomMonitor : public ParamSever
{

private:
    ros::Subscriber set_dist_sub;
    ros::Publisher dist_pub;
    ros::Subscriber odom_sub;
    ros::Publisher robot_pose_pub;

    ros::ServiceServer set_dist_srv;

    float total_dist = 0;
    float last_x = FLT_MAX;
    float last_y = FLT_MAX;
    float current_x;
    float current_y;
    std_msgs::Float32 dist_msg;

    Eigen::Affine3d init_pose = Eigen::Affine3d::Identity();;
    Eigen::Affine3d curr_pose, odom_pose;

public:
    OdomMonitor(ros::NodeHandle *nh)
    {
        /* initiate service */
        set_dist_srv = nh->advertiseService(SET_DIST_SERVER, &OdomMonitor::setDistance, this);
        odom_sub = nh->subscribe<nav_msgs::Odometry>(ODOM_TOPIC, 10, boost::bind(&OdomMonitor::odomCallBack, this, _1));
        dist_pub = nh->advertise<std_msgs::Float32>(DIST_TOPIC, 1);
        robot_pose_pub = nh->advertise<geometry_msgs::PoseStamped>(ROBOT_POSE_TOPIC, 1);
    }

    void setDistance(const std_msgs::Float32::ConstPtr &dist_msg);
    void odomCallBack(const nav_msgs::Odometry::ConstPtr &odom_msg);
    bool setDistance(vtr_lite::SetDistance::Request &req, vtr_lite::SetDistance::Response &res);
};

/* service for set/reset the distance */

bool OdomMonitor::setDistance(vtr_lite::SetDistance::Request &req, vtr_lite::SetDistance::Response &res)
{
    res.distance = req.distance;
    total_dist = req.distance;
    last_x = current_x;
    last_y = current_y;

    ROS_INFO("Setting current travelled distance to %.3f", (float)req.distance);

    dist_msg.data = total_dist;
    dist_pub.publish(dist_msg);

    // reset the init_pose for the map
    init_pose = odom_pose;

    return true;
}

void OdomMonitor::odomCallBack(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    /*if last_x and last_y are not initialized*/
    if (last_x == FLT_MAX && last_y == FLT_MAX)
    {
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
    dist_msg.data = total_dist;
    dist_pub.publish(dist_msg);

    /* calculate the map poses */
    tf::poseMsgToEigen(odom_msg->pose.pose, odom_pose);
    curr_pose = init_pose.inverse()*odom_pose;

    // Publish 
    geometry_msgs::PoseStamped curr_pose_msg;
    curr_pose_msg.header.frame_id = "/odom";
    tf::poseEigenToMsg(curr_pose, curr_pose_msg.pose);
    robot_pose_pub.publish(curr_pose_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vtr_odom_monitor");

    ros::NodeHandle nh;
    OdomMonitor om = OdomMonitor(&nh);

    ROS_INFO("Odometry Monitor Started.");

    ros::spin();

    return 0;
}
