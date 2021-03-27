//
// Created by jaguar on 26/12/2020.
//

#include "param.h"

#include <vtr_lite/PathProfile.h>
#include <vtr_lite/SetDistance.h>
#include <vtr_lite/Mapping.h>

#include <geometry_msgs/PoseStamped.h>

using namespace std;
using namespace cv;

typedef enum
{
    PAUSED,
    MAPPING,
    COMPLETED
} EMappingState;

class Mapper : public ParamSever
{
private:
    /* Service for set/reset distance */
    ros::ServiceClient dist_client;
    vtr_lite::SetDistance dist_srv;

    /* Subscibers and publishers */
    ros::Subscriber dist_sub;
    ros::Subscriber robot_pose_sub;
    image_transport::Subscriber image_sub;
    ros::Subscriber joy_sub;
    ros::Publisher vel_cmd_pub;

    /* run-time info */
    float linear_acc;
    float linear_vel;
    float angular_vel;
    float last_linear_vel;
    float last_angular_vel;
    Mat current_img;
    float dist_travelled;

    /* map image and path profile */
    vector<Mat> map_images;
    geometry_msgs::Pose robot_pose;
    vector<double> map_poses;
    vector<float> images_dist;
    vector<float> event_dist;
    vector<float> event_linear_vel;
    vector<float> event_angular_vel;
    ros::Time last_event_time;

    /* Mapping state */
    EMappingState state;
    geometry_msgs::Twist twist;
    int image_count;
    int event_count;

    // map name
    string map_name;

public:
    Mapper(ros::NodeHandle *nh)
    {
        /* Initiate distance service client */
        dist_client = nh->serviceClient<vtr_lite::SetDistance>(SET_DIST_SERVER);
        image_transport::ImageTransport img_trans(*nh);

        /* initiate service */
        ROS_INFO("subscribe to %s", JOY_TOPIC.c_str());
        ROS_INFO("subscribe to %s", DIST_TOPIC.c_str());
        ROS_INFO("subscribe to %s", IMAGE_TOPIC.c_str());
        ROS_INFO("subscribe to %s", ROBOT_POSE_TOPIC.c_str());
        ROS_INFO("subscribe to %s", VEL_CMD_TOPIC.c_str());

        dist_sub = nh->subscribe<std_msgs::Float32>(DIST_TOPIC, 1, boost::bind(&Mapper::distanceCallBack, this, _1));
        robot_pose_sub = nh->subscribe<geometry_msgs::PoseStamped>(ROBOT_POSE_TOPIC, 1, boost::bind(&Mapper::robotPoseCallBack, this, _1));
        joy_sub = nh->subscribe<sensor_msgs::Joy>(JOY_TOPIC, 1, boost::bind(&Mapper::joyCallBack, this, _1));
        image_sub = img_trans.subscribe(IMAGE_TOPIC, 1, boost::bind(&Mapper::imageCallBack, this, _1));
        vel_cmd_pub = nh->advertise<geometry_msgs::Twist>(VEL_CMD_TOPIC, 1);
    }

    void distanceCallBack(const std_msgs::Float32::ConstPtr &dist_msg);
    void robotPoseCallBack(const geometry_msgs::PoseStamped::ConstPtr &pose_msg);
    void joyCallBack(const sensor_msgs::Joy::ConstPtr &joy);
    void imageCallBack(const sensor_msgs::ImageConstPtr &img_msg);
    bool mapping(vtr_lite::Mapping::Request &req, vtr_lite::Mapping::Response &res);
    void setMapName(string &file_name);
    void saveMap();

    template <class T>
    T f_min_max(T x, T min, T max) { return fmin(fmax(x, min), max); }
};

/* class functions */

// set the name of the map
void Mapper::setMapName(string &file_name) { map_name = file_name; }

// distance currently travelled
void Mapper::distanceCallBack(const std_msgs::Float32::ConstPtr &dist_msg) { dist_travelled = dist_msg->data; }
void Mapper::robotPoseCallBack(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) { robot_pose = pose_msg->pose; }

// joystick dcallback
void Mapper::joyCallBack(const sensor_msgs::Joy::ConstPtr &joy)
{
    // angular velocity will increase with linear velocity
    angular_vel = MAX_ANGULAR_VEL * linear_vel * 1.0 * joy->axes[ANGULAR_AXIS];
    // define the linear acceleration
    linear_acc = MAX_LINEAR_ACC * joy->axes[LINEAR_AXIS];

    // accumulate the linear acceleration
    linear_vel += linear_acc;
    linear_vel = f_min_max(linear_vel, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
    angular_vel = f_min_max(angular_vel, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);

    // two times rotation mode
    if (joy->buttons[ROT_ACC_BUTTON])
        angular_vel *= 2.0;
    // max rotation mode
    if (joy->buttons[ROT_MAX_BUTTON])
        angular_vel = MAX_ANGULAR_VEL * joy->axes[ANGULAR_AXIS];
    // pause or stop
    if (joy->buttons[PAUSE_BUTTON])
        state = PAUSED;
    else
        state = MAPPING;

    if (joy->buttons[STOP_BUTTON])
        state = COMPLETED;

    linear_vel = f_min_max(linear_vel, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
    angular_vel = f_min_max(angular_vel, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);
    ROS_INFO("Joystick pressed");
}

// image call back function
void Mapper::imageCallBack(const sensor_msgs::ImageConstPtr &img_msg)
{
    if (state == MAPPING)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        current_img = cv_ptr->image;

        if (dist_travelled > image_count * TOPO_INTERVAL)
        {
            Mat map_img;
            if (IMG_RESIZE_FACTOR == -1)
                map_img = current_img;
            else
            {
                resize(current_img, map_img, cv::Size(), IMG_RESIZE_FACTOR, IMG_RESIZE_FACTOR);
            }

            map_images.push_back(map_img);
            map_poses.insert(map_poses.end(), {robot_pose.position.x, robot_pose.position.y, robot_pose.position.z, robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w});

            images_dist.push_back(dist_travelled);
            ROS_INFO("Image %i is record at %f.", image_count, dist_travelled);
            image_count++;
        }
    }
}

/* Mapping function */
bool Mapper::mapping(vtr_lite::Mapping::Request &req, vtr_lite::Mapping::Response &res)
{
    // set the name of the map
    setMapName(req.map_name);
    res.status = false;

    /* reset distance using service*/
    dist_srv.request.distance = dist_travelled = 0;

    if (!dist_client.call(dist_srv))
        ROS_ERROR("Failed to call service SetDistance provided by odometry_monitor node!");

    // map info
    map_images.clear();
    images_dist.clear();
    event_dist.clear();
    event_linear_vel.clear();
    event_angular_vel.clear();

    state = MAPPING;
    image_count = event_count = 0;
    linear_vel = 0.;
    angular_vel = 0.;

    ros::Rate rate(50);
    while (!ros::isShuttingDown())
    {
        if (state == MAPPING)
        {
            twist.linear.x = linear_vel;
            twist.angular.z = angular_vel;
        }
        else
        {
            twist.linear.x = twist.angular.z = 0;
            ROS_WARN("Robot is paused.");
        }
        vel_cmd_pub.publish(twist);

        if (state == MAPPING)
        {
            /* saving path profile */
            if (fabs(last_linear_vel - linear_vel) > 0.001 || fabs(last_angular_vel - angular_vel) > 0.001 || (fabs(angular_vel) >= 0.001 & ros::Time::now() > last_event_time + ros::Duration(0.1)))
            {

                if (fabs(last_linear_vel - linear_vel) > 0.001)
                    ROS_DEBUG("Recording - Accelerating/Decelerating.");
                if (fabs(last_angular_vel - angular_vel) > 0.001)
                    ROS_DEBUG("Recording - Turning");
                if (fabs(angular_vel) >= 0.001 & ros::Time::now() > last_event_time + ros::Duration(0.1))
                    ROS_DEBUG("Recording - Keep Turning more than 0.1s");

                if (fabs(linear_vel) > MIN_LINEAR_VEL)
                {
                    event_dist.push_back(dist_travelled);
                    event_linear_vel.push_back(linear_vel);
                    event_angular_vel.push_back(angular_vel);
                    event_count++;
                    ROS_INFO("Event %i is record.", event_count);
                    last_event_time = ros::Time::now();
                }
                else
                {
                    ROS_ERROR("linear_vel %f", linear_vel);
                    ROS_WARN("Robot is not moving.");
                }
            }
            last_linear_vel = linear_vel;
            last_angular_vel = angular_vel;
        }
        /*on preempt request end mapping and save current map */
        if (state == COMPLETED)
        {
            ROS_INFO("Mapping completed, flushing map.");
            saveMap();
            res.status = true;
            return true;
        }

        //rate.sleep();
        ros::spinOnce();
    }
    return false;
}

void Mapper::saveMap()
{
    /*save the images map as well*/
    ofstream outfile;
    string image_file_name = FOLDER + "/" + map_name + ".bin";
    outfile.open(image_file_name, ios::binary);
    ROS_INFO("Saving images map to %s", image_file_name.c_str());

    for (int i = 0; i < map_images.size(); i++)
    {
        for (int r = 0; r < map_images[i].rows; r++)
            outfile.write(reinterpret_cast<const char *>(map_images[i].ptr(r)), map_images[i].cols * map_images[i].elemSize());
    }
    outfile.close();

    /*save the path profile as well*/
    string path_file_name = FOLDER + "/" + map_name + ".yaml";
    ROS_INFO("Saving path profile to %s", path_file_name.c_str());
    FileStorage pfs(path_file_name.c_str(), FileStorage::WRITE);
    write(pfs, "map_distance", vector<float>{dist_travelled});
    write(pfs, "image_size", vector<int>({map_images[0].rows, map_images[0].cols}));
    write(pfs, "image_distance", images_dist);
    write(pfs, "event_distance", event_dist);
    write(pfs, "linear_vel", event_linear_vel);
    write(pfs, "angular_vel", event_angular_vel);
    // if save the map keyframe poses
    //write(pfs, "map_pose", map_poses);

    ROS_INFO("Done!");

    pfs.release();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vtr_mapper");

    ROS_INFO("Mapping Server started.");

    ros::NodeHandle nh;
    Mapper m = Mapper(&nh);

    ros::ServiceServer ss = nh.advertiseService("vtr_lite/mapper", &Mapper::mapping, &m);
    ros::spin();

    return 0;
}
