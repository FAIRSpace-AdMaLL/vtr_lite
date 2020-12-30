//
// Created by jaguar on 26/12/2020.
//

#include "param.h"

#include <vtr_lite/PathProfile.h>
#include <vtr_lite/SetDistance.h>

using namespace std;
using namespace cv;

typedef enum
{
    PAUSED,
    NAVIGATING,
    COMPLETED
} ENAVIGATINGState;

class Navigator : public ParamSever
{
private:
    /* Service for set/reset distance */
    ros::ServiceClient dist_client;
    vtr_lite::SetDistance dist_srv;

    /* Subscibers and publishers */
    ros::Subscriber dist_sub;
    image_transport::Subscriber image_sub;
    image_transport::Publisher map_pub;
    image_transport::Publisher matcher_vis_pub;

    ros::Subscriber joy_sub;
    ros::Publisher vel_cmd_pub;

    /* run-time info */
    float linear_vel;
    float angular_vel;
    Mat current_img;
    Mat map_img;
    float dist_travelled;

    /* map image and path profile */
    vector<Mat> images_map;
    vector<int> image_size;
    vector<float> map_dist;
    vector<float> images_dist;
    vector<float> event_dist;
    vector<float> event_linear_vel;
    vector<float> event_angular_vel;
    ros::Time last_event_time;

    int num_event;
    int num_img;
    int event_idx;
    int map_img_idx;
    int last_map_img_idx;

    /* Navigation state */
    ENAVIGATINGState state;
    geometry_msgs::Twist twist;

public:
    string map_name;

public:
    Navigator(ros::NodeHandle *nh)
    {
        /* Initiate distance service client */
        dist_client = nh->serviceClient<vtr_lite::SetDistance>(SET_DIST_SERVER);
        image_transport::ImageTransport img_trans(*nh);

        /* initiate service */
        dist_sub = nh->subscribe<std_msgs::Float32>(DIST_TOPIC, 1, boost::bind(&Navigator::distanceCallBack, this, _1));
        joy_sub = nh->subscribe<sensor_msgs::Joy>(JOY_TOPIC, 1, boost::bind(&Navigator::joyCallBack, this, _1));
        image_sub = img_trans.subscribe(IMAGE_TOPIC, 1, boost::bind(&Navigator::imageCallBack, this, _1));
        map_pub = img_trans.advertise(MAP_TOPIC, 1);
        matcher_vis_pub = img_trans.advertise(MATCHER_VIS_TOPIC, 1);
        vel_cmd_pub = nh->advertise<geometry_msgs::Twist>(VEL_CMD_TOPIC, 1);
    }

    void distanceCallBack(const std_msgs::Float32::ConstPtr &dist_msg);
    void joyCallBack(const sensor_msgs::Joy::ConstPtr &joy);
    void imageCallBack(const sensor_msgs::ImageConstPtr &img_msg);
    void initializeNav();
    void loadMap();

    template <class T>
    T f_min_max(T x, T min, T max) { return fmin(fmax(x, min), max); }
};

/* utility functions */
int binarySearch(const vector<float> &array, int i, int j, float val)
{
    if (i == j)
        return i;

    if (j - i == 1)
        return fabs(val - array[i]) > fabs(val - array[j]) ? j : i;

    if (i < j)
    {
        int mid = i + (j - i) / 2;
        if (val == array[mid])
            return mid;
        else if (val > array[mid])
            return binarySearch(array, mid, j, val);
        else
            return binarySearch(array, i, mid, val);
    }
    return -1;
}

/* class functions */

void Navigator::initializeNav()
{
    event_idx = 0;
    last_map_img_idx = -1;

    /* reset distance using service*/
    dist_srv.request.distance = dist_travelled = 0;

    if (!dist_client.call(dist_srv))
        ROS_ERROR("Failed to call service SetDistance provided by odometry_monitor node!");

    state = NAVIGATING;
}

/* load map */
void Navigator::loadMap()
{
    // load path profile
    string path_file_name = FOLDER + "/" + map_name + ".ymal";
    ROS_INFO("Loading path profile from %s", path_file_name.c_str());
    FileStorage fsp(path_file_name, FileStorage::READ);

    image_size.clear();
    map_dist.clear();
    images_dist.clear();
    event_dist.clear();
    event_linear_vel.clear();
    event_angular_vel.clear();

    if (fsp.isOpened())
    {
        fsp["map_distance"] >> map_dist;
        fsp["image_size"] >> image_size;
        fsp["image_distance"] >> images_dist;
        fsp["event_distance"] >> event_dist;
        fsp["linear_vel"] >> event_linear_vel;
        fsp["angular_vel"] >> event_angular_vel;
        fsp.release();
    }

    num_img = images_dist.size();
    num_event = event_linear_vel.size();

    // load map images
    string image_file_name = FOLDER + "/" + map_name + ".bin";
    ROS_INFO("Loading map images from %s", image_file_name.c_str());

    ifstream in_file;
    in_file.open(image_file_name, ios::binary);
    Mat img_i = Mat::zeros(image_size[0], image_size[1], CV_8UC3);

    for (int num = 0; num < num_img; num++)
    {
        for (int r = num; r < num + img_i.rows; r++)
            in_file.read(reinterpret_cast<char *>(img_i.ptr(r - num)), img_i.cols * img_i.elemSize());

        images_map.push_back(img_i.clone());
    }
    ROS_INFO("Done!");
}

// joystick dcallback
void Navigator::joyCallBack(const sensor_msgs::Joy::ConstPtr &joy)
{
    state = NAVIGATING;
    // pause or stop
    if (joy->buttons[PAUSE_BUTTON])
        state = PAUSED;
    if (joy->buttons[STOP_BUTTON])
        state = COMPLETED;
}

// image call back function
void Navigator::imageCallBack(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    current_img = cv_ptr->image;
}

void Navigator::distanceCallBack(const std_msgs::Float32::ConstPtr &dist_msg)
{
    if (state == NAVIGATING)
    {
        dist_travelled = dist_msg->data;

        // Searching for the nearest map image using BinarySearch
        int map_img_idx = binarySearch(images_dist, 0, num_img - 1, dist_travelled);

        // and publish it
        if (map_img_idx > -1 && map_img_idx != last_map_img_idx)
        {
            map_img = images_map[map_img_idx];
            ROS_INFO("Load %i th image at distance %f", map_img_idx, dist_travelled);

            /*if someone listens, then publish loaded image too*/
            if (map_pub.getNumSubscribers() > 0)
            {
                std_msgs::Header header;
                cv_bridge::CvImage bridge(header, sensor_msgs::image_encodings::BGR8, map_img);
                map_pub.publish(bridge.toImageMsg());
            }
        }

        if (dist_travelled > event_dist[event_idx] && event_idx < num_event)
        {
            twist.linear.x = twist.linear.y = twist.linear.z = 0.0;
            twist.angular.z = twist.angular.y = twist.angular.x = 0.0;
            // retrieval event velocities
            twist.linear.x = event_linear_vel[event_idx];
            twist.angular.z = event_angular_vel[event_idx];

            ROS_INFO("Load %i th event at distance %f", event_idx, dist_travelled);

            event_idx++;
        }

        if (dist_travelled >= map_dist[0])
            state = COMPLETED;

        last_map_img_idx = map_img_idx;
    }

    if (state == COMPLETED)
    {
        twist.linear.x = twist.linear.y = twist.linear.z = 0.0;
        twist.angular.z = twist.angular.y = twist.angular.x = 0.0;
    }

    vel_cmd_pub.publish(twist);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vtr_navigator");

    ROS_INFO("Navigation started.");

    ros::NodeHandle nh;
    Navigator nav = Navigator(&nh);

    if (argc > 1)
        nav.map_name = argv[1];
    else
        ROS_ERROR("Map name is not provided!");
    
    nav.initializeNav();
    nav.loadMap();

    ros::spin();

    ROS_INFO("Navigation completed.");
    return 0;
}
