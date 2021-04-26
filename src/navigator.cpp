//
// Created by jaguar on 26/12/2020.
//

#include "param.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vtr_lite/SetDistance.h>
#include <vtr_lite/NNImageMatching.h>
#include <vtr_lite/Navigation.h>

using namespace std;
using namespace cv;

typedef enum
{
    PREPARING,
    PAUSED,
    NAVIGATING,
    COMPLETED
} ENAVIGATINGState;

class Navigator : public ParamSever
{
private:
    /* Service for set/reset distance */
    ros::ServiceClient dist_client;
    ros::ServiceClient matcher_client;
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
    float goal_dist;

    /* Navigation state */
    ENAVIGATINGState state;
    geometry_msgs::Twist twist;

    vector<int> differences;
    vector<int> histogram;

    float visual_offset;
    float last_visual_offset;
    float error_accumlation;

public:
    bool is_reverse = false;

public:
    Navigator(ros::NodeHandle *nh)
    {
        /* Initiate distance service client */
        dist_client = nh->serviceClient<vtr_lite::SetDistance>(SET_DIST_SERVER);
        matcher_client = nh->serviceClient<vtr_lite::NNImageMatching>(NN_MATCHER_SERVER);

        image_transport::ImageTransport img_trans(*nh);

        /* initiate service */
        dist_sub = nh->subscribe<std_msgs::Float32>(DIST_TOPIC, 1, boost::bind(&Navigator::distanceCallBack, this, _1));
        //joy_sub = nh->subscribe<sensor_msgs::Joy>(JOY_TOPIC, 1, boost::bind(&Navigator::joyCallBack, this, _1));
        image_sub = img_trans.subscribe(IMAGE_TOPIC, 1, boost::bind(&Navigator::imageCallBack, this, _1));
        map_pub = img_trans.advertise(MAP_TOPIC, 1);
        matcher_vis_pub = img_trans.advertise(MATCHER_VIS_TOPIC, 1);
        vel_cmd_pub = nh->advertise<geometry_msgs::Twist>(VEL_CMD_TOPIC, 1);
    }

    void distanceCallBack(const std_msgs::Float32::ConstPtr &dist_msg);
    //void joyCallBack(const sensor_msgs::Joy::ConstPtr &joy);
    void imageCallBack(const sensor_msgs::ImageConstPtr &img_msg);
    bool navigate(vtr_lite::Navigation::Request& req, vtr_lite::Navigation::Response& res);
    void loadMap(const string map_name);
    float PID(float error, float last_error);

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
bool Navigator::navigate(vtr_lite::Navigation::Request& req, vtr_lite::Navigation::Response& res)
{
    // load the map
    loadMap(req.map_name);
    // set the reverse mode or not
    is_reverse = req.reverse;
    res.status = false;

    event_idx = is_reverse ? num_event : 0;
    map_img_idx = is_reverse ? num_img : 0;

    last_map_img_idx = -1;
    error_accumlation = 0; // Reseting the PID integral at the start of navigation

    /* reset distance using service*/
    dist_srv.request.distance = dist_travelled = 0;
    goal_dist = map_dist[0];

    /* initialise the variables*/
    visual_offset = 0.;

    if (!dist_client.call(dist_srv))
        ROS_ERROR("Failed to call service SetDistance provided by odometry_monitor node!");

    ROS_INFO("Navigation Initialized: reverse mode %i, current distance: %f, goal distance %f", is_reverse, dist_travelled, goal_dist);

    state = NAVIGATING;

    ros::Rate rate(50);

    while(!ros::isShuttingDown())
    {
        if(state == COMPLETED)
        {
            res.status = true;
            return true;
        }

        //rate.sleep();
        ros::spinOnce();
    }

    return false;
}

/* load map */
void Navigator::loadMap(const string map_name)
{
    // load path profile
    string path_file_name = FOLDER + "/" + map_name + ".yaml";
    ROS_INFO("Loading path profile from %s", path_file_name.c_str());
    FileStorage fsp(path_file_name, FileStorage::READ);

    image_size.clear();
    map_dist.clear();
    images_dist.clear();
    event_dist.clear();
    event_linear_vel.clear();
    event_angular_vel.clear();
    images_map.clear();

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
    Mat img_i = Mat::zeros(image_size[0], image_size[1], CV_8UC1);

    for (int num = 0; num < num_img; num++)
    {
        for (int r = num; r < num + img_i.rows; r++) {
            in_file.read(reinterpret_cast<char *>(img_i.ptr(r - num)), img_i.cols * img_i.elemSize());
        }

        images_map.push_back(img_i.clone());
    }
    ROS_INFO("Done!");
    state = PREPARING;
}

// image call back function
void Navigator::imageCallBack(const sensor_msgs::ImageConstPtr &img_msg)
{
    if (state == NAVIGATING)
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

        if (IMG_RESIZE_FACTOR != -1)
        {
            resize(current_img, current_img, cv::Size(), IMG_RESIZE_FACTOR, IMG_RESIZE_FACTOR);
        }

        vtr_lite::NNImageMatching srv;
        srv.request.image_camera = *(cv_bridge::CvImage(std_msgs::Header(), "mono8", current_img).toImageMsg());

        //imwrite( "/home/jaguar/maps/currentImage_"+to_string(map_img_idx) + "_" + to_string(ros::Time::now().toSec())+".png", current_img);
        srv.request.image_map = *(cv_bridge::CvImage(std_msgs::Header(), "mono8", map_img).toImageMsg());

        // call feature maching service
        if (!matcher_client.call(srv))
        {
            ROS_ERROR("Failed to call service NN Matcher.");
            return;
        }

        int num_bins = srv.response.differences.size();
        int num_matches = srv.response.differences.size();
        int granularity = 20;

        differences.resize(0);
        histogram.resize(0);

        for (int i = 0; i < num_bins; i++)
            histogram.push_back(0);

        for (int i = 0; i < srv.response.differences.size(); i++)
            differences.push_back(int(srv.response.differences[i]));

        /*building histogram*/
        for (int i = 0; i < num_matches; i++)
        {
            int index = (differences[i] + granularity / 2) / granularity + num_bins / 2;

            if (index >= 0 && index < num_bins)
                histogram[index]++;
        }

        int num_inliner = 0;

        int max_bin = distance(histogram.begin(), max_element(histogram.begin(), histogram.end()));
        float max_bin_rot = (max_bin - num_bins / 2) * granularity;

        float difference = 0;
        int count = 0;
        /*histogram printing*/
        for (vector<int>::const_iterator it = histogram.begin(); it != histogram.end(); ++it)
        {
            cout << *it << " ";
            if (fabs(*it - max_bin_rot) < granularity)
            {
                difference += *it;
                count++;
            }
        }
        cout << endl;

        ROS_ERROR("max_bin_rot %f", max_bin_rot);
        ROS_ERROR("difference %f", difference);
        ROS_ERROR("count %i", count);

        visual_offset = difference / count; // calculate the mean value within the max bin (2x)

        float visual_offset_raw = visual_offset;
        visual_offset = PID(visual_offset, last_visual_offset);
        last_visual_offset = visual_offset;
        ROS_ERROR("PID control, visual offset before %f after %f", visual_offset_raw, visual_offset);
    }
}

void Navigator::distanceCallBack(const std_msgs::Float32::ConstPtr &dist_msg)
{
    if (state == NAVIGATING)
    {
        is_reverse ? dist_travelled = goal_dist - dist_msg->data : dist_travelled = dist_msg->data;

        // Searching for the nearest map image using BinarySearch
        map_img_idx = binarySearch(images_dist, 0, num_img - 1, dist_travelled);

        // and publish it
        if (map_img_idx > -1 && map_img_idx != last_map_img_idx)
        {
            map_img = images_map[map_img_idx];
            //imwrite( "/home/jaguar/maps/mapImage_"+to_string(map_img_idx)+".png", map_img);

            ROS_INFO("Load %i th image at distance %f", map_img_idx, dist_travelled);

            /*if someone listens, then publish loaded image too*/
            if (map_pub.getNumSubscribers() > 0)
            {
                std_msgs::Header header;
                cv_bridge::CvImage bridge(header, sensor_msgs::image_encodings::MONO8, map_img);
                map_pub.publish(bridge.toImageMsg());
            }
        }

        if (dist_travelled >= event_dist[event_idx] && event_idx < num_event)
        {
            twist.linear.x = twist.linear.y = twist.linear.z = 0.0;
            twist.angular.z = twist.angular.y = twist.angular.x = 0.0;
            // retrieval event velocities
            twist.linear.x = is_reverse ? -event_linear_vel[event_idx]:event_linear_vel[event_idx]; // if reverse mode, apply inverse linear vel
            twist.angular.z = event_angular_vel[event_idx];

            ROS_INFO("Load %i th event at distance %f", event_idx, dist_travelled);

            is_reverse ? event_idx-- : event_idx++;
        }

        /* Visual control */
        ROS_INFO("The visual offset %f", visual_offset);
        ROS_INFO("The pixel-wise angular velocity gain %f", PIXEL_VEL_GAIN);

        // add the visual compensation (important)
        twist.angular.z += visual_offset * PIXEL_VEL_GAIN;

        if ((!is_reverse & dist_travelled > goal_dist) || (is_reverse & dist_travelled <= 0))
            state = COMPLETED;

        last_map_img_idx = map_img_idx;
    }

    if (state==COMPLETED || state==PAUSED)
    {
        twist.linear.x = twist.linear.y = twist.linear.z = 0.0;
        twist.angular.z = twist.angular.y = twist.angular.x = 0.0;

        if(state==COMPLETED)
            ROS_INFO("Navigation Task Completed!");
        else
            ROS_INFO("Navigation Task Paused!");
    }

    cout << "robot state: " << state << endl;
    cout<<"robot twist: " << twist.linear.x << endl;

    vel_cmd_pub.publish(twist);
}

float Navigator::PID(float error, float last_error)
{
    if (isnan(error))
        error = 0;

    error_accumlation += error;

    float delta = error - last_error;

    std_msgs::Float32 msg;

    msg.data = PID_Kp * error;
    //kp_pub.publish(msg);

    msg.data = PID_Ki * error_accumlation;
    //ki_pub.publish(msg);

    //msg.data = max(min(PID_Kp*error + PID_Ki*error_accumlation + PID_Kd*delta, float(500)), float(-500));
    //control_output.publish(msg);

    return max(min(PID_Kp * error + PID_Ki * error_accumlation + PID_Kd * delta, float(500)), float(-500));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vtr_navigator");

    ROS_INFO("Navigation Server started.");

    ros::NodeHandle nh;
    Navigator nav = Navigator(&nh);

    ros::ServiceServer ss = nh.advertiseService("vtr_lite/navigator", &Navigator::navigate, &nav);
    ros::spin();

    return 0;
}
