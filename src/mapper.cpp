//
// Created by jaguar on 26/12/2020.
//

#include "param.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <vtr_lite/PathProfile.h>
#include <vtr_lite/SetDistance.h>

#include <iostream>
#include <cmath>

using namespace std;
using namespace cv;


typedef enum
{
    PAUSED,
    MAPPING,
    COMPLETED
}EMappingState;


class Mapper : public ParamSever {
private:
    /* Service for set/reset distance */
    ros::ServiceClient dist_client;
    vtr_lite::SetDistance dist_srv;

    /* Subscibers and publishers */
    ros::Subscriber dist_sub;
    image_transport::Subscriber image_sub;
    ros::Subscriber joy_sub;
    ros::Publisher vel_cmd_pub;

    /* run-time info */
    float linear_acc;
    float linear_vel;
    float angular_vel;
    float last_linear_vel;
    float last_angular_vel;
    Mat current_image;
    float distance_travelled;

    /* map image and path profile */
    string map_name;
    vector<Mat> images_map;
    vector<float> images_dist;
    vector<float> path_dist;
    vector<float> path_forward_vel;
    vector<float> path_angular_vel;
    ros::Time last_event_time;

    /* Mapping state */
    EMappingState state;
    geometry_msgs::Twist twist;
    int image_count;
    int event_count;

public:

    Mapper (ros::NodeHandle *nh) 
    {
        /* Initiate distance service client */
        dist_client = nh->serviceClient<vtr_lite::SetDistance>(SET_DIST_SERVER);
        image_transport::ImageTransport img_trans(*nh);

        /* initiate service */
        cout << JOY_TOPIC << endl;
        cout << DIST_TOPIC << endl;

        dist_sub = nh->subscribe<std_msgs::Float32>(DIST_TOPIC, 1, boost::bind(&Mapper::distanceCallBack, this, _1));
        joy_sub = nh->subscribe<sensor_msgs::Joy>(JOY_TOPIC, 1, boost::bind(&Mapper::joyCallBack, this, _1));
        image_sub = img_trans.subscribe(IMAGE_TOPIC, 1, boost::bind(&Mapper::imageCallBack, this, _1));
        vel_cmd_pub = nh->advertise<geometry_msgs::Twist>(VEL_CMD_TOPIC, 1);

        // execute the main mapping function
        mapping();
    }

    void distanceCallBack(const std_msgs::Float32::ConstPtr &dist_msg);
    void joyCallBack(const sensor_msgs::Joy::ConstPtr &joy);
    void imageCallBack(const sensor_msgs::ImageConstPtr &img_msg);
    // void actionCallBack(const vtr_lite::mapperGoalConstPtr &goal, ActionServer *serv);
    void mapping();
    
    template <class T>
    T f_min_max (T x, T min, T max) {return fmin(fmax(x, min), max);}
};

/* class functions */

// distance currently travelled
void Mapper::distanceCallBack(const std_msgs::Float32::ConstPtr &dist_msg) 
{
	distance_travelled = dist_msg->data;
}

// joystick dcallback
void Mapper::joyCallBack(const sensor_msgs::Joy::ConstPtr &joy)
{    
    state = MAPPING;
    // angular velocity will increase with linear velocity
	angular_vel = MAX_ANGULAR_VEL * linear_vel * 1.0 * joy->axes[ANGULAR_AXIS]; 
    // accumulate the linear acceleration
	linear_acc = MAX_LINEAR_ACC * joy->axes[LINEAR_AXIS];
    // two times rotation mode
    if(joy->buttons[ROT_ACC_BUTTON])   angular_vel *= 2.0;
    // max rotation mode
	if(joy->buttons[ROT_MAX_BUTTON])   angular_vel = MAX_ANGULAR_VEL * joy->axes[ANGULAR_AXIS];
    // pause or stop
    if(joy->buttons[PAUSE_BUTTON])   state = PAUSED;
	if(joy->buttons[STOP_BUTTON])   state = COMPLETED;
	ROS_INFO("Joystick pressed");
} 

// image call back function
void Mapper::imageCallBack(const sensor_msgs::ImageConstPtr &img_msg) 
{
    cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
    current_image = cv_ptr->image;

    if(distance_travelled > image_count * TOPO_INTERVAL)
    {
        images_map.push_back(current_image);
        images_dist.push_back(distance_travelled);
        ROS_INFO("Image %i is record at %f.", image_count, distance_travelled);
        image_count ++;
    }
}

/* Mapping function */
void Mapper::mapping()
{
	/* reset distance using service*/
	dist_srv.request.distance = distance_travelled = 0;

    if (!dist_client.call(dist_srv)) 
        ROS_ERROR("Failed to call service SetDistance provided by odometry_monitor node!");

    // map info
    map_name = "tmp";

	images_map.clear();
    images_dist.clear();
    path_dist.clear();
    path_forward_vel.clear();
    path_angular_vel.clear();
    char name[100];

    state = MAPPING;
    image_count = event_count = 0;

    while (ros::ok())
    {
        if(state == MAPPING)
        {
		    linear_vel += linear_acc;
            linear_vel = f_min_max(linear_vel, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
            angular_vel = f_min_max(angular_vel, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);

            twist.linear.x = linear_vel;
            twist.angular.z = angular_vel;
        }
        else
        {
            twist.linear.x = twist.angular.z = 0;
        }
		vel_cmd_pub.publish(twist);

        if(state == MAPPING)
        {
			/* saving path profile */
			if (last_linear_vel != linear_vel || last_angular_vel != angular_vel || (angular_vel !=0 & ros::Time::now() > last_event_time + ros::Duration(0.1)))
			{
				if(angular_vel !=0 & ros::Time::now() > last_event_time + ros::Duration(0.1))
					ROS_DEBUG("Robot is turing.");

				if(fabs(linear_vel) > MIN_LINEAR_VEL) {
					path_dist.push_back(distance_travelled);
					path_forward_vel.push_back(linear_vel);
					path_angular_vel.push_back(angular_vel);
					event_count++;
					ROS_INFO("Event %i is record.", event_count);
					last_event_time = ros::Time::now();
				}
				else {
					ROS_WARN("Robot is not moving.");
				}
			}
			last_linear_vel = linear_vel;
			last_angular_vel = angular_vel;
		}
		/*on preempt request end mapping and save current map */
		if(state == COMPLETED)
		{
			ROS_INFO("Map complete, flushing maps.");

			/*add last data to the map*/
			for(int i = 0; i < images_dist.size(); i++)
                cout << images_dist[i] << " ";

            cout << endl;

			/*and flush it to the disk*/
			for (int i = 0;i<images_dist.size();i++){
				sprintf(name,"%s/%s_%.3f.yaml", FOLDER.c_str(),map_name.c_str(),images_dist[i]);
				ROS_INFO("Saving map to %s",name);
				FileStorage fs(name,FileStorage::WRITE);
				write(fs, "Image", images_map[i]);
				fs.release();
			}

			/*save the path profile as well*/
			sprintf(name,"%s/%s.yaml", FOLDER.c_str(),map_name.c_str());
			ROS_INFO("Saving path profile to %s", name);
			FileStorage pfs(name, FileStorage::WRITE);
			write(pfs, "distance", path_dist);
			write(pfs, "forward_vel", path_forward_vel);
			write(pfs, "angular_vel", path_angular_vel);

			pfs.release();
			
            return;
		}

        ros::spinOnce();
		
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "vtr_mapper");

    ROS_INFO("Mapping started.");

    ros::NodeHandle nh;
    Mapper om = Mapper(&nh);

    ROS_INFO("Mapping completed.");

    return 0;
}


