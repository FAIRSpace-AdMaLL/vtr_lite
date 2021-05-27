# V-TR_Lite

## Robust and Long-Term Monocular Teach-and-Repeat Navigation using a Single-Experience Map 
[Paper](http://eprints.whiterose.ac.uk/168162/)  
[Video Demo](https://youtu.be/4oTsYiRGueI)

+ Day2Night navigation
+ Fisheye-Panorama navigation

### Authors: Li Sun, Marwan Taher, et al.

### Ubuntu 18 + ROS Melodic 

### Usage

1. launch odometry monitor, mapper and navigator: `roslaunch vtr_lite vtr_lite.launch`
1. Mapping: `rosservice call /vtr_lite/mapper "map_name: 'church'"`
1. Navigate: `rosservice call /vtr_lite/navigator "map_name: 'church' reverse: false"`
1. Note, if you use the rosbags, you will need to firstly play the rosbag then call the services.

