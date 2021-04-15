# V-TR_Lite

## Robust and Long-Term Monocular Teach-and-Repeat Navigation using a Single-Experience Map 
[Paper](http://eprints.whiterose.ac.uk/168162/)  
[Video Demo](https://youtu.be/4oTsYiRGueI)

+ Day2Night navigation
+ Fisheye-Panorama navigation

### Authors: Li Sun, Marwan Taher, et al.

### Ubuntu 18 + ROS Melodic + Setup OpenCV

If you are using Ubuntu 18, you will need to compile opencv with opencv-contrib:

1. Create a folder to perform the compilation and switch to it: `mkdir ~/opencv;cd ~/opencv`
1. Download opencv: `git clone -b 3.4 --single-branch https://github.com/opencv/opencv.git`
1. Download opencv-contrib: `git clone -b 3.4 --single-branch https://github.com/opencv/opencv_contrib.git`
1. Go to opencv folder, create a build folder and switch to it: `mkdir opencv/build;cd opencv/build`
1. Tell opencv to compile with the contrib: `cmake -DOPENCV_ENABLE_NONFREE:BOOL=ON -DOPENCV_EXTRA_MODULES_PATH=~/opencv/opencv_contrib/modules ~/opencv/opencv`
1. Compile it: `make -j5`.
1. Install it: `sudo make install`


### Usage

1. launch odometry monitor, mapper and navigator: `roslaunch vtr_lite vtr_lite.launch`
1. Mapping: `rosservice call /vtr_lite/mapper "map_name: 'church'"`
1. Navigate: `rosservice call /vtr_lite/navigator "map_name: 'church' reverse: false"`
1. Note, if you use the rosbags, you will need to firstly play the rosbag then call the services.

