# ubbo_ROS

This is a ROS package for driving the Ubbo Maker telepresence robot. It has been developed and tested on a raspberry pi 3 b running ubuntu mate 16.04.
The project is created as a part of a bachelor project in electronics engineering. 

* Author: Andreas Just Rønning
* ROS driver: [ubbo_ros](https://github.com/JustElectron/ubbo_ros)

## Dependencies

* [libubbo](https://github.com/wjwwood/serial) cross-platrofm c++ library.
* [CMake](https://cmake.org/) Build system

### Build


    mkdir -p ~/ubbo_ws/src
    cd ~/ubbo_ws/src
    git clone https://github.com/JustElectron/ubbo_ros.git
    cd ..
    catkin_make
