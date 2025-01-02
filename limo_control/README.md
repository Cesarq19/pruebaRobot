# Limo Control

Package for basic robot control as well as default Kalman filter for odometry

## Set Up

Clone this repo into your workspace, install dependencies, build and source

    git clone --recursive http://github.com/limo-agx/limo.git
    cd ..
    rosdep install --from-paths src --ignore-src -y
    catkin_make
    source devel/setup.bash

## Usage

    roslaunch limo_vision darknet_ros.launch

## More Documentation

[Limo Startup - Limo Docs](https://limo-agx.github.io/starting_limo.html)