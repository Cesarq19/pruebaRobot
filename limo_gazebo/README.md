# Limo Gazebo

![Apartment](https://limo-agx.github.io/_images/simulation_apartment.png)

This package contains a simulation environment for developing and learning with a Limo robot

## Set Up

Clone this repo into your workspace, install dependencies, build and source

    git clone https://github.com/limo-agx/limo_simulator.git
    cd  ~/catkin_ws
    rosdep install --from-paths src --ignore-src -y
    catkin_make
    source devel/setup.bash

## Usage

### Differential Drive

    roslaunch limo_gazebo limo_four_diff.launch 

### Ackerman Drive

    roslaunch limo_gazebo limo_ackerman.launch 

## More Documentation

[Simulation - Limo Docs](https://limo-agx.github.io/simulating_limo.html)