# Limo Navigation

![Yolo](https://limo-agx.github.io/_images/yolo.png)

Example navigation configurations for autonomous navigation of the Limo.  Uses either Gmapping with AMCL or RTABMap for SLAM.  Uses move_base for path planning and execution

## Set Up

Clone this repo into your workspace, install dependencies, build and source

    git clone --recursive http://github.com/limo-agx/limo.git
    cd ..
    rosdep install --from-paths src --ignore-src -y
    catkin_make
    source devel/setup.bash

## Usage

### Gmapping (2D SLAM)

    roslaunch limo_navigation limo_gmapping.launch

[Gmapping - Limo Docs](https://limo-agx.github.io/navigation_gmapping.html)

### RTABMap (3D SLAM)

    roslaunch limo_navigation limo_rtabmap.launch

[RTABMap - Limo Docs](https://limo-agx.github.io/navigation_rtabmap.html)
