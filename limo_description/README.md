# Limo Description

![Diff](https://limo-agx.github.io/_images/URDF_RVIZ.png)

Package containing all URDF files, meshes, and launches for defining a Limo robot

## Set Up

Clone this repo into your workspace, install dependencies, build and source

    git clone --recursive http://github.com/limo-agx/limo.git
    cd ..
    rosdep install --from-paths src --ignore-src -y
    catkin_make
    source devel/setup.bash

## More Documentation

[Custom URDF - Limo Docs](https://limo-agx.github.io/custom_urdf.html)