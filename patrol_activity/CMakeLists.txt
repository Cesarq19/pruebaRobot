cmake_minimum_required(VERSION 3.0.2)
project(patrol_activity)

## Encuentra dependencias de catkin
find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  move_base_msgs
)

## Declaración del paquete catkin
catkin_package()

## Instalar scripts ejecutables
catkin_install_python(PROGRAMS
  scripts/patrol_node.py
  scripts/person_follow_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
