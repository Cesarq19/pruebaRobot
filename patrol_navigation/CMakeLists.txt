cmake_minimum_required(VERSION 3.0.2)
project(patrol_navigation)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES patrol_navigation
#  CATKIN_DEPENDS gmapping map_server move_base
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/patrol_navigation_amcl_diff.launch)
  roslaunch_add_file_check(launch/patrol_gmapping.launch)
  roslaunch_add_file_check(launch/patrol_move_base_diff.launch)
  roslaunch_add_file_check(launch/patrol_navigation_rtabmap_diff.launch)
  roslaunch_add_file_check(launch/patrol_rtabmap.launch)
  roslaunch_add_file_check(launch/patrol_slam.launch)
  roslaunch_add_file_check(launch/patrol_nav.launch)
endif()