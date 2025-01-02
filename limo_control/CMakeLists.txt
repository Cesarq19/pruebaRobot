cmake_minimum_required(VERSION 3.0.2)
project(limo_control)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES limo_control
#  CATKIN_DEPENDS interactive_marker_twist_server robot_localization twist_mux
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/control.launch)
  roslaunch_add_file_check(launch/limo_teletop_keyboard.launch)
endif()