cmake_minimum_required(VERSION 3.0.2)
project(patrol_gazebo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS    
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  gazebo_dev
  gazebo_msgs
)

add_executable(dummy_cmd_vel src/dummy_cmd_vel.cpp)
target_link_libraries(dummy_cmd_vel ${catkin_LIBRARIES})

install(TARGETS dummy_cmd_vel
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/apartment.launch)
  roslaunch_add_file_check(launch/patrol_diff.launch)
endif()
