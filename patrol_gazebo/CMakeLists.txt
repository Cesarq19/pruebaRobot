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

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(dummy_cmd src/dummy_cmd_vel.cpp)
add_dependencies(dummy_cmd ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dummy_cmd ${catkin_LIBRARIES})

install(TARGETS dummy_cmd
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/apartment.launch)
  roslaunch_add_file_check(launch/patrol_diff.launch)
endif()
