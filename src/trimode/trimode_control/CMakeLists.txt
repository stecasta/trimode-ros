cmake_minimum_required(VERSION 2.8.3)
project(trimode_control)

find_package(catkin REQUIRED COMPONENTS rospy tf roscpp std_msgs roslaunch message_generation )

add_message_files(
  FILES
  FloatList.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime roscpp tf)

roslaunch_add_file_check(launch/control.launch)
roslaunch_add_file_check(launch/teleop.launch)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(odometry src/odometry.cpp)
add_executable(wheel_transf_control src/wheel_transf_control.cpp)
target_link_libraries(odometry ${catkin_LIBRARIES})
target_link_libraries(wheel_transf_control ${catkin_LIBRARIES})

add_dependencies(odometry odometry_gencpp)

add_dependencies(odometry ${catkin_EXPORTED_TARGETS})
add_dependencies(wheel_transf_control ${catkin_EXPORTED_TARGETS})
#add_dependencies(grid2occupancy grid_map_ros)
add_dependencies(odometry trimode_control_msgs_generate_messages_cpp)
add_dependencies(wheel_transf_control trimode_control_msgs_generate_messages_cpp)
#find_package(grid_map_ros REQUIRED)
#include_directories(${grid_map_ros_INCLUDE_DIRS})

install(DIRECTORY config launch src msg srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

