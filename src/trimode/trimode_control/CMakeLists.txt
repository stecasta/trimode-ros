cmake_minimum_required(VERSION 2.8.3)
project(trimode_control)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs roslaunch message_generation)

add_message_files(
  FILES
  FloatList.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

roslaunch_add_file_check(launch/control.launch)
roslaunch_add_file_check(launch/teleop.launch)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY config launch src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

