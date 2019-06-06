cmake_minimum_required(VERSION 2.8.3)
project(trimode_description)

find_package(catkin REQUIRED COMPONENTS roslaunch jackal_description gazebo_ros gazebo_msgs gazebo_plugins gazebo_ros_control)

catkin_package()

roslaunch_add_file_check(launch/description.launch)

install(DIRECTORY meshes launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/env_run
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
