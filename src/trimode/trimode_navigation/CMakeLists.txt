cmake_minimum_required(VERSION 2.8.3)
project(trimode_navigation)

#add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch/include/amcl.launch)
roslaunch_add_file_check(launch/amcl_demo.launch)
roslaunch_add_file_check(launch/include/gmapping.launch)
roslaunch_add_file_check(launch/gmapping_demo.launch)
roslaunch_add_file_check(launch/include/move_base.launch)
roslaunch_add_file_check(launch/odom_navigation_demo.launch)

#add_executable(grid2occupancy src/grid2occupancy.cpp)
#target_link_libraries(grid2occupancy ${catkin_LIBRARIES})
#add_dependencies(grid2occupancy grid_map_ros)

#find_package(grid_map_ros REQUIRED)
#include_directories(${grid_map_ros_INCLUDE_DIRS})

install(
DIRECTORY launch maps params
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
