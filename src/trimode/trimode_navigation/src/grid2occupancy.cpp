#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include "Eigen/Eigen"

#include <grid_map_core/grid_map_core.hpp>

// ROS
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
nav_msgs::OccupancyGrid occupancyGrid;

using namespace std;
using namespace Eigen;

void gridCallback(const grid_map::GridMap& msg)
{
    const grid_map::GridMap gridMap = msg;
    const std::string layer = "traversability";
    float dataMin = 0;
    float dataMax = 255;
    grid_map::GridMapRosConverter::toOccupancyGrid(gridMap, layer, dataMin, dataMax, occupancyGrid);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "grid2occupancy converter");


  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("grid", 1000, gridCallback);
  
  ros::Publisher pub = n.advertise<nav_msgs::OccupancyGrid>("trav_map", 1000);  
  
  ros::Rate loop_rate(5);

//  ros::spin();

  while (ros::ok())
  {

//    ROS_INFO("%s", msg.data.c_str());

    pub.publish(occupancyGrid);

    ros::spinOnce();

    loop_rate.sleep();
  }  

  return 0;
}

