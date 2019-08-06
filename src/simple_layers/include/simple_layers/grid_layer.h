#ifndef GRID_LAYER_H_
#define GRID_LAYER_H_
#include <ros/ros.h>
#include <atomic>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>

namespace simple_layer_namespace
{

class GridLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  GridLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y) override;
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;

  virtual void updateWithMax(costmap_2d::Costmap2D&master_grid, int min_i, int min_j, int max_i, int max_j);


  bool isDiscretized()
  {                                     
    return true;
  }

  virtual void matchSize();
  
  unsigned char interpretValue(unsigned char value);

  /*!
   * Callback to receive the occupancy grid msg from traversability_map.
   * @param occupancy_grid msg from traversability_map.
   */
  void traversabilityMapCallback(const nav_msgs::OccupancyGridConstPtr& traversability);

protected:

  //! Whether the traversability_map msg was received.
  std::atomic_bool traversabilityMapReceived_;  

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
  
  ros::NodeHandle nodeHandle_;
  
  //! The traversability_map from which to take the information abut the environment.
  nav_msgs::OccupancyGridConstPtr traversability_;
  
  //! Ros subscriber to traversability map.
  ros::Subscriber traversabilitySubscriber_;  
  
  //! Topic_name of the traversability_map incoming msg.
  std::string traversabilityTopic_;

  std::string global_frame_;  ///< @brief The global frame for the costmap
  std::string map_frame_;  /// @brief frame that map is located in
  unsigned int x_, y_, width_, height_, size_x_, size_y_, resolution_, origin_x_, origin_y_;
  bool first_update;
  unsigned char lethal_threshold_;
  
  costmap_2d::Costmap2D old_map;

};
}
#endif

