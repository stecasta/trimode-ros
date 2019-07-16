#include<simple_layers/grid_layer.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::GridLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace simple_layer_namespace
{

GridLayer::GridLayer() {}

void GridLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  global_frame_ = layered_costmap_->getGlobalFrameID();


  if (!nodeHandle_.param("traversability_topic", traversabilityTopic_, std::string("/traversability_map_visualization/traversability_map"))) {
    ROS_WARN("did not find elevation_topic, using default");
  }
  // Subscribe to topic.
  traversabilitySubscriber_ = nodeHandle_.subscribe(traversabilityTopic_, 10, &GridLayer::traversabilityMapCallback, this);

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &GridLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}


void GridLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

void GridLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void GridLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;
// To do: replace the hardcoded values.
  *min_x = robot_x - 2.5;
  *min_y = robot_y - 2.5;
  *max_x = robot_x + 2.5;
  *max_y = robot_y + 2.5;

}

void GridLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                            int max_j)
{
    if (!enabled_ || !traversabilityMapReceived_) {
      return;
    }

  unsigned int mx, my;
  double wx, wy;
  // Might be in a different frame
  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_->lookupTransform(map_frame_, global_frame_, ros::Time(0));
  }
  catch (tf2::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }
  // Copy map data given proper transformations
  tf2::Transform tf2_transform;
  tf2::convert(transform.transform, tf2_transform);
  for (unsigned int i = min_i; i < max_i; ++i)
  {
    for (unsigned int j = min_j; j < max_j; ++j)
    {
      // Convert master_grid coordinates (i,j) into global_frame_(wx,wy) coordinates
      layered_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);
      // Transform from global_frame_ to map_frame_
      tf2::Vector3 p(wx, wy, 0);
      p = tf2_transform*p;
      // Set master_grid with cell from map
      if (worldToMap(p.x(), p.y(), mx, my))
      {
          master_grid.setCost(i, j, getCost(mx, my));
      }
    }
  }

}

unsigned char GridLayer::interpretValue(unsigned char value)
{

  if (value == -1){
      return costmap_2d::NO_INFORMATION;
  }

  double scale = (double) value / 100;
  return scale * 255;
}

void GridLayer::traversabilityMapCallback(const nav_msgs::OccupancyGridConstPtr
& traversability) {

  traversability_ = traversability;

  if (size_x_ != traversability->info.width || size_y_ != traversability->info.height ||
         resolution_ != traversability->info.resolution ||
         origin_x_ != traversability->info.origin.position.x ||
         origin_y_ != traversability->info.origin.position.y)
  {
    // update the size of the costmap stored locally in this layer
    ROS_INFO("Resizing traversability layer to %d X %d at %f m/pix", traversability->info.width, traversability->info.height, traversability->info.resolution);
    resizeMap(traversability->info.width, traversability->info.height, traversability->info.resolution,
         traversability->info.origin.position.x, traversability->info.origin.position.y);
  }

  size_x_ = traversability->info.width;
  size_y_ = traversability->info.height;
  resolution_ = traversability->info.resolution;
  origin_x_ = traversability->info.origin.position.x;
  origin_y_ = traversability->info.origin.position.y;
  map_frame_ = traversability->header.frame_id;

  unsigned int index = 0;

  // initialize the costmap with static data
  for (unsigned int i = 0; i < traversability->info.height; ++i)
  {
    for (unsigned int j = 0; j < traversability->info.width; ++j)
    {
      unsigned char value = traversability->data[index];
      costmap_[index] = interpretValue(value);
      ++index;
    }
  }

  if (!traversabilityMapReceived_) {
    traversabilityMapReceived_ = true;
  }
}

} // end namespace