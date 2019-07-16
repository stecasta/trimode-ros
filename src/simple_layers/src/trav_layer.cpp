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
  nodeHandle_ = ros::NodeHandle("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  GridLayer::matchSize();

  global_frame_ = layered_costmap_->getGlobalFrameID();

  if (!nodeHandle_.param("traversability_topic", traversabilityTopic_, std::string("/traversability_map_visualization/traversability_map"))) {
    ROS_WARN("did not find elevation_topic, using default");
  }
  // Subscribe to topic.
  traversabilitySubscriber_ = nodeHandle_.subscribe(traversabilityTopic_, 1, &GridLayer::traversabilityMapCallback, this);
//  dsrv_ = nullptr;
//  setupDynamicReconfigure(nodeHandle_);

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nodeHandle_);
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
      if (!(enabled_ && traversabilityMapReceived_)) {
        return;
      }

//    *min_x = robot_x - width_ / 2;
//    *min_y = robot_y - height_ / 2;
//    *max_x = robot_x + width_ / 2;
//    *max_y = robot_y + height_ / 2;
      double wx, wy;
      mapToWorld(robot_x, robot_y, wx, wy);

      *min_x = wx - 5 / 2;
      *min_y = wy - 5 / 2;
      *max_x = wx + 5 / 2;
      *max_y = wy + 5 / 2;

    ROS_ERROR("min x: %f, min y: %f, max x: %f, max y: %f", *min_x, *min_y, *max_x, *max_y);
    ROS_ERROR("width: %d", width_);

}

//void GridLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
//                               double* max_x, double* max_y)
//{

////  if( !layered_costmap_->isRolling() ){
////    if (!map_received_ || !(has_updated_data_ || has_extra_bounds_))
////      return;
////  }

//  if (!(enabled_ && traversabilityMapReceived_)) {
//    return;
//  }

//  updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);

////  useExtraBounds(min_x, min_y, max_x, max_y);

////  double wx, wy;
////  ROS_ERROR("1min x: %f, min y: %f, max x: %f, max y: %f", wx, wy, *max_x, *max_y);
////  mapToWorld(robot_x, robot_y, wx, wy);
////  *min_x = std::min(wx, *min_x);
////  *min_y = std::min(wy, *min_y);
////  ROS_ERROR("wx: %f, wy: %f", wx, wy);
////  mapToWorld(robot_x + width_, robot_y + height_, wx, wy);
////  *max_x = std::max(wx, *max_x);
////  *max_y = std::max(wy, *max_y);
////ROS_ERROR("wx: %f, wy: %f", wx, wy);
////  has_updated_data_ = false;
//}

void GridLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!(enabled_ && traversabilityMapReceived_)) {
      return;
  }
  int sum = 0;
  ROS_ERROR("min i: %d, min j: %d, max i: %d, max j: %d", min_i, min_j, max_i, max_j);
//  ROS_ERROR("i+j: %d", i+j);
  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);
////      if (costmap_[index] == NO_INFORMATION)
////        continue;
      sum++;
      master_grid.setCost(i, j, costmap_[index]);
//        master_grid.setCost(i, j, traversability_->data[index]);
    }
  }

//  // If rolling window, the master_grid is unlikely to have same coordinates as this layer
//  unsigned int mx, my;
//  double wx, wy;
//  // Might even be in a different frame
//  geometry_msgs::TransformStamped transform;
//  try
//  {
//    transform = tf_->lookupTransform(map_frame_, global_frame_, ros::Time(0));
//  }
//  catch (tf2::TransformException ex)
//  {
//    ROS_ERROR("%s", ex.what());
//    return;
//  }
//  // Copy map data given proper transformations
//  tf2::Transform tf2_transform;
//  tf2::convert(transform.transform, tf2_transform);
//  for (unsigned int i = min_i; i < max_i; ++i)
//  {
//    for (unsigned int j = min_j; j < max_j; ++j)
//    {
//      // Convert master_grid coordinates (i,j) into global_frame_(wx,wy) coordinates
//      layered_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);
//      // Transform from global_frame_ to map_frame_
//      tf2::Vector3 p(wx, wy, 0);
////      p = tf2_transform*p;
//      // Set master_grid with cell from mapk
//      if (worldToMap(p.x(), p.y(), mx, my))
//      {
////        if (!use_maximum_)
//          master_grid.setCost(i, j, getCost(mx, my));
////        else
////          master_grid.setCost(i, j, std::max(getCost(mx, my), master_grid.getCost(i, j)));
////          ROS_ERROR("updt");
//      }
//    }
//  }
}

unsigned char GridLayer::interpretValue(unsigned char value)
{

  if (value == -1){
      return -1;
  }

  double scale = (double) value / 100;
  return scale * 255;
}

void GridLayer::traversabilityMapCallback(const nav_msgs::OccupancyGridConstPtr
& traversability) {
  unsigned int width_ = traversability->info.width, height_ = traversability->info.height;

//  traversability_ = traversability;


  if (size_x_ != width_ || size_y_ != height_ ||
         resolution_ != traversability->info.resolution ||
         origin_x_ != traversability->info.origin.position.x ||
         origin_y_ != traversability->info.origin.position.y)
  {
    // only update the size of the costmap stored locally in this layer
    ROS_INFO("Resizing traversability layer to %d X %d at %f m/pix", width_, height_, traversability->info.resolution);
    resizeMap(width_, height_, traversability->info.resolution,
         traversability->info.origin.position.x, traversability->info.origin.position.y);
  }

  size_x_ = width_;
  size_y_ = height_;
  resolution_ = traversability->info.resolution;
  origin_x_ = traversability->info.origin.position.x;
  origin_y_ = traversability->info.origin.position.y;
  map_frame_ = traversability->header.frame_id;

  unsigned int index = 0;

  // initialize the costmap with static data
  for (unsigned int i = 0; i < height_; ++i)
  {
    for (unsigned int j = 0; j < width_; ++j)
    {
      unsigned char value = traversability->data[index] / 100 * 254;
      costmap_[index] = interpretValue(value);
      ++index;
    }
  }

  if (!traversabilityMapReceived_) {
    traversabilityMapReceived_ = true;
  }
}

} // end namespace
