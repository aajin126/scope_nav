#ifndef SOCIAL_NAVIGATION_LAYERS_GRID_PREDICTION_LAYER_H
#define SOCIAL_NAVIGATION_LAYERS_GRID_PREDICTION_LAYER_H

#include <boost/thread.hpp>
#include <costmap_2d/costmap_layer.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <social_navigation_layers/GridPredictionLayerConfig.h>
#include <string>

namespace social_navigation_layers
{
class GridPredictionLayer : public costmap_2d::CostmapLayer
{
public:
  GridPredictionLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual void matchSize();
  virtual void reset();

  virtual bool isDiscretized()
  {
    return true;
  }

private:
  void configure(GridPredictionLayerConfig& config, uint32_t level);
  void gridCallback(const nav_msgs::OccupancyGridConstPtr& grid_msg);

  ros::Subscriber grid_sub_;
  nav_msgs::OccupancyGrid latest_grid_;
  boost::recursive_mutex lock_;
  bool has_grid_;
  std::string grid_topic_;
  double transform_tolerance_;
  int min_value_;
  dynamic_reconfigure::Server<GridPredictionLayerConfig>* server_;
  dynamic_reconfigure::Server<GridPredictionLayerConfig>::CallbackType reconfigure_cb_;
};
}  // namespace social_navigation_layers

#endif  // SOCIAL_NAVIGATION_LAYERS_GRID_PREDICTION_LAYER_H
