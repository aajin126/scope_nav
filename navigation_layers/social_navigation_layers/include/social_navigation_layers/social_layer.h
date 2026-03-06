// Copyright 2018 David V. Lu!!
#ifndef SOCIAL_NAVIGATION_LAYERS__SOCIAL_LAYER_HPP_
#define SOCIAL_NAVIGATION_LAYERS__SOCIAL_LAYER_HPP_

#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include "nav2_costmap_2d/layered_costmap.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <people_msgs/msg/people.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <list>
#include <string>
#include <vector>
#include <mutex>

namespace social_navigation_layers
{

class SocialLayer : public nav2_costmap_2d::Layer
{
public:
  SocialLayer() = default;
  ~SocialLayer() override = default;

  void reset() override
  {
    current_ = false;
  }

  bool isClearable() override
  {
    return true;
  }

  void onInitialize() override;

  void updateBounds(double origin_x, double origin_y, double origin_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;

  // derived classes implement this
  void updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                   int min_i, int min_j, int max_i, int max_j) override = 0;

  virtual void updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y) = 0;

  bool isDiscretized() { return false; }

protected:
  void peopleCallback(const people_msgs::msg::People::SharedPtr msg);

protected:
  rclcpp::Subscription<people_msgs::msg::People>::SharedPtr people_sub_;
  people_msgs::msg::People people_list_;
  std::list<people_msgs::msg::Person> transformed_people_;

  rclcpp::Duration people_keep_time_{0, 0};  // optional usage in derived layers

  std::recursive_mutex lock_;
  bool first_time_{true};
  double last_min_x_{0.0}, last_min_y_{0.0}, last_max_x_{0.0}, last_max_y_{0.0};

  // TF2 in ROS 2 (Nav2 doesn't give you tf_ like ROS1 costmap_2d did)
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace social_navigation_layers

#endif  // SOCIAL_NAVIGATION_LAYERS__SOCIAL_LAYER_HPP_
