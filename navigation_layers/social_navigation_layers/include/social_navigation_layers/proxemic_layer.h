// Copyright 2018 David V. Lu!!
#ifndef SOCIAL_NAVIGATION_LAYERS__PROXEMIC_LAYER_HPP_
#define SOCIAL_NAVIGATION_LAYERS__PROXEMIC_LAYER_HPP_

#include "social_navigation_layers/social_layer.h"

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "nav2_costmap_2d/layered_costmap.hpp"

#include <angles/angles.h>

#include <algorithm>
#include <cmath>

namespace social_navigation_layers
{

double gaussian(double x, double y, double x0, double y0, double A,
                double varx, double vary, double skew);

double get_radius(double cutoff, double A, double var);

class ProxemicLayer : public SocialLayer
{
public:
  ProxemicLayer() = default;
  ~ProxemicLayer() override = default;

  void onInitialize() override;
  void updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;

private:
  rcl_interfaces::msg::SetParametersResult
  onParamChange(const std::vector<rclcpp::Parameter>& params);

protected:
  double cutoff_{10.0};
  double amplitude_{100.0};
  double covar_{0.5};
  double factor_{1.0};
  double keep_time_{0.5};  // seconds

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace social_navigation_layers

#endif  // SOCIAL_NAVIGATION_LAYERS__PROXEMIC_LAYER_HPP_
