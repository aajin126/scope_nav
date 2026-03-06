#include "social_navigation_layers/proxemic_layer.h"
#include "nav2_costmap_2d/layered_costmap.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(social_navigation_layers::ProxemicLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

namespace social_navigation_layers
{

double gaussian(double x, double y, double x0, double y0, double A,
                double varx, double vary, double skew)
{
  const double dx = x - x0, dy = y - y0;
  const double h = std::sqrt(dx * dx + dy * dy);
  const double angle = std::atan2(dy, dx);
  const double mx = std::cos(angle - skew) * h;
  const double my = std::sin(angle - skew) * h;

  const double f1 = std::pow(mx, 2.0) / (2.0 * varx);
  const double f2 = std::pow(my, 2.0) / (2.0 * vary);
  return A * std::exp(-(f1 + f2));
}

double get_radius(double cutoff, double A, double var)
{
  return std::sqrt(-2.0 * var * std::log(cutoff / A));
}

void ProxemicLayer::onInitialize()
{
  SocialLayer::onInitialize();

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("ProxemicLayer: node is null");
  }

  // Declare parameters under "<layer_name>.<param>" (Nav2 convention)
  node->declare_parameter<double>(name_ + ".cutoff", cutoff_);
  node->declare_parameter<double>(name_ + ".amplitude", amplitude_);
  node->declare_parameter<double>(name_ + ".covariance", covar_);
  node->declare_parameter<double>(name_ + ".factor", factor_);
  node->declare_parameter<double>(name_ + ".keep_time", keep_time_);
  node->declare_parameter<bool>(name_ + ".enabled", true);

  // Load initial values
  cutoff_ = node->get_parameter(name_ + ".cutoff").as_double();
  amplitude_ = node->get_parameter(name_ + ".amplitude").as_double();
  covar_ = node->get_parameter(name_ + ".covariance").as_double();
  factor_ = node->get_parameter(name_ + ".factor").as_double();
  keep_time_ = node->get_parameter(name_ + ".keep_time").as_double();
  enabled_ = node->get_parameter(name_ + ".enabled").as_bool();

  // keep_time_ is used by SocialLayer (originally ros::Duration)
  // Assume SocialLayer stores a Duration-like variable people_keep_time_
  people_keep_time_ = rclcpp::Duration::from_seconds(keep_time_);

  // Parameter update callback (dynamic_reconfigure replacement)
  param_cb_handle_ = node->add_on_set_parameters_callback(
    std::bind(&ProxemicLayer::onParamChange, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult
ProxemicLayer::onParamChange(const std::vector<rclcpp::Parameter>& params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto& p : params) {
    const auto& n = p.get_name();

    if (n == name_ + ".cutoff") cutoff_ = p.as_double();
    else if (n == name_ + ".amplitude") amplitude_ = p.as_double();
    else if (n == name_ + ".covariance") covar_ = p.as_double();
    else if (n == name_ + ".factor") factor_ = p.as_double();
    else if (n == name_ + ".keep_time") {
      keep_time_ = p.as_double();
      people_keep_time_ = rclcpp::Duration::from_seconds(keep_time_);
    }
    else if (n == name_ + ".enabled") enabled_ = p.as_bool();
  }

  return result;
}

void ProxemicLayer::updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y)
{
  for (auto it = transformed_people_.begin(); it != transformed_people_.end(); ++it) {
    const auto person = *it;

    const double mag = std::sqrt(std::pow(person.velocity.x, 2) + std::pow(person.velocity.y, 2));
    const double factor = 1.0 + mag * factor_;
    const double point = get_radius(cutoff_, amplitude_, covar_ * factor);

    *min_x = std::min(*min_x, person.position.x - point);
    *min_y = std::min(*min_y, person.position.y - point);
    *max_x = std::max(*max_x, person.position.x + point);
    *max_y = std::max(*max_y, person.position.y + point);
  }
}

void ProxemicLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                               int min_i, int min_j, int max_i, int max_j)
{
  // Keep the same locking pattern as ROS1 version (assumes SocialLayer defines lock_)
  std::lock_guard<std::recursive_mutex> guard(lock_);

  if (!enabled_) return;
  if (people_list_.people.empty()) return;
  if (cutoff_ >= amplitude_) return;

  nav2_costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
  const double res = costmap->getResolution();

  for (auto it = transformed_people_.begin(); it != transformed_people_.end(); ++it) {
    const auto person = *it;

    const double angle = std::atan2(person.velocity.y, person.velocity.x);
    const double mag = std::sqrt(std::pow(person.velocity.x, 2) + std::pow(person.velocity.y, 2));
    const double factor = 1.0 + mag * factor_;

    const double base = get_radius(cutoff_, amplitude_, covar_);
    const double point = get_radius(cutoff_, amplitude_, covar_ * factor);

    const unsigned int width  = std::max(1, static_cast<int>((base + point) / res));
    const unsigned int height = std::max(1, static_cast<int>((base + point) / res));

    const double cx = person.position.x, cy = person.position.y;

    double ox, oy;
    if (std::sin(angle) > 0) oy = cy - base;
    else oy = cy + (point - base) * std::sin(angle) - base;

    if (std::cos(angle) >= 0) ox = cx - base;
    else ox = cx + (point - base) * std::cos(angle) - base;

    int dx, dy;
    costmap->worldToMapNoBounds(ox, oy, dx, dy);

    int start_x = 0, start_y = 0;
    int end_x = static_cast<int>(width), end_y = static_cast<int>(height);

    if (dx < 0) start_x = -dx;
    else if (dx + static_cast<int>(width) > static_cast<int>(costmap->getSizeInCellsX()))
      end_x = std::max(0, static_cast<int>(costmap->getSizeInCellsX()) - dx);

    if (static_cast<int>(start_x + dx) < min_i) start_x = min_i - dx;
    if (static_cast<int>(end_x + dx) > max_i)   end_x = max_i - dx;

    if (dy < 0) start_y = -dy;
    else if (dy + static_cast<int>(height) > static_cast<int>(costmap->getSizeInCellsY()))
      end_y = std::max(0, static_cast<int>(costmap->getSizeInCellsY()) - dy);

    if (static_cast<int>(start_y + dy) < min_j) start_y = min_j - dy;
    if (static_cast<int>(end_y + dy) > max_j)   end_y = max_j - dy;

    const double bx = ox + res / 2.0;
    const double by = oy + res / 2.0;

    for (int i = start_x; i < end_x; i++) {
      for (int j = start_y; j < end_y; j++) {
        const unsigned char old_cost = costmap->getCost(i + dx, j + dy);
        if (old_cost == nav2_costmap_2d::NO_INFORMATION) continue;

        const double x = bx + i * res;
        const double y = by + j * res;

        const double ma = std::atan2(y - cy, x - cx);
        const double diff = angles::shortest_angular_distance(angle, ma);

        double a;
        if (std::fabs(diff) < M_PI / 2.0)
          a = gaussian(x, y, cx, cy, amplitude_, covar_ * factor, covar_, angle);
        else
          a = gaussian(x, y, cx, cy, amplitude_, covar_, covar_, 0.0);

        if (a < cutoff_) continue;

        const unsigned char cvalue = static_cast<unsigned char>(a);
        costmap->setCost(i + dx, j + dy, std::max(cvalue, old_cost));
      }
    }
  }

  (void)master_grid;  // kept signature; we update underlying costmap like original code
}

}  // namespace social_navigation_layers
