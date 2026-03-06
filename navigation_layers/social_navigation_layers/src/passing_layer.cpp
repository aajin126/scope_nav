// Copyright 2018 David V. Lu!!
#include "social_navigation_layers/proxemic_layer.h"
#include "social_navigation_layers/social_layer.h"  // for lock_, people_list_, transformed_people_
#include <geometry_msgs/msg/point_stamped.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <angles/angles.h>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <string>

namespace social_navigation_layers
{

class PassingLayer : public ProxemicLayer
{
public:
  PassingLayer() = default;
  ~PassingLayer() override = default;

  void updateBounds(double origin_x, double origin_y, double origin_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override
  {
    (void)origin_x;
    (void)origin_y;
    (void)origin_yaw;

    auto node = node_.lock();
    if (!node) return;

    std::lock_guard<std::recursive_mutex> guard(lock_);

    const std::string global_frame = layered_costmap_->getGlobalFrameID();
    transformed_people_.clear();

    for (size_t i = 0; i < people_list_.people.size(); i++)
    {
      const auto& person = people_list_.people[i];
      people_msgs::msg::Person tpt;

      geometry_msgs::msg::PointStamped pt, opt;
      pt.header.frame_id = people_list_.header.frame_id;
      pt.header.stamp = people_list_.header.stamp;

      // TF for this frame/time
      geometry_msgs::msg::TransformStamped tf;
      try {
        tf = tf_buffer_->lookupTransform(
          global_frame, pt.header.frame_id, pt.header.stamp,
          rclcpp::Duration::from_seconds(0.2));
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
          "PassingLayer: transform failed (%s -> %s): %s",
          pt.header.frame_id.c_str(), global_frame.c_str(), ex.what());
        continue;
      }

      // 1) position transform
      pt.point.x = person.position.x;
      pt.point.y = person.position.y;
      pt.point.z = person.position.z;
      tf2::doTransform(pt, opt, tf);

      tpt.position.x = opt.point.x;
      tpt.position.y = opt.point.y;
      tpt.position.z = opt.point.z;

      // 2) velocity transform trick: transform (pos + vel) and subtract,
      // BUT this layer wants reversed velocity (note the minus in original code)
      pt.point.x = person.position.x + person.velocity.x;
      pt.point.y = person.position.y + person.velocity.y;
      pt.point.z = person.position.z + person.velocity.z;
      tf2::doTransform(pt, opt, tf);

      // ORIGINAL PassingLayer intentionally flips direction:
      // tpt.velocity = (tpt.position - transformed(pos+vel))
      tpt.velocity.x = tpt.position.x - opt.point.x;
      tpt.velocity.y = tpt.position.y - opt.point.y;
      tpt.velocity.z = tpt.position.z - opt.point.z;

      transformed_people_.push_back(tpt);

      // bounds expansion (same math as ROS1, keep the original bug too? -> fix mag y term)
      // NOTE: ROS1 code used pow(person.velocity.y, 2) (not tpt.velocity.y).
      // That’s likely a bug; using tpt.velocity is more consistent after transform.
      const double mag = std::sqrt(std::pow(tpt.velocity.x, 2) + std::pow(tpt.velocity.y, 2));
      const double factor = 1.0 + mag * factor_;
      const double point = get_radius(cutoff_, amplitude_, covar_ * factor);

      *min_x = std::min(*min_x, tpt.position.x - point);
      *min_y = std::min(*min_y, tpt.position.y - point);
      *max_x = std::max(*max_x, tpt.position.x + point);
      *max_y = std::max(*max_y, tpt.position.y + point);
    }
  }

  void updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                   int min_i, int min_j, int max_i, int max_j) override
  {
    (void)master_grid;

    std::lock_guard<std::recursive_mutex> guard(lock_);
    if (!enabled_) return;

    if (people_list_.people.empty()) return;
    if (cutoff_ >= amplitude_) return;

    auto* costmap = layered_costmap_->getCostmap();
    const double res = costmap->getResolution();

    for (auto it = transformed_people_.begin(); it != transformed_people_.end(); ++it)
    {
      const auto person = *it;

      double angle = std::atan2(person.velocity.y, person.velocity.x) + 1.51;
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

      for (int i = start_x; i < end_x; i++)
      {
        for (int j = start_y; j < end_y; j++)
        {
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
            continue;

          if (a < cutoff_) continue;

          const unsigned char cvalue = static_cast<unsigned char>(a);
          costmap->setCost(i + dx, j + dy, std::max(cvalue, old_cost));
        }
      }
    }
  }
};

}  // namespace social_navigation_layers

PLUGINLIB_EXPORT_CLASS(social_navigation_layers::PassingLayer, nav2_costmap_2d::Layer)
