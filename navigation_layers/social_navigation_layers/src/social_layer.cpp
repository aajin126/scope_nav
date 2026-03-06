// Copyright 2018 David V. Lu!!
#include "social_navigation_layers/social_layer.h"

#include <geometry_msgs/msg/point_stamped.hpp>

#include <algorithm>

namespace social_navigation_layers
{

void SocialLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("SocialLayer: node is null");
  }

  current_ = true;
  first_time_ = true;

  // TF2 setup
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Parameters
  node->declare_parameter<std::string>(name_ + ".people_topic", "sogmp_prediction");
  node->declare_parameter<double>(name_ + ".keep_time", 0.0);

  const std::string people_topic = node->get_parameter(name_ + ".people_topic").as_string();
  const double keep_time = node->get_parameter(name_ + ".keep_time").as_double();
  people_keep_time_ = rclcpp::Duration::from_seconds(keep_time);

  // Subscribe (your ROS1 code forced "/" + topic; keep same behavior)
  const std::string full_topic = "/" + people_topic;

  people_sub_ = node->create_subscription<people_msgs::msg::People>(
    full_topic, rclcpp::QoS(1),
    std::bind(&SocialLayer::peopleCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node->get_logger(), "%s: subscribed to %s",
              name_.c_str(), full_topic.c_str());
}

void SocialLayer::peopleCallback(const people_msgs::msg::People::SharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> guard(lock_);
  people_list_ = *msg;
}

void SocialLayer::updateBounds(double origin_x, double origin_y, double origin_yaw,
                               double* min_x, double* min_y, double* max_x, double* max_y)
{
  (void)origin_x;
  (void)origin_y;
  (void)origin_yaw;

  auto node = node_.lock();
  if (!node) return;

  std::lock_guard<std::recursive_mutex> guard(lock_);

  const std::string global_frame = layered_costmap_->getGlobalFrameID();
  transformed_people_.clear();

  // If no data, just keep previous bounds union behavior minimal
  if (people_list_.people.empty()) {
    // still need to “touch” last bounds once to clear old region; keep original behavior below
  }

  for (size_t i = 0; i < people_list_.people.size(); i++) {
    const auto& person = people_list_.people[i];

    people_msgs::msg::Person tpt;

    geometry_msgs::msg::PointStamped pt, opt;
    pt.header.frame_id = people_list_.header.frame_id;
    pt.header.stamp = people_list_.header.stamp;

    // 1) transform person position
    pt.point.x = person.position.x;
    pt.point.y = person.position.y;
    pt.point.z = person.position.z;

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        global_frame, pt.header.frame_id, pt.header.stamp,
        rclcpp::Duration::from_seconds(0.2));  // small timeout (tune if needed)
      tf2::doTransform(pt, opt, tf);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                           "Transform failed (%s -> %s): %s",
                           pt.header.frame_id.c_str(), global_frame.c_str(), ex.what());
      continue;
    }

    tpt.position.x = opt.point.x;
    tpt.position.y = opt.point.y;
    tpt.position.z = opt.point.z;

    // 2) approximate velocity transform by transforming (position + velocity) and subtracting
    pt.point.x = person.position.x + person.velocity.x;
    pt.point.y = person.position.y + person.velocity.y;
    pt.point.z = person.position.z + person.velocity.z;

    try {
      tf2::doTransform(pt, opt, tf);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                           "Transform (vel) failed (%s -> %s): %s",
                           pt.header.frame_id.c_str(), global_frame.c_str(), ex.what());
      continue;
    }

    tpt.velocity.x = opt.point.x - tpt.position.x;
    tpt.velocity.y = opt.point.y - tpt.position.y;
    tpt.velocity.z = opt.point.z - tpt.position.z;

    transformed_people_.push_back(tpt);
  }

  // Let derived layer expand bounds around transformed_people_
  updateBoundsFromPeople(min_x, min_y, max_x, max_y);

  // Keep “union with last bounds” logic to ensure old footprints get cleared/updated
  if (first_time_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    first_time_ = false;
  } else {
    const double a = *min_x, b = *min_y, c = *max_x, d = *max_y;

    *min_x = std::min(last_min_x_, *min_x);
    *min_y = std::min(last_min_y_, *min_y);
    *max_x = std::max(last_max_x_, *max_x);
    *max_y = std::max(last_max_y_, *max_y);

    last_min_x_ = a;
    last_min_y_ = b;
    last_max_x_ = c;
    last_max_y_ = d;
  }
}

}  // namespace social_navigation_layers
