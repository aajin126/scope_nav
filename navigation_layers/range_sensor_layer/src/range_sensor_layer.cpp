#include "range_sensor_layer/range_sensor_layer.h"

#include <pluginlib/class_list_macros.hpp>
#include <boost/algorithm/string.hpp>

using nav2_costmap_2d::NO_INFORMATION;

PLUGINLIB_EXPORT_CLASS(range_sensor_layer::RangeSensorLayer, nav2_costmap_2d::Layer)

namespace range_sensor_layer
{

void RangeSensorLayer::onInitialize()
{
  // Nav2 CostmapLayer gives you a lifecycle node weak ptr
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("RangeSensorLayer: node is null");
  }

  current_ = true;
  buffered_readings_ = 0;
  last_reading_time_ = node->now();
  default_value_ = to_cost(0.5);

  matchSize();
  resetRange();

  // TF buffer/listener (use node clock)
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Declare parameters (dynamic_reconfigure -> parameters)
  node->declare_parameter<std::string>(name_ + ".ns", "");
  node->declare_parameter<std::vector<std::string>>(name_ + ".topics", std::vector<std::string>{"/sonar"});
  node->declare_parameter<std::string>(name_ + ".input_sensor_type", "ALL");

  node->declare_parameter<bool>(name_ + ".use_decay", false);
  node->declare_parameter<double>(name_ + ".pixel_decay", 10.0);
  node->declare_parameter<double>(name_ + ".transform_tolerance", 0.3);

  node->declare_parameter<double>(name_ + ".phi", 0.0);
  node->declare_parameter<double>(name_ + ".inflate_cone", 1.0);
  node->declare_parameter<double>(name_ + ".no_readings_timeout", 0.0);
  node->declare_parameter<double>(name_ + ".clear_threshold", 0.2);
  node->declare_parameter<double>(name_ + ".mark_threshold", 0.8);
  node->declare_parameter<bool>(name_ + ".clear_on_max_reading", false);
  node->declare_parameter<bool>(name_ + ".enabled", true);

  // Load params
  std::string topics_ns = node->get_parameter(name_ + ".ns").as_string();
  auto topics = node->get_parameter(name_ + ".topics").as_string_array();

  std::string sensor_type_name = node->get_parameter(name_ + ".input_sensor_type").as_string();
  use_decay_ = node->get_parameter(name_ + ".use_decay").as_bool();
  pixel_decay_ = node->get_parameter(name_ + ".pixel_decay").as_double();
  transform_tolerance_ = node->get_parameter(name_ + ".transform_tolerance").as_double();

  phi_v_ = node->get_parameter(name_ + ".phi").as_double();
  inflate_cone_ = node->get_parameter(name_ + ".inflate_cone").as_double();
  no_readings_timeout_ = node->get_parameter(name_ + ".no_readings_timeout").as_double();
  clear_threshold_ = node->get_parameter(name_ + ".clear_threshold").as_double();
  mark_threshold_ = node->get_parameter(name_ + ".mark_threshold").as_double();
  clear_on_max_reading_ = node->get_parameter(name_ + ".clear_on_max_reading").as_bool();
  enabled_ = node->get_parameter(name_ + ".enabled").as_bool();

  boost::to_upper(sensor_type_name);
  RCLCPP_INFO(node->get_logger(), "%s: %s as input_sensor_type given", name_.c_str(), sensor_type_name.c_str());

  InputSensorType input_sensor_type = ALL;
  if (sensor_type_name == "VARIABLE") input_sensor_type = VARIABLE;
  else if (sensor_type_name == "FIXED") input_sensor_type = FIXED;
  else if (sensor_type_name == "ALL") input_sensor_type = ALL;
  else {
    RCLCPP_ERROR(node->get_logger(), "%s: Invalid input sensor type: %s",
                 name_.c_str(), sensor_type_name.c_str());
  }

  if (input_sensor_type == VARIABLE)
    processRangeMessageFunc_ = [this](sensor_msgs::msg::Range& m){ processVariableRangeMsg(m); };
  else if (input_sensor_type == FIXED)
    processRangeMessageFunc_ = [this](sensor_msgs::msg::Range& m){ processFixedRangeMsg(m); };
  else
    processRangeMessageFunc_ = [this](sensor_msgs::msg::Range& m){ processRangeMsg(m); };

  // Subscribe to topics
  range_subs_.clear();
  for (const auto& t : topics) {
    std::string topic_name = topics_ns;
    if (!topic_name.empty() && topic_name.back() != '/') topic_name += "/";
    topic_name += t;

    auto sub = node->create_subscription<sensor_msgs::msg::Range>(
      topic_name, rclcpp::SensorDataQoS(),
      std::bind(&RangeSensorLayer::bufferIncomingRangeMsg, this, std::placeholders::_1));

    range_subs_.push_back(sub);
    RCLCPP_INFO(node->get_logger(), "RangeSensorLayer: subscribed to topic %s", topic_name.c_str());
  }

  // Global frame from layered costmap
  global_frame_ = layered_costmap_->getGlobalFrameID();

  // Parameter callback
  param_cb_handle_ = node->add_on_set_parameters_callback(
    std::bind(&RangeSensorLayer::onParamChange, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult
RangeSensorLayer::onParamChange(const std::vector<rclcpp::Parameter>& params)
{
  auto node = node_.lock();
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto& p : params) {
    const auto& n = p.get_name();

    // only react to our namespace: "<layer_name>.<param>"
    if (n == name_ + ".phi") phi_v_ = p.as_double();
    else if (n == name_ + ".inflate_cone") inflate_cone_ = p.as_double();
    else if (n == name_ + ".no_readings_timeout") no_readings_timeout_ = p.as_double();
    else if (n == name_ + ".clear_threshold") clear_threshold_ = p.as_double();
    else if (n == name_ + ".mark_threshold") mark_threshold_ = p.as_double();
    else if (n == name_ + ".clear_on_max_reading") clear_on_max_reading_ = p.as_bool();
    else if (n == name_ + ".use_decay") use_decay_ = p.as_bool();
    else if (n == name_ + ".pixel_decay") pixel_decay_ = p.as_double();
    else if (n == name_ + ".transform_tolerance") transform_tolerance_ = p.as_double();
    else if (n == name_ + ".enabled") {
      if (enabled_ != p.as_bool()) {
        enabled_ = p.as_bool();
        current_ = false;
      }
    }
  }

  (void)node;
  return result;
}

void RangeSensorLayer::bufferIncomingRangeMsg(const sensor_msgs::msg::Range::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(range_mutex_);
  range_msgs_buffer_.push_back(*msg);
}

void RangeSensorLayer::updateCostmap()
{
  std::list<sensor_msgs::msg::Range> copy;
  {
    std::lock_guard<std::mutex> lock(range_mutex_);
    copy = range_msgs_buffer_;
    range_msgs_buffer_.clear();
  }

  for (auto& m : copy) {
    processRangeMessageFunc_(m);
  }
}

double RangeSensorLayer::gamma(double theta)
{
  if (std::fabs(theta) > max_angle_) return 0.0;
  return 1.0 - std::pow(theta / max_angle_, 2);
}

double RangeSensorLayer::delta(double phi)
{
  return 1.0 - (1.0 + std::tanh(2.0 * (phi - phi_v_))) / 2.0;
}

void RangeSensorLayer::get_deltas(double angle, double* dx, double* dy)
{
  double ta = std::tan(angle);
  if (ta == 0) *dx = 0;
  else *dx = resolution_ / ta;

  *dx = std::copysign(*dx, std::cos(angle));
  *dy = std::copysign(resolution_, std::sin(angle));
}

double RangeSensorLayer::sensor_model(double r, double phi, double theta)
{
  double lbda = delta(phi) * gamma(theta);
  double d = resolution_;

  if (phi >= 0.0 && phi < r - 2 * d * r) return (1 - lbda) * 0.5;
  else if (phi < r - d * r)
    return lbda * 0.5 * std::pow((phi - (r - 2 * d * r)) / (d * r), 2) + (1 - lbda) * 0.5;
  else if (phi < r + d * r) {
    double J = (r - phi) / (d * r);
    return lbda * ((1 - 0.5 * std::pow(J, 2)) - 0.5) + 0.5;
  }
  return 0.5;
}

void RangeSensorLayer::processRangeMsg(sensor_msgs::msg::Range& msg)
{
  if (msg.min_range == msg.max_range) processFixedRangeMsg(msg);
  else processVariableRangeMsg(msg);
}

void RangeSensorLayer::processFixedRangeMsg(sensor_msgs::msg::Range& msg)
{
  auto node = node_.lock();
  if (!node) return;

  if (!std::isinf(msg.range)) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
      "Fixed distance ranger (min_range == max_range) in frame %s sent invalid value. "
      "Only -Inf (== object detected) and Inf (== no object detected) are valid.",
      msg.header.frame_id.c_str());
    return;
  }

  bool clear_sensor_cone = false;
  if (msg.range > 0) { // +inf
    if (!clear_on_max_reading_) return;
    clear_sensor_cone = true;
  }

  msg.range = msg.min_range;
  updateCostmap(msg, clear_sensor_cone);
}

void RangeSensorLayer::processVariableRangeMsg(sensor_msgs::msg::Range& msg)
{
  if (msg.range < msg.min_range || msg.range > msg.max_range) return;

  bool clear_sensor_cone = (msg.range == msg.max_range && clear_on_max_reading_);
  updateCostmap(msg, clear_sensor_cone);
}

void RangeSensorLayer::updateCostmap(sensor_msgs::msg::Range& msg, bool clear_sensor_cone)
{
  auto node = node_.lock();
  if (!node) return;

  max_angle_ = msg.field_of_view / 2.0;

  geometry_msgs::msg::PointStamped in, out;
  in.header.stamp = msg.header.stamp;
  in.header.frame_id = msg.header.frame_id;

  // TF2 lookup
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(global_frame_, in.header.frame_id, in.header.stamp,
                                     rclcpp::Duration::from_seconds(transform_tolerance_));
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
      "Range sensor layer can't transform from %s to %s: %s",
      global_frame_.c_str(), in.header.frame_id.c_str(), ex.what());
    return;
  }

  tf2::doTransform(in, out, tf);
  double ox = out.point.x, oy = out.point.y;

  in.point.x = msg.range;
  tf2::doTransform(in, out, tf);
  double tx = out.point.x, ty = out.point.y;

  double dx = tx - ox, dy = ty - oy;
  double theta = std::atan2(dy, dx);
  double d = std::sqrt(dx * dx + dy * dy);

  int bx0, by0, bx1, by1;
  int Ox, Oy, Ax, Ay, Bx, By;

  worldToMapNoBounds(ox, oy, Ox, Oy);
  bx1 = bx0 = Ox;
  by1 = by0 = Oy;
  touch(ox, oy, &min_x_, &min_y_, &max_x_, &max_y_);

  unsigned int aa, ab;
  if (worldToMap(tx, ty, aa, ab)) {
    setCost(aa, ab, 233);
    touch(tx, ty, &min_x_, &min_y_, &max_x_, &max_y_);
  }

  double mx, my;

  mx = ox + std::cos(theta - max_angle_) * d * 1.2;
  my = oy + std::sin(theta - max_angle_) * d * 1.2;
  worldToMapNoBounds(mx, my, Ax, Ay);
  bx0 = std::min(bx0, Ax); bx1 = std::max(bx1, Ax);
  by0 = std::min(by0, Ay); by1 = std::max(by1, Ay);
  touch(mx, my, &min_x_, &min_y_, &max_x_, &max_y_);

  mx = ox + std::cos(theta + max_angle_) * d * 1.2;
  my = oy + std::sin(theta + max_angle_) * d * 1.2;
  worldToMapNoBounds(mx, my, Bx, By);
  bx0 = std::min(bx0, Bx); bx1 = std::max(bx1, Bx);
  by0 = std::min(by0, By); by1 = std::max(by1, By);
  touch(mx, my, &min_x_, &min_y_, &max_x_, &max_y_);

  bx0 = std::max(0, bx0);
  by0 = std::max(0, by0);
  bx1 = std::min(static_cast<int>(size_x_), bx1);
  by1 = std::min(static_cast<int>(size_y_), by1);

  for (unsigned int x = bx0; x <= static_cast<unsigned int>(bx1); x++) {
    for (unsigned int y = by0; y <= static_cast<unsigned int>(by1); y++) {
      bool update_xy_cell = true;

      if (inflate_cone_ < 1.0) {
        int w0 = orient2d(Ax, Ay, Bx, By, x, y);
        int w1 = orient2d(Bx, By, Ox, Oy, x, y);
        int w2 = orient2d(Ox, Oy, Ax, Ay, x, y);

        float bcciath = -inflate_cone_ * area(Ax, Ay, Bx, By, Ox, Oy);
        update_xy_cell = w0 >= bcciath && w1 >= bcciath && w2 >= bcciath;
      }

      if (update_xy_cell) {
        double wx, wy;
        mapToWorld(x, y, wx, wy);
        update_cell(ox, oy, theta, msg.range, wx, wy, clear_sensor_cone);
      }
    }
  }

  buffered_readings_++;
  last_reading_time_ = node->now();
  if (use_decay_) removeOutdatedReadings();
}

void RangeSensorLayer::removeOutdatedReadings()
{
  // WARNING: your original code had erase-while-iterating UB.
  // Fix it with iterator-safe loop.
  auto node = node_.lock();
  if (!node) return;

  const double removal_time = last_reading_time_.seconds() - pixel_decay_;

  for (auto it = marked_point_history_.begin(); it != marked_point_history_.end(); ) {
    if (it->second < removal_time) {
      const auto cell = it->first;
      it = marked_point_history_.erase(it);
      setCost(cell.first, cell.second, nav2_costmap_2d::FREE_SPACE);
    } else {
      ++it;
    }
  }
}

void RangeSensorLayer::update_cell(double ox, double oy, double ot, double r,
                                  double nx, double ny, bool clear)
{
  unsigned int x, y;
  if (!worldToMap(nx, ny, x, y)) return;

  double dx = nx - ox, dy = ny - oy;
  double theta = std::atan2(dy, dx) - ot;
  theta = angles::normalize_angle(theta);
  double phi = std::sqrt(dx * dx + dy * dy);

  double sensor = 0.0;
  if (!clear) sensor = sensor_model(r, phi, theta);

  double prior = to_prob(getCost(x, y));
  double prob_occ = sensor * prior;
  double prob_not = (1 - sensor) * (1 - prior);
  double new_prob = prob_occ / (prob_occ + prob_not);

  unsigned char c = to_cost(new_prob);
  setCost(x, y, c);

  if (use_decay_) {
    std::pair<unsigned int, unsigned int> p(x, y);
    if (c > to_cost(mark_threshold_)) {
      marked_point_history_[p] = last_reading_time_.seconds();
    } else if (c < to_cost(clear_threshold_)) {
      auto it = marked_point_history_.find(p);
      if (it != marked_point_history_.end()) marked_point_history_.erase(it);
    }
  }
}

void RangeSensorLayer::resetRange()
{
  min_x_ = min_y_ =  std::numeric_limits<double>::max();
  max_x_ = max_y_ = -std::numeric_limits<double>::max();
}

void RangeSensorLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                   double* min_x, double* min_y, double* max_x, double* max_y)
{
  (void)robot_yaw;
  auto node = node_.lock();
  if (!node) return;

  if (layered_costmap_->isRolling()) {
    updateOrigin(robot_x - getSizeInMetersX() / 2.0, robot_y - getSizeInMetersY() / 2.0);
  }

  updateCostmap();

  *min_x = std::min(*min_x, min_x_);
  *min_y = std::min(*min_y, min_y_);
  *max_x = std::max(*max_x, max_x_);
  *max_y = std::max(*max_y, max_y_);

  resetRange();

  if (!enabled_) {
    current_ = true;
    return;
  }

  if (buffered_readings_ == 0) {
    if (no_readings_timeout_ > 0.0 &&
        (node->now() - last_reading_time_).seconds() > no_readings_timeout_) {
      RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
        "No range readings received for %.2f seconds, while expected at least every %.2f seconds.",
        (node->now() - last_reading_time_).seconds(), no_readings_timeout_);
      current_ = false;
    }
  }
}

void RangeSensorLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                                  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) return;

  unsigned char* master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();
  unsigned char clear = to_cost(clear_threshold_);
  unsigned char mark  = to_cost(mark_threshold_);

  for (int j = min_j; j < max_j; j++) {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++) {
      unsigned char prob = costmap_[it];
      unsigned char current;

      if (prob == nav2_costmap_2d::NO_INFORMATION) { it++; continue; }
      else if (prob > mark) current = nav2_costmap_2d::LETHAL_OBSTACLE;
      else if (prob < clear) current = nav2_costmap_2d::FREE_SPACE;
      else { it++; continue; }

      unsigned char old_cost = master_array[it];
      if (old_cost == NO_INFORMATION || old_cost < current) master_array[it] = current;
      it++;
    }
  }

  buffered_readings_ = 0;
  current_ = true;
}

void RangeSensorLayer::reset()
{
  auto node = node_.lock();
  if (node) RCLCPP_DEBUG(node->get_logger(), "Resetting range sensor layer...");
  deactivate();
  resetMaps();
  current_ = true;
  activate();
}

void RangeSensorLayer::deactivate()
{
  std::lock_guard<std::mutex> lock(range_mutex_);
  range_msgs_buffer_.clear();
}

void RangeSensorLayer::activate()
{
  std::lock_guard<std::mutex> lock(range_mutex_);
  range_msgs_buffer_.clear();
}

}  // namespace range_sensor_layer
