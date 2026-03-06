#ifndef RANGE_SENSOR_LAYER__RANGE_SENSOR_LAYER_HPP_
#define RANGE_SENSOR_LAYER__RANGE_SENSOR_LAYER_HPP_

#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <sensor_msgs/msg/range.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <angles/angles.h>

#include <list>
#include <map>
#include <string>
#include <utility>
#include <vector>
#include <mutex>
#include <functional>
#include <limits>
#include <cmath>

namespace range_sensor_layer
{

class RangeSensorLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  enum InputSensorType { VARIABLE, FIXED, ALL };

  RangeSensorLayer() = default;
  ~RangeSensorLayer() override = default;
  bool isClearable() override { return true; }
  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;

  void reset() override;
  void deactivate() override;
  void activate() override;

private:
  // ROS2 parameter update callback
  rcl_interfaces::msg::SetParametersResult
  onParamChange(const std::vector<rclcpp::Parameter>& params);

  void bufferIncomingRangeMsg(const sensor_msgs::msg::Range::SharedPtr msg);

  void updateCostmap();
  void processRangeMsg(sensor_msgs::msg::Range& msg);
  void processFixedRangeMsg(sensor_msgs::msg::Range& msg);
  void processVariableRangeMsg(sensor_msgs::msg::Range& msg);

  void resetRange();
  void updateCostmap(sensor_msgs::msg::Range& msg, bool clear_sensor_cone);
  void removeOutdatedReadings();

  double gamma(double theta);
  double delta(double phi);
  double sensor_model(double r, double phi, double theta);

  void get_deltas(double angle, double* dx, double* dy);
  void update_cell(double ox, double oy, double ot, double r,
                   double nx, double ny, bool clear);

  inline double to_prob(unsigned char c) const
  {
    return static_cast<double>(c) / nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  inline unsigned char to_cost(double p) const
  {
    return static_cast<unsigned char>(p * nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  // Triangle helpers
  float area(int x1, int y1, int x2, int y2, int x3, int y3)
  {
    return std::fabs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0);
  }
  int orient2d(int Ax, int Ay, int Bx, int By, int Cx, int Cy)
  {
    return (Bx - Ax) * (Cy - Ay) - (By - Ay) * (Cx - Ax);
  }

private:
  // function pointer to choose processing mode
  std::function<void(sensor_msgs::msg::Range&)> processRangeMessageFunc_;

  std::mutex range_mutex_;
  std::list<sensor_msgs::msg::Range> range_msgs_buffer_;

  std::map<std::pair<unsigned int, unsigned int>, double> marked_point_history_;

  double max_angle_{0.0}, phi_v_{0.0};
  double inflate_cone_{1.0};
  std::string global_frame_;

  double clear_threshold_{0.2}, mark_threshold_{0.8};
  bool clear_on_max_reading_{false};

  double no_readings_timeout_{0.0};
  rclcpp::Time last_reading_time_{0, 0, RCL_ROS_TIME};
  unsigned int buffered_readings_{0};

  std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr> range_subs_;

  double min_x_{0.0}, min_y_{0.0}, max_x_{0.0}, max_y_{0.0};

  bool use_decay_{false};
  double pixel_decay_{10.0};
  double transform_tolerance_{0.3};

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace range_sensor_layer

#endif  // RANGE_SENSOR_LAYER__RANGE_SENSOR_LAYER_HPP_
