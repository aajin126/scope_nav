#include <social_navigation_layers/grid_prediction_layer.h>

#include <algorithm>
#include <string>
#include <vector>

#include <costmap_2d/cost_values.h>
#include <geometry_msgs/TransformStamped.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(social_navigation_layers::GridPredictionLayer, costmap_2d::Layer)

namespace social_navigation_layers
{
GridPredictionLayer::GridPredictionLayer()
: has_grid_(false), transform_tolerance_(0.3), min_value_(1), server_(NULL)
{
}

void GridPredictionLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);

  current_ = true;
  enabled_ = true;
  default_value_ = costmap_2d::NO_INFORMATION;

  matchSize();

  nh.param("grid_topic", grid_topic_, std::string("local_map"));
  nh.param("transform_tolerance", transform_tolerance_, 0.3);
  nh.param("min_value", min_value_, 1);

  server_ = new dynamic_reconfigure::Server<GridPredictionLayerConfig>(nh);
  reconfigure_cb_ = boost::bind(&GridPredictionLayer::configure, this, _1, _2);
  server_->setCallback(reconfigure_cb_);

  const std::string resolved_topic = grid_topic_.empty() || grid_topic_[0] == '/' ? grid_topic_ : "/" + grid_topic_;
  grid_sub_ = nh.subscribe(resolved_topic, 1, &GridPredictionLayer::gridCallback, this);
}

void GridPredictionLayer::configure(GridPredictionLayerConfig& config, uint32_t level)
{
  (void)level;
  boost::recursive_mutex::scoped_lock lock(lock_);
  enabled_ = config.enabled;
  min_value_ = config.min_value;
  transform_tolerance_ = config.transform_tolerance;
  current_ = false;
}

void GridPredictionLayer::matchSize()
{
  CostmapLayer::matchSize();
  default_value_ = costmap_2d::NO_INFORMATION;
  resetMaps();
}

void GridPredictionLayer::reset()
{
  boost::recursive_mutex::scoped_lock lock(lock_);
  resetMaps();
  current_ = false;
}

void GridPredictionLayer::gridCallback(const nav_msgs::OccupancyGridConstPtr& grid_msg)
{
  boost::recursive_mutex::scoped_lock lock(lock_);
  latest_grid_ = *grid_msg;
  has_grid_ = true;
  current_ = false;
}

void GridPredictionLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                       double* min_x, double* min_y, double* max_x, double* max_y)
{
  (void)robot_x;
  (void)robot_y;
  (void)robot_yaw;

  boost::recursive_mutex::scoped_lock lock(lock_);
  if (!enabled_ || !has_grid_)
  {
    return;
  }

  nav_msgs::OccupancyGrid grid = latest_grid_;
  const costmap_2d::Costmap2D* master = layered_costmap_->getCostmap();
  if (master->getSizeInCellsX() != getSizeInCellsX() || master->getSizeInCellsY() != getSizeInCellsY() ||
      master->getResolution() != getResolution() || master->getOriginX() != getOriginX() ||
      master->getOriginY() != getOriginY())
  {
    matchSize();
  }

  try
  {
    const std::string global_frame = layered_costmap_->getGlobalFrameID();
    const geometry_msgs::TransformStamped tf_msg = tf_->lookupTransform(
      global_frame, grid.header.frame_id, grid.header.stamp, ros::Duration(transform_tolerance_));

    tf2::Transform frame_to_global;
    tf2::fromMsg(tf_msg.transform, frame_to_global);

    tf2::Transform grid_origin;
    tf2::fromMsg(grid.info.origin, grid_origin);

    const tf2::Transform grid_to_global = frame_to_global * grid_origin;
    const double grid_width = static_cast<double>(grid.info.width) * grid.info.resolution;
    const double grid_height = static_cast<double>(grid.info.height) * grid.info.resolution;

    const std::vector<tf2::Vector3> corners{
      tf2::Vector3(0.0, 0.0, 0.0),
      tf2::Vector3(grid_width, 0.0, 0.0),
      tf2::Vector3(0.0, grid_height, 0.0),
      tf2::Vector3(grid_width, grid_height, 0.0),
    };

    for (std::vector<tf2::Vector3>::const_iterator it = corners.begin(); it != corners.end(); ++it)
    {
      const tf2::Vector3 global_corner = grid_to_global * (*it);
      touch(global_corner.x(), global_corner.y(), min_x, min_y, max_x, max_y);
    }

    current_ = true;
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1.0, "%s: failed to transform prediction grid from %s to %s: %s",
                      name_.c_str(), grid.header.frame_id.c_str(), layered_costmap_->getGlobalFrameID().c_str(),
                      ex.what());
  }
}

void GridPredictionLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  boost::recursive_mutex::scoped_lock lock(lock_);
  if (!enabled_ || !has_grid_)
  {
    return;
  }

  nav_msgs::OccupancyGrid grid = latest_grid_;
  resetMaps();

  try
  {
    const std::string global_frame = layered_costmap_->getGlobalFrameID();
    const geometry_msgs::TransformStamped tf_msg = tf_->lookupTransform(
      global_frame, grid.header.frame_id, grid.header.stamp, ros::Duration(transform_tolerance_));

    tf2::Transform frame_to_global;
    tf2::fromMsg(tf_msg.transform, frame_to_global);

    tf2::Transform grid_origin;
    tf2::fromMsg(grid.info.origin, grid_origin);

    const tf2::Transform grid_to_global = frame_to_global * grid_origin;
    const tf2::Transform global_to_grid = grid_to_global.inverse();

    const int bounded_min_i = std::max(0, min_i);
    const int bounded_min_j = std::max(0, min_j);
    const int bounded_max_i = std::min(static_cast<int>(getSizeInCellsX()), max_i);
    const int bounded_max_j = std::min(static_cast<int>(getSizeInCellsY()), max_j);

    unsigned char* costmap = getCharMap();

    const double master_origin_x = master_grid.getOriginX();
    const double master_origin_y = master_grid.getOriginY();
    const double master_resolution = master_grid.getResolution();

    const double grid_resolution = grid.info.resolution;
    const int grid_width = static_cast<int>(grid.info.width);
    const int grid_height = static_cast<int>(grid.info.height);

    #pragma omp parallel for collapse(2) num_threads(8)
    for (int mx = bounded_min_i; mx < bounded_max_i; ++mx)
    {
        for (int my = bounded_min_j; my < bounded_max_j; ++my)
        {
            const double wx =
            master_origin_x + (static_cast<double>(mx) + 0.5) * master_resolution;
            const double wy =
            master_origin_y + (static_cast<double>(my) + 0.5) * master_resolution;

            const tf2::Vector3 global_cell(wx, wy, 0.0);
            const tf2::Vector3 grid_cell = global_to_grid * global_cell;

            if (grid_cell.x() < 0.0 || grid_cell.y() < 0.0)
            {
            continue;
            }

            const int source_x = static_cast<int>(grid_cell.x() / grid_resolution);
            const int source_y = static_cast<int>(grid_cell.y() / grid_resolution);

            if (source_x < 0 || source_y < 0 ||
                source_x >= grid_width || source_y >= grid_height)
            {
            continue;
            }

            const unsigned int source_index =
            static_cast<unsigned int>(source_y) * grid.info.width +
            static_cast<unsigned int>(source_x);

            const int value = grid.data[source_index];

            if (value < min_value_)
            {
            continue;
            }

            const unsigned char cost = static_cast<unsigned char>(
            std::min<int>(
                costmap_2d::LETHAL_OBSTACLE,
                (value * costmap_2d::LETHAL_OBSTACLE) / 100));

            const unsigned int target_index = getIndex(mx, my);
            costmap[target_index] = cost;
        }
    }
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1.0, "%s: failed to transform prediction grid from %s to %s: %s",
                      name_.c_str(), grid.header.frame_id.c_str(), layered_costmap_->getGlobalFrameID().c_str(),
                      ex.what());
    return;
  }

  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}
}  // namespace social_navigation_layers
