#include <social_navigation_layers/probability_layer.h>
#include <social_navigation_layers/utils.h>
#include <math.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <list>
PLUGINLIB_EXPORT_CLASS(social_navigation_layers::ProbabilityLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace social_navigation_layers
{
void ProbabilityLayer::onInitialize()
{
  CrowdLayer::onInitialize();
  ros::NodeHandle nh("~/" + name_), g_nh;
  server_ = new dynamic_reconfigure::Server<ProbabilityLayerConfig>(nh);
  f_ = boost::bind(&ProbabilityLayer::configure, this, _1, _2);
  server_->setCallback(f_);
}

void ProbabilityLayer::updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y)
{
  std::list<people_msgs::Person>::iterator p_it;

  for (p_it = transformed_people_.begin(); p_it != transformed_people_.end(); ++p_it)
  {
    people_msgs::Person person = *p_it;

    // Calculate influence radius based on probability (0~254)
    double prob = person.probability;
    double prob_norm = prob / 254.0; // 0~1
    double base_point = get_radius(cutoff_, amplitude_, covar_);
    double point = base_point * prob_norm;

    *min_x = std::min(*min_x, person.position.x - point);
    *min_y = std::min(*min_y, person.position.y - point);
    *max_x = std::max(*max_x, person.position.x + point);
    *max_y = std::max(*max_y, person.position.y + point);
  }
}

void ProbabilityLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  boost::recursive_mutex::scoped_lock lock(lock_);
  if (!enabled_) return;

  if (people_list_.people.size() == 0)
    return;
  if (cutoff_ >= amplitude_)
    return;

  std::list<people_msgs::Person>::iterator p_it;
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
  double res = costmap->getResolution();

  for (p_it = transformed_people_.begin(); p_it != transformed_people_.end(); ++p_it)
  {
    people_msgs::Person person = *p_it;
    double amplitude = person.probability; // 0~254
    if (amplitude <= 0.0)
      continue; 

    double base = get_radius(cutoff_, amplitude, covar_);
    double point = base;

    unsigned int width = std::max(1, static_cast<int>((2 * point) / res));
    unsigned int height = std::max(1, static_cast<int>((2 * point) / res));

    double cx = person.position.x, cy = person.position.y;
    double ox = cx - point;
    double oy = cy - point;

    int dx, dy;
    costmap->worldToMapNoBounds(ox, oy, dx, dy);

    int start_x = 0, start_y = 0, end_x = width, end_y = height;
    if (dx < 0)
      start_x = -dx;
    else if (dx + width > costmap->getSizeInCellsX())
      end_x = std::max(0, static_cast<int>(costmap->getSizeInCellsX()) - dx);

    if (static_cast<int>(start_x + dx) < min_i)
      start_x = min_i - dx;
    if (static_cast<int>(end_x + dx) > max_i)
      end_x = max_i - dx;

    if (dy < 0)
      start_y = -dy;
    else if (dy + height > costmap->getSizeInCellsY())
      end_y = std::max(0, static_cast<int>(costmap->getSizeInCellsY()) - dy);

    if (static_cast<int>(start_y + dy) < min_j)
      start_y = min_j - dy;
    if (static_cast<int>(end_y + dy) > max_j)
      end_y = max_j - dy;

    double bx = ox + res / 2,
           by = oy + res / 2;
    for (int i = start_x; i < end_x; i++)
    {
      for (int j = start_y; j < end_y; j++)
      {
        unsigned char old_cost = costmap->getCost(i + dx, j + dy);
        if (old_cost == costmap_2d::NO_INFORMATION)
          continue;

        double x = bx + i * res, y = by + j * res;
        double a = gaussian(x, y, cx, cy, amplitude, covar_, covar_, 0);
        if (a < cutoff_)
          continue;
        unsigned char cvalue = (unsigned char)a;
        costmap->setCost(i + dx, j + dy, std::max(cvalue, old_cost));
      }
    }
  }
}

void ProbabilityLayer::configure(ProbabilityLayerConfig &config, uint32_t level)
{
  cutoff_ = config.cutoff;
  amplitude_ = config.amplitude;
  covar_ = config.covariance;
  people_keep_time_ = ros::Duration(config.keep_time);
  enabled_ = config.enabled;
}
};  // namespace social_navigation_layers
