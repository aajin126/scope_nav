#ifndef SOCIAL_NAVIGATION_LAYERS_PROBABILITY_LAYER_H
#define SOCIAL_NAVIGATION_LAYERS_PROBABILITY_LAYER_H
#include <ros/ros.h>
#include <social_navigation_layers/crowd_layer.h>
#include <dynamic_reconfigure/server.h>
#include <social_navigation_layers/ProbabilityLayerConfig.h>

double gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew);
double get_radius(double cutoff, double A, double var);

namespace social_navigation_layers
{
class ProbabilityLayer : public CrowdLayer
{
public:
  ProbabilityLayer()
  {
    layered_costmap_ = NULL;
  }

  virtual void onInitialize();
  virtual void updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

protected:
  void configure(ProbabilityLayerConfig &config, uint32_t level);
  double cutoff_, amplitude_, covar_;
  dynamic_reconfigure::Server<ProbabilityLayerConfig>* server_;
  dynamic_reconfigure::Server<ProbabilityLayerConfig>::CallbackType f_;
};
}  // namespace social_navigation_layers

#endif  // SOCIAL_NAVIGATION_LAYERS_PROBABILITY_LAYER_H
