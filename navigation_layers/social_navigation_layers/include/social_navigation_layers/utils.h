#ifndef SOCIAL_NAVIGATION_LAYERS_UTILS_H
#define SOCIAL_NAVIGATION_LAYERS_UTILS_H
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <people_msgs/People.h>
#include <boost/thread.hpp>
#include <list>

namespace social_navigation_layers
{
double gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew);
double get_radius(double cutoff, double A, double var);

}  // namespace social_navigation_layers

#endif  // SOCIAL_NAVIGATION_LAYERS_UTILS_H
