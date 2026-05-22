/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <temporal_risk_aware_planner/planner_core.h>
#include <pluginlib/class_list_macros.hpp>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include <temporal_risk_aware_planner/dijkstra.h>
#include <temporal_risk_aware_planner/astar.h>
#include <temporal_risk_aware_planner/grid_path.h>
#include <temporal_risk_aware_planner/gradient_path.h>
#include <temporal_risk_aware_planner/quadratic_calculator.h>

#include <cmath>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(temporal_risk_aware_planner::TemporalRiskAwarePlanner, nav_core::BaseGlobalPlanner)

namespace temporal_risk_aware_planner {

void TemporalRiskAwarePlanner::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value) {
    unsigned char* pc = costarr;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
}

TemporalRiskAwarePlanner::TemporalRiskAwarePlanner() :
        costmap_(NULL), initialized_(false), allow_unknown_(true),
        p_calc_(NULL), planner_(NULL), path_maker_(NULL), orientation_filter_(NULL),
        potential_array_(NULL), has_global_goal_(false), has_subgoal_(false), last_nearest_idx_(0), has_voxgrid_(false), current_robot_speed_(0.0),
        lookahead_dist_(4.0), subgoal_reached_dist_(1.5), global_goal_near_dist_(3.0), goal_tolerance_(0.2) {
}

TemporalRiskAwarePlanner::TemporalRiskAwarePlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) :
        TemporalRiskAwarePlanner() {
    //initialize the planner
    initialize(name, costmap, frame_id);
}

TemporalRiskAwarePlanner::~TemporalRiskAwarePlanner() {
    if (p_calc_)
        delete p_calc_;
    if (planner_)
        delete planner_;
    if (path_maker_)
        delete path_maker_;
    if (dsrv_)
        delete dsrv_;
}

void TemporalRiskAwarePlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void TemporalRiskAwarePlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) {
    if (!initialized_) {
        ros::NodeHandle private_nh("~/" + name);
        costmap_ = costmap;
        frame_id_ = frame_id;

        unsigned int cx = costmap->getSizeInCellsX(), cy = costmap->getSizeInCellsY();

        private_nh.param("old_navfn_behavior", old_navfn_behavior_, false);
        if(!old_navfn_behavior_)
            convert_offset_ = 0.5;
        else
            convert_offset_ = 0.0;

        bool use_quadratic;
        private_nh.param("use_quadratic", use_quadratic, true);
        if (use_quadratic)
            p_calc_ = new QuadraticCalculator(cx, cy);
        else
            p_calc_ = new PotentialCalculator(cx, cy);

        bool use_dijkstra;
        private_nh.param("use_dijkstra", use_dijkstra, false);
        if (use_dijkstra)
        {
            DijkstraExpansion* de = new DijkstraExpansion(p_calc_, cx, cy);
            if(!old_navfn_behavior_)
                de->setPreciseStart(true);
            planner_ = de;
        }
        else
            planner_ = new AStarExpansion(p_calc_, cx, cy);

        bool use_grid_path;
        private_nh.param("use_grid_path", use_grid_path, true);
        if (use_grid_path)
            path_maker_ = new GridPath(p_calc_);
        else
            path_maker_ = new GradientPath(p_calc_);

        orientation_filter_ = new OrientationFilter();

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);
        subgoal_marker_pub_ = private_nh.advertise<visualization_msgs::Marker>("subgoal_marker", 1);
        nearest_marker_pub_ = private_nh.advertise<visualization_msgs::Marker>("nearest_marker", 1);

        private_nh.param("allow_unknown", allow_unknown_, true);
        planner_->setHasUnknown(allow_unknown_);
        private_nh.param("planner_window_x", planner_window_x_, 0.0);
        private_nh.param("planner_window_y", planner_window_y_, 0.0);
        private_nh.param("default_tolerance", default_tolerance_, 0.0);
        private_nh.param("publish_scale", publish_scale_, 100);
        private_nh.param("outline_map", outline_map_, true);
        voxgrid_sub_ = private_nh.subscribe("/temporal_grid_local_map",1,&TemporalRiskAwarePlanner::voxGridCallback,this);
        std::string odom_topic;
        private_nh.param("odom_topic", odom_topic, std::string("/odom"));
        odom_sub_ = private_nh.subscribe(odom_topic, 1, &TemporalRiskAwarePlanner::odomCallback, this);

        make_plan_srv_ = private_nh.advertiseService("make_plan", &TemporalRiskAwarePlanner::makePlanService, this);

        dsrv_ = new dynamic_reconfigure::Server<temporal_risk_aware_planner::TemporalRiskAwarePlannerConfig>(ros::NodeHandle("~/" + name));
        dynamic_reconfigure::Server<temporal_risk_aware_planner::TemporalRiskAwarePlannerConfig>::CallbackType cb =
                [this](auto& config, auto level){ reconfigureCB(config, level); };
        dsrv_->setCallback(cb);

        initialized_ = true;
    } else
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");

}

void TemporalRiskAwarePlanner::reconfigureCB(temporal_risk_aware_planner::TemporalRiskAwarePlannerConfig& config, uint32_t level) {
    planner_->setLethalCost(config.lethal_cost);
    path_maker_->setLethalCost(config.lethal_cost);
    planner_->setNeutralCost(config.neutral_cost);
    planner_->setFactor(config.cost_factor);
    publish_potential_ = config.publish_potential;
    orientation_filter_->setMode(config.orientation_mode);
    orientation_filter_->setWindowSize(config.orientation_window_size);
}

void TemporalRiskAwarePlanner::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

bool TemporalRiskAwarePlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = frame_id_;

    return true;
}

void TemporalRiskAwarePlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + (mx+convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my+convert_offset_) * costmap_->getResolution();
}

bool TemporalRiskAwarePlanner::worldToMap(double wx, double wy, double& mx, double& my) {
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}

bool TemporalRiskAwarePlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           std::vector<geometry_msgs::PoseStamped>& plan) {
    return makePlan(start, goal, default_tolerance_, plan);
}

bool TemporalRiskAwarePlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           double tolerance, std::vector<geometry_msgs::PoseStamped>& plan) {

    boost::mutex::scoped_lock lock(mutex_);

    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized yet.");
        return false;
    }

    std::vector<geometry_msgs::PoseStamped> sub_path;


    if (!has_global_goal_ || isGoalChanged(goal)) 
    {
      
        initial_start_ = start;
        global_goal_ = goal;

        has_global_goal_ = true;
        last_global_goal_ = goal;
        has_subgoal_ = false;

        previous_subpath_.clear();
        reference_path_.clear();

        last_nearest_idx_ = 0;

        if (!buildPlan(initial_start_, global_goal_, reference_path_))  
        {
            return false;
        }
    }

    if (!has_subgoal_ || isPoseNear(start, subgoal_, subgoal_reached_dist_) || !isSubgoalSafe(subgoal_)) 
    {

        if (isPoseNear(start, global_goal_, global_goal_near_dist_)) {
            subgoal_ = global_goal_;
        } 
        else 
        {

            if (!selectSubgoal(start, subgoal_)) {
                return false;
            }
        }  

        has_subgoal_ = true;

        // Publish subgoal marker for visualization
        publishSubgoalMarker(subgoal_);

        if (!buildPlan(start, subgoal_, sub_path)) 
        {
            return false;
        }

        previous_subpath_ = sub_path;
        plan = sub_path;
        publishPlan(plan);

        return !plan.empty();

    }


    std::vector<geometry_msgs::PoseStamped> pruned_subpath;
    pruneSubpath(start, pruned_subpath);

    if (isTemporalRiskAcceptable(pruned_subpath) && !pruned_subpath.empty()) 
    {
        ROS_INFO("[DebugPlanner] [Zone B] Path is SAFE. Reusing the pruned subpath.");
        sub_path = pruned_subpath;
    } 
    else 
    {
        buildPlan(start, subgoal_, sub_path);
    }

    previous_subpath_ = sub_path;
    plan = sub_path;
    publishPlan(plan);

    ROS_INFO("[DebugPlanner] [Zone B - FINAL RETURN] Returning plan with %lu points.", plan.size());
    return true;

}

bool TemporalRiskAwarePlanner::buildPlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
        std::vector<geometry_msgs::PoseStamped>& plan) {

    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    //clear the plan, just in case
    plan.clear();

    ros::NodeHandle n;
    std::string global_frame = frame_id_;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (goal.header.frame_id != global_frame) {
        ROS_ERROR(
                "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), goal.header.frame_id.c_str());
        return false;
    }

    if (start.header.frame_id != global_frame) {
        ROS_ERROR(
                "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), start.header.frame_id.c_str());
        return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;

    if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
        ROS_WARN_THROTTLE(1.0,
                "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }
    if(old_navfn_behavior_){
        start_x = start_x_i;
        start_y = start_y_i;
    }else{
        worldToMap(wx, wy, start_x, start_y);
    }

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
        ROS_WARN_THROTTLE(1.0,
                "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }
    if(old_navfn_behavior_){
        goal_x = goal_x_i;
        goal_y = goal_y_i;
    }else{
        worldToMap(wx, wy, goal_x, goal_y);
    }

    //clear the starting cell within the costmap because we know it can't be an obstacle
    clearRobotCell(start, start_x_i, start_y_i);

    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();

    //make sure to resize the underlying array that Navfn uses
    p_calc_->setSize(nx, ny);
    planner_->setSize(nx, ny);
    path_maker_->setSize(nx, ny);
    potential_array_ = new float[nx * ny];

    if(outline_map_)
        outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);

    bool found_legal = planner_->calculatePotentials(costmap_->getCharMap(), start_x, start_y, goal_x, goal_y,
                                                    nx * ny * 2, potential_array_);

    if(!old_navfn_behavior_)
        planner_->clearEndpoint(costmap_->getCharMap(), potential_array_, goal_x_i, goal_y_i, 2);
    if(publish_potential_)
        publishPotential(potential_array_);

    if (found_legal) {
        //extract the plan
        if (getPlanFromPotential(start_x, start_y, goal_x, goal_y, goal, plan)) {
            //make sure the goal we push on has the same timestamp as the rest of the plan
            geometry_msgs::PoseStamped goal_copy = goal;
            goal_copy.header.stamp = ros::Time::now();
            plan.push_back(goal_copy);
        } else {
            ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
        }
    }else{
        ROS_ERROR_THROTTLE(5.0, "Failed to get a plan.");
    }

    // add orientations if needed
    orientation_filter_->processPath(start, plan);

    //publish the plan for visualization purposes
    publishPlan(plan);
    delete[] potential_array_;

    return !plan.empty();
}

bool TemporalRiskAwarePlanner::isGoalChanged(const geometry_msgs::PoseStamped& goal) const 
{

    if (!has_global_goal_) {
        return true;
    }

    return distance2D(goal, last_global_goal_) > goal_tolerance_;
}

double TemporalRiskAwarePlanner::distance2D(const geometry_msgs::PoseStamped& a,
                                     const geometry_msgs::PoseStamped& b) const
{
  const double dx = a.pose.position.x - b.pose.position.x;
  const double dy = a.pose.position.y - b.pose.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

bool TemporalRiskAwarePlanner::isPoseNear(const geometry_msgs::PoseStamped& a,
                                          const geometry_msgs::PoseStamped& b,
                                          double tolerance) const 
{
    double dist= distance2D(a, b);
    ROS_INFO("--- [DEBUG 2] Dist to Subgoal: %.2f ---", dist);
    return dist <= tolerance;
}

bool TemporalRiskAwarePlanner::isSubgoalSafe(const geometry_msgs::PoseStamped& pose) {

    if (!costmap_) return false;

    unsigned int mx, my;
    if (costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
        unsigned char cost = costmap_->getCost(mx, my);
        
        if (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
            return false;
        }
        return true;
    }
    return false;
}

bool TemporalRiskAwarePlanner::selectSubgoal(const geometry_msgs::PoseStamped& start, geometry_msgs::PoseStamped& subgoal) 
{
    const size_t search_begin = last_nearest_idx_;

    size_t nearest_idx = findNearestIdx(start, reference_path_, search_begin);
    
    last_nearest_idx_ = nearest_idx;
    
    publishNearestMarker(reference_path_[nearest_idx]);

    size_t target_idx = nearest_idx;
    double accumulated_dist = 0.0;
    
    for (size_t i = nearest_idx; i + 1 < reference_path_.size(); ++i) {
        double dx = reference_path_[i+1].pose.position.x - reference_path_[i].pose.position.x;
        double dy = reference_path_[i+1].pose.position.y - reference_path_[i].pose.position.y;
        accumulated_dist += std::sqrt(dx*dx + dy*dy);
        
        if (accumulated_dist >= lookahead_dist_) {
            target_idx = i + 1;
            break;
        }
    }

    if (isSubgoalSafe(reference_path_[target_idx])) {
        subgoal = reference_path_[target_idx];
        return true;
    }
    else {

        ROS_WARN("[TemporalPlanner] Target subgoal at idx %lu is UNSAFE, reselecting...", target_idx);
        
        for (size_t j = target_idx + 1; j < reference_path_.size(); ++j) {
            if (isSubgoalSafe(reference_path_[j])) {
                subgoal = reference_path_[j];
                return true;
            }
        }
        
        if (isSubgoalSafe(global_goal_)) {
            subgoal = global_goal_;
            return true;
        }
    }

    return false;
                                                
}

size_t TemporalRiskAwarePlanner::findNearestIdx(
    const geometry_msgs::PoseStamped& start,
    const std::vector<geometry_msgs::PoseStamped>& path,
    size_t search_begin) const
{
  if (path.empty())
  {
    return 0;
  }

  if (search_begin >= path.size())
  {
    search_begin = path.size() - 1;
  }

  size_t best_index = search_begin;
  double best_dist = distance2D(start, path[search_begin]);

  size_t previous_index = search_begin;
  double previous_dist = best_dist;

  for (size_t i = search_begin + 1; i < path.size(); ++i)
  {
    const double current_dist = distance2D(start, path[i]);

    if (previous_dist <= current_dist)
    {
      return previous_index;
    }

    if (current_dist < best_dist)
    {
      best_dist = current_dist;
      best_index = i;
    }

    previous_index = i;
    previous_dist = current_dist;
  }

  return best_index;
}


void TemporalRiskAwarePlanner::pruneSubpath(const geometry_msgs::PoseStamped& start,
                                            std::vector<geometry_msgs::PoseStamped>& pruned_subpath) const 
{
    pruned_subpath.clear();

    if (previous_subpath_.empty()) {
        ROS_WARN("Previous subpath is empty. Cannot prune subpath.");
        return;
    }

    const size_t nearest_idx = findNearestIdx(start, previous_subpath_, 0);

    pruned_subpath.assign(
        previous_subpath_.begin() + nearest_idx,
        previous_subpath_.end());
}

bool TemporalRiskAwarePlanner::isTemporalRiskAcceptable(const std::vector<geometry_msgs::PoseStamped>& path) const
{

    const int width  = static_cast<int>(latest_voxgrid_.width);
    const int height = static_cast<int>(latest_voxgrid_.height);
    const int depth  = static_cast<int>(latest_voxgrid_.depth);

    if (!has_voxgrid_) {
        ROS_WARN("[TemporalRisk] Return TRUE: has_voxgrid_ is FALSE (No data available yet).");
        return true; 
    }

    if (width <= 0 || height <= 0 || depth <= 0 ||
        latest_voxgrid_.dl <= 0.0 || latest_voxgrid_.dt <= 0.0) {
        return true;
    }

    if (width <= 0 || height <= 0 || depth <= 0 ||
        latest_voxgrid_.dl <= 0.0 || latest_voxgrid_.dt <= 0.0) {
        ROS_WARN("[TemporalRisk] Return TRUE: Invalid Grid Dimensions or Resolutions. (W:%d, H:%d, D:%d, dl:%.3f, dt:%.3f)",
                        width, height, depth, latest_voxgrid_.dl, latest_voxgrid_.dt);
        return true;
    }

    const size_t slice_size = static_cast<size_t>(width) * height;

    const double speed = std::max(current_robot_speed_, 0.1);
    
    std::vector<double> time_risk_sum(depth, 0.0);
    std::vector<int> time_risk_count(depth, 0);

    double accumulated_dist = 0.0;
    double max_risk = 0.0;

    ROS_INFO("[TemporalRisk] ===== Starting Path Evaluation =====");
    ROS_INFO("[TemporalRisk] Path Size: %lu points, Current Speed: %.2f m/s", path.size(), speed);
    
    for (size_t i = 0; i < path.size(); ++i) 
    {
        
        if (i > 0) 
        {
            const double dx = path[i].pose.position.x - path[i - 1].pose.position.x;
            const double dy = path[i].pose.position.y - path[i - 1].pose.position.y;

            accumulated_dist += std::sqrt(dx * dx + dy * dy);
        }

        const double arrival_time = accumulated_dist / speed;

        int t_idx = static_cast<int>(
            std::floor(arrival_time / latest_voxgrid_.dt));

        t_idx = std::min(t_idx, depth - 1);

        const double wx = path[i].pose.position.x;
        const double wy = path[i].pose.position.y;

        const int x_idx = static_cast<int>(std::floor((wx - latest_voxgrid_.origin.x) / latest_voxgrid_.dl));

        const int y_idx = static_cast<int>(std::floor((wy - latest_voxgrid_.origin.y) / latest_voxgrid_.dl));

        if (x_idx < 0 || y_idx < 0 || x_idx >= width || y_idx >= height) 
        {
            ROS_INFO("[TemporalRisk] Loop CONTINUE: Pt %lu is Out of Bounds. (x_idx:%d, y_idx:%d, W:%d, H:%d)", 
                                i, x_idx, y_idx, width, height);
            continue;
        }

        const size_t index =
            static_cast<size_t>(t_idx) * slice_size +
            static_cast<size_t>(y_idx) * width +
            static_cast<size_t>(x_idx);

        const double risk = static_cast<double>(latest_voxgrid_.data[index]) / 255.0;

        time_risk_sum[t_idx] += risk;
        time_risk_count[t_idx]++;

        max_risk = std::max(max_risk, risk);

        ROS_INFO("[TemporalRisk] Time step %d, risk: %.2f", t_idx, risk);

        if (max_risk > 0.6) 
        {
            ROS_WARN("[TemporalRisk] Return FALSE: Immediate Lethal Collision Detected! Pt %lu Max Risk (%.2f) > 0.6", i, max_risk);
            return false;
        }
    }

    double first_risk = -1.0;
    double max_time_risk = 0.0;

    for (int t = 0; t < depth; ++t) {
        if (time_risk_count[t] == 0) {
            continue;
        }

        const double mean_risk_t = time_risk_sum[t] / static_cast<double>(time_risk_count[t]);

        if (first_risk < 0.0) {
            first_risk = mean_risk_t;
        }

        max_time_risk = std::max(max_time_risk, mean_risk_t);
    }

    if (first_risk < 0.0) {
        return true;
    }

    const double risk_trend = max_time_risk - first_risk;

    ROS_INFO("[TemporalRisk] Risk trend: %.2f", risk_trend);

    if (risk_trend > 0.2) {
        return false;
    }

    return true;
}


void TemporalRiskAwarePlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    boost::mutex::scoped_lock lock(mutex_);

    current_robot_speed_ = std::fabs(msg->twist.twist.linear.x);
}

void TemporalRiskAwarePlanner::voxGridCallback(const vox_msgs::VoxGrid::ConstPtr& msg)
{
    boost::mutex::scoped_lock lock(mutex_);

    latest_voxgrid_ = *msg;

    has_voxgrid_ = !latest_voxgrid_.data.empty();
}

// Publish subgoal as a Marker for RViz visualization
void TemporalRiskAwarePlanner::publishSubgoalMarker(const geometry_msgs::PoseStamped& subgoal) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = subgoal.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "subgoal";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = subgoal.pose;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.8f;
    marker.lifetime = ros::Duration(0.0);
    subgoal_marker_pub_.publish(marker);
}

void TemporalRiskAwarePlanner::publishNearestMarker(const geometry_msgs::PoseStamped& nearest) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = nearest.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "nearest";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = nearest.pose;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.8f;
    marker.lifetime = ros::Duration(0.0);
    nearest_marker_pub_.publish(marker);
}

void TemporalRiskAwarePlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

bool TemporalRiskAwarePlanner::getPlanFromPotential(double start_x, double start_y, double goal_x, double goal_y,
                                      const geometry_msgs::PoseStamped& goal,
                                       std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    std::string global_frame = frame_id_;

    //clear the plan, just in case
    plan.clear();

    std::vector<std::pair<float, float> > path;

    if (!path_maker_->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path)) {
        ROS_ERROR("NO PATH!");
        return false;
    }

    ros::Time plan_time = ros::Time::now();
    for (int i = path.size() -1; i>=0; i--) {
        std::pair<float, float> point = path[i];
        //convert the plan to world coordinates
        double world_x, world_y;
        mapToWorld(point.first, point.second, world_x, world_y);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
    }
    if(old_navfn_behavior_){
            plan.push_back(goal);
    }
    return !plan.empty();
}

void TemporalRiskAwarePlanner::publishPotential(float* potential)
{
    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
    double resolution = costmap_->getResolution();
    nav_msgs::OccupancyGrid grid;
    // Publish Whole Grid
    grid.header.frame_id = frame_id_;
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution;

    grid.info.width = nx;
    grid.info.height = ny;

    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - resolution / 2;
    grid.info.origin.position.y = wy - resolution / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx * ny);

    float max = 0.0;
    for (unsigned int i = 0; i < grid.data.size(); i++) {
        float potential = potential_array_[i];
        if (potential < POT_HIGH) {
            if (potential > max) {
                max = potential;
            }
        }
    }

    for (unsigned int i = 0; i < grid.data.size(); i++) {
        if (potential_array_[i] >= POT_HIGH) {
            grid.data[i] = -1;
        } else {
            if (fabs(max) < DBL_EPSILON) {
                grid.data[i] = -1;
            } else {
                grid.data[i] = potential_array_[i] * publish_scale_ / max;
            }
        }
    }
    potential_pub_.publish(grid);
}

} //end namespace temporal_risk_aware_planner
