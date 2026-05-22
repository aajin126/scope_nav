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

#include <cfloat>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(temporal_risk_aware_planner::TemporalRiskAwarePlanner, nav_core::BaseGlobalPlanner)

namespace temporal_risk_aware_planner {

namespace {

struct double2 {
    double x;
    double y;
};

double2 make_double2(double x, double y) {
    double2 point = {x, y};
    return point;
}

geometry_msgs::Point markerPoint(const geometry_msgs::PoseStamped& pose, double z_offset) {
    geometry_msgs::Point point = pose.pose.position;
    point.z += z_offset;
    return point;
}

void setMarkerColor(visualization_msgs::Marker& marker, float r, float g, float b, float a) {
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
}

}  // namespace

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
        potential_array_(NULL), has_global_goal_(false), has_voxgrid_(false), 
        current_robot_speed_(0.0), lookahead_dist_(4.0), global_goal_near_dist_(1.0), goal_tolerance_(0.2) {
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
        planning_debug_marker_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("planning_debug_markers", 1, true);

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

    plan.clear();
    std::vector<geometry_msgs::PoseStamped> sub_path;
    std::vector<geometry_msgs::PoseStamped> local_goal_line;


    if (!has_global_goal_ || isGoalChanged(goal)) 
    {
      
        global_goal_ = goal;
        last_global_goal_ = goal;
        has_global_goal_ = true;
        previous_subpath_.clear();
    }

    std::vector<geometry_msgs::PoseStamped> endpoints;
    bool is_near = isPoseNear(start, global_goal_, global_goal_near_dist_);

    if (is_near)
    {
        ROS_INFO("Goal is very near. Switching to Direct Approach Mode.");

        if (buildPlan(start, global_goal_, sub_path)) {
            plan = sub_path;           
            previous_subpath_ = plan; 
            publishPlan(plan); 
            publishPlanningDebugMarkers(local_goal_line, endpoints, std::vector<PathCandidate>(), plan);
            return !plan.empty();
        }
        else {
            ROS_WARN("Direct approach failed. Falling back to homotopy logic.");
        }

    }

    ROS_INFO("Goal is far. Generating local goal line and evaluating homotopy classes.");

    createLocalGoalLine(start, global_goal_, is_near, endpoints, local_goal_line);

    std::vector<PathCandidate> candidates = findHomotopyPaths(start, endpoints);

    if (candidates.empty()) {
        ROS_ERROR("No valid paths found through any endpoints.");
        publishPlanningDebugMarkers(local_goal_line, endpoints, candidates, std::vector<geometry_msgs::PoseStamped>());
        return false;
    }

    std::vector<geometry_msgs::PoseStamped> lowest_risk_path = evalTemporalRisk(candidates);

    if (previous_subpath_.empty()) {
        plan = lowest_risk_path;
    } else {
        bool h_match_found = false;
        PathCandidate best_matching_candidate;
        
        for (const auto& cand : candidates) {
            if (isSameHomotopy(previous_subpath_, cand.path, start)) {
                if (cand.temporal_risk_score < 0.6) { 
                    best_matching_candidate = cand;
                    h_match_found = true;
                    break;
                }
            }
        }

        if (h_match_found) {
            ROS_INFO("Same Homotopy path found. Reusing current homotopy class.");
            plan = best_matching_candidate.path;
        } else {
            ROS_INFO("Homotopy changed or block detected. Re-evaluating lowest risk path.");
            plan = lowest_risk_path;
        }
    }

    previous_subpath_ = plan;
    publishPlan(plan);
    publishPlanningDebugMarkers(local_goal_line, endpoints, candidates, plan);
    return !plan.empty();

}

std::vector<TemporalRiskAwarePlanner::PathCandidate> TemporalRiskAwarePlanner::findHomotopyPaths(
                                                                       const geometry_msgs::PoseStamped& start,
                                                                       const std::vector<geometry_msgs::PoseStamped>& endpoints)
{

    std::vector<PathCandidate> all_paths;
    std::vector<std::vector<geometry_msgs::PoseStamped>> raw_paths(endpoints.size());
    
    // buildPlan uses shared planner state, so candidate generation must stay sequential.
    for (size_t i = 0; i < endpoints.size(); ++i) {
        std::vector<geometry_msgs::PoseStamped> single_plan;
        if (buildPlan(start, endpoints[i], single_plan)) {
            raw_paths[i] = single_plan;
        }
    }

    // 2. Homotopy filtering
    for (const auto& path : raw_paths) {
        if (path.empty()) continue;

        bool duplicate = false;
        for (const auto& unique_cand : all_paths) {
            if (isSameHomotopy(path, unique_cand.path, start)) {
                duplicate = true;
                break;
            }
        }

        if (!duplicate) {
            PathCandidate cand;
            cand.path = path;
            cand.homotopy_id = static_cast<int>(all_paths.size());
            cand.temporal_risk_score = 0.0;
            all_paths.push_back(cand);
        }
    }

    return all_paths;

}

bool TemporalRiskAwarePlanner::isSameHomotopy(const std::vector<geometry_msgs::PoseStamped>& patha, 
                                              const std::vector<geometry_msgs::PoseStamped>& pathb,
                                              const geometry_msgs::PoseStamped& start) 
{
    if (patha.size() < 2 || pathb.size() < 2) return true;

    // Directly pass the two paths into the obstacle checker (No polygon generation needed)
    return !hasObstacleInside(patha, pathb, start);
}

bool TemporalRiskAwarePlanner::hasObstacleInside(const std::vector<geometry_msgs::PoseStamped>& patha,
                                                 const std::vector<geometry_msgs::PoseStamped>& pathb,
                                                 const geometry_msgs::PoseStamped& start) 
{
    if (patha.empty() || pathb.empty()) return false;

    // 1. Convert ROS paths to simple double2 types for faster access
    std::vector<double2> pathA, pathB;
    for (const auto& p : patha) pathA.push_back(make_double2(p.pose.position.x, p.pose.position.y));
    for (const auto& p : pathb) pathB.push_back(make_double2(p.pose.position.x, p.pose.position.y));

    // 2. Calculate the center points of the endpoints of both paths
    double2 endA = pathA.back();
    double2 endB = pathB.back();
    double center_end_x = (endA.x + endB.x) / 2.0;
    double center_end_y = (endA.y + endB.y) / 2.0;

    // 3. Compute the heading vector (V) from start (robot) to the center endpoint
    double robot_x = start.pose.position.x;
    double robot_y = start.pose.position.y;
    double v_x = center_end_x - robot_x;
    double v_y = center_end_y - robot_y;

    // 4. Gather obstacle coordinates ONLY in the forward direction based on the heading vector
    std::vector<double2> obstacles;
    int map_w = costmap_->getSizeInCellsX();
    int map_h = costmap_->getSizeInCellsY();
    double ox = costmap_->getOriginX();
    double oy = costmap_->getOriginY();
    double res = costmap_->getResolution();
    const unsigned char* char_map = costmap_->getCharMap();

    for (int my = 0; my < map_h; ++my) {
        for (int mx = 0; mx < map_w; ++mx) {
            unsigned char cost = char_map[(size_t)my * map_w + mx];
            
            // Filter legal obstacle costs
            if (cost >= 253) { 
                double wx = ox + (mx + 0.5) * res;
                double wy = oy + (my + 0.5) * res;

                // Vector from robot to the current obstacle cell (W)
                double w_x = wx - robot_x;
                double w_y = wy - robot_y;

                // Dot product calculation to determine if the obstacle is in front of the robot
                double dot = v_x * w_x + v_y * w_y;

                // Skip if the obstacle is behind the robot (dot product < 0)
                if (dot < 0.0) continue;

                obstacles.push_back(make_double2(wx, wy));
            }
        }
    }

    if (obstacles.empty()) return false;

    bool obstacle_found = false;
    int num_obstacles = obstacles.size();

    // 5. OpenMP parallelized vertical ray-casting check using cross and dot products
    #pragma omp parallel for shared(obstacle_found)
    for (int i = 0; i < num_obstacles; ++i) {
        if (obstacle_found) continue; // Early exit if another thread found an obstacle

        double2 obs = obstacles[i];
        int crossA = 0;
        int crossB = 0;

        // Check against PathA using perpendicular projection
        for (size_t j = 0; j < pathA.size() - 1; ++j) {
            double2 p1 = pathA[j];
            double2 p2 = pathA[j+1];

            double dx = p2.x - p1.x;
            double dy = p2.y - p1.y;

            if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6) continue;

            // Cross product to check if the obstacle lies on the left side of the segment
            double cross_product = dx * (obs.y - p1.y) - dy * (obs.x - p1.x);
            // Dot product to check if the projection falls within the segment bounds [p1, p2]
            double dot_product = (obs.x - p1.x) * dx + (obs.y - p1.y) * dy;
            double segment_len_sq = dx * dx + dy * dy;

            if (dot_product >= 0 && dot_product <= segment_len_sq) {
                if (cross_product > 0) { 
                    crossA++;
                }
            }
        }

        // Check against PathB using perpendicular projection
        for (size_t j = 0; j < pathB.size() - 1; ++j) {
            double2 p1 = pathB[j];
            double2 p2 = pathB[j+1];

            double dx = p2.x - p1.x;
            double dy = p2.y - p1.y;

            if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6) continue;

            double cross_product = dx * (obs.y - p1.y) - dy * (obs.x - p1.x);
            double dot_product = (obs.x - p1.x) * dx + (obs.y - p1.y) * dy;
            double segment_len_sq = dx * dx + dy * dy;

            if (dot_product >= 0 && dot_product <= segment_len_sq) {
                if (cross_product > 0) {
                    crossB++;
                }
            }
        }

        // If parity of crossings differs, the obstacle is blocked between two paths
        if ((crossA % 2) != (crossB % 2)) {
            obstacle_found = true;
        }
    }

    return obstacle_found;
}

void TemporalRiskAwarePlanner::createLocalGoalLine(const geometry_msgs::PoseStamped& start, 
                                                   const geometry_msgs::PoseStamped& global_goal, 
                                                   bool is_near, 
                                                   std::vector<geometry_msgs::PoseStamped>& endpoints,
                                                   std::vector<geometry_msgs::PoseStamped>& local_goal_line)
{

    endpoints.clear();
    local_goal_line.clear();

    // 1. Calculate the center and direction of the local goal line
    double center_x = 0.0, center_y = 0.0;
    double dx_w = global_goal.pose.position.x - start.pose.position.x;
    double dy_w = global_goal.pose.position.y - start.pose.position.y;
    double dist_to_goal = std::hypot(dx_w, dy_w);

    if (dist_to_goal < 1e-3) {
        endpoints.push_back(global_goal);
        local_goal_line.push_back(global_goal);
        return;
    }

    if (is_near || dist_to_goal < lookahead_dist_) {
        center_x = global_goal.pose.position.x;
        center_y = global_goal.pose.position.y;
    } else {
        center_x = start.pose.position.x + (dx_w / dist_to_goal) * lookahead_dist_;
        center_y = start.pose.position.y + (dy_w / dist_to_goal) * lookahead_dist_;
    }

    double heading_angle = std::atan2(dy_w, dx_w);
    double perp_angle = heading_angle + M_PI / 2.0;
    tf2::Quaternion goal_line_q;
    goal_line_q.setRPY(0.0, 0.0, heading_angle);
    const geometry_msgs::Quaternion goal_line_orientation = tf2::toMsg(goal_line_q);

    // Determine the width of the goal line (e.g., 2.5m for each side, 5.0m in total)
    double half_line_width = 2.5; 
    double end1_x = center_x + half_line_width * std::cos(perp_angle);
    double end1_y = center_y + half_line_width * std::sin(perp_angle);
    double end2_x = center_x - half_line_width * std::cos(perp_angle);
    double end2_y = center_y - half_line_width * std::sin(perp_angle);

    auto make_goal_line_pose = [&](double wx, double wy) {
        geometry_msgs::PoseStamped pt;
        pt.header.frame_id = frame_id_;
        pt.header.stamp = ros::Time::now();
        pt.pose.position.x = wx;
        pt.pose.position.y = wy;
        pt.pose.position.z = 0.0;
        pt.pose.orientation = goal_line_orientation;
        return pt;
    };

    local_goal_line.push_back(make_goal_line_pose(end1_x, end1_y));
    local_goal_line.push_back(make_goal_line_pose(end2_x, end2_y));

    // 2. Convert world coordinates to costmap grid coordinates
    unsigned int m_x1, m_y1, m_x2, m_y2;
    if (!costmap_->worldToMap(end1_x, end1_y, m_x1, m_y1) ||
        !costmap_->worldToMap(end2_x, end2_y, m_x2, m_y2)) {
        ROS_WARN_THROTTLE(1.0, "Goal line endpoints are out of costmap bounds.");
        return;
    }

    // 3. Using Bresenham to get all grid cells on the line
    std::pair<int, int> p1 = {static_cast<int>(m_x1), static_cast<int>(m_y1)};
    std::pair<int, int> p2 = {static_cast<int>(m_x2), static_cast<int>(m_y2)};
    std::vector<std::pair<int, int>> bresenham_line = Bresenham(p1, p2);

    // 4. Iterate through cells to check costs and perform obstacle segmentation
    std::vector<std::vector<geometry_msgs::PoseStamped>> safe_segments;
    std::vector<geometry_msgs::PoseStamped> current_segment;
    std::vector<geometry_msgs::PoseStamped> rasterized_goal_line;

    for (const auto& cell : bresenham_line) {
        int x = cell.first;
        int y = cell.second;

        // Boundary check for the costmap
        if (x < 0 || x >= static_cast<int>(costmap_->getSizeInCellsX()) || 
            y < 0 || y >= static_cast<int>(costmap_->getSizeInCellsY())) {
            if (!current_segment.empty()) {
                safe_segments.push_back(current_segment);
                current_segment.clear();
            }
            continue;
        }

        unsigned char cost = costmap_->getCost(x, y);
        geometry_msgs::PoseStamped pt;
        pt.header.frame_id = frame_id_;
        pt.header.stamp = ros::Time::now();
        costmap_->mapToWorld(x, y, pt.pose.position.x, pt.pose.position.y);
        pt.pose.position.z = 0.0;
        pt.pose.orientation = goal_line_orientation;
        rasterized_goal_line.push_back(pt);

        // Check if the cell is free from lethal or inscribed obstacles
        if (cost < 128) {
            current_segment.push_back(pt);
        } else {
            // Split the line into segments when hitting an obstacle
            if (!current_segment.empty()) {
                safe_segments.push_back(current_segment);
                current_segment.clear();
            }
        }
    }
    
    // Add the remaining segment after the loop completes
    if (!current_segment.empty()) {
        safe_segments.push_back(current_segment);
    }

    if (!rasterized_goal_line.empty()) {
        local_goal_line = rasterized_goal_line;
    }

    // 5. Uniformly sample endpoints from each obstacle-free safe segment
    int samples_per_segment = 1;

    for (const auto& segment : safe_segments) {
        // Noise filtering: Accept only valid passages with at least 4 continuous cells
        if (segment.size() < 3) continue; 

        for (int i = 0; i < samples_per_segment; ++i) {
            int target_idx = (samples_per_segment > 1) ? 
                             (i * (segment.size() - 1)) / (samples_per_segment - 1) : 0;
            
            endpoints.push_back(segment[target_idx]);
        }
    }

}

std::vector<geometry_msgs::PoseStamped> TemporalRiskAwarePlanner::evalTemporalRisk(
    std::vector<TemporalRiskAwarePlanner::PathCandidate>& candidates)
{
    if (candidates.empty()) {
        return std::vector<geometry_msgs::PoseStamped>();
    }

    auto use_first_candidate = [&]() {
        for (auto& cand : candidates) {
            cand.temporal_risk_score = 0.0;
        }
        return candidates.front().path;
    };

    const int width  = static_cast<int>(latest_voxgrid_.width);
    const int height = static_cast<int>(latest_voxgrid_.height);
    const int depth  = static_cast<int>(latest_voxgrid_.depth);

    if (!has_voxgrid_) {
        ROS_WARN("[TemporalRisk] No voxgrid data yet. Selecting the first candidate.");
        return use_first_candidate();
    }

    if (width <= 0 || height <= 0 || depth <= 0 ||
        latest_voxgrid_.dl <= 0.0 || latest_voxgrid_.dt <= 0.0) {
        ROS_WARN("[TemporalRisk] Invalid grid dimensions or resolutions. (W:%d, H:%d, D:%d, dl:%.3f, dt:%.3f)",
                        width, height, depth, latest_voxgrid_.dl, latest_voxgrid_.dt);
        return use_first_candidate();
    }

    const size_t slice_size = static_cast<size_t>(width) * height;
    const size_t expected_size = slice_size * static_cast<size_t>(depth);

    if (latest_voxgrid_.data.size() < expected_size) {
        ROS_WARN("[TemporalRisk] Voxgrid data is smaller than expected. (%lu < %lu)",
                 latest_voxgrid_.data.size(), expected_size);
        return use_first_candidate();
    }

    const double speed = std::max(current_robot_speed_, 0.1);

    ROS_INFO("[TemporalRisk] ===== Starting Candidate Evaluation =====");
    ROS_INFO("[TemporalRisk] Candidates: %lu, Current Speed: %.2f m/s", candidates.size(), speed);

    #pragma omp parallel for 
    for (int cand_idx = 0; cand_idx < static_cast<int>(candidates.size()); ++cand_idx) {
        PathCandidate& candidate = candidates[cand_idx];
        const std::vector<geometry_msgs::PoseStamped>& path = candidate.path;

        if (path.empty()) {
            candidate.temporal_risk_score = 1.0;
            continue;
        }

        std::vector<double> time_risk_sum(depth, 0.0);
        std::vector<int> time_risk_count(depth, 0);

        double accumulated_dist = 0.0;
        double max_risk = 0.0;
        double total_risk = 0.0;
        int valid_risk_count = 0;

        for (size_t i = 0; i < path.size(); ++i) {
            if (i > 0) {
                const double dx = path[i].pose.position.x - path[i - 1].pose.position.x;
                const double dy = path[i].pose.position.y - path[i - 1].pose.position.y;
                accumulated_dist += std::sqrt(dx * dx + dy * dy);
            }

            const double arrival_time = accumulated_dist / speed;

            int t_idx = static_cast<int>(
                std::floor(arrival_time / latest_voxgrid_.dt));

            t_idx = std::max(0, std::min(t_idx, depth - 1));

            const double wx = path[i].pose.position.x;
            const double wy = path[i].pose.position.y;

            const int x_idx = static_cast<int>(
                std::floor((wx - latest_voxgrid_.origin.x) / latest_voxgrid_.dl));

            const int y_idx = static_cast<int>(
                std::floor((wy - latest_voxgrid_.origin.y) / latest_voxgrid_.dl));

            if (x_idx < 0 || y_idx < 0 || x_idx >= width || y_idx >= height) {
                continue;
            }

            const size_t index =
                static_cast<size_t>(t_idx) * slice_size +
                static_cast<size_t>(y_idx) * static_cast<size_t>(width) +
                static_cast<size_t>(x_idx);

            const double risk =
                static_cast<double>(latest_voxgrid_.data[index]) / 255.0;

            time_risk_sum[t_idx] += risk;
            time_risk_count[t_idx]++;

            max_risk = std::max(max_risk, risk);
            total_risk += risk;
            valid_risk_count++;
        }

        const double mean_path_risk =
            (valid_risk_count > 0)
            ? total_risk / static_cast<double>(valid_risk_count)
            : 1.0;

        /*
         * Temporal trend calculation using linear regression.
         *
         * Each valid time step has:
         *   x = time_sec
         *   y = mean_risk_t
         *
         * slope > 0 means risk increases over time.
         * slope <= 0 means risk is stable or decreasing.
         */
        double sum_t = 0.0;
        double sum_r = 0.0;
        double sum_tt = 0.0;
        double sum_tr = 0.0;
        int n_time_samples = 0;

        for (int t = 0; t < depth; ++t) {
            if (time_risk_count[t] == 0) {
                continue;
            }

            const double mean_risk_t =
                time_risk_sum[t] / static_cast<double>(time_risk_count[t]);

            const double time_sec =
                static_cast<double>(t) * latest_voxgrid_.dt;

            sum_t += time_sec;
            sum_r += mean_risk_t;
            sum_tt += time_sec * time_sec;
            sum_tr += time_sec * mean_risk_t;

            n_time_samples++;
        }

        double slope = 0.0;
        double risk_trend = 0.0;

        if (n_time_samples >= 2) {
            const double n = static_cast<double>(n_time_samples);
            const double denom = n * sum_tt - sum_t * sum_t;

            if (std::fabs(denom) > 1e-9) {
                slope = (n * sum_tr - sum_t * sum_r) / denom;

                const double horizon_time =
                    static_cast<double>(depth - 1) * latest_voxgrid_.dt;

                risk_trend =
                    std::min(1.0, std::max(0.0, slope * horizon_time));
            }
        }

        /*
         * Static risk part:
         *   w_max + w_mean = 1.0
         *
         * Temporal trend is added separately.
         */
        const double w_max = 0.4;
        const double w_mean = 0.6;
        const double w_trend = 0.5;

        const double static_risk =
            w_max * max_risk +
            w_mean * mean_path_risk;

        candidate.temporal_risk_score =
            static_risk +
            w_trend * risk_trend;

        #pragma omp critical
        {
            ROS_INFO("[TemporalRisk] Candidate %d score: %.3f "
                     "(static: %.3f, max: %.3f, mean: %.3f, trend: %.3f, slope: %.3f)",
                     cand_idx,
                     candidate.temporal_risk_score,
                     static_risk,
                     max_risk,
                     mean_path_risk,
                     risk_trend,
                     slope);
        }
    }

    const auto best_candidate = std::min_element(
        candidates.begin(), candidates.end(),
        [](const PathCandidate& a, const PathCandidate& b) {
            return a.temporal_risk_score < b.temporal_risk_score;
        });

    return best_candidate->path;
}

std::vector<std::pair<int, int>> TemporalRiskAwarePlanner::Bresenham(const std::pair<int, int>& p1, const std::pair<int, int>& p2)
{
    std::vector<std::pair<int, int>> line_vec = {p1};
    
    int dx = std::abs(p2.first - p1.first);
    int dy = std::abs(p2.second - p1.second);
    int sx = (p2.first > p1.first) ? 1 : -1;
    int sy = (p2.second > p1.second) ? 1 : -1;
    
    int x = p1.first;
    int y = p1.second;

    if (dx > dy) {
        int e = -dx;
        for (int i = 0; i < dx; i++) {
            x += sx;
            e += 2 * dy;
            line_vec.push_back({x, y});
            if (e >= 0) {
                y += sy;
                e -= 2 * dx;
                line_vec.push_back({x, y});
            }
        }
    }
    else if (dx < dy) {
        int e = -dy;
        for (int i = 0; i < dy; i++) {
            y += sy;
            e += 2 * dx;
            line_vec.push_back({x, y});
            if (e >= 0) {
                x += sx;
                e -= 2 * dy;
                line_vec.push_back({x, y});
            }
        }
    }
    else { // dx == dy
        for (int i = 0; i < dx; i++) {
            x += sx;
            y += sy;
            line_vec.push_back({x, y});
        }
    }

    return line_vec;
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

void TemporalRiskAwarePlanner::publishPlanningDebugMarkers(
    const std::vector<geometry_msgs::PoseStamped>& local_goal_line,
    const std::vector<geometry_msgs::PoseStamped>& endpoints,
    const std::vector<TemporalRiskAwarePlanner::PathCandidate>& candidates,
    const std::vector<geometry_msgs::PoseStamped>& selected_path) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    visualization_msgs::MarkerArray marker_array;
    const ros::Time stamp = ros::Time::now();

    visualization_msgs::Marker clear_marker;
    clear_marker.header.frame_id = frame_id_;
    clear_marker.header.stamp = stamp;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    auto make_marker = [&](const std::string& ns, int id, int type) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = stamp;
        marker.ns = ns;
        marker.id = id;
        marker.type = type;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.lifetime = ros::Duration(0.0);
        return marker;
    };

    if (local_goal_line.size() >= 2) {
        visualization_msgs::Marker marker =
            make_marker("local_goal_line", 0, visualization_msgs::Marker::LINE_STRIP);
        marker.scale.x = 0.06;
        setMarkerColor(marker, 1.0f, 0.85f, 0.0f, 0.9f);

        for (const auto& pose : local_goal_line) {
            marker.points.push_back(markerPoint(pose, 0.04));
        }
        marker_array.markers.push_back(marker);
    }

    if (!endpoints.empty()) {
        visualization_msgs::Marker marker =
            make_marker("end_points", 0, visualization_msgs::Marker::SPHERE_LIST);
        marker.scale.x = 0.22;
        marker.scale.y = 0.22;
        marker.scale.z = 0.22;
        setMarkerColor(marker, 0.0f, 0.9f, 1.0f, 0.9f);

        for (const auto& pose : endpoints) {
            marker.points.push_back(markerPoint(pose, 0.08));
        }
        marker_array.markers.push_back(marker);
    }

    const float palette[][3] = {
        {0.85f, 0.35f, 0.15f},
        {0.55f, 0.45f, 0.95f},
        {0.0f, 0.65f, 0.95f},
        {0.95f, 0.25f, 0.45f},
        {0.55f, 0.70f, 0.20f}
    };
    const size_t palette_size = sizeof(palette) / sizeof(palette[0]);

    for (size_t i = 0; i < candidates.size(); ++i) {
        if (candidates[i].path.size() < 2) {
            continue;
        }

        visualization_msgs::Marker marker =
            make_marker("sub_paths", static_cast<int>(i), visualization_msgs::Marker::LINE_STRIP);
        marker.scale.x = 0.035;
        const float* color = palette[i % palette_size];
        setMarkerColor(marker, color[0], color[1], color[2], 0.45f);

        for (const auto& pose : candidates[i].path) {
            marker.points.push_back(markerPoint(pose, 0.06));
        }
        marker_array.markers.push_back(marker);
    }

    if (selected_path.size() >= 2) {
        visualization_msgs::Marker marker =
            make_marker("selected_path", 0, visualization_msgs::Marker::LINE_STRIP);
        marker.scale.x = 0.10;
        setMarkerColor(marker, 0.0f, 1.0f, 0.25f, 1.0f);

        for (const auto& pose : selected_path) {
            marker.points.push_back(markerPoint(pose, 0.12));
        }
        marker_array.markers.push_back(marker);
    }

    planning_debug_marker_pub_.publish(marker_array);
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
