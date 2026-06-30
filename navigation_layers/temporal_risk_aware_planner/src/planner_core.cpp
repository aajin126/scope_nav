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
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(temporal_risk_aware_planner::TemporalRiskAwarePlanner, nav_core::BaseGlobalPlanner)

namespace temporal_risk_aware_planner {

namespace {

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

double elapsedMs(const ros::WallTime& start) {
    return (ros::WallTime::now() - start).toSec() * 1000.0;
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
        current_robot_speed_(0.0), lookahead_dist_(4.0), global_goal_near_dist_(1.5), goal_tolerance_(0.2) {
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
        planning_track_marker_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("planning_track_markers", 1, true);
        planning_time_pub_ = private_nh.advertise<std_msgs::Float64>("planning_time_ms", 1);

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

bool TemporalRiskAwarePlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
    const ros::WallTime planning_start = ros::WallTime::now();
    bool result = makePlan(start, goal, default_tolerance_, plan);
    const double planning_time_ms = (ros::WallTime::now() - planning_start).toSec() * 1000.0;
    std_msgs::Float64 planning_time_msg;
    planning_time_msg.data = planning_time_ms;
    planning_time_pub_.publish(planning_time_msg);

    return result;
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

    // 1. Create local goal line and sample endpoints

    std::vector<geometry_msgs::PoseStamped> endpoints;

    bool is_near = isPoseNear(start, global_goal_, global_goal_near_dist_);

    double create_local_goal_line_time_ms = 0.0;
    double lazy_replan_time_ms = 0.0;
    double find_multiple_paths_time_ms = 0.0;
    double group_paths_by_homotopy_time_ms = 0.0;
    double compute_temporal_risk_time_ms = 0.0;
    double select_same_homotopy_time_ms = 0.0;
    double select_lowest_risk_time_ms = 0.0;

    ros::WallTime timer = ros::WallTime::now();
    createLocalGoalLine(start, global_goal_, is_near, endpoints, local_goal_line);
    create_local_goal_line_time_ms = elapsedMs(timer);

    timer = ros::WallTime::now();

    // 2. Lazy replanning : If the current subgoal is still safe and reachable, keep using it
    if (!previous_subpath_.empty())
    {
        double prev_risk = evalTemporalRisk(previous_subpath_);

        if (prev_risk < 4.0)
        {
            geometry_msgs::PoseStamped nearest_endpoint;

            if (findNearestPose(previous_subpath_.back(), endpoints, nearest_endpoint)) 
            {
                std::vector<geometry_msgs::PoseStamped> tail_plan;

                if (buildPlan(0, previous_subpath_.back(), nearest_endpoint, tail_plan)) 
                {
                    plan = previous_subpath_;

                    for (size_t i = 0; i < tail_plan.size(); ++i) {
                        plan.push_back(tail_plan[i]);
                    }

                    previous_subpath_ = plan;
                    publishPlan(plan);
                    lazy_replan_time_ms = elapsedMs(timer);
                    ROS_INFO("makePlan timing [createLocalGoalLine=%.2fms lazyReplan=%.2fms]", create_local_goal_line_time_ms, lazy_replan_time_ms);
                    return !plan.empty();
                }
            }
        }    

    }
    lazy_replan_time_ms = elapsedMs(timer);

    // 3. Generate multiple path candidates

    timer = ros::WallTime::now();
    std::vector<std::vector<geometry_msgs::PoseStamped>> valid_paths = findMultiplePaths(start, endpoints);
    find_multiple_paths_time_ms = elapsedMs(timer);

    if (valid_paths.empty()) {
        ROS_ERROR("No valid paths found through any endpoints.");
        publishPlanningDebugMarkers(local_goal_line, endpoints, std::vector<PathCandidate>(), std::vector<geometry_msgs::PoseStamped>());
        ROS_INFO("makePlan timing [createLocalGoalLine=%.2fms lazyReplan=%.2fms findMultiplePaths=%.2fms]", create_local_goal_line_time_ms, lazy_replan_time_ms, find_multiple_paths_time_ms);

        return false;
    }

    // 4.  Group paths by homotopy class
    int previous_homotopy_id = -1;

    timer = ros::WallTime::now();
    std::vector<PathCandidate> candidates = groupPathsByHomotopy(valid_paths, start, previous_subpath_, previous_homotopy_id);
    group_paths_by_homotopy_time_ms = elapsedMs(timer);
    
    bool selected_plan = false;
    int selected_homotopy_id = -1;

    // 5. Compute temporal risk score for every candidate and select the one with same homotopy with previous subpath if it has acceptable risk, otherwise select the one with the lowest temporal risk score

    timer = ros::WallTime::now();

    for (int cand_idx = 0; cand_idx < static_cast<int>(candidates.size()); ++cand_idx) {
        candidates[cand_idx].temporal_risk_score = evalTemporalRisk(candidates[cand_idx].path);
    }
    std::sort(candidates.begin(), candidates.end(),
        [](const PathCandidate& a, const PathCandidate& b) {
            return a.temporal_risk_score < b.temporal_risk_score;
        });

    compute_temporal_risk_time_ms = elapsedMs(timer);

    // Select path
    timer = ros::WallTime::now();

    int selected_idx = -1;

    if (previous_homotopy_id >= 0) 
    {
        for (int i = 0; i < static_cast<int>(candidates.size()); ++i) 
        {
            if (candidates[i].homotopy_id == previous_homotopy_id) 
            {
                if (candidates[i].temporal_risk_score < 3.0)
                {
                    selected_idx = i;
                    ROS_INFO("Acceptable same homotopy path found. Reusing current homotopy class.");
                    break;
                }

            }
       }

    }

    if (selected_idx < 0) 
    {
        selected_idx = 0;
        ROS_INFO("No acceptable same homotopy path found. Selecting the lowest temporal-risk path.");
    }

    plan = candidates[selected_idx].path;
    selected_plan = true;
    selected_homotopy_id = candidates[selected_idx].homotopy_id;

    select_same_homotopy_time_ms = elapsedMs(timer);

    previous_subpath_ = plan;
    publishPlan(plan);
    publishPlanningDebugMarkers(local_goal_line, endpoints, candidates, plan, selected_homotopy_id);

    ROS_INFO("[createLocalGoalLine=%.2fms lazyReplan=%.2fms findMultiplePaths=%.2fms groupPathsByHomotopy=%.2fms computeTemporalRisk=%.2fms selectSameHomotopy=%.2fms]",
             create_local_goal_line_time_ms,
             lazy_replan_time_ms,
             find_multiple_paths_time_ms,
             group_paths_by_homotopy_time_ms,
             compute_temporal_risk_time_ms,
             select_same_homotopy_time_ms);

    return !plan.empty();

}

std::vector<std::vector<geometry_msgs::PoseStamped>> TemporalRiskAwarePlanner::findMultiplePaths(
                                                                       const geometry_msgs::PoseStamped& start,
                                                                       const std::vector<geometry_msgs::PoseStamped>& endpoints)
{
    std::vector<std::vector<geometry_msgs::PoseStamped>> raw_paths(endpoints.size());

    if (endpoints.empty()) {
        ROS_WARN("findMultiplePaths: endpoints is empty.");
        return {};
    }

    const int nx = static_cast<int>(costmap_->getSizeInCellsX());
    const int ny = static_cast<int>(costmap_->getSizeInCellsY());

    int num_threads = omp_get_max_threads();
    num_threads = std::min(num_threads, static_cast<int>(endpoints.size()));
    num_threads = std::max(1, num_threads);

    if (static_cast<int>(workspaces_.size()) < num_threads) {
        workspaces_.resize(num_threads);
    }

    for (int tid = 0; tid < num_threads; ++tid) {
        getWorkspace(tid, nx, ny);
    }

    #pragma omp parallel for 
    for (int i = 0; i < static_cast<int>(endpoints.size()); ++i) {
        const int tid = omp_get_thread_num();

        std::vector<geometry_msgs::PoseStamped> single_plan;

        if (buildPlan(tid, start, endpoints[i], single_plan)) {
            raw_paths[i] = std::move(single_plan);
        }
    }

    std::vector<std::vector<geometry_msgs::PoseStamped>> valid_paths;
    valid_paths.reserve(raw_paths.size());

    for (auto& path : raw_paths) {
        if (!path.empty()) {
            valid_paths.push_back(std::move(path));
        }
    }

    ROS_INFO("findMultiplePaths: endpoints=%zu valid_paths=%zu",
             endpoints.size(), valid_paths.size());

    return valid_paths;
}

std::vector<TemporalRiskAwarePlanner::PathCandidate>TemporalRiskAwarePlanner::groupPathsByHomotopy(
                            const std::vector<std::vector<geometry_msgs::PoseStamped>>& valid_paths,
                            const geometry_msgs::PoseStamped& start,
                            const std::vector<geometry_msgs::PoseStamped>& previous_path,
                            int& previous_homotopy_id)
{
    std::vector<PathCandidate> cand_paths;
    previous_homotopy_id = -1;

    const bool has_previous = !previous_path.empty();

    std::vector<std::vector<geometry_msgs::PoseStamped>> grouped_paths;
    grouped_paths.reserve(valid_paths.size() + (has_previous ? 1 : 0));

    if (has_previous) {
        grouped_paths.push_back(previous_path);
    }

    for (const auto& path : valid_paths) {
        if (!path.empty()) {
            grouped_paths.push_back(path);
        }
    }

    const int n = static_cast<int>(grouped_paths.size());
    if (n == 0 || (has_previous && n == 1)) {
        return cand_paths;
    }

    const int candidate_offset = has_previous ? 1 : 0;

    std::vector<uint8_t> same(
        static_cast<size_t>(n) * static_cast<size_t>(n),
        0);

    std::vector<std::pair<int, int>> pairs;
    pairs.reserve(static_cast<size_t>(n) * static_cast<size_t>(n - 1) / 2);

    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            pairs.push_back(std::make_pair(i, j));
        }
    }

    #pragma omp parallel for 
    for (int k = 0; k < static_cast<int>(pairs.size()); ++k) {
        const int i = pairs[k].first;
        const int j = pairs[k].second;

        if (isSameHomotopy(grouped_paths[i], grouped_paths[j], start)) {
            same[static_cast<size_t>(i) * n + j] = 1;
        }
    }

    std::vector<int> parent(n);
    std::vector<int> rank(n, 0);

    for (int i = 0; i < n; ++i) {
        parent[i] = i;
    }

    auto find_root = [&](int x) {
        int root = x;

        while (parent[root] != root) {
            root = parent[root];
        }

        while (parent[x] != x) {
            const int next = parent[x];
            parent[x] = root;
            x = next;
        }

        return root;
    };

    auto unite = [&](int a, int b) {
        int root_a = find_root(a);
        int root_b = find_root(b);

        if (root_a == root_b) {
            return;
        }

        if (rank[root_a] < rank[root_b]) {
            std::swap(root_a, root_b);
        }

        parent[root_b] = root_a;

        if (rank[root_a] == rank[root_b]) {
            ++rank[root_a];
        }
    };

    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (same[static_cast<size_t>(i) * n + j]) {
                unite(i, j);
            }
        }
    }

    std::vector<int> root_to_homotopy_id(n, -1);
    std::vector<int> homotopy_ids(n, -1);
    int next_id = 0;

    for (int i = 0; i < n; ++i) {
        const int root = find_root(i);

        if (root_to_homotopy_id[root] < 0) {
            root_to_homotopy_id[root] = next_id++;
        }

        homotopy_ids[i] = root_to_homotopy_id[root];
    }

    if (has_previous) {
        previous_homotopy_id = homotopy_ids[0];
    }

    cand_paths.reserve(n - candidate_offset);

    for (int i = candidate_offset; i < n; ++i) {
        PathCandidate cand;
        cand.path = grouped_paths[i];
        cand.homotopy_id = homotopy_ids[i];
        cand.temporal_risk_score = 0.0;
        cand_paths.push_back(cand);
    }

    return cand_paths;
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
    const auto& endA = patha.back().pose.position;
    const auto& endB = pathb.back().pose.position;

    double goal_x = 0.5 * (endA.x + endB.x);
    double goal_y = 0.5 * (endA.y + endB.y);

    double robot_x = start.pose.position.x;
    double robot_y = start.pose.position.y;

    // V: direction perpendicular to the local goal line
    double v_x = goal_x - robot_x;
    double v_y = goal_y - robot_y;

    double v_len = std::hypot(v_x, v_y);
    if (v_len < 1e-6) {
        return false;
    }

    // U: direction parallel to the local goal line
    double u_x = -v_y / v_len;
    double u_y =  v_x / v_len;

    int map_w = costmap_->getSizeInCellsX();
    int map_h = costmap_->getSizeInCellsY();

    double ox  = costmap_->getOriginX();
    double oy  = costmap_->getOriginY();
    double res = costmap_->getResolution();

    const unsigned char* char_map = costmap_->getCharMap();

    // 0: empty, 1: Path A, 2: Path B, 3: both
    std::vector<uint8_t> path_grid(map_w * map_h, 0);

    auto markPath = [&](const std::vector<geometry_msgs::PoseStamped>& path, uint8_t value)
    {
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            unsigned int mx0, my0, mx1, my1;

            if (!costmap_->worldToMap(path[i].pose.position.x,
                                      path[i].pose.position.y,
                                      mx0, my0)) {
                continue;
            }

            if (!costmap_->worldToMap(path[i + 1].pose.position.x,
                                      path[i + 1].pose.position.y,
                                      mx1, my1)) {
                continue;
            }

            auto cells = Bresenham(
                {static_cast<int>(mx0), static_cast<int>(my0)},
                {static_cast<int>(mx1), static_cast<int>(my1)}
            );

            for (const auto& c : cells) {
                int mx = c.first;
                int my = c.second;

                if (mx >= 0 && mx < map_w && my >= 0 && my < map_h) {
                    path_grid[my * map_w + mx] |= value;
                }
            }
        }
    };

    markPath(patha, 1);
    markPath(pathb, 2);

    auto firstPathOnBresenhamLine = [&](int x0, int y0, int x1, int y1) -> uint8_t
    {
        int dx = std::abs(x1 - x0);
        int dy = std::abs(y1 - y0);
        int sx = (x1 > x0) ? 1 : -1;
        int sy = (y1 > y0) ? 1 : -1;

        int x = x0;
        int y = y0;

        auto check = [&](int mx, int my) -> uint8_t
        {
            if (mx < 0 || mx >= map_w || my < 0 || my >= map_h) {
                return 0;
            }

            uint8_t v = path_grid[my * map_w + mx];

            if (v & 1) return 1;
            if (v & 2) return 2;

            return 0;
        };

        if (dx > dy) {
            int e = -dx;
            for (int i = 0; i < dx; ++i) {
                x += sx;
                e += 2 * dy;

                if (x < 0 || x >= map_w || y < 0 || y >= map_h) return 0;

                uint8_t hit = check(x, y);
                if (hit != 0) return hit;

                if (e >= 0) {
                    y += sy;
                    e -= 2 * dx;

                    if (x < 0 || x >= map_w || y < 0 || y >= map_h) return 0;

                    hit = check(x, y);
                    if (hit != 0) return hit;
                }
            }
        } else if (dx < dy) {
            int e = -dy;
            for (int i = 0; i < dy; ++i) {
                y += sy;
                e += 2 * dx;

                if (x < 0 || x >= map_w || y < 0 || y >= map_h) return 0;

                uint8_t hit = check(x, y);
                if (hit != 0) return hit;

                if (e >= 0) {
                    x += sx;
                    e -= 2 * dy;

                    if (x < 0 || x >= map_w || y < 0 || y >= map_h) return 0;

                    hit = check(x, y);
                    if (hit != 0) return hit;
                }
            }
        } else {
            for (int i = 0; i < dx; ++i) {
                x += sx;
                y += sy;

                if (x < 0 || x >= map_w || y < 0 || y >= map_h) return 0;

                uint8_t hit = check(x, y);
                if (hit != 0) return hit;
            }
        }

        return 0;
    };

    std::vector<std::pair<int, int>> obstacles;

    for (int obs_my = 0; obs_my < map_h; obs_my += 3) {
        for (int obs_mx = 0; obs_mx < map_w; obs_mx += 3) {
            int obs_idx = obs_my * map_w + obs_mx;

            if (char_map[obs_idx] != costmap_2d::LETHAL_OBSTACLE) {
                continue;
            }

            double obs_x = ox + (static_cast<double>(obs_mx) + 0.5) * res;
            double obs_y = oy + (static_cast<double>(obs_my) + 0.5) * res;

            double rel_x = obs_x - robot_x;
            double rel_y = obs_y - robot_y;

            // Ignore obstacles behind the robot
            if (rel_x * v_x + rel_y * v_y <= 0.0) {
                continue;
            }

            obstacles.push_back({obs_mx, obs_my});
        }
    }

    bool obstacle_found = false;

    for (int i = 0; i < static_cast<int>(obstacles.size()); ++i) {
        int obs_mx = obstacles[i].first;
        int obs_my = obstacles[i].second;

        int ray_len = std::max(map_w, map_h)/20;

        int pos_end_x = obs_mx + static_cast<int>(std::round(u_x * ray_len));
        int pos_end_y = obs_my + static_cast<int>(std::round(u_y * ray_len));

        int neg_end_x = obs_mx - static_cast<int>(std::round(u_x * ray_len));
        int neg_end_y = obs_my - static_cast<int>(std::round(u_y * ray_len));

        uint8_t pos_hit = firstPathOnBresenhamLine(obs_mx, obs_my, pos_end_x, pos_end_y);
        uint8_t neg_hit = firstPathOnBresenhamLine(obs_mx, obs_my, neg_end_x, neg_end_y);

        if ((pos_hit == 1 && neg_hit == 2) ||
            (pos_hit == 2 && neg_hit == 1)) {
            return true;
        }
    }


    return false;
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

    // Determine the width of the goal line 
    const double resolution = costmap_->getResolution();
    const int map_size = std::min(static_cast<int>(costmap_->getSizeInCellsX()), static_cast<int>(costmap_->getSizeInCellsY()));

    int goal_line_half_length_cells =
        std::min(
            static_cast<int>(0.6 * dist_to_goal / resolution),
            static_cast<int>(0.055 * map_size)
        ) - 2;

    goal_line_half_length_cells = std::max(1, goal_line_half_length_cells);

    double half_line_width =
        static_cast<double>(goal_line_half_length_cells) * resolution;
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

    local_goal_line.clear();
    for (const auto& segment : safe_segments) {
        if (segment.size() < 2) continue;

        for (size_t i = 1; i < segment.size(); ++i) {
            local_goal_line.push_back(segment[i - 1]);
            local_goal_line.push_back(segment[i]);
        }
    }

    // 5. Uniformly sample endpoints based on segment length from each obstacle-free safe segment
    const int min_segment_size = 3;
    const int cells_per_sample = 20;
    const int max_samples_per_segment = 4;

    for (const auto& segment : safe_segments) {
        if (segment.size() < min_segment_size) continue;

        int samples_per_segment =
            static_cast<int>(segment.size()) / cells_per_sample;

        samples_per_segment =
            std::max(1, std::min(max_samples_per_segment, samples_per_segment));

        if (samples_per_segment == 1) {
            endpoints.push_back(segment[segment.size() / 2]);
            continue;
        }

        for (int i = 0; i < samples_per_segment; ++i) {
            const int target_idx =
                (i * (static_cast<int>(segment.size()) - 1)) /
                (samples_per_segment - 1);

            endpoints.push_back(segment[target_idx]);
        }
    }

}

double TemporalRiskAwarePlanner::evalTemporalRisk(const std::vector<geometry_msgs::PoseStamped>& path) const
{
    // ------------------------------------------------------------
    // 0. Initial Checks & Data Validation
    // ------------------------------------------------------------
    if (path.empty()) return 1e9;
    if (!has_voxgrid_) return 0.0;

    const int width  = static_cast<int>(latest_voxgrid_.width);
    const int height = static_cast<int>(latest_voxgrid_.height);
    const int depth  = static_cast<int>(latest_voxgrid_.depth);

    if (width <= 0 || height <= 0 || depth <= 0 ||
        latest_voxgrid_.dl <= 0.0 || latest_voxgrid_.dt <= 0.0) {
        return 0.0;
    }

    const size_t slice_size = static_cast<size_t>(width) * static_cast<size_t>(height);
    const size_t expected_size = slice_size * static_cast<size_t>(depth);
    if (latest_voxgrid_.data.size() < expected_size) return 0.0;

    // ------------------------------------------------------------
    // Parameters (Tuned for Simple & Robust Tracking)
    // ------------------------------------------------------------
    const double speed = std::max(current_robot_speed_, 0.1);
    const size_t sample_stride = 3;
    const int spatial_radius = 1;          
    const double risk_threshold = 0.30;    
    const int temporal_window_steps = 4;   
    const int min_event_length = 2;        
    
    // Static filter: Ignore obstacles persisting > 80% of prediction time
    const int static_duration_threshold = static_cast<int>(depth * 0.8); 
    
    // Weights
    const double w_arrival = 2.0;
    const double w_temporal_motion = 7.0;
    const double alpha_length = 1.0;       

    // Helper Lambdas
    auto clampTimeIndex = [&](int t_idx) -> int {
        return std::max(0, std::min(t_idx, depth - 1));
    };

    auto getRiskAt = [&](int x, int y, int t) -> double {
        if (x < 0 || y < 0 || x >= width || y >= height) return 0.0;
        t = clampTimeIndex(t);
        const size_t index = static_cast<size_t>(t) * slice_size + static_cast<size_t>(y) * width + static_cast<size_t>(x);
        if (index >= latest_voxgrid_.data.size()) return 0.0;
        return static_cast<double>(latest_voxgrid_.data[index]) / 255.0;
    };

    auto getSpatialKernelMax = [&](int x, int y, int t) -> double {
        double max_risk = 0.0;
        t = clampTimeIndex(t);
        for (int dy = -spatial_radius; dy <= spatial_radius; ++dy) {
            for (int dx = -spatial_radius; dx <= spatial_radius; ++dx) {
                max_risk = std::max(max_risk, getRiskAt(x + dx, y + dy, t));
            }
        }
        return max_risk;
    };

    auto worldToVoxIndex = [&](double wx, double wy, int& x, int& y) -> bool {
        x = static_cast<int>(std::floor((wx - latest_voxgrid_.origin.x) / latest_voxgrid_.dl));
        y = static_cast<int>(std::floor((wy - latest_voxgrid_.origin.y) / latest_voxgrid_.dl));
        return !(x < 0 || y < 0 || x >= width || y >= height);
    };

    // ------------------------------------------------------------
    // Path Sampling
    // ------------------------------------------------------------
    struct SamplePoint { size_t path_idx; int x_idx, y_idx; double s, arrival_time; };
    std::vector<SamplePoint> samples;
    std::vector<double> cumulative_dist(path.size(), 0.0);
    
    for (size_t i = 1; i < path.size(); ++i) {
        cumulative_dist[i] = cumulative_dist[i - 1] + distance2D(path[i - 1], path[i]);
    }

    for (size_t i = 0; i < path.size(); i += sample_stride) {
        int x, y;
        if (!worldToVoxIndex(path[i].pose.position.x, path[i].pose.position.y, x, y)) continue;
        samples.push_back({i, x, y, cumulative_dist[i], cumulative_dist[i] / speed});
    }

    if (samples.empty()) return 1e9;

    // ------------------------------------------------------------
    // Part 1. Continuous Time-Margin Arrival Cost
    // ------------------------------------------------------------
    double arrival_cost = 0.0;
    bool has_valid_risk = false;

    for (const auto& sp : samples) {
        double peak_risk = 0.0;
        int peak_t_idx = 0;

        for (int tk = 0; tk < depth; ++tk) {
            double r_curr = getSpatialKernelMax(sp.x_idx, sp.y_idx, tk);
            if (r_curr > peak_risk) {
                peak_risk = r_curr;
                peak_t_idx = tk;
            }
        }

        if (peak_risk > 0.0) {
            has_valid_risk = true;
            double peak_time = static_cast<double>(peak_t_idx) * latest_voxgrid_.dt;
            double time_margin = std::fabs(sp.arrival_time - peak_time);
            arrival_cost += (peak_risk / (1.0 + time_margin)); // Continuous overlap penalty
        }
    }

    // ------------------------------------------------------------
    // Part 2. Candidate Extraction with Static-Dynamic Disentanglement
    // ------------------------------------------------------------
    struct Candidate { int t_idx; double risk; };
    std::vector<std::vector<Candidate>> candidates(samples.size());

    for (size_t si = 0; si < samples.size(); ++si) {
        bool in_segment = false;
        int segment_start_t = 0;
        int peak_t_idx = 0;
        double peak_risk = 0.0;

        for (int tk = 0; tk < depth; ++tk) {
            double r_curr = getSpatialKernelMax(samples[si].x_idx, samples[si].y_idx, tk);
            
            if (r_curr >= risk_threshold) {
                if (!in_segment) {
                    in_segment = true;
                    segment_start_t = tk;
                    peak_t_idx = tk;
                    peak_risk = r_curr;
                } else if (r_curr > peak_risk) {
                    peak_t_idx = tk;
                    peak_risk = r_curr;
                }
            } else if (in_segment) {
                // Reject static obstacles (persisting too long)
                if ((tk - segment_start_t) < static_duration_threshold) {
                    candidates[si].push_back({peak_t_idx, peak_risk});
                }
                in_segment = false;
                peak_risk = 0.0;
            }
        }
        if (in_segment && (depth - segment_start_t) < static_duration_threshold) {
            candidates[si].push_back({peak_t_idx, peak_risk});
        }
    }

    // ------------------------------------------------------------
    // Part 3. Tracking with Greedy Nearest-Neighbor (1:1 Matching)
    // ------------------------------------------------------------
    struct Track {
        int last_t_idx;
        int length;
        double sum_risk;
        std::vector<size_t> sample_history;
        std::vector<int> t_idx_history;
    };

    std::vector<Track> active_tracks;
    std::vector<Track> completed_tracks;

    for (size_t si = 0; si < samples.size(); ++si) {
        std::vector<Track> next_active_tracks;
        std::vector<bool> is_candidate_used(candidates[si].size(), false);

        // Extend active tracks with the best matching candidate (1:1)
        for (auto& tr : active_tracks) {
            int best_ci = -1;
            int min_dt = temporal_window_steps + 1;
            double max_risk = -1.0;

            for (int ci = 0; ci < static_cast<int>(candidates[si].size()); ++ci) {
                if (is_candidate_used[ci]) continue; // Prevent multiple tracks claiming one candidate

                const Candidate& cand = candidates[si][ci];
                int dt = std::abs(cand.t_idx - tr.last_t_idx);

                if (dt <= temporal_window_steps) {
                    if (dt < min_dt || (dt == min_dt && cand.risk > max_risk)) {
                        min_dt = dt;
                        max_risk = cand.risk;
                        best_ci = ci;
                    }
                }
            }

            if (best_ci != -1) {
                tr.last_t_idx = candidates[si][best_ci].t_idx;
                tr.length += 1;
                tr.sum_risk += candidates[si][best_ci].risk;
                tr.sample_history.push_back(si);
                tr.t_idx_history.push_back(candidates[si][best_ci].t_idx);
                
                next_active_tracks.push_back(tr);
                is_candidate_used[best_ci] = true;
            } else {
                if (tr.length >= min_event_length) {
                    completed_tracks.push_back(tr);
                }
            }
        }

        // Start new tracks from unused candidates
        for (int ci = 0; ci < static_cast<int>(candidates[si].size()); ++ci) {
            if (!is_candidate_used[ci]) {
                const Candidate& cand = candidates[si][ci];
                Track tr;
                tr.last_t_idx = cand.t_idx;
                tr.length = 1;
                tr.sum_risk = cand.risk;
                tr.sample_history = {si};
                tr.t_idx_history = {cand.t_idx};
                
                next_active_tracks.push_back(tr);
            }
        }
        active_tracks.swap(next_active_tracks);
    }

    for (const auto& tr : active_tracks) {
        if (tr.length >= min_event_length) completed_tracks.push_back(tr);
    }

    // ------------------------------------------------------------
    // Part 4. Micro-Slope Integration Cost (Path Integral)
    // ------------------------------------------------------------
    double temporal_motion_cost = 0.0;

    for (const auto& tr : completed_tracks) {
        double track_step_cost_sum = 0.0;
        double mean_event_risk = tr.sum_risk;

        for (size_t j = 1; j < tr.sample_history.size(); ++j) {
            size_t prev_s_idx = tr.sample_history[j - 1];
            size_t curr_s_idx = tr.sample_history[j];

            double ds = samples[curr_s_idx].s - samples[prev_s_idx].s;
            if (ds <= 1e-3) continue;

            double t_prev = static_cast<double>(tr.t_idx_history[j - 1]) * latest_voxgrid_.dt;
            double t_curr = static_cast<double>(tr.t_idx_history[j]) * latest_voxgrid_.dt;

            double local_slope = (t_curr - t_prev) / ds;
            double time_margin = std::fabs(samples[curr_s_idx].arrival_time - t_curr);
            double margin_weight = 1.0 / (1.0 + time_margin);

            double step_risk = mean_event_risk * margin_weight;
            track_step_cost_sum += (-step_risk * local_slope);
        }

        // Soft weight to filter out short track noise
        double L = static_cast<double>(tr.length);
        double w_track = 1.0 - std::exp(-alpha_length * (L - 1.0));

        temporal_motion_cost += (track_step_cost_sum * w_track);
    }

    return (w_arrival * arrival_cost) + (w_temporal_motion * temporal_motion_cost);
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

TemporalRiskAwarePlanner::PlannerWorkspace& TemporalRiskAwarePlanner::getWorkspace(int tid, int nx, int ny)
{
    const int ns = nx * ny;

    if (tid < 0 || tid >= static_cast<int>(workspaces_.size())) {
        ROS_ERROR("getWorkspace: invalid tid=%d workspaces.size=%zu",
                  tid, workspaces_.size());
        tid = 0;
    }

    PlannerWorkspace& ws = workspaces_[tid];

    if (ws.nx != nx || ws.ny != ny || ws.ns != ns ||
        !ws.p_calc || !ws.planner || !ws.path_maker) {

        ws.nx = nx;
        ws.ny = ny;
        ws.ns = ns;

        ws.costmap_copy.resize(ns);
        ws.potential_array.resize(ns);

        if (dynamic_cast<QuadraticCalculator*>(p_calc_) != NULL) {
            ws.p_calc.reset(new QuadraticCalculator(nx, ny));
        } else {
            ws.p_calc.reset(new PotentialCalculator(nx, ny));
        }

        if (dynamic_cast<AStarExpansion*>(planner_) != NULL) {
            ws.planner.reset(new AStarExpansion(ws.p_calc.get(), nx, ny));
        } else {
            DijkstraExpansion* local_dijkstra =
                new DijkstraExpansion(ws.p_calc.get(), nx, ny);

            if (!old_navfn_behavior_) {
                local_dijkstra->setPreciseStart(true);
            }

            ws.planner.reset(local_dijkstra);
        }

        if (dynamic_cast<GridPath*>(path_maker_) != NULL) {
            ws.path_maker.reset(new GridPath(ws.p_calc.get()));
        } else {
            ws.path_maker.reset(new GradientPath(ws.p_calc.get()));
        }

        ws.p_calc->setSize(nx, ny);
        ws.planner->setSize(nx, ny);
        ws.path_maker->setSize(nx, ny);
        ws.planner->setHasUnknown(allow_unknown_);
    }

    return ws;
}


bool TemporalRiskAwarePlanner::buildPlan(
    int tid,
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan)
{
    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized yet.");
        return false;
    }

    plan.clear();

    const std::string global_frame = frame_id_;

    if (goal.header.frame_id != global_frame ||
        start.header.frame_id != global_frame) {
        return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;

    if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
        return false;
    }

    if (old_navfn_behavior_) {
        start_x = start_x_i;
        start_y = start_y_i;
    } else {
        worldToMap(wx, wy, start_x, start_y);
    }

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
        return false;
    }

    if (old_navfn_behavior_) {
        goal_x = goal_x_i;
        goal_y = goal_y_i;
    } else {
        worldToMap(wx, wy, goal_x, goal_y);
    }

    const int nx = costmap_->getSizeInCellsX();
    const int ny = costmap_->getSizeInCellsY();
    const int ns = nx * ny;

    if (nx <= 0 || ny <= 0 || ns <= 0) {
        return false;
    }

    PlannerWorkspace& ws = getWorkspace(tid, nx, ny);

    const unsigned char* original_costmap = costmap_->getCharMap();

    std::copy(
        original_costmap,
        original_costmap + ns,
        ws.costmap_copy.begin());

    ws.costmap_copy[start_y_i * nx + start_x_i] = costmap_2d::FREE_SPACE;

    if (outline_map_) {
        outlineMap(ws.costmap_copy.data(), nx, ny, costmap_2d::LETHAL_OBSTACLE);
    }

    std::fill(
        ws.potential_array.begin(),
        ws.potential_array.end(),
        POT_HIGH);

    const bool found_legal =
        ws.planner->calculatePotentials(
            ws.costmap_copy.data(),
            start_x,
            start_y,
            goal_x,
            goal_y,
            ns * 2,
            ws.potential_array.data());

    if (!old_navfn_behavior_) {
        ws.planner->clearEndpoint(
            ws.costmap_copy.data(),
            ws.potential_array.data(),
            goal_x_i,
            goal_y_i,
            2);
    }

    if (!found_legal) {
        return false;
    }

    if (!getPlanFromPotentialThreadSafe(
            ws.path_maker.get(),
            ws.potential_array.data(),
            start_x,
            start_y,
            goal_x,
            goal_y,
            goal,
            plan)) {
        return false;
    }

    geometry_msgs::PoseStamped goal_copy = goal;
    goal_copy.header.stamp = ros::Time::now();
    plan.push_back(goal_copy);

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

bool TemporalRiskAwarePlanner::findNearestPose(const geometry_msgs::PoseStamped& query,
                                          const std::vector<geometry_msgs::PoseStamped>& poses,
                                          geometry_msgs::PoseStamped& nearest_pose) const 
{
    double min_dist = std::numeric_limits<double>::infinity();
    size_t nearest_idx = 0;

    for (size_t i = 0; i < poses.size(); ++i) {
        const double dist = distance2D(query, poses[i]);

        if (dist < min_dist) {
            min_dist = dist;
            nearest_idx = i;
        }
    }

    nearest_pose = poses[nearest_idx];
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

void TemporalRiskAwarePlanner::publishTrackMarkers(
    const std::vector<Track>& tracks, 
    const std::vector<SamplePoint>& samples, 
    const std::vector<geometry_msgs::PoseStamped>& path) const 
{
    visualization_msgs::MarkerArray marker_array;
    
    visualization_msgs::Marker delete_marker;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    int marker_id = 0;

    for (size_t i = 0; i < tracks.size(); ++i) {
        const auto& tr = tracks[i];

        visualization_msgs::Marker points_marker;
        points_marker.header.frame_id = "odom";
        points_marker.header.stamp = ros::Time::now();
        points_marker.ns = "active_track_points";
        points_marker.id = marker_id++;
        points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        points_marker.action = visualization_msgs::Marker::ADD;
        
        points_marker.scale.x = 0.2; 
        points_marker.scale.y = 0.2;
        points_marker.scale.z = 0.2;
        
        points_marker.color.r = 0.0; 
        points_marker.color.g = 0.0;
        points_marker.color.b = 1.0;
        points_marker.color.a = 1.0;

        visualization_msgs::Marker line_marker;
        line_marker.header = points_marker.header;
        line_marker.ns = "active_track_lines";
        line_marker.id = marker_id++;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        
        line_marker.scale.x = 0.05; 
        
        line_marker.color.r = 0.0; 
        line_marker.color.g = 0.0;
        line_marker.color.b = 1.0;
        line_marker.color.a = 0.8;

        for (size_t j = 0; j < tr.sample_history.size(); ++j) {
            size_t s_idx = tr.sample_history[j];
            int t_idx = tr.t_idx_history[j];
            
            size_t original_path_idx = samples[s_idx].path_idx;

            geometry_msgs::Point p;
            p.x = path[original_path_idx].pose.position.x;
            p.y = path[original_path_idx].pose.position.y;

            p.z = static_cast<double>(t_idx) * 0.1; 

            points_marker.points.push_back(p);
            line_marker.points.push_back(p);
        }

        marker_array.markers.push_back(points_marker);
        marker_array.markers.push_back(line_marker);
    }

    planning_track_marker_pub_.publish(marker_array);
}

void TemporalRiskAwarePlanner::publishPlanningDebugMarkers(
    const std::vector<geometry_msgs::PoseStamped>& local_goal_line,
    const std::vector<geometry_msgs::PoseStamped>& endpoints,
    const std::vector<TemporalRiskAwarePlanner::PathCandidate>& candidates,
    const std::vector<geometry_msgs::PoseStamped>& selected_path,
    int selected_homotopy_id) {
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
            make_marker("local_goal_line", 0, visualization_msgs::Marker::LINE_LIST);
        marker.scale.x = 0.06;
        setMarkerColor(marker, 1.0f, 0.85f, 0.0f, 0.9f);

        for (size_t i = 0; i + 1 < local_goal_line.size(); i += 2) {
            marker.points.push_back(markerPoint(local_goal_line[i], 0.04));
            marker.points.push_back(markerPoint(local_goal_line[i + 1], 0.04));
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
        {0.55f, 0.70f, 0.20f},
        {0.35f, 0.85f, 0.55f},
        {0.95f, 0.75f, 0.20f},
        {0.45f, 0.25f, 0.95f},
        {0.15f, 0.85f, 0.85f},
        {0.80f, 0.30f, 0.70f}
    };
    const size_t palette_size = sizeof(palette) / sizeof(palette[0]);

    for (size_t i = 0; i < candidates.size(); ++i) {
        if (candidates[i].path.size() < 2) {
            continue;
        }

        const int homotopy_id = candidates[i].homotopy_id;
        const float* color = palette[homotopy_id % palette_size];

        visualization_msgs::Marker marker =
            make_marker("sub_paths", static_cast<int>(i), visualization_msgs::Marker::LINE_STRIP);
        marker.scale.x = (homotopy_id == selected_homotopy_id) ? 0.06 : 0.035;
        setMarkerColor(marker, color[0], color[1], color[2], (homotopy_id == selected_homotopy_id) ? 0.9f : 0.45f);

        for (const auto& pose : candidates[i].path) {
            marker.points.push_back(markerPoint(pose, 0.06));
        }
        marker_array.markers.push_back(marker);

        const size_t label_idx = candidates[i].path.size() / 2;
        visualization_msgs::Marker text_marker =
            make_marker("sub_path_costs", static_cast<int>(i), visualization_msgs::Marker::TEXT_VIEW_FACING);
        text_marker.pose = candidates[i].path[label_idx].pose;
        text_marker.pose.position. y += 0.15;
        text_marker.pose.position.z += 0.45;
        text_marker.scale.z = 0.24;
        setMarkerColor(text_marker, color[0], color[1], color[2], 1.0f);

        char label[64];
        std::snprintf(label, sizeof(label), "H%d  R %.2f", homotopy_id, candidates[i].temporal_risk_score);
        text_marker.text = label;
        marker_array.markers.push_back(text_marker);
    }

    if (selected_path.size() >= 2) {
        const float* selected_color = (selected_homotopy_id >= 0)
            ? palette[selected_homotopy_id % palette_size]
            : palette[0];

        visualization_msgs::Marker marker =
            make_marker("selected_path", 0, visualization_msgs::Marker::LINE_STRIP);
        marker.scale.x = 0.12;
        setMarkerColor(marker, selected_color[0], selected_color[1], selected_color[2], 1.0f);

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

bool TemporalRiskAwarePlanner::getPlanFromPotentialThreadSafe(
    Traceback* local_path_maker,
    float* local_potential_array,
    double start_x,
    double start_y,
    double goal_x,
    double goal_y,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan)
{
    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized yet.");
        return false;
    }

    const std::string global_frame = frame_id_;

    plan.clear();

    std::vector<std::pair<float, float>> path;

    if (!local_path_maker->getPath(
            local_potential_array,
            start_x,
            start_y,
            goal_x,
            goal_y,
            path)) {
        return false;
    }

    const ros::Time plan_time = ros::Time::now();

    for (int i = static_cast<int>(path.size()) - 1; i >= 0; --i) {
        const std::pair<float, float>& point = path[i];

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

    if (old_navfn_behavior_) {
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
