#ifndef _PLANNERCORE_H
#define _PLANNERCORE_H
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
#define POT_HIGH 1.0e10        // unassigned cell potential
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <dynamic_reconfigure/server.h>
#include <temporal_risk_aware_planner/potential_calculator.h>
#include <temporal_risk_aware_planner/expander.h>
#include <temporal_risk_aware_planner/traceback.h>
#include <temporal_risk_aware_planner/orientation_filter.h>
#include <temporal_risk_aware_planner/TemporalRiskAwarePlannerConfig.h>
#include <vox_msgs/VoxGrid.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <omp.h>
#include <std_msgs/Float64.h>

namespace temporal_risk_aware_planner {

class Expander;
class GridPath;

/**
 * @class PlannerCore
 * @brief Provides a ROS wrapper for the temporal_risk_aware_planner planner which runs a fast, interpolated navigation function on a costmap.
 */

class TemporalRiskAwarePlanner : public nav_core::BaseGlobalPlanner {
    public:
        /**
         * @brief  Default constructor for the PlannerCore object
         */
        TemporalRiskAwarePlanner();

        /**
         * @brief  Constructor for the PlannerCore object
         * @param  name The name of this planner
         * @param  costmap A pointer to the costmap to use
         * @param  frame_id Frame of the costmap
         */
        TemporalRiskAwarePlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

        /**
         * @brief  Default deconstructor for the PlannerCore object
         */
        ~TemporalRiskAwarePlanner();

        /**
         * @brief  Initialization function for the PlannerCore object
         * @param  name The name of this planner
         * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
         */
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan);

        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param tolerance The tolerance on the goal point for the planner
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance,
                      std::vector<geometry_msgs::PoseStamped>& plan);

        /**
         * @brief  Computes the full navigation function for the map given a point in the world to start from
         * @param world_point The point to use for seeding the navigation function
         * @return True if the navigation function was computed successfully, false otherwise
         */
        bool computePotential(const geometry_msgs::Point& world_point);

        /**
         * @brief Compute a plan to a goal after the potential for a start point has already been computed (Note: You should call computePotential first)
         * @param start_x
         * @param start_y
         * @param end_x
         * @param end_y
         * @param goal The goal pose to create a plan to
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */

        bool getPlanFromPotentialThreadSafe(
            Traceback* local_path_maker,
            float* local_potential_array,
            double start_x,
            double start_y,
            double goal_x,
            double goal_y,
            const geometry_msgs::PoseStamped& goal,
            std::vector<geometry_msgs::PoseStamped>& plan);
        bool getPlanFromPotential(double start_x, double start_y, double end_x, double end_y,
                                  const geometry_msgs::PoseStamped& goal,
                                  std::vector<geometry_msgs::PoseStamped>& plan);

        /**
         * @brief Get the potential, or naviagation cost, at a given point in the world (Note: You should call computePotential first)
         * @param world_point The point to get the potential for
         * @return The navigation function's value at that point in the world
         */
        double getPointPotential(const geometry_msgs::Point& world_point);

        /**
         * @brief Check for a valid potential value at a given point in the world (Note: You should call computePotential first)
         * @param world_point The point to get the potential for
         * @return True if the navigation function is valid at that point in the world, false otherwise
         */
        bool validPointPotential(const geometry_msgs::Point& world_point);

        /**
         * @brief Check for a valid potential value at a given point in the world (Note: You should call computePotential first)
         * @param world_point The point to get the potential for
         * @param tolerance The tolerance on searching around the world_point specified
         * @return True if the navigation function is valid at that point in the world, false otherwise
         */
        bool validPointPotential(const geometry_msgs::Point& world_point, double tolerance);

        /**
         * @brief  Publish a path for visualization purposes
         */
        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

        bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);

        // struct Track {
        //     int start_t_idx;
        //     int last_t_idx;
        //     size_t start_sample_idx;
        //     size_t last_sample_idx;
        //     int length;
        //     double sum_risk;
        //     std::vector<size_t> sample_history;
        //     std::vector<int> t_idx_history;
        // };

    protected:

        /**
         * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
         */
        costmap_2d::Costmap2D* costmap_;
        std::string frame_id_;
        ros::Publisher plan_pub_;
        bool initialized_, allow_unknown_;

    private:
        void mapToWorld(double mx, double my, double& wx, double& wy);
        bool worldToMap(double wx, double wy, double& mx, double& my);
        void clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my);
        void publishPotential(float* potential);
        struct Track {
            int last_t_idx;
            int length;
            double sum_risk;
            std::vector<size_t> sample_history;
            std::vector<int> t_idx_history;
        };
        struct PathCandidate {
            std::vector<geometry_msgs::PoseStamped> path;
            int homotopy_id = -1;
            double temporal_risk_score = 0.0;
        };
        struct SamplePoint {
            size_t path_idx;
            int x_idx;
            int y_idx;
            double s;
            double arrival_time;
            int arrival_t_idx;
        };
        struct PlannerWorkspace
        {
            int nx = 0;
            int ny = 0;
            int ns = 0;

            std::vector<unsigned char> costmap_copy;
            std::vector<float> potential_array;

            std::unique_ptr<PotentialCalculator> p_calc;
            std::unique_ptr<Expander> planner;
            std::unique_ptr<Traceback> path_maker;
        };
        double distance2D(const geometry_msgs::PoseStamped& a,
                          const geometry_msgs::PoseStamped& b) const;
        bool buildPlan(int tid, const geometry_msgs::PoseStamped& start,
                       const geometry_msgs::PoseStamped& goal,
                       std::vector<geometry_msgs::PoseStamped>& plan);
        bool isGoalChanged(const geometry_msgs::PoseStamped& goal) const;
        bool isPoseNear(const geometry_msgs::PoseStamped& a,
                        const geometry_msgs::PoseStamped& b,
                        double tolerance) const;
        bool isSubgoalSafe(const geometry_msgs::PoseStamped& pose);
        bool selectSubgoal(const geometry_msgs::PoseStamped& start,
                           geometry_msgs::PoseStamped& subgoal);
        bool reselectSubgoal(const std::vector<geometry_msgs::PoseStamped>& path, size_t current_idx, geometry_msgs::PoseStamped& subgoal);
        size_t findNearestIdx(
            const geometry_msgs::PoseStamped& start,
            const std::vector<geometry_msgs::PoseStamped>& path,
            size_t search_begin) const;     
        void publishSubgoalMarker(const geometry_msgs::PoseStamped& subgoal);
        void publishNearestMarker(const geometry_msgs::PoseStamped& nearest);
        void publishTrackMarkers(
            const std::vector<Track>& tracks, 
            const std::vector<SamplePoint>& samples, 
            const std::vector<geometry_msgs::PoseStamped>& path) const;
        void publishPlanningDebugMarkers(
            const std::vector<geometry_msgs::PoseStamped>& local_goal_line,
            const std::vector<geometry_msgs::PoseStamped>& endpoints,
            const std::vector<PathCandidate>& candidates,
            const std::vector<geometry_msgs::PoseStamped>& selected_path,
            int selected_homotopy_id = -1);
        void voxGridCallback(const vox_msgs::VoxGrid::ConstPtr& msg);
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        bool getTemporalRiskAt(double wx, double wy, double time_from_now, double& risk) const;
        void pruneSubpath(const geometry_msgs::PoseStamped& start,
                          std::vector<geometry_msgs::PoseStamped>& pruned_subpath) const;
        bool isTemporalRiskAcceptable(const std::vector<geometry_msgs::PoseStamped>& path) const;
        std::vector<std::vector<geometry_msgs::PoseStamped>> findMultiplePaths(const geometry_msgs::PoseStamped& start, const std::vector<geometry_msgs::PoseStamped>& endpoints);
        std::vector<PathCandidate> groupPathsByHomotopy(const std::vector<std::vector<geometry_msgs::PoseStamped>>& valid_paths,
                                                        const geometry_msgs::PoseStamped& start,
                                                        const std::vector<geometry_msgs::PoseStamped>& previous_path,
                                                        int& previous_homotopy_id);
        bool isSameHomotopy(const std::vector<geometry_msgs::PoseStamped>& patha, 
                              const std::vector<geometry_msgs::PoseStamped>& pathb,
                              const geometry_msgs::PoseStamped& start);
        bool findNearestPose(const geometry_msgs::PoseStamped& query,
                                          const std::vector<geometry_msgs::PoseStamped>& poses,
                                          geometry_msgs::PoseStamped& nearest_pose) const ;
        bool hasObstacleInside(const std::vector<geometry_msgs::PoseStamped>& patha,
                               const std::vector<geometry_msgs::PoseStamped>& pathb,
                               const geometry_msgs::PoseStamped& start);
        void createLocalGoalLine(const geometry_msgs::PoseStamped& start, 
                                                   const geometry_msgs::PoseStamped& global_goal, 
                                                   bool is_near, 
                                                   std::vector<geometry_msgs::PoseStamped>& endpoints,
                                                   std::vector<geometry_msgs::PoseStamped>& local_goal_line);
        double evalTemporalRisk(const std::vector<geometry_msgs::PoseStamped>& path) const;
        std::vector<std::pair<int, int>> Bresenham(const std::pair<int, int>& p1, const std::pair<int, int>& p2);
        TemporalRiskAwarePlanner::PlannerWorkspace& getWorkspace(int tid, int nx, int ny);

        std::vector<PlannerWorkspace> workspaces_;

        double planner_window_x_, planner_window_y_, default_tolerance_;
        boost::mutex mutex_;
        ros::ServiceServer make_plan_srv_;

        ros::Subscriber voxgrid_sub_;
        vox_msgs::VoxGrid latest_voxgrid_;
        bool has_voxgrid_;

        ros::Subscriber odom_sub_;
        double current_robot_speed_;

        ros::Publisher subgoal_marker_pub_;
        ros::Publisher nearest_marker_pub_;
        ros::Publisher planning_debug_marker_pub_;
        ros::Publisher planning_track_marker_pub_;

        PotentialCalculator* p_calc_;
        Expander* planner_;
        Traceback* path_maker_;
        OrientationFilter* orientation_filter_;

        bool publish_potential_;
        ros::Publisher potential_pub_;
        int publish_scale_;

        void outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);

        float* potential_array_;
        unsigned int start_x_, start_y_, end_x_, end_y_;

        bool old_navfn_behavior_;
        float convert_offset_;

        bool outline_map_;

        bool has_global_goal_;
        double global_goal_near_dist_;
        double goal_tolerance_;
        double lookahead_dist_;
        geometry_msgs::PoseStamped initial_start_;
        geometry_msgs::PoseStamped global_goal_;
        geometry_msgs::PoseStamped last_global_goal_;
        std::vector<geometry_msgs::PoseStamped> previous_subpath_;
        ros::Publisher planning_time_pub_;

        dynamic_reconfigure::Server<temporal_risk_aware_planner::TemporalRiskAwarePlannerConfig> *dsrv_;
        void reconfigureCB(temporal_risk_aware_planner::TemporalRiskAwarePlannerConfig &config, uint32_t level);

};

} //end namespace temporal_risk_aware_planner

#endif
