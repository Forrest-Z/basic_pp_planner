/*********************************************************************
 *
 * Software License Agreement 
 *
 *  Copyright (c) 2009, Botsync.
 *  All rights reserved.
 *
 * Authors: botsync.co 
 *
 *********************************************************************/

#ifndef PP_LOCAL_PLANNER_H
#define PP_LOCAL_PLANNER_H

#include <vector>
#include <Eigen/Core>


#include <pp_local_planner/PPLocalPlannerConfig.h>

//for creating a local cost grid
#include <base_local_planner/map_grid_visualizer.h>

//for transformation to be updated to tf2.
#include <tf/transform_listener.h>

//for obstacle data access
#include <costmap_2d/costmap_2d.h>

#include <base_local_planner/trajectory.h>
#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/simple_trajectory_generator.h>

#include <base_local_planner/oscillation_cost_function.h>
#include <base_local_planner/map_grid_cost_function.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/twirling_cost_function.h>
#include <base_local_planner/simple_scored_sampling_planner.h>

#include <nav_msgs/Path.h>

namespace pp_local_planner {
    
    /*double operator*(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2)
    {
        return (sqrt(pose1.pose.position.x * pose2.pose.position.x + pose1.pose.position.y * pose2.pose.position.y));
    }*/
    /**
     * @class PPLocalPlanner
     * @brief A class implementing a local planner using the purepursuit approach. 
     */
    class PPLocalPlanner {
        public:
            /**
             * @brief  Constructor for the planner
             * @param name The name of the planner 
             * @param costmap_ros A pointer to the costmap instance the planner should use
             * @param global_frame the frame id of the tf frame to use
             */
            PPLocalPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util);

            /**
             * @brief Reconfigures the trajectory planner
             */
            void reconfigure(PPLocalPlannerConfig &cfg);

            /**
             * sets new plan and resets state
             */
            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
            bool ppUpdate(const tf2_ros::Buffer* tf, const std::vector<geometry_msgs::PoseStamped>& transformed_plan, const
            geometry_msgs::PoseStamped& global_pose, const std::vector<geometry_msgs::Point>& footprint_spec, const geometry_msgs::Twist& robot_vel, geometry_msgs::Twist& cmd_vel);

        private:

            /*
             *@brief this method checks whether robot will hit the obstacle or not at a point in world frame with current footprint specification.
             * @param x coordinate of the pose in the map frame to be checked.
             * @param y coordinate of the pose in the map frame to be checked.
             * @param theta orientation of the position w.r.to map frame to be checked. 
             * @param current footprint of the robot.
             * @return cost of the footprint at the given point. Less than zero represents footprint is in obstacle, greater than zero is free from obstacle.
             */
            double checkFootprintCost(double x, double y, double theta, std::vector<geometry_msgs::Point> footprint_spec);
            double calculateDynamicLookahead(const geometry_msgs::Twist& robot_vel);
            bool computeLinearVelocity(const std::vector<geometry_msgs::PoseStamped>& transformed_plan, const
            std::vector<geometry_msgs::Point>& footprint_spec, const geometry_msgs::Twist& robot_vel, const geometry_msgs::PoseStamped& global_pose, double& linear_vel);
            bool computeAngularVelocity(const tf2_ros::Buffer* tf, const geometry_msgs::PoseStamped& lookahead_pose, const geometry_msgs::PoseStamped&
            global_pose, const geometry_msgs::Twist&, double linear_vel, double& angular_vel);
            bool getLookaheadPoint(const std::vector<geometry_msgs::PoseStamped>&
            transformed_plan, const geometry_msgs::PoseStamped& global_pose, const geometry_msgs::Twist& robot_vel,
            geometry_msgs::PoseStamped& lookahead_pose);
            bool makeVelocityPlan(const std::vector<geometry_msgs::PoseStamped>& transformed_plan, const
            std::vector<geometry_msgs::Point>& footprint_spec, const geometry_msgs::Twist& robot_vel); 
            inline double euclidean(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2)
            {
                return (sqrt(pow(pose1.pose.position.x - pose2.pose.position.x, 2) + pow(pose1.pose.position.y -
                pose2.pose.position.y, 2)));
            }
            boost::mutex configuration_mutex_;
            base_local_planner::LocalPlannerUtil *planner_util_;
            std::vector<geometry_msgs::PoseStamped> global_plan_;
            std::string frame_id_;
            bool publish_traj;
            typedef base_local_planner::LocalPlannerLimits planner_limits; 
            struct PurepursuitConfig
            {
                double min_lookahead;
                double max_lookahead;
                double kla;
                double kct;
                planner_limits limits_;
            };
            PurepursuitConfig pp_config;


    };
    
};
#endif
