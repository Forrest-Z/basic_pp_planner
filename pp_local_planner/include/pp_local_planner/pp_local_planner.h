/*********************************************************************
 *
 * Software License Agreement 
 *
 *  Copyright (c) 2020, Botsync.
 *  All rights reserved.
 *
 * Author: Gopan B.Chandran 
 *
 *********************************************************************/

#ifndef PP_LOCAL_PLANNER_H
#define PP_LOCAL_PLANNER_H

#include <vector>
#include <Eigen/Core>

#include "motion_planner.h"
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
    
   /* struct PoseCompare
    {
        bool operator ()(const geometry_msgs::PoseStamped& pose_a, const geometry_msgs::PoseStamped pose_b)
        {
            return (pose_a.pose.position.x == pose_b.pose.position.x && pose_a.pose.position.y == pose_b.pose.position.y
            && pose_a.pose.position.z == pose_b.pose.position.z && pose_a.pose.orientation.x == pose_b.pose.orientation.y ==
            pose_b.pose.orientation.y && pose_a.pose.orientation.w == pose_b.pose.orientation.w && pose_a.pose.orientation.z
            == pose_b.pose.orientation.z);
        }
    };*/
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
            PPLocalPlanner(std::string name, tf2_ros::Buffer* tf, base_local_planner::LocalPlannerUtil *planner_util, std::string
            motion_frame);

            ~PPLocalPlanner();

            /**
             * @brief Reconfigures the trajectory planner
             */
            void reconfigure(PPLocalPlannerConfig &cfg);

            /**
             * sets new plan and resets state
             */
            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
            bool ppUpdate(const tf2_ros::Buffer* tf, const geometry_msgs::PoseStamped& global_pose, std::vector<geometry_msgs::Point> footprint_spec, const geometry_msgs::Twist& robot_vel, std::vector<geometry_msgs::PoseStamped>& local_plan, geometry_msgs::Twist& cmd_vel);
            bool isGoalReached(const geometry_msgs::PoseStamped& robot_pose);
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
            bool computeLinearVelocity(std::vector<geometry_msgs::PoseStamped>& transformed_plan, std::vector<geometry_msgs::Point> footprint_spec, const geometry_msgs::Twist& robot_vel, const
            geometry_msgs::PoseStamped& global_pose, mpd::MotionPlan& mpl, geometry_msgs::Twist& base_command);
            bool computeAngularVelocity(const tf2_ros::Buffer* tf, const geometry_msgs::PoseStamped& lookahead_pose, const geometry_msgs::PoseStamped&
            global_pose, const geometry_msgs::Twist& robot_vel, double linear_vel, double& angular_vel);
            bool getLookaheadPoint(const tf2_ros::Buffer* tf, std::vector<geometry_msgs::PoseStamped>&
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
            motion_planner::MotionPlanner* mplnr;
            std::vector<geometry_msgs::PoseStamped> global_plan_;
            std::string frame_id_;
            bool publish_traj;
            
            //object to get local planner limit parameters 
            typedef base_local_planner::LocalPlannerLimits planner_limits; 
            //structure containing purepursuit configurations.
            struct PurepursuitConfig
            {
                double min_lookahead;
                double max_lookahead;
                double kla;
                double kct;
                planner_limits limits_;
            };

            //container holding information for debugging.
            struct PurepursuitDebug
            {
                PurepursuitDebug()
                {
                    this->pose_debug_pub = this->nh_debug.advertise<geometry_msgs::PoseStamped>("/lookahead_pose", 1);
                }
               
                /*
                 * @brief method to update infos to publish
                 * @param dynamic lookahead information.
                 * @param dynamic latertal shif information.
                 * @param curvature info to publish.
                 * @param lookahead info
                 */
                //typename<class Info, class Data>
                //void updateDebug(Info debug_info, Data debug_data)
                void updateDebug(double dynamic_lookahead, double lateral_shift, double curvature)
                {
                    this->dynamic_lookahead = dynamic_lookahead;
                    this->lateral_shift = lateral_shift;
                    this->curvature = curvature;
                }

                void updateDebug(geometry_msgs::PoseStamped pose)
                {
                    this->lookahead_pose = pose;
                }


                void publishDebug()
                {
                    this->pose_debug_pub.publish(this->lookahead_pose);
                }

                private:
                    ros::NodeHandle nh_debug;
                    ros::Publisher pose_debug_pub; 
                    double dynamic_lookahead;
                    double lateral_shift;
                    double curvature;
                    geometry_msgs::PoseStamped lookahead_pose;
            };

            PurepursuitConfig pp_config;
            PurepursuitDebug* pp_debug;


    };
    
};
#endif
