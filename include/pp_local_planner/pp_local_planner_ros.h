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

#ifndef PP_PLANNER_ROS_H
#define PP_PLANNER_ROS_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <pp_local_planner/PPLocalPlannerConfig.h>

#include <angles/angles.h>

#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/latched_stop_rotate_controller.h>

#include <base_local_planner/odometry_helper_ros.h>

#include <pp_local_planner/pp_local_planner.h>

namespace pp_local_planner {
    /**
     * @class PPLocalPlannerROS
     * @brief ROS Wrapper for the PPLocalPlanner that adheres to the
     * BaseLocalPlanner interface and can be used as a plugin for move_base.
     */
    class PPLocalPlannerROS : public nav_core::BaseLocalPlanner {
        public:
            /**
             * @brief  Constructor for PPLocalPlannerROS wrapper
             */
            PPLocalPlannerROS();

            /**
             * @brief  Constructs the ros wrapper
             * @param name The name to give this instance of the trajectory planner
             * @param tf A pointer to a transform listener
             * @param costmap The cost map to use for assigning costs to trajectories
             */
            void initialize(std::string name, tf::TransformListener* tf,
                    costmap_2d::Costmap2DROS* costmap_ros);

            /**
             * @brief  Destructor for the wrapper
             */
            ~PPLocalPlannerROS();

            /**
             * @brief  Given the current position, orientation, and velocity of the robot,
             * compute velocity commands to send to the base
             * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
             * @return True if a valid trajectory was found, false otherwise
             */
            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);


            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

            /**
             * @brief  Check if the goal pose has been achieved
             * @return True if achieved, false otherwise
             */
            bool isGoalReached();



            bool isInitialized() {
                return initialized_;
            }

            /**
             * @brief Given the current position, orientation, and velocity of the robot,
             * compute velocity commands to send to the base, using purepursuit controller approach.
             * This method only follows global plans and stops if obstacle is there.
             * @param position of the robot in the global frame.
             * @param cmd_vel will be filled with the velocity commands to be passed to the robot base.
             * @return True if a valid control is found, false otherwise.
             */
            bool ppComputeVelocityCommands(const geometry_msgs::PoseStamped &global_pose, geometry_msgs::Twist& cmd_vel);

            /**
             * @brief override method to update 
             * robot loaded state
             */
            void setLoadedState(bool isloaded);

        private:
            /**
             * @brief Callback to update the local planner's parameters based on dynamic reconfigure
             */
            void reconfigureCB(PPLocalPlannerConfig &config, uint32_t level);

            void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

            void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

            tf::TransformListener* tf_; ///< @brief Used for transforming point clouds

            // for visualisation, publishers of global and local plan
            ros::Publisher g_plan_pub_, l_plan_pub_;

            base_local_planner::LocalPlannerUtil planner_util_;

            boost::shared_ptr<PPLocalPlanner> pp_; ///< @brief The trajectory controller

            costmap_2d::Costmap2DROS* costmap_ros_;

            dynamic_reconfigure::Server<PPLocalPlannerConfig> *dsrv_;
            pp_local_planner::PPLocalPlannerConfig default_config_;
            bool setup_;
            tf::Stamped<tf::Pose> current_pose_;

            base_local_planner::LatchedStopRotateController latchedStopRotateController_;


            bool initialized_;


            base_local_planner::OdometryHelperRos odom_helper_;
            std::string odom_topic_;
    };

};
#endif
