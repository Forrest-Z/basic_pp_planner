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

#include <pp_local_planner/pp_local_planner_ros.h>
#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>

//#include <nav_core/parameter_magic.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(pp_local_planner::PPLocalPlannerROS, nav_core::BaseLocalPlanner)

    namespace pp_local_planner {

        void PPLocalPlannerROS::reconfigureCB(PPLocalPlannerConfig &config, uint32_t level) {
            if (setup_ && config.restore_defaults) {
                config = default_config_;
                config.restore_defaults = false;
            }
            if ( ! setup_) {
                default_config_ = config;
                setup_ = true;
            } 

            // update generic local planner params
            /*base_local_planner::LocalPlannerLimits limits;
            limits.max_vel_x = config.max_vel_x;
            limits.min_vel_x = config.min_vel_x;
            limits.max_vel_y = config.max_vel_y;
            limits.min_vel_y = config.min_vel_y;
            limits.max_rot_vel = config.max_rot_vel;
            limits.min_rot_vel = config.min_rot_vel;
            limits.acc_lim_x = config.acc_lim_x;
            limits.acc_lim_y = config.acc_lim_y;
            limits.acc_lim_theta = config.acc_lim_theta;
            limits.xy_goal_tolerance = config.xy_goal_tolerance;
            limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
            limits.prune_plan = config.prune_plan;
            planner_util_.reconfigureCB(limits, config.restore_defaults);*/
            pp_->reconfigure(config);
        }

        PPLocalPlannerROS::PPLocalPlannerROS() : initialized_(false),
        odom_helper_("odom"), setup_(false) {

        }

        void PPLocalPlannerROS::initialize(
                std::string name,
                tf::TransformListener* tf,
                costmap_2d::Costmap2DROS* costmap_ros) {
            if (!isInitialized()) {

                ros::NodeHandle private_nh("~/" + name);
                ros::NodeHandle private_nh_("~/");
                g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
                l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
                tf_ = tf;
                costmap_ros_ = costmap_ros;
                costmap_ros_->getRobotPose(current_pose_);

                // make sure to update the costmap we'll use for this cycle
                costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

                planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

                //create the actual planner that we'll use.. it'll configure itself from the parameter server
                pp_  = boost::shared_ptr<PPLocalPlanner>(new PPLocalPlanner(name, tf_, &planner_util_, costmap_ros_->getGlobalFrameID()));

                if( private_nh_.getParam( "odom_topic", odom_topic_ ))
                {
                    odom_helper_.setOdomTopic( odom_topic_ );
                }

                initialized_ = true;

                // Warn about deprecated parameters -- remove this block in N-turtle
                /*nav_core::warnRenamedParameter(private_nh, "max_vel_trans", "max_trans_vel");
                nav_core::warnRenamedParameter(private_nh, "min_vel_trans", "min_trans_vel");
                nav_core::warnRenamedParameter(private_nh, "max_vel_theta", "max_rot_vel");
                nav_core::warnRenamedParameter(private_nh, "min_vel_theta", "min_rot_vel");
                nav_core::warnRenamedParameter(private_nh, "acc_lim_trans", "acc_limit_trans");
                nav_core::warnRenamedParameter(private_nh, "theta_stopped_vel", "rot_stopped_vel");*/

                dsrv_ = new dynamic_reconfigure::Server<PPLocalPlannerConfig>(private_nh);
                dynamic_reconfigure::Server<PPLocalPlannerConfig>::CallbackType cb = boost::bind(&PPLocalPlannerROS::reconfigureCB, this, _1, _2);
                dsrv_->setCallback(cb);
            }
            else{
                ROS_WARN("This planner has already been initialized, doing nothing.");
            }
        }

        bool PPLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
            if (! isInitialized()) {
                ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
                return false;
            }
            //when we get a new plan, we also want to clear any latch we may have on goal tolerances
            latchedStopRotateController_.resetLatching();

            ROS_INFO("Got new plan");
            return pp_->setPlan(orig_global_plan);
        }

        bool PPLocalPlannerROS::isGoalReached() {
            if (! isInitialized()) {
                ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
                return false;
            }
            if ( ! costmap_ros_->getRobotPose(current_pose_)) {
                ROS_ERROR("Could not get robot pose");
                return false;
            }
            geometry_msgs::PoseStamped current_pose;
            poseStampedTFToMsg(current_pose_, current_pose);
            if(pp_->isGoalReached(current_pose))
            { 
               return true;
            }

            return false;

            /*if(latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
                ROS_INFO("Goal reached");
                return true;
            } else {
                ROS_INFO("Goal not reached");
                return false;
            }*/
        }

        void PPLocalPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
            base_local_planner::publishPlan(path, l_plan_pub_);
        }


        void PPLocalPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
            base_local_planner::publishPlan(path, g_plan_pub_);
        }

        PPLocalPlannerROS::~PPLocalPlannerROS(){
            //make sure to clean things up
            delete dsrv_;
        }


        bool PPLocalPlannerROS::ppComputeVelocityCommands(const geometry_msgs::PoseStamped& global_pose, geometry_msgs::Twist& cmd_vel)
        {

            if(!isInitialized()){
                ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
                return false;
            }
            geometry_msgs::Twist robot_vel;
            odom_helper_.getRobotVelTwist(robot_vel);
            std::vector<geometry_msgs::Point> footprint_spec = costmap_ros_->getRobotFootprint();
            std::vector<geometry_msgs::PoseStamped> local_plan;
            if(pp_->ppUpdate(tf_, global_pose, footprint_spec, robot_vel, local_plan, cmd_vel))
            {
                //updateLocalPlan(local_plan, cmd_vel);
                publishLocalPlan(local_plan);
                return true;
            }
            else
            {
                ROS_WARN("purepursuit failed to find a valid plan");
                return false;
            }
        }


        bool PPLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) 
        {
            //get robot postion in global frame. 
            if ( ! costmap_ros_->getRobotPose(current_pose_)) {
                ROS_ERROR("Could not get robot pose");
                return false;
            }
            //get the global plan that robot has to follow.
            /*std::vector<geometry_msgs::PoseStamped> transformed_plan;
            if ( ! planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
                ROS_ERROR("Could not get local plan");
                return false;
            }*/

            //if the global plan passed in is empty... we won't do anything
            /*if(transformed_plan.empty()) {
                ROS_WARN_NAMED("pp_local_planner", "Received an empty transformed plan.");
                return false;
            }*/

            //ROS_DEBUG_NAMED("pp_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());
            bool planner_ok;
            geometry_msgs::PoseStamped current_pose;
            poseStampedTFToMsg(current_pose_, current_pose);
            planner_ok = ppComputeVelocityCommands(current_pose, cmd_vel);
            if(planner_ok)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        void PPLocalPlannerROS::setLoadedState(bool isloaded)
        {
           pp_->passLoadedState(isloaded); 
        }

    };
