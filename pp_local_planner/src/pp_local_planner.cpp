/*********************************************************************
 *
 * Software License Agreement 
 *
 *  Copyright (c) Botsync.
 *  All rights reserved.
 *
 * Author: botsync.co 
 *
 *********************************************************************/

#include <pp_local_planner/pp_local_planner.h>
#include "pp_local_planner/motion_planner.h"
#include <base_local_planner/goal_functions.h>
#include <cmath>

//for computing path distance
#include <queue>

#include <angles/angles.h>

#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace pp_local_planner {
    void PPLocalPlanner::reconfigure(PPLocalPlannerConfig &config)
    {
        boost::mutex::scoped_lock l(configuration_mutex_);
        pp_config.min_lookahead = config.min_lookahead;
        pp_config.max_lookahead = config.max_lookahead;
        pp_config.kla = config.kla;
        pp_config.kct = config.kct;
        pp_config.limits_ = planner_util_->getCurrentLimits();
    }

    PPLocalPlanner::PPLocalPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util) :
        planner_util_(planner_util)
    {
        ros::NodeHandle private_nh("~/" + name);


        //Assuming this planner is being run within the navigation stack, we can
        //just do an upward search for the frequency at which its being run. This
        //also allows the frequency to be overwritten locally.
        std::string controller_frequency_param_name;


        private_nh.param("global_frame_id", frame_id_, std::string("odom"));

        private_nh.param("publish_traj_pc", publish_traj, false);

        pp_debug = new PurepursuitDebug;

        mplnr = new motion_planner::MotionPlanner(planner_util_, 1.0);


    }

    PPLocalPlanner::~PPLocalPlanner()
    {
        delete pp_debug;
        delete mplnr;
    }


    bool PPLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
        return planner_util_->setPlan(orig_global_plan);
    }


    /*double PPLocalPlanner::checkFootprintCost(double x, double y, double theta, std::vector<geometry_msgs::Point> footprint_spec)
    {
        //current costmap information
        costmap_2d::Costmap2D* costmap = planner_util_->getCostmap();
        //world map that we are using. Configured with costmap.
        base_local_planner::WorldModel* world_model = new base_local_planner::CostmapModel(*costmap);
        double footprint_cost = world_model->footprintCost(x, y, theta, footprint_spec);  	
        //deleting the world model pointer.	
        if(world_model != NULL)
        { 
            delete world_model;
        }
        return footprint_cost;	
    }*/

    bool PPLocalPlanner::ppUpdate(const tf2_ros::Buffer* tf, std::vector<geometry_msgs::PoseStamped>& transformed_plan, const geometry_msgs::PoseStamped&
            global_pose, std::vector<geometry_msgs::Point> footprint_spec, const geometry_msgs::Twist& robot_vel, geometry_msgs::Twist& cmd_vel)
    {
        geometry_msgs::PoseStamped lookahead_pose;
        lookahead_pose.header.frame_id = transformed_plan.at(0).header.frame_id;
        //lookahead_pose.header.frame_id = "base_footprint";
        if(!getLookaheadPoint(tf, transformed_plan, global_pose, robot_vel, lookahead_pose))
        {
            ROS_WARN("No lookahead point returning");
            return false;
        }
        geometry_msgs::Twist base_command;
        if(!computeLinearVelocity(transformed_plan, footprint_spec, robot_vel, global_pose, base_command))
        {
            ROS_WARN("not able to make a valid linear velocity plan");
            return false;
        }

        double angular_vel;
        if(!computeAngularVelocity(tf, lookahead_pose, global_pose, robot_vel, base_command.linear.x, angular_vel))
        {
            ROS_WARN("not able to make a valid angular velocity plan");
            return false;
        }

        cmd_vel.linear.x = base_command.linear.x;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = angular_vel;
        pp_debug->publishDebug();


        return true;
    }


    bool PPLocalPlanner::computeAngularVelocity(const tf2_ros::Buffer* tf, const geometry_msgs::PoseStamped& lookahead_pose, const
            geometry_msgs::PoseStamped& global_pose, const geometry_msgs::Twist& robot_vel, double linear_vel, double& angular_vel)
    {
        //run with try catch.
        geometry_msgs::PoseStamped lookahead_pose_global;
        //ROS_WARN("robot pose x: %f, y: %f", global_pose.pose.position.x, global_pose.pose.position.y);
        lookahead_pose_global.header.frame_id = lookahead_pose.header.frame_id;
        lookahead_pose_global.header.stamp = ros::Time(0);
        lookahead_pose_global.pose = lookahead_pose.pose;
        geometry_msgs::PoseStamped lookahead_pose_robot;
        //tf->transform(lookahead_pose_global, lookahead_pose_robot, "mag250_tf/base_link"); //frame should be updated
        //tf->transform(lookahead_pose_global, lookahead_pose_robot, "base_link"); //frame should be updated
        ROS_WARN("lookahead pose global x: %f, y: %f", lookahead_pose_global.pose.position.x, lookahead_pose_global.pose.position.y);
        ROS_WARN("lookahead pose robot x: %f, y: %f", lookahead_pose_robot.pose.position.x, lookahead_pose_robot.pose.position.y);
        //purepursuit calculations
        double alpha = atan2(lookahead_pose_global.pose.position.y - global_pose.pose.position.y, lookahead_pose_global.pose.position.x - global_pose.pose.position.x) - tf2::getYaw(global_pose.pose.orientation);

        //double alpha = atan2(lookahead_pose_robot.pose.position.y, lookahead_pose_robot.pose.position.x);

        double dynamic_lookahead = calculateDynamicLookahead(robot_vel);
        double lateral_shift = dynamic_lookahead * sin(alpha); 
        ROS_WARN("lateral shift : %f", lateral_shift);
        double curvature = 2 * lateral_shift / pow(dynamic_lookahead, 2); 
        angular_vel = linear_vel * curvature;

        //bounding angular velocity values below the limits.
        //pp_config.max_angular
        pp_debug->updateDebug(dynamic_lookahead, lateral_shift, curvature);
        return true;

    }

    bool PPLocalPlanner::computeLinearVelocity(std::vector<geometry_msgs::PoseStamped>& transformed_plan, std::vector<geometry_msgs::Point> footprint_spec, const geometry_msgs::Twist& robot_vel, const
            geometry_msgs::PoseStamped& global_pose, geometry_msgs::Twist& base_command)
    {

        //if(makeVelocityPlan(transformed_plan, footprint_spec, robot_vel))
        {
            //linear_vel = 0.3;
            mpd::MotionPlan mpl;
            if(!mplnr->constructMotionPlan(transformed_plan, global_pose, robot_vel, footprint_spec, mpl))
            {
                return false;
            }
            mplnr->getInstantaneousCommand(mpl, global_pose, robot_vel, base_command);
            ROS_WARN("cmd_vel %f", base_command.linear.x);
            return true; 
        }
    }


    bool PPLocalPlanner::getLookaheadPoint(const tf2_ros::Buffer* tf, std::vector<geometry_msgs::PoseStamped>& transformed_plan, const
            geometry_msgs::PoseStamped& global_pose, const geometry_msgs::Twist& robot_vel, geometry_msgs::PoseStamped& lookahead_pose)
    {
        //double path_length = 0.0;
        //geometry_msgs::PoseStamped pose_ = transformed_plan.at(0);
        std::vector<geometry_msgs::PoseStamped>::const_iterator plan_it;

        geometry_msgs::PoseStamped prev_ldp;
        //iterating over the global path for finding the approximate point at the lookahead distance from the robot.
        for(plan_it = transformed_plan.begin(); plan_it != transformed_plan.end() -1; plan_it++)
        {
            //ROS_WARN("robot pose x: %f, y: %f", global_pose.pose.position.x, global_pose.pose.position.y);
            //start.header.frame_id = (*plan_it).header.frame_id;
            //start.header.stamp = ros::Time::now();
            //(*plan_it).header.stamp = ros::Time(0);
            //end.header.frame_id = (*plan_it).header.frame_id;
            //end.header.stamp = ros::Time::now();
            //(*(++plan_it)).header.stamp = ros::Time(0); 
            //lookahead_pose_global.pose = lookahead_pose.pose;
            //geometry_msgs::TransformStamped transform = tf->lookupTransform("mag250_tf/base_link", ros::Time(),
            //(*plan_it).header.frame_id, (*plan_it).header.stamp, (*plan_it).header.frame_id, ros::Duration(0.5));

            //tf->transform(*plan_it, start, "mag250_tf/base_link"); //frame should be updated
            //tf2::doTransform(*plan_it, start, transform);
            //tf2::doTransform(*(++plan_it), end, transform); //frame should be updated
            motion_planner::MotionPlanner mp;
            //int points_counted = plan_it - transformed_plan.begin() + 1;
            //double pose_diff = mpd::euclidean(*plan_it, *(plan_it++));
            //ROS_WARN("pose diff : %f", pose_diff);
            //path_length = path_length + pose_diff;
            //ROS_WARN("path_length : %f", path_length);
            //ROS_WARN("points counted : %d", points_counted);
            double dynamic_lookahead = calculateDynamicLookahead(robot_vel);
            //ROS_WARN("dynamic_lookahead : %f", dynamic_lookahead);
            //checking the arc length is greater than lookahead distance then taking that point as 
            //lookahead point in path.
            //ROS_WARN("plan x : %f y : %f", (*plan_it).pose.position.x, (*plan_it).pose.position.y);
            if(mp.getPlaneDistance(global_pose, *plan_it) > dynamic_lookahead)
            {
                geometry_msgs::PoseStamped start = *(std::prev(plan_it, 1));
                geometry_msgs::PoseStamped end = *(plan_it);
                double scale = dynamic_lookahead - mp.getPlaneDistance(global_pose, start); 
                /*double scale = dynamic_lookahead; 
                geometry_msgs::PoseStamped start;
                geometry_msgs::PoseStamped end;
                geometry_msgs::TransformStamped transform = tf->lookupTransform("base_footprint", ros::Time(),
                        (*plan_it).header.frame_id, (*plan_it).header.stamp, (*plan_it).header.frame_id, ros::Duration(0.5));
                tf2::doTransform(*(std::prev(plan_it)), start, transform);
                tf2::doTransform(*plan_it, end, transform); //frame should be updated */

                if (mp.linInterpolatedPose(start, end, global_pose, scale, lookahead_pose))
                {
                    ROS_WARN("lookahead pose x: %f, y: %f", lookahead_pose.pose.position.x, lookahead_pose.pose.position.y);
                    ROS_WARN("dis : %f", mpd::euclidean(lookahead_pose, prev_ldp));
                    pp_debug->updateDebug(lookahead_pose);
                    prev_ldp = lookahead_pose;
                    return true;
                }
            }
            //plan over but could not find the look ahead point. Assuming robot is near the goal
            //extend the global plan in the direction of final points.

        }
            return false;
    }

        double PPLocalPlanner::calculateDynamicLookahead(const geometry_msgs::Twist& robot_vel)
        {
            double dynamic_lookahead;
            double schange = pp_config.min_lookahead / pp_config.kla; 
            if(robot_vel.linear.x < schange){
                dynamic_lookahead = pp_config.min_lookahead;	
            }
            else
            {
                dynamic_lookahead = pp_config.kla * robot_vel.linear.x;
            }
            if(dynamic_lookahead > pp_config.max_lookahead)
            {
                dynamic_lookahead = pp_config.max_lookahead;
            }
            return dynamic_lookahead;
        }
    };
