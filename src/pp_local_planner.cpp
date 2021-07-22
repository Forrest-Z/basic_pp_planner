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
        pp_config.load_vmax = config.load_vmax; 
        pp_config.noload_vmax = config.noload_vmax; 
        pp_config.load_wmax = config.load_wmax; 
        pp_config.noload_wmax = config.noload_wmax; 
        pp_config.load_acc_x = config.load_acc_x; 
        pp_config.noload_acc_x = config.noload_acc_x; 
        pp_config.load_acc_w = config.load_acc_w; 
        pp_config.noload_acc_w = config.noload_acc_w;
        pp_config.vmin = config.vmin;
        pp_config.wmin = config.wmin;
        pp_config.min_lookahead = config.min_lookahead;
        pp_config.max_lookahead = config.max_lookahead;
        pp_config.kla = config.kla;
        pp_config.lat_acc = config.lat_acc;
        pp_config.obst_stop_dist = config.safety_distance;
        pp_config.cross_track_warn = config.cross_track_warn;
        pp_config.cross_track_error = config.cross_track_error;
        pp_config.xy_goal_tolerance = config.xy_goal_tolerance;
        pp_config.yaw_goal_tolerance = config.yaw_goal_tolerance;
        pp_config.update_config = config.change_config;
        pp_config.robot_type = config.robot_type;
        //pp_config.limits_ = planner_util_->getCurrentLimits();
        if(pp_config.update_config)
        {
            mplnr->updateConfig(pp_config);
        }
    }

    PPLocalPlanner::PPLocalPlanner(std::string name, tf::TransformListener* tf, base_local_planner::LocalPlannerUtil
            *planner_util, std::string motion_frame): tf_{tf}, planner_util_{planner_util}, motion_frame_{motion_frame}
    {
        ros::NodeHandle private_nh("~/" + name);


        //Assuming this planner is being run within the navigation stack, we can
        //just do an upward search for the frequency at which its being run. This
        //also allows the frequency to be overwritten locally.
        std::string controller_frequency_param_name;


        private_nh.param("global_frame_id", frame_id_, std::string("odom"));
        private_nh.param("publish_traj_pc", publish_traj, false);

        ros::NodeHandle node; 
        visualise_pub = node.advertise<visualization_msgs::Marker>("cmd_vel_viz", 1); 

        // Create pointers to each of the other classes that will be use in this script 
        pp_debug = new PurepursuitDebug;
        mplnr = new motion_planner::MotionPlanner(tf_, planner_util_, 2.0, motion_frame_);
        mtf = new motion_target_follower::MotionTargetFollower(0.0, 1.0, 0.4, 2.0);

    }

    PPLocalPlanner::~PPLocalPlanner()
    {
        delete pp_debug;
        delete mplnr;
    }

    void PPLocalPlanner::passLoadedState(bool isloaded)
    {
        mplnr->updateLoadedState(isloaded);
        loaded = isloaded;
    }

    bool PPLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
        if(planner_util_->setPlan(orig_global_plan))
        {
            return mplnr->setGlobalPlan(orig_global_plan); 
        }
        return false;
    }

    bool PPLocalPlanner::ppUpdate(const tf::TransformListener* tf, const geometry_msgs::PoseStamped& global_pose,
            std::vector<geometry_msgs::Point> footprint_spec, const geometry_msgs::Twist& robot_vel,
            std::vector<geometry_msgs::PoseStamped>& transformed_plan, geometry_msgs::Twist& cmd_vel)
    {
        geometry_msgs::Twist base_command;
        mpd::MotionPlan mpl;
        if(!computeLinearVelocity(transformed_plan, footprint_spec, robot_vel, global_pose, mpl, base_command))
        {
            ROS_WARN("FAILED TO GENERATE A VALID VELOCITY PLAN");
            return false;
        }

        if(mpl.at(0).in_place || mpl.at(0).obstacle || mpl.at(0).error || mpl.at(0).pause)
        {
            cmd_vel.linear.x = base_command.linear.x;
            //cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = base_command.angular.z;
            //cmd_vel.angular.z = 0.0;
        }

        else
        {
            geometry_msgs::PoseStamped lookahead_pose;
            lookahead_pose.header.frame_id = transformed_plan.at(0).header.frame_id;
            if(!getLookaheadPoint(tf, transformed_plan, global_pose, robot_vel, lookahead_pose))
            {
                ROS_WARN("FAILED TO FIND LOOKAHEAD POINT");
                return false;
            }
            double angular_vel;
            if(!computeAngularVelocity(tf, lookahead_pose, global_pose, robot_vel, base_command.linear.x, angular_vel))
            {
                ROS_WARN("FAILED TO GENERATE A VALID ANGULAR VELOCITY PLAN");
                return false;
            }

            cmd_vel.linear.x = base_command.linear.x;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = angular_vel;

        }
        if(mplnr->trajectoryCollision(robot_vel.linear.x, cmd_vel.angular.z, 5.0, footprint_spec, global_pose))
        {
            ROS_WARN("POSSIBLE TRAJECTORY COLLISION");
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
            std_msgs::UInt8 led_msg;
            led_msg.data = 4; //red colour
        }
        //bound control inputs
        mplnr->boundControlInput(cmd_vel.linear.x, cmd_vel.angular.z);
        pp_debug->publishDebug();
        //pp_debug->publishLED(led_msg);
        publishVisualization(cmd_vel); 
        return true;
    }

    void PPLocalPlanner::publishVisualization(const geometry_msgs::Twist& cmd_vel)    
    {
        std::string linear (std::to_string(cmd_vel.linear.x));
        std::string angular (std::to_string(cmd_vel.angular.z)); 

        visualization_msgs::Marker obstacle_marker; 

        obstacle_marker.header.frame_id = "mag250_tf/base_link"; 
        obstacle_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING; 
        // obstacle_marker.type = obstacle_marker.ADD;

        obstacle_marker.pose.position.x = -1.0;
        obstacle_marker.pose.position.y = 0.0; 

        tf::Quaternion q_t;

        q_t.setRPY(0, 0, 0);
        q_t.normalize();

        //populate the orientation values
        obstacle_marker.pose.orientation.x = q_t.getX();
        obstacle_marker.pose.orientation.y = q_t.getY();
        obstacle_marker.pose.orientation.z = q_t.getZ();
        obstacle_marker.pose.orientation.w = q_t.getW();

        obstacle_marker.scale.x = 0.5;
        obstacle_marker.scale.y = 0.5;
        obstacle_marker.scale.z = 0.5;

        obstacle_marker.color.r = 255;
        obstacle_marker.color.g = 255;
        obstacle_marker.color.b = 255;
        obstacle_marker.color.a = 1;

        obstacle_marker.text = "Linear vel: " + linear + ", " + "Angular vel: " + angular;

        obstacle_marker.lifetime = ros::Duration(15); 						
        visualise_pub.publish(obstacle_marker);
    }


    bool PPLocalPlanner::computeAngularVelocity(const tf::TransformListener* tf, const geometry_msgs::PoseStamped& lookahead_pose, const
            geometry_msgs::PoseStamped& global_pose, const geometry_msgs::Twist& robot_vel, double& linear_vel, double& angular_vel)
    {
        geometry_msgs::PoseStamped lookahead_pose_global;
        lookahead_pose_global.header.frame_id = lookahead_pose.header.frame_id;
        lookahead_pose_global.header.stamp = ros::Time::now();
        lookahead_pose_global.pose = lookahead_pose.pose;

        //purepursuit calculations
        /*double alpha = atan2(lookahead_pose_global.pose.position.y - global_pose.pose.position.y, lookahead_pose_global.pose.position.x - global_pose.pose.position.x) - tf2::getYaw(global_pose.pose.orientation);

        //double alpha = atan2(lookahead_pose_robot.pose.position.y, lookahead_pose_robot.pose.position.x);

        double dynamic_lookahead = calculateDynamicLookahead(robot_vel);
        double lateral_shift = dynamic_lookahead * sin(alpha); 
        double curvature = 2 * lateral_shift / pow(dynamic_lookahead, 2); 
        //angular_vel = robot_vel.linear.x * curvature;
        angular_vel = linear_vel * curvature;

        //bounding angular velocity values below the limits.
        //pp_config.max_angular
        pp_debug->updateDebug(dynamic_lookahead, lateral_shift, curvature);*/

        //auto limits = planner_util_->getCurrentLimits();
        //mtf->updateControlLimits(limits.max_vel_x, limits.min_vel_x, limits.max_rot_vel, limits.min_rot_vel);
        double vmax = (loaded == true) ? pp_config.load_vmax : pp_config.noload_vmax;
        double wmax = (loaded == true) ? pp_config.load_wmax : pp_config.noload_wmax;
        mtf->updateControlLimits(vmax, pp_config.vmin, wmax, pp_config.wmin);

        geometry_msgs::PoseStamped lookahead_pose_base;
        //geometry_msgs::PoseStamped local_target_pose;
        //mplnr->getReferencePose(local_target_pose);
        mplnr->transformPose(pp_config.robot_type+ "_tf/base_link", lookahead_pose_global, lookahead_pose_base);
        double linear_velocity = robot_vel.linear.x;
        mtf->getControlCommands(lookahead_pose_base, linear_vel, angular_vel);
        return true;
    }

    bool PPLocalPlanner::computeLinearVelocity(std::vector<geometry_msgs::PoseStamped>& transformed_plan, std::vector<geometry_msgs::Point> footprint_spec, const geometry_msgs::Twist& robot_vel, const
            geometry_msgs::PoseStamped& global_pose, mpd::MotionPlan& mpl, geometry_msgs::Twist& base_command)
    {
        if(!mplnr->constructMotionPlan(transformed_plan, global_pose, robot_vel, footprint_spec, mpl))
        {
            return false;
        }
        mplnr->getInstantaneousCommand(mpl, global_pose, robot_vel, footprint_spec, base_command);
        return true; 
    }

    bool PPLocalPlanner::getLookaheadPoint(const tf::TransformListener* tf, std::vector<geometry_msgs::PoseStamped>& transformed_plan, const geometry_msgs::PoseStamped& global_pose, const geometry_msgs::Twist& robot_vel, geometry_msgs::PoseStamped& lookahead_pose)
    {
        if(transformed_plan.size() < 1)
        {
            ROS_ERROR("NO PLAN TO TRAVEL : ERROR");
            return false;
        }

        if(transformed_plan.size() >= 1 && transformed_plan.size() < 4)
        {
            lookahead_pose = transformed_plan.at(transformed_plan.size() - 1);
            return true;
        }

        std::vector<geometry_msgs::PoseStamped>::const_iterator plan_it;
        geometry_msgs::PoseStamped prev_ldp;
        motion_planner::MotionPlanner mp;
        double dynamic_lookahead = calculateDynamicLookahead(robot_vel);
        //iterating over the global path for finding the approximate point at the lookahead distance from the robot.
        //extended lookahead point.
        //fixed bad allocation error when robot is deviating too much away from path.
        //in this case first point is selected point then applying std::prev() was giving error
        //now plan is started from the second point.
        for(plan_it = transformed_plan.begin() + 1; plan_it != transformed_plan.end() -1; plan_it++)
        {
            if(mp.getPlaneDistance(global_pose, *plan_it) >= dynamic_lookahead)
            {
                geometry_msgs::PoseStamped start = *(std::prev(plan_it, 1));
                geometry_msgs::PoseStamped end = *(plan_it);
                double scale = dynamic_lookahead - mp.getPlaneDistance(global_pose, start); 
                if (mp.linInterpolatedPose(start, end, global_pose, scale, lookahead_pose))
                {
                    pp_debug->updateDebug(lookahead_pose);
                    prev_ldp = lookahead_pose;
                    return true;
                }
                else
                {
                    ROS_ERROR("LOOKAHEAD CALCULATION FAILED");
                    return false;
                }
            }
        } 
        lookahead_pose = transformed_plan.at(transformed_plan.size() - 1);
        return true;
        /*mpd::PosePair plan_extend_pair;
          plan_extend_pair = mp.getPlanExtendPosePair(transformed_plan);
          geometry_msgs::PoseStamped end = plan_extend_pair.second;
          geometry_msgs::PoseStamped start = plan_extend_pair.first;
          pp_debug->updateLineExtendDebug(start, end);
          double scale = dynamic_lookahead - mp.getPlaneDistance(global_pose, end); 
          if (mp.linInterpolatedPose(start, end, global_pose, scale, lookahead_pose))
          {
          pp_debug->updateDebug(lookahead_pose);
          prev_ldp = lookahead_pose;
          return true;
          }
          else
          {
          ROS_ERROR("LOOKAHEAD CALCULATION FAILED");
          return false;
          }*/
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
        ROS_INFO("Dynamic lookeahd is %f, %f", dynamic_lookahead, robot_vel.linear.x); 
        return dynamic_lookahead;
    }

    bool PPLocalPlanner::isGoalReached(const geometry_msgs::PoseStamped& robot_pose)
    {
        return mplnr->isGoalReached(robot_pose);
    }

};
