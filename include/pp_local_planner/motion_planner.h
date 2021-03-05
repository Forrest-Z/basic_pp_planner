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

#ifndef MOTION_PLANNER_H 
#define MOTION_PLANNER_H 

#include "geometry_msgs/PoseStamped.h"
#include "motion_planner_data.h"
#include "pp_local_planner/motion_planner_config.h"
#include <std_msgs/Bool.h>
#include "std_srvs/SetBoolRequest.h"
#include "std_srvs/SetBoolResponse.h"

#include <boost/thread/pthread/mutex.hpp>
#include <boost/thread/pthread/recursive_mutex.hpp>
#include<cmath>

//STL includes
#include <tuple>
#include <algorithm>

//angles
#include <angles/angles.h>

//nav stack includes
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/costmap_model.h>

#include <std_srvs/SetBool.h>

//tf include
#include <tf2/utils.h>

namespace motion_planner 
{
    /*
     *@class VelocityGenetrator
     *@brief class genereate motion plan for robot base.
     */
    class MotionPlanner 
    {

        public:
            /*
             *@brief motion planner constructor
             *@param pointer to local planner utility to get local planner parameters.
             *@param extra distance added with the minimum stopping distance of the robot for safe stopping.
             */
            MotionPlanner(tf::TransformListener* tf, base_local_planner::LocalPlannerUtil* planer_util, double safe_factor, std::string motion_frame);

            /*
             *@brief constructor
             */
            MotionPlanner();
            virtual ~MotionPlanner();

            /*
             *@brief method to generate a motion plan that robot should apply.
             *@param global plan in world frame.
             *@param position of robot in world frame.
             *@param robot velocity
             *@param reference to update the motion plan.
             *@return true if generated a safe motion plan for the robot.
             */
            virtual bool constructMotionPlan(mpd::Plan& plan, const geometry_msgs::PoseStamped&
                    global_pose, const geometry_msgs::Twist& robot_vel, std::vector<geometry_msgs::Point>
                    footprint_spec, mpd::MotionPlan& motion_plan);
            
            void boundControlInput(double& v, double& w);
            
            double getDisFromPointToLine(const geometry_msgs::PoseStamped& pose, double a, double b, double c);
            
            void getGlobalPlan(mpd::Plan& global_plan);

            geometry_msgs::PoseStamped getInplacePose(const mpd::Plan& plan, mpd::Plan::const_iterator it, int&
            pose_count);
            
            /*
             *@brief method to generate control commands for the robot base. Tracking angular velocity is not supported
             *now to be added.
             *@param local motion plan
             *@param global robot position
             *@param reference to cmd_vel to be updated.
             */
            bool getInstantaneousCommand(mpd::MotionPlan& mp, const geometry_msgs::PoseStamped& global_pose, const
            geometry_msgs::Twist& robot_vel, const std::vector<geometry_msgs::Point>& footprint_spec, geometry_msgs::Twist& cmd_vel);

            bool getLinearEquation(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& end,
                    double& a, double& b, double& c);
            
            bool getLocalPlan(const geometry_msgs::PoseStamped& global_pose, mpd::Plan& local_plan);

            /*
             *@brief method to get distance between two poses.
             *@param geometry pose 1
             *@param geometry pose 2.
             *@return distance between two poses.
             */
            double getPlaneDistance(const geometry_msgs::PoseStamped& pose_a, const geometry_msgs::PoseStamped& pose_b);

            mpd::PosePair getPlanExtendPosePair(const mpd::Plan& plan);

            bool getUnitVector(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& end,
                    tf2::Vector3& unit_vector);

            bool isGoalReached(const geometry_msgs::PoseStamped& robot_pose);

            bool linInterpolatedPose(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& end, const geometry_msgs::PoseStamped& global_pose, const double dynamic_lookahead, geometry_msgs::PoseStamped& interpolated_pose);

            bool setGlobalPlan(const mpd::Plan& orig_global_plan);
            
            /*bool operator == (const geometry_msgs::PoseStamped& pose_a, const geometry_msgs::PoseStamped& pose_b)
             {   
                return (pose_a.pose.position.x == pose_b.pose.position.x && pose_a.pose.position.y ==
                  pose_b.pose.position.y && pose_a.pose.position.z == pose_b.pose.position.z && pose_a.pose.orientation.x ==
                  pose_b.pose.orientation.y == pose_b.pose.orientation.y && pose_a.pose.orientation.w ==
                  pose_b.pose.orientation.w && pose_a.pose.orientation.z == pose_b.pose.orientation.z);
             }*/


            /*
             *@brief method to get the robot cross track error.
             *@param plan that robot is following.
             *@param robot position in the plan frame.
             *@return crosstrack error value.
             */
            mpd::CrossTrackInfo crossTrackError(const mpd::Plan& plan, const geometry_msgs::PoseStamped& global_pose);

            void getMinDistancePoseIt(const mpd::MotionPlan& search_plan, const geometry_msgs::PoseStamped& origin,
                    mpd::MotionPlan::const_iterator& it);
            
            void getMinDistancePoseIt(const mpd::Plan& search_plan, const geometry_msgs::PoseStamped& origin,
                    mpd::Plan::const_iterator& it);

            void getReferencePose(geometry_msgs::PoseStamped& ref_pose);

            mpd::Plan::const_iterator getPlanPoseIt(const mpd::Plan& plan, const geometry_msgs::PoseStamped& search_pose);


            /*
             *@brief method to get the forward path curvature information.
             *@param plan that robot is following.
             *@return magnitude path curvature.
             */
            double pathCurvature(const mpd::MengerPoints& path_points);

	    void initializeConfig();
            
            bool trajectoryCollision(double linear_vel, double angular_vel, double frequency,
            const std::vector<geometry_msgs::Point>& footprint_spec, const geometry_msgs::PoseStamped& robot_pose);
            
            bool transformPose(const std::string& global_frame, const
                    geometry_msgs::PoseStamped &pose, geometry_msgs::PoseStamped& transformed_pose);
	    
	    void updateConfig(struct MotionPlannerConfig& latest_config);
        
        private:

            /*
             *@brief method to check and update plan for in place turn.
             *@param reference motion pose to update the in place turn plan.
             *@return return true if in place turn requires.
             */
            bool inPlace(const geometry_msgs::PoseStamped& pose, const geometry_msgs::PoseStamped& global_pose, const double xy_goal_tolerance, const double yaw_goal_tolerance, double& yaw_dif);

            bool isGoal(const geometry_msgs::PoseStamped& check_pose, const geometry_msgs::PoseStamped& end_pose,
                    const double xy_goal_tolerance);

            bool isStart(const geometry_msgs::PoseStamped& start_pose, const geometry_msgs::PoseStamped& global_pose,
                    const double xy_goal_tolerance);


            /*
             *@brief method to trim motion plan portion based on arc length.
             *@param motion plan.
             *@param maximum allowed arc length of the plan.
             */
            void trimMotionPlan(mpd::MotionPlan& motion_plan, double safe_arc_length);

            void profileVelocity(const double& ref_vel, double& profiled_vel);


            void clearVisitedPlan(const mpd::Plan::const_iterator upto_it);
            
            void clearVisitedPlan(int size);

            double getGoalDistance(const geometry_msgs::PoseStamped& robot_pose);

            /*
             *@brief method to update information about the obstacle present in the robot path. 
             *@param critical obstacle information.
             *@return true if there is critical obstacle.
             */
            bool updateObstacleInfo(const geometry_msgs::PoseStamped& plan_pose, std::vector<geometry_msgs::Point>
                    footprint_spec, mpd::ObstacleInfo& obstacle_info);

            bool updateMBPlan();

            bool warningFieldCb(std_srvs::SetBoolRequest& field_status, std_srvs::SetBoolResponse& response);
            bool navPauseCb(std_srvs::SetBoolRequest& pause, std_srvs::SetBoolResponse& response);

            virtual void setLoadedState(bool isloaded) override;

            

            tf::TransformListener* tf_;
            base_local_planner::LocalPlannerUtil* planner_util_;
            base_local_planner::WorldModel* world_model;
            boost::recursive_mutex warning_field_mutex;
            boost::mutex config_mutex;
            boost::mutex nav_pause_mutex;
            boost::mutex loaded_state_mutex;
            costmap_2d::Costmap2D* costmap;
            mpd::Plan global_plan_;
            mpd::Plan mb_global_plan_;
	        MotionPlannerConfig config;
            geometry_msgs::PoseStamped start_pose_;
            geometry_msgs::PoseStamped end_pose_;
            geometry_msgs::PoseStamped ref_pose_;
            ros::Publisher ref_pose_pub;
            ros::Publisher closest_pose_pub;
            ros::Publisher obstacle_info_pub;
            ros::ServiceServer warning_field_server;
            ros::ServiceServer nav_pause_server;
            std::string motion_frame_;
            double safe_factor_;
            double xy_goal_tolerance_;
            double yaw_goal_tolerance_;
            double max_xy_tolerance;
            double max_yaw_goal_tolerance;
            double last_control_v;
            double last_control_w;
            bool accept_plan;
            bool debug;
            bool plan_executed;
            bool critical_error;
            bool warning_field_status;
            bool pause_motion;
            bool loaded;

    };
};

#endif
