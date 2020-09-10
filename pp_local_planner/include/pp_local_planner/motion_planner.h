#ifndef MOTION_PLANNER_H 
#define MOTION_PLANNER_H 

#include "motion_planner_data.h"

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
            MotionPlanner(base_local_planner::LocalPlannerUtil* planer_util, double safe_factor);
            
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
            /*
             *@brief method to generate control commands for the robot base. Tracking angular velocity is not supported
             *now to be added.
             *@param local motion plan
             *@param global robot position
             *@param reference to cmd_vel to be updated.
             */
            bool getInstantaneousCommand(mpd::MotionPlan& mp, const geometry_msgs::PoseStamped& global_pose, const
            geometry_msgs::Twist& robot_vel, geometry_msgs::Twist& cmd_vel);

            bool getLinearEquation(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& end,
            double& a, double& b, double& c);
            
            /*
             *@brief method to get distance between two poses.
             *@param geometry pose 1
             *@param geometry pose 2.
             *@return distance between two poses.
             */
            double getPlaneDistance(const geometry_msgs::PoseStamped& pose_a, const geometry_msgs::PoseStamped& pose_b);

            double getDisFromPointToLine(const geometry_msgs::PoseStamped& pose, double a, double b, double c);
            
            bool getUnitVector(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& end,
            tf2::Vector3& unit_vector);
            
            bool linInterpolatedPose(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& end, const geometry_msgs::PoseStamped& global_pose, const double dynamic_lookahead, geometry_msgs::PoseStamped& interpolated_pose);

        private:

            /*
             *@brief method to get the robot cross track error.
             *@param plan that robot is following.
             *@param robot position in the plan frame.
             *@return crosstrack error value.
             */
            mpd::CrossTrackInfo crossTrackError(const mpd::Plan& plan, const geometry_msgs::PoseStamped& global_pose);

            void getMinDistancePoseIt(const mpd::MotionPlan& search_plan, const geometry_msgs::PoseStamped& origin,
            mpd::MotionPlan::const_iterator& it);

            void rampMotionPlan(mpd::MotionPlan& mpl, const geometry_msgs::PoseStamped& global_pose, const geometry_msgs::Twist& robot_vel, mpd::MotionPlan& ramp_plan);

            void getMinForwardVelPose(mpd::MotionPlan& mpl, const geometry_msgs::PoseStamped& global_pose,
            mpd::MotionPlan::iterator it);

            /*
             *@brief method to get the forward path curvature information.
             *@param plan that robot is following.
             *@return magnitude path curvature.
             */
            double pathCurvature(const mpd::MengerPoints& path_points);
            
            /*
             *@brief method to check and update plan for in place turn.
             *@param reference motion pose to update the in place turn plan.
             *@return return true if in place turn requires.
             */
            bool inPlace(const geometry_msgs::PoseStamped& initial_pose, const geometry_msgs::PoseStamped&
            final_pose, const double xy_goal_tolerance, const double yaw_goal_tolerance, double& yaw_dif);

            bool isGoal(const geometry_msgs::PoseStamped& goal_pose, const geometry_msgs::PoseStamped& global_pose,
            const double xy_goal_tolerance, const double yaw_goal_tolerance);
           
            /*
             *@brief method to trim motion plan portion based on arc length.
             *@param motion plan.
             *@param maximum allowed arc length of the plan.
             */
            void trimMotionPlan(mpd::MotionPlan& motion_plan, double safe_arc_length);
            
            void updatePlan(mpd::Plan& global_plan, const geometry_msgs::PoseStamped& global_pose);
            
            /*
             *@brief method to update information about the obstacle present in the robot path. 
             *@param critical obstacle information.
             *@return true if there is critical obstacle.
             */
            bool updateObstacleInfo(const geometry_msgs::PoseStamped& plan_pose, std::vector<geometry_msgs::Point>
            footprint_spec, mpd::ObstacleInfo& obstacle_info);

            base_local_planner::LocalPlannerUtil* planner_util_;
            base_local_planner::WorldModel* world_model;
            costmap_2d::Costmap2D* costmap;
            double safe_factor_;

    };
};

#endif
