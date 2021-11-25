#ifndef PP_PLANNER_ROS_H
#define PP_PLANNER_ROS_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>


#include <angles/angles.h>

#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/latched_stop_rotate_controller.h>
#include <base_local_planner/odometry_helper_ros.h>

#include <tf/transform_listener.h>




namespace pp_local_planner {

    class PPLocalPlannerROS : public nav_core::BaseLocalPlanner {
        
        


        public:

            PPLocalPlannerROS();

            void initialize(std::string name, tf::TransformListener* tf,
                    costmap_2d::Costmap2DROS* costmap_ros);
            ~PPLocalPlannerROS();

            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);


            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
            bool isGoalReached();

            void setLoadedState(bool isloaded);

        private:

            
            bool update_next_look_ahead_point(geometry_msgs::PoseStamped &next_look_ahead_pose_ );
            void get_angular_velocity_command(const geometry_msgs::PoseStamped &next_look_ahead_pose, double &w_);
            bool get_global_goal_(geometry_msgs::PoseStamped &goal_pt);


        //inline functions  

            inline double euclidean(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2){
                return (sqrt(pow(pose1.pose.position.x - pose2.pose.position.x, 2) + pow(pose1.pose.position.y -
                        pose2.pose.position.y, 2)));
            }
            
            costmap_2d::Costmap2DROS* costmap_ros_;
            costmap_2d::Costmap2D* costmap_;
            tf::TransformListener* tf_;
            bool initialized_;         
            
            //Params for launch file
            std::string odom_topic_ = "/bi/mag_base_controller/odom";
            
            //Tuning params
            double look_ahead_dis_ = 1;

            //std::vector<geometry_msgs::PoseStamped>& global_plan_;

            base_local_planner::LocalPlannerUtil planner_util_;
            base_local_planner::LatchedStopRotateController latchedStopRotateController_;
            ros::Publisher global_plan_pub_, look_ahead_point_pub, global_pose_pub_, global_goal_pub_ ;
            std::vector<geometry_msgs::PoseStamped> global_plan_, orig_global_plan_;

            geometry_msgs::PoseStamped global_pose_;
            tf::Stamped<tf::Pose> global_pose_tf_;

           

        
    };

};
#endif
