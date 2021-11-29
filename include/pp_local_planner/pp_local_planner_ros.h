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

            

            bool get_closest_pt_idx_in_global_plan_(int &mn_index);
            bool get_lookahead_pt_idx_in_global_plan_(const double &la_dis_, const int &closest_pt_idx_, int &la_pt_idx);
            void process_global_path_points_(std::vector<tracker_functions::PathPoint> &path_points_);
            bool get_path_curvature_at_index(const int &idx, double &r_);

            costmap_2d::Costmap2DROS* costmap_ros_;
            costmap_2d::Costmap2D* costmap_;
            tf::TransformListener* tf_;
            bool initialized_;         
            
           
            base_local_planner::LocalPlannerUtil planner_util_;
            base_local_planner::LatchedStopRotateController latchedStopRotateController_;
            
            geometry_msgs::PoseStamped global_pose_stamped_; 
            tf::Stamped<tf::Pose> global_pose_tf_;
            std::pair<double, double> global_pose_;

            std::vector<geometry_msgs::PoseStamped> global_plan_;

            ros::Publisher global_plan_pub_, point_pub_, unfilled_circle_pub_;
            ros::Publisher closest_pt_pub_,lookahead_pose_pub_;
            
            ros::NodeHandle nh_;

    };

};
#endif
