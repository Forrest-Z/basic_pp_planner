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

#include <pp_local_planner/vis_functions.h>
#include <pp_local_planner/pp_tracker_functions.h>


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


            void advertise_publishers();
            
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
            ros::Publisher closest_pt_pub_,lookahead_pose_pub_, ct_error_pub_, la_pt_line_pub;

            ros::NodeHandle nh_;

            pp_ds::Limits pp_limits_;

    };

};
#endif
