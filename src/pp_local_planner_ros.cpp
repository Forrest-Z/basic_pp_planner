#include <pp_local_planner/pp_local_planner_ros.h>
#include <Eigen/Core>
#include <cmath>


#include <pp_local_planner/vis_functions.h>
#include <pp_local_planner/helper_functions.h>
#include <pp_local_planner/geometry_functions.h>
#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>
#include <visualization_msgs/Marker.h>

PLUGINLIB_EXPORT_CLASS(pp_local_planner::PPLocalPlannerROS, nav_core::BaseLocalPlanner)

namespace pp_local_planner
{

    PPLocalPlannerROS::PPLocalPlannerROS() : initialized_(false)
    {
    }

    void PPLocalPlannerROS::initialize(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros)
    {

        if (initialized_)
        {

            ROS_WARN("The planner has already been initialized!\n");
            return;
        }

        tf_ = tf;
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();

        //ros::NodeHandle private_nh_("~" + name);
        nh_ = ros::NodeHandle{name};

        planner_util_.initialize(tf_, costmap_, costmap_ros_->getGlobalFrameID());

        initialized_ = true;

        circle_pub_ = nh_.advertise<visualization_msgs::Marker>("circle_marker", 1000, true);
        global_plan_pub_ = nh_.advertise<nav_msgs::Path>("global_plan", 1000, true);
        point_pub_ = nh_.advertise<visualization_msgs::Marker>("point_marker", 1000, true);
        unfilled_circle_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("unfilled_circle_markerarray_", 1000, true);
        
    }

    
    
    
   

    bool PPLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {

        ROS_INFO("Inside the computeVelocityCommand function!\n");

        if (!costmap_ros_->getRobotPose(global_pose_tf_))
        {

            ROS_ERROR("Could not get robot pose!\n");
            return false;
        }

        if (!planner_util_.getLocalPlan(global_pose_tf_, global_plan_))
        {

            ROS_ERROR("planner_util_ could not find transformed_plan_!\n");
            return false;
        }

        base_local_planner::publishPlan(global_plan_, global_plan_pub_);

        double mn_ = 10000, mx_ = -1;
        int y_cnt_ =0, cc_cnt_ = 0 ;


        for (int i = 1; i < (int)global_plan_.size() - 1; i++)
        {


            ROS_INFO("planner_util_.getGlobalFrameID(): %s\n", planner_util_.getGlobalFrame());
            geometry_msgs::PoseStamped pose_a_, pose_b_, pose_c_;
            
            pose_a_ = global_plan_.at(i - 1);
            pose_b_ = global_plan_.at(i);
            pose_c_ = global_plan_.at(i + 1);

            bool flag_;

            std::pair<double, double> a_, b_, c_;

            helper_functions::convert_pose_stamped_to_pair_double(pose_a_, a_); 
            helper_functions::convert_pose_stamped_to_pair_double(pose_b_, b_); 
            helper_functions::convert_pose_stamped_to_pair_double(pose_c_, c_); 
            
            double r_;
            flag_ = geometry_functions::get_cr_(a_, b_ ,c_, r_);

            if(flag_){
                
                std::pair<double, double> cc_; 

                flag_ = geometry_functions::get_cc_(a_, b_, c_, cc_);
                
                
                /*if(cc1_flag_) {

                    ROS_WARN("i: %d\n", i);
                    ROS_INFO("a_: (%f,%f) b_: (%f,%f) c_: (%f,%f)\n", a_.first, a_.second, b_.first, b_.second, c_.first, c_.second);
                    ROS_INFO("y_: %f r_: %f cc_1_: (%f,%f)\n", y_, r_, cc_1_.first, cc_1_.second);

                }*/ 

                if(flag_ && r_ < 20) {

                    ROS_WARN("i: %d\n", i);
                    ROS_INFO("a_: (%f,%f) b_: (%f,%f) c_: (%f,%f)\n", a_.first, a_.second, b_.first, b_.second, c_.first, c_.second);
                    ROS_INFO("r_: %f cc_: (%f,%f)\n", r_, cc_.first, cc_.second);

                    std::pair<double, double> pt_; 
                    helper_functions::convert_pose_stamped_to_pair_double(global_plan_.at(i), pt_);
                    vis_functions::publish_point_(pt_, point_pub_, planner_util_, nh_);
                    //vis_functions::publish_circle_(cc_, r_, circle_pub_, planner_util_, nh_);
                    vis_functions::publish_unfilled_circle_(cc_, r_, unfilled_circle_pub_, planner_util_, nh_);


                    ROS_WARN("Sleeping for 1s\n");
                    ros::Duration(1.0).sleep();
                } 
                
                mn_ = std::min(mn_, r_);
                mx_ = std::max(mx_, r_);

                
            }
        }

        ROS_WARN("mx_: %f mn_: %f\n", mx_, mn_);
        ROS_WARN("y_cnt_: %d cc_cnt_: %d\n", y_cnt_, cc_cnt_);
        
        return true;
    }

    bool PPLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {

        if (!initialized_)
        {

            ROS_ERROR("Initialized the planner before calling the setPlan function!\n");
            return false;
        }

        latchedStopRotateController_.resetLatching();

        ROS_INFO("Got new plan! -- Sleeping for 2 seconds!\n");

        ros::Duration(2.0).sleep();

        if (planner_util_.setPlan(orig_global_plan))
        {

            //orig_global_plan_ = orig_global_plan;
            return true;
        }

        else
        {
            return false;
        }
    }

    bool PPLocalPlannerROS::isGoalReached()
    {

        //ROS_WARN("Returning false from the isGoalReached function!\n");

        return false;
    }

    PPLocalPlannerROS::~PPLocalPlannerROS()
    {
    }

    void PPLocalPlannerROS::setLoadedState(bool isloaded)
    {

        return;
    };

};
