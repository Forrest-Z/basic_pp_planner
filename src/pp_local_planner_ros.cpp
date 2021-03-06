#include <pp_local_planner/pp_local_planner_ros.h>
#include <Eigen/Core>
#include <cmath>



#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>


PLUGINLIB_EXPORT_CLASS(pp_local_planner::PPLocalPlannerROS, nav_core::BaseLocalPlanner)

namespace pp_local_planner
{

    PPLocalPlannerROS::PPLocalPlannerROS() : initialized_(false)
    {
    }


    void PPLocalPlannerROS::advertise_publishers(){

        global_plan_pub_ = nh_.advertise<nav_msgs::Path>("global_plan", 1000, true);

        point_pub_ = nh_.advertise<visualization_msgs::Marker>("point_marker", 1000, true);
        unfilled_circle_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("unfilled_circle_markerarray_", 1000, true);

        lookahead_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("lookahed_pose_", 1000, true);
        closest_pt_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("closest_pose_", 1000, true);
        ct_error_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("crossstrack_error", 1000, true);
        la_pt_line_pub= nh_.advertise<visualization_msgs::MarkerArray>("la_pt_line", 1000, true);

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

        advertise_publishers();

        pp_tracker_functions::initialize_pp_limits(pp_limits_);
        pp_tracker_functions::print_pp_limits(pp_limits_);

        
        
        initialized_ = true;

    }

    
    bool PPLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {

        ROS_INFO("Inside the computeVelocityCommand function!\n");

        if (!costmap_ros_->getRobotPose(global_pose_tf_))
        {

            ROS_ERROR("Could not get robot pose!\n");
            return false;
        }

        tf::poseStampedTFToMsg(global_pose_tf_, global_pose_stamped_);

        if (!planner_util_.getLocalPlan(global_pose_tf_, global_plan_))
        {

            ROS_ERROR("planner_util_ could not find transformed_plan_!\n");
            return false;
        }

        base_local_planner::publishPlan(global_plan_, global_plan_pub_);


        int closest_pt_idx_, la_pt_idx;

        bool flag_; 

        flag_ = pp_tracker_functions::get_closest_pt_idx_in_global_plan_(closest_pt_idx_, global_pose_stamped_, global_plan_);

        if(!flag_) {

            ROS_ERROR("Unable to find the closest_pt_idx in the global_plan_!\n");
            return false;

        }

        flag_ = pp_tracker_functions::get_lookahead_pt_idx_in_global_plan_(pp_limits_, closest_pt_idx_, la_pt_idx, global_plan_);

        if(!flag_) {

            ROS_ERROR("Unable to find a suitable lookahead point!\n");
            return false;

        }

        ROS_WARN("Path processing started!\n");

        std::vector<pp_ds::PathPoint> path_points_;
        pp_tracker_functions::process_global_path_points(global_plan_, path_points_, pp_limits_);
        
        double v_ = path_points_[closest_pt_idx_].vx_;

        geometry_msgs::PoseStamped closest_stamped_pose_ = global_plan_.at(closest_pt_idx_); 
        geometry_msgs::PoseStamped la_stamped_pose_ = global_plan_.at(la_pt_idx);
        
        std::pair<double, double> closest_pose_, la_pose_; 

        helper_functions::convert_pose_stamped_to_pair_double(closest_stamped_pose_, closest_pose_); 
        helper_functions::convert_pose_stamped_to_pair_double(la_stamped_pose_, la_pose_);

        double la_dis_ = geometry_functions::get_euclidean_dis(closest_pose_ , la_pose_);

        lookahead_pose_pub_.publish(global_plan_.at(la_pt_idx));
        closest_pt_pub_.publish(global_plan_.at(closest_pt_idx_));

        double e_, alpha_;  
        pp_tracker_functions::get_cross_track_error_(closest_pt_idx_, la_pt_idx, global_plan_, e_, alpha_);
        
        ROS_WARN("e_: %f\n", e_);

        v_ = 0.2;

        //if(v_ == 1.0) {v_ = 0.1; }

        double r_ = (la_dis_ * la_dis_)/ (2 * e_);

        double sig_ = pp_tracker_functions::get_ct_error_signum(global_pose_stamped_, la_pt_idx, global_plan_);

        double mult_ = (sig_ > 0 ? -1 : 1);

        double w_ = mult_ * abs(v_/r_);

        vis_functions::publish_la_point_line(closest_pt_idx_, la_pt_idx, global_plan_, la_pt_line_pub, planner_util_, nh_);
        vis_functions::publish_ct_error_line(la_pt_idx, global_pose_stamped_, global_plan_, alpha_, e_, ct_error_pub_, planner_util_, nh_);



        ROS_WARN("v_: %f w_: %f r_: %f\n", v_, w_, r_);
        cmd_vel.linear.x = v_; 
        cmd_vel.angular.z = w_;
        
        ros::Duration(2.0).sleep();

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
