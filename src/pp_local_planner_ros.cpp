#include <pp_local_planner/pp_local_planner_ros.h>
#include <Eigen/Core>
#include <cmath>


#include <pp_local_planner/vis_functions.h>
#include <pp_local_planner/helper_functions.h>
#include <pp_local_planner/geometry_functions.h>
#include <pp_local_planner/tracker_functions.h>

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

        global_plan_pub_ = nh_.advertise<nav_msgs::Path>("global_plan", 1000, true);
        
        point_pub_ = nh_.advertise<visualization_msgs::Marker>("point_marker", 1000, true);
        unfilled_circle_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("unfilled_circle_markerarray_", 1000, true);

        lookahead_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("lookahed_pose_", 1000, true);
        closest_pt_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("closest_pose_", 1000, true);


    }

    bool PPLocalPlannerROS::get_closest_pt_idx_in_global_plan_(int &mn_index){

        double mn_dis_ = 1000;
        mn_index = -1;

        std::pair<double, double> global_pose_; 
        helper_functions::convert_pose_stamped_to_pair_double(global_pose_stamped_, global_pose_);


        for(int i = 0; i < (int)global_plan_.size(); i++) {

            geometry_msgs::PoseStamped stamped_pose_ = global_plan_.at(i);

            std::pair<double, double> pose_ ;
            helper_functions::convert_pose_stamped_to_pair_double(stamped_pose_, pose_);

            double dis_ = geometry_functions::get_euclidean_dis(global_pose_, pose_);

            if(dis_ < mn_dis_){

                mn_dis_ = dis_; 
                mn_index = i; 

            }

        }

        if(mn_index == -1) {return false; }

        return true; 

        
    }


    bool PPLocalPlannerROS::get_lookahead_pt_idx_in_global_plan_(const double &la_dis_, const int &closest_pt_idx_, int &la_pt_idx){
        
        bool flag_ = false; 

        double dis_ ;

        geometry_msgs::PoseStamped closest_stamped_pose_ = global_plan_.at(closest_pt_idx_);
        std::pair<double, double> closest_pose_; 
        helper_functions::convert_pose_stamped_to_pair_double(closest_stamped_pose_, closest_pose_);

        for(int i = closest_pt_idx_; i < (int)global_plan_.size(); i++) {
            
            geometry_msgs::PoseStamped stamped_pose_ = global_plan_.at(i);

            std::pair<double, double> pose_; 

            helper_functions::convert_pose_stamped_to_pair_double(stamped_pose_, pose_);

            dis_ = geometry_functions::get_euclidean_dis(closest_pose_, pose_);

            if(dis_ >= la_dis_) {

                la_pt_idx = i; 
                return true;


            }

        }

        return false;

    }

    bool PPLocalPlannerROS::get_path_curvature_at_index(const int &idx, double &r_){

        int len_ = (int)global_plan_.size(); 

        if (idx == 0) {return false; }

        if(idx == (int)global_plan_.size() - 1) {return false;}

        geometry_msgs::PoseStamped pose_a_, pose_b_, pose_c_;
            
        pose_a_ = global_plan_.at(idx - 1);
        pose_b_ = global_plan_.at(idx);
        pose_c_ = global_plan_.at(idx + 1);

            
        std::pair<double, double> a_, b_, c_;

        helper_functions::convert_pose_stamped_to_pair_double(pose_a_, a_); 
        helper_functions::convert_pose_stamped_to_pair_double(pose_b_, b_); 
        helper_functions::convert_pose_stamped_to_pair_double(pose_c_, c_); 
            
        
        bool flag_ = geometry_functions::get_cr_(a_, b_ ,c_, r_);

        return flag_;


    }

    void PPLocalPlannerROS::process_global_path_points_(std::vector<tracker_functions::PathPoint> &path_points_){

        helper_functions::convert_pose_stamped_to_pair_double(global_pose_stamped_, global_pose_);

        int sz_ = (int)global_plan_.size(); 

        if(sz_ < 3) {

            ROS_WARN("The size of global_plan_ < 3 -- Something might be wrong!\n");

        }

        for(int i  =0 ; i< (int)global_plan_.size(); i++) {

            double r_, dis_; 
            bool flag_;
            int idx = -1;
            
            if(i == 0) {idx = i + 1; }
            
            else if(i == (int)global_plan_.size() - 1) {idx = i - 1; }
            
            else {idx = i ;}


            flag_ = get_path_curvature_at_index(idx, r_);
            
            if(!flag_) {r_ = -1;}

            geometry_msgs::PoseStamped pose_stamped_ = global_plan_.at(i); 
            std::pair<double, double> pose_; 

            helper_functions::convert_pose_stamped_to_pair_double(pose_stamped_, pose_);    

            dis_ = geometry_functions::get_euclidean_dis(global_pose_, pose_);

            tracker_functions::PathPoint path_pt = {pose_stamped_, pose_, dis_, r_};

            path_points_.push_back(path_pt);


        }

z`
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

        std::vector<tracker_functions::path_point> path_points_;

        helper_functions::convert_pose_stamped_to_pair_double(global_pose_stamped_, global_pose_);

        /*int cnt_a =0, cnt_b = 0 ; 

        int closest_pt_idx_; 
        int flag_ = get_closest_pt_idx_in_global_plan_(closest_pt_idx_);
        
        if(!flag_) {
            
            cnt_a++;
            ROS_ERROR("Unable to find the closest point to the robot pose in the global plan!\n");
            return false;
        
        }

        ROS_WARN("closest_pt_index: %d\n", closest_pt_idx_);


        closest_pt_pub_.publish(global_plan_.at(closest_pt_idx_));
        
        double la_dis_ = 2.0;
        int la_pt_idx = -1;

        flag_ = get_lookahead_pt_idx_in_global_plan_(la_dis_, closest_pt_idx_, la_pt_idx);

        if(!flag_) {
            
            cnt_b++;
            ROS_ERROR("Unable to find a suitable lookahead pose!\n");
            return false;

        }

        lookahead_pose_pub_.publish(global_plan_.at(la_pt_idx));
        
        ROS_WARN("cnt_a: %d cnt_b: %d\n",cnt_a, cnt_b);
        ROS_WARN("Sleeping for 1s \n");
        ros::Duration(1.0).sleep();
        */


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
