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

        std::pair<double,double> last_pt_;
        
        helper_functions::convert_pose_stamped_to_pair_double(global_plan_.back(),last_pt_);

        vis_functions::publish_point_(last_pt_, point_pub_, planner_util_, nh_);

        ROS_INFO("Size of global_plan_: %d\n", (int)global_plan_.size());

        std::pair<double, double> a_  = {-1, 0}, b_ = {0,1}, c_ = {1, 0}, cc_; 

        bool flag_ = math_functions::findCircumCenter(a_, b_, c_, cc_);

        ROS_WARN("flag_: %d\n", flag_);
        
        double y_;

        flag_ = geometry_functions::get_menger_curvature(a_, b_, c_, y_);

        if(flag_){

            double r_ = 1.0/y_;

            vis_functions::publish_circle_(cc_, r_, circle_pub_, planner_util_, nh_);

        }

        
        /*for (int i = 1; i < (int)global_plan_.size() - 1; i++)
        {

            geometry_msgs::PoseStamped pose_a_, pose_b_, pose_c_;
            pose_a_ = global_plan_.at(i - 1);
            pose_b_ = global_plan_.at(i);
            pose_c_ = global_plan_.at(i + 1);

            bool flag_;

            double inst_menger_curvature_, inst_radius_of_curvature_ = -1.0;
            std::pair<double, double> inst_circumcentre_;

            flag_ = get_menger_curvature(pose_a_, pose_b_, pose_c_, inst_menger_curvature_);

            if (!flag_){
                
                inst_menger_curvature_ = 0;
                ROS_WARN("Merger curvature set to 0!\n");
                //return false;s
            }
            else{

                inst_radius_of_curvature_ = (1.0/inst_menger_curvature_);

                ROS_INFO("inst_radius_of_curvature_: %f inst_menger_curvature: %f\n", inst_radius_of_curvature_, inst_menger_curvature_);
            
            }

            if(inst_radius_of_curvature_ == -1) {
                
                ROS_WARN("i: %d inst_radius_of_radius: -1\n");
                continue;
            }
            
            else if(inst_radius_of_curvature_ > 10) {
                
                ROS_WARN("inst_radius_of_curvature: > 10"); 
                continue;
            }

            std::pair<double, double> a_ = {pose_a_.pose.position.x, pose_a_.pose.position.y};
            std::pair<double, double> b_  = {pose_b_.pose.position.x, pose_b_.pose.position.y};
            std::pair<double, double> c_ = {pose_c_.pose.position.x, pose_c_.pose.position.y};

            ROS_INFO("Publishing circle corresponding to the %dth waypoint\n", i);
            
            flag_ = math_functions::findCircumCenter(a_, b_,c_ ,inst_circumcentre_);            
            
            if (!flag_)
            {

                ROS_ERROR("Unable to find the circumcentre! -- Something might be wrong!\n");
                return false;
            
            }

            publish_circle_(inst_circumcentre_, inst_radius_of_curvature_);

            cmd_vel.angular.z = 1;

            ROS_INFO("Sleeping for 1 second!\n");
            ros::Duration(1.0).sleep();
        }*/

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
