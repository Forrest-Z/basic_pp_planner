#include <pp_local_planner/pp_local_planner_ros.h>
#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>


PLUGINLIB_EXPORT_CLASS(pp_local_planner::PPLocalPlannerROS, nav_core::BaseLocalPlanner)

    namespace pp_local_planner {

        PPLocalPlannerROS::PPLocalPlannerROS():initialized_(false){

        }

        void PPLocalPlannerROS::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) {
            
            if(initialized_){

                ROS_WARN("The planner has already been initialized!\n");
                return;
                
            }

            tf_ = tf; 
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            
            ros::NodeHandle private_nh_("~" + name);

        
            global_plan_pub_ =  private_nh_.advertise<nav_msgs::Path>("global_plan_", 1000, true);   
            look_ahead_point_pub = private_nh_.advertise<geometry_msgs::PoseStamped>("lookahead_pose", 1000, true);     
            global_pose_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("global_pose_", 1000, true);
            global_goal_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("global_goal", 1000, true);
            
            planner_util_.initialize(tf_, costmap_, costmap_ros_->getGlobalFrameID());
            
            initialized_ = true;



        }


        bool PPLocalPlannerROS::update_next_look_ahead_point(geometry_msgs::PoseStamped &next_look_ahead_pose_){

            tf::poseStampedTFToMsg(global_pose_tf_, global_pose_);

            for(auto it_ = global_plan_.begin(); it_ != global_plan_.end(); ++it_){

                double dis_from_robot_pose_ = euclidean(*it_, global_pose_);

                if(dis_from_robot_pose_ > look_ahead_dis_) {
                    
                    geometry_msgs::PoseStamped curr_path_pt = *it_;
                    next_look_ahead_pose_ = *it_;
                    return true;

                }

            }

            ROS_ERROR("Could not find the next look-ahead point!\n");

            return false;


        }

        bool PPLocalPlannerROS::get_global_goal_(geometry_msgs::PoseStamped &goal_pt){
            
            tf::Stamped<tf::Pose> global_goal_;
            
            std::vector<geometry_msgs::PoseStamped> local_plan_;
            bool f_ = planner_util_.getLocalPlan(global_pose_tf_, local_plan_);

            if(!f_) {
                
                ROS_ERROR("planner_util_ could not find the local_plan_!\n");
                return false;

            }

            bool flag_ = base_local_planner::getGoalPose(*tf_, local_plan_, planner_util_.getGlobalFrame(), global_goal_);

            if(!flag_) {return false;}
            
            tf::poseStampedTFToMsg(global_goal_, goal_pt);  
            return true;

        }

        bool PPLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {

            ROS_INFO("Inside the computeVelocityCommand function!\n");

            if(!costmap_ros_->getRobotPose(global_pose_tf_)) {

                ROS_ERROR("Could not get robot pose!\n");
                return false;
            }

            if(!planner_util_.getLocalPlan(global_pose_tf_, global_plan_)) { 

                ROS_ERROR("planner_util_ could not find transformed_plan_!\n"); 
                return false;

            }
            
            base_local_planner::publishPlan(global_plan_, global_plan_pub_);

            geometry_msgs::PoseStamped next_look_ahead_pose_;
            bool flag_; 
            
            flag_ = update_next_look_ahead_point(next_look_ahead_pose_);

            if(!flag_) {
                
               ROS_ERROR("Could not find the next look ahead pose!\n");
                return false;
            }

            //geometry_msgs::PoseStamped global_goal_;
            
            //flag_ = get_global_goal_(global_goal_);

            /*if(!flag_) {

                ROS_ERROR("Could not find the global goal!\n");
                return false;
            
            }*/

            look_ahead_point_pub.publish(next_look_ahead_pose_);
            global_pose_pub_.publish(global_pose_);
            //global_goal_pub_.publish(global_goal_);

            double del_y = next_look_ahead_pose_.pose.position.y - global_pose_.pose.position.y;
            double r_ = 2 * (del_y)/(look_ahead_dis_ * look_ahead_dis_);

            double look_ahead_yaw_ = tf::getYaw(next_look_ahead_pose_.pose.orientation);
            double global_pose_yaw_ = tf::getYaw(global_pose_.pose.orientation);

            double w_, v_; 

            if(look_ahead_yaw_ > global_pose_yaw_) {w_=1;  }

            else {w_ = -1;}

            //v_ = abs(w_/r_);

            v_ = 0.1;

            w_ = w_ * (v_ * r_);

            ROS_INFO("r_: %f\n", r_);
            ROS_INFO("v_: %f w_ : %f\n", v_, w_);

            cmd_vel.angular.z = w_; 
            cmd_vel.linear.x = v_;
            


            return true;

}
       
        bool PPLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {

            if(!initialized_) {
                
                ROS_ERROR("Initialized the planner before calling the setPlan function!\n");    
                return false;

            }
            
            latchedStopRotateController_.resetLatching();

            ROS_INFO("Got new plan! -- Sleeping for 2 seconds!\n");

            ros::Duration(2.0).sleep();
            
            if(planner_util_.setPlan(orig_global_plan)) { 

                orig_global_plan_ = orig_global_plan;    
                return true; 
                
            }

            else {return false;}

        }

        
        bool PPLocalPlannerROS::isGoalReached() {
            
            
            //ROS_WARN("Returning false from the isGoalReached function!\n");
            
            return false;

        }

        
        PPLocalPlannerROS::~PPLocalPlannerROS(){
        
        }

       

        void PPLocalPlannerROS::setLoadedState(bool isloaded){

            return;

        };


    };
