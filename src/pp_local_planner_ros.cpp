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
    

            planner_util_.initialize(tf, costmap_, costmap_ros_->getGlobalFrameID());
            

            
            initialized_ = true;



        }

        bool PPLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {

            if(!initialized_) {
                
                ROS_ERROR("Initialized the planner before calling the setPlan function!\n");    
                return false;

            }
            
            latchedStopRotateController_.resetLatching();

            ROS_INFO("Got new plan! -- Sleeping for 2 seconds!\n");

            ros::Duration(2.0).sleep();

            if(planner_util_.setPlan(orig_global_plan)) { return true; }

            else {return false;}

        }

        bool PPLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {

            if(!costmap_ros_->getRobotPose(global_pose_tf_)) {

                ROS_ERROR("Could not get robot pose!\n");
                return false;
            }


            tf::poseStampedTFToMsg(global_pose_tf_, global_pose_);
            std::vector<geometry_msgs::PoseStamped> transformed_plan_;



            if(!planner_util_.getLocalPlan(global_pose_tf_, transformed_plan_)) { 

                ROS_ERROR("planner_util_ could not find transformed_plan_!\n"); 
                return false;

            }

            base_local_planner::publishPlan(transformed_plan_, global_plan_pub_);
                    
            ROS_WARN("Inside the computeVelocityCommand function!\n");
            cmd_vel.angular.z = 1;
            return true;

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
