#ifndef VIS_FUNCTIONS
#define VIS_FUNCTIONS

#include <ros/ros.h>
#include <pp_local_planner/pp_ds.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <base_local_planner/local_planner_util.h>
#include <pp_local_planner/helper_functions.h>
#include <pp_local_planner/geometry_functions.h>

namespace vis_functions{

    void publish_circle_(const std::pair<double, double> &cc_, const double &r_, ros::Publisher &circle_pub_, base_local_planner::LocalPlannerUtil &planner_util_, ros::NodeHandle &nh_){

            visualization_msgs::Marker marker;
            marker.header.frame_id = planner_util_.getGlobalFrame();
            marker.header.stamp = ros::Time();
            marker.ns = nh_.getNamespace() + "_circle_";
            marker.id = 0;

            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = cc_.first;
            marker.pose.position.y = cc_.second;
            marker.pose.position.z = 1;

            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 2.0 * r_;
            marker.scale.y = 2.0 * r_;
            marker.scale.z = 0.2;

            marker.color.a = 1.0; // Don't forget to set the alpha!

            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            circle_pub_.publish(marker);
    }

    void publish_unfilled_circle_(const std::pair<double, double> &cc_, const double &r_, ros::Publisher &unfilled_circle_pub_, base_local_planner::LocalPlannerUtil &planner_util_, ros::NodeHandle &nh_){

            visualization_msgs::MarkerArray marker_array;
            
            int num_markers_ = 50;
            double theta_i_ = 0 ;
            double del_theta_ = (2 * 3.14)/(1.0 * num_markers_);

            int marker_id_ =0 ; 

            for(int i =0 ; i < num_markers_ ; i++) {

                double theta_ = theta_i_ + (i * del_theta_);
                
                visualization_msgs::Marker marker;

                marker.header.frame_id = planner_util_.getGlobalFrame();
                marker.header.stamp = ros::Time();
                marker.ns = nh_.getNamespace() + "unfilled_circle_";
                marker.id = marker_id_++;

                marker.type = visualization_msgs::Marker::CYLINDER;
                marker.action = visualization_msgs::Marker::ADD;

                marker.pose.position.x = cc_.first + (r_ * cos(theta_));
                marker.pose.position.y = cc_.second + (r_ * sin(theta_));
                marker.pose.position.z = 1;

                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;

                marker.scale.x = 0.02;
                marker.scale.y = 0.02;
                marker.scale.z = 1.0;

                marker.color.a = 1.0; // Don't forget to set the alpha!

                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;

                marker_array.markers.push_back(marker);

            }
            
            unfilled_circle_pub_.publish(marker_array);
    }

    void publish_point_(const std::pair<double, double> &pt_, ros::Publisher &point_pub_, base_local_planner::LocalPlannerUtil &planner_util_, ros::NodeHandle &nh_){

            visualization_msgs::Marker marker;
            marker.header.frame_id = planner_util_.getGlobalFrame();
            marker.header.stamp = ros::Time();
            marker.ns = nh_.getNamespace() + "_point_";
            marker.id = 0;

            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = pt_.first;
            marker.pose.position.y = pt_.second;
            marker.pose.position.z = 1;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.0;

            marker.color.a = 1.0; // Don't forget to set the alpha!

            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;

            point_pub_.publish(marker);
    }

    void publish_la_point_line(const int &closest_pt_idx, const int &la_pt_idx,  const pp_ds::Plan_ &global_plan_,  ros::Publisher &la_pt_line_pub, base_local_planner::LocalPlannerUtil &planner_util_, ros::NodeHandle &nh_){
        
        visualization_msgs::MarkerArray marker_array;
            

        geometry_msgs::PoseStamped closest_stamped_pose_ , la_stamped_pose_; 
        std::pair<double, double> closest_pose_, la_pose_; 

        closest_stamped_pose_ = global_plan_.at(closest_pt_idx); 
        la_stamped_pose_ = global_plan_.at(la_pt_idx);

        helper_functions::convert_pose_stamped_to_pair_double(closest_stamped_pose_, closest_pose_);
        helper_functions::convert_pose_stamped_to_pair_double(la_stamped_pose_, la_pose_);

        double dis_ = geometry_functions::get_euclidean_dis(closest_pose_, la_pose_);

        int num_pts = 50; 

        double theta_; 

        double dx_ = la_pose_.first - closest_pose_.first; 
        double dy_ = la_pose_.second - closest_pose_.second;

        if(dx_ == 0) {

                theta_ = 1.57; 

        }

        else {

                theta_ = atan2(dy_, dx_);

        }

        double step_sz_ = (dis_/num_pts);

        int marker_id_ = 0 ;

        for(int i = 0 ;i < num_pts; i++) {
                

                double x_ = closest_pose_.first + (i * step_sz_ * cos(theta_));
                double y_ = closest_pose_.second + (i * step_sz_ * sin(theta_));  
                
                visualization_msgs::Marker marker;

                marker.header.frame_id = planner_util_.getGlobalFrame();
                marker.header.stamp = ros::Time();
                marker.ns = nh_.getNamespace() + "_ct_error_";
                marker.id = marker_id_++;

                marker.type = visualization_msgs::Marker::CUBE;
                marker.action = visualization_msgs::Marker::ADD;

                marker.pose.position.x = x_;
                marker.pose.position.y = y_;
                marker.pose.position.z = 0;

                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;

                marker.scale.x = 0.02;
                marker.scale.y = 0.02;
                marker.scale.z = 1.0;

                marker.color.a = 1.0; // Don't forget to set the alpha!

                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 1.0;

                marker_array.markers.push_back(marker);
                

        }

        la_pt_line_pub.publish(marker_array);

        
    }


    void publish_ct_error_line(const int &la_pt_idx, const int &closest_pt_idx, const pp_ds::Plan_ &global_plan_, const double &la_dis_, const double &e_, ros::Publisher &crosstrack_error_pub_, base_local_planner::LocalPlannerUtil &planner_util_, ros::NodeHandle &nh_){
        

        visualization_msgs::MarkerArray marker_array;

        std::pair<double, double> closest_pt_, la_pt_;    
        
        int marker_id_ =0 ; 
        
        geometry_msgs::PoseStamped closest_stamped_pose_ = global_plan_.at(closest_pt_idx);
        geometry_msgs::PoseStamped la_stamped_pose_ = global_plan_.at(closest_pt_idx);
        
        helper_functions::convert_pose_stamped_to_pair_double(closest_stamped_pose_, closest_pt_);
        helper_functions::convert_pose_stamped_to_pair_double(la_stamped_pose_, la_pt_);

        double la_line_slope_angle = geometry_functions::get_slope_angle_from_two_points(closest_pt_, la_pt_);

        double m_angle_ = la_line_slope_angle + acos(0);

        int num_pts = 50;

        double sep_ = e_/num_pts;
        
        for(double len_ = 0; len_ <= e_ ; len_ += sep_){
                
                visualization_msgs::Marker marker;

                double x_ = la_pt_.first -  len_ * cos(m_angle_); 
                double y_ = la_pt_.second  - len_ * sin(m_angle_);

                std::pair<double, double> pt_ = {x_, y_};

                marker.header.frame_id = planner_util_.getGlobalFrame();
                marker.header.stamp = ros::Time();
                marker.ns = nh_.getNamespace() + "_ct_error_";
                marker.id = marker_id_++;

                marker.type = visualization_msgs::Marker::CYLINDER;
                marker.action = visualization_msgs::Marker::ADD;

                marker.pose.position.x = x_;
                marker.pose.position.y = y_;
                marker.pose.position.z = 0;

                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;

                marker.scale.x = 0.02;
                marker.scale.y = 0.02;
                marker.scale.z = 0.02;

                marker.color.a = 1.0; // Don't forget to set the alpha!

                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 1.0;

                marker_array.markers.push_back(marker);

        }

        crosstrack_error_pub_.publish(marker_array);

    }


};

#endif

