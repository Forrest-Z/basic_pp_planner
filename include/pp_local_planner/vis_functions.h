#ifndef VIS_FUNCTIONS
#define VIS_FUNCTIONS

#include <ros/ros.h>
#include<visualization_msgs/Marker.h>
#include <base_local_planner/local_planner_util.h>

namespace vis_functions{

    void publish_circle_(const std::pair<double, double> &center_, const double &inst_radius_, ros::Publisher &circle_pub_, base_local_planner::LocalPlannerUtil &planner_util_, ros::NodeHandle &nh_){

            visualization_msgs::Marker marker;
            marker.header.frame_id = planner_util_.getGlobalFrame();
            marker.header.stamp = ros::Time();
            marker.ns = nh_.getNamespace() + "_circle_";
            marker.id = 0;

            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = center_.first;
            marker.pose.position.y = center_.second;
            marker.pose.position.z = 1;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 1.0 * inst_radius_;
            marker.scale.y = 1.0 * inst_radius_;
            marker.scale.z = 0.0;

            marker.color.a = 1.0; // Don't forget to set the alpha!

            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            circle_pub_.publish(marker);
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

            marker.scale.x = 2.0;
            marker.scale.y = 2.0;
            marker.scale.z = 0.0;

            marker.color.a = 1.0; // Don't forget to set the alpha!

            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;

            point_pub_.publish(marker);
    }


};

#endif

