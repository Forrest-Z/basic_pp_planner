#ifndef TRACKER_FUNCTIONS
#define TRACKER_FUNCTIONS

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace tracker_functions {

    struct PathPoint {

                geometry_msgs::PoseStamped stamped_pose_; 
                std::pair<double,double> pose_; 
                double len_;
                double r_;               

    };




};

#endif