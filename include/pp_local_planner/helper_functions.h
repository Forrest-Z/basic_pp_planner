#ifndef HELPER_FUNCTIONS
#define HELPER_FUNCTIONS

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace helper_functions{

    void convert_pose_stamped_to_pair_double(const geometry_msgs::PoseStamped &pose_,std::pair<double, double> &point_){

        point_  = {pose_.pose.position.x, pose_.pose.position.y};

    }


};





#endif

