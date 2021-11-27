#ifndef GEOMETRY_FUNCTIONS
#define GEOMETRY_FUNCTIONS

#include <iostream>
#include <cfloat>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <pp_local_planner/math_functions.h>

namespace geometry_functions{

    double get_euclidean_dis(const std::pair<double, double> &a_, const std::pair<double, double> &b_)
    {

        double dis = sqrt(pow(a_.first - b_.first, 2) + pow(a_.second - b_.second, 2));
    }

    bool find_circumcentre(const int global_plan_index_, std::pair<double, double> &circumcentre_, const std::vector<geometry_msgs::PoseStamped> &global_plan_)
    {

        if (global_plan_index_ >= (int)global_plan_.size() - 1)
        {

            ROS_ERROR("global_plan_index_ >= (int)global_plan_.size() - 1\n");
            return false;
        }

        if (global_plan_index_ <= 0)
        {

            ROS_ERROR("global_plan_index_ <= 0\n");
            return false;
        }

        geometry_msgs::PoseStamped pose_a, pose_b, pose_c;

        pose_a = global_plan_[global_plan_index_ - 1];
        pose_b = global_plan_[global_plan_index_];
        pose_c = global_plan_[global_plan_index_ + 1];

        std::pair<double, double> a = std::make_pair(pose_a.pose.position.x, pose_a.pose.position.y);
        std::pair<double, double> b = std::make_pair(pose_b.pose.position.x, pose_b.pose.position.y);
        std::pair<double, double> c = std::make_pair(pose_c.pose.position.x, pose_c.pose.position.y);

        if (math_functions::findCircumCenter(a, b, c, circumcentre_))
        {

            return true;
        }
        else
        {

            ROS_ERROR("Unable to find the circumcentre!\n");
            return false;
        }
    }

    bool get_menger_curvature(const std::pair<double, double> &a_, const std::pair<double, double> &b_, const std::pair<double, double> &c_, double &y_)
    {

        double area_ = ((b_.first - a_.first) * (c_.second - a_.second)) - ((b_.second - a_.second) - (c_.first - a_.first));

        y_ = 1.0;

        double denom_ = get_euclidean_dis(a_, b_) * get_euclidean_dis(b_, c_) * get_euclidean_dis(c_, a_);

        if (denom_ == 0)
        {

            ROS_ERROR("denom_ is zero -- menger_curvature can't be calculated!\n");
            return false;
        }

        y_ = 2 * area_ / denom_;

        return true;
    }

};
#endif
