#ifndef GEOMETRY_FUNCTIONS
#define GEOMETRY_FUNCTIONS

#include <iostream>
#include <cfloat>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace geometry_functions{

    double get_euclidean_dis(const std::pair<double, double> &a_, const std::pair<double, double> &b_)
    {

        double dis = sqrt(pow(a_.first - b_.first, 2) + pow(a_.second - b_.second, 2));
    }

    std::tuple<double, double, double> get_line_cofficients_from_points(const std::pair<double, double> &pt1_, const std::pair<double, double> &pt2_){

        double x1_ = pt1_.first, x2_ = pt2_.first; 
        double y1_ = pt1_.second, y2_ = pt2_.second;

        double m_, c_;

        double dx_ = (x2_ - x1_); 
        double dy_ = (y2_ - y1_);

        if(dy_ == 0) {m_ = 0 ;}

        else  if(dx_ == 0) {m_ = std::numeric_limits<double>::infinity ;}

        else {m_ = (dy_/dx_);}

        
        return std::make_tuple(a_,b_,c_);

    }

    bool get_cr_(const std::pair<double, double> &a_, const std::pair<double, double> &b_, const std::pair<double, double> &c_, double &r_){

        double x1 = a_.first, x2 = b_.first, x3 = c_.first; 
        double y1 = a_.second, y2 = b_.second, y3 = c_.second;

        double den_  = 2*((x2-x1)*(y3-y2)-(y2-y1)*(x3-x2));

        double num_ =  ((std::pow((x2-x1),2)) + (std::pow((y2-y1),2))) * ((std::pow((x3-x2),2))+(std::pow((y3-y2),2))) * ((std::pow((x1-x3),2))+ (std::pow((y1-y3),2)));

        num_ = std::pow(num_, 0.5);
    
        if(den_ == 0) {

            ROS_WARN("Failed:: Points are either not collinear or not distinct.");
            return false;

        }

        r_ = abs(num_/den_);
        return true;

    }

    bool get_cc_(const std::pair<double, double> &a_, const std::pair<double, double> &b_, const std::pair<double, double> &c_, std::pair<double, double> &cc_){

        double x1 = a_.first, x2 = b_.first, x3 = c_.first; 
        double y1 = a_.second, y2 = b_.second, y3 = c_.second;

        double A = 0.5*((x2-x1)*(y3-y2)-(y2-y1)*(x3-x2));

        if ( A == 0 ) {

            ROS_WARN("Failed: points are either collinear or not distinct");
            return false;

        }

        double xnum = ((y3 - y1)*(y2 - y1)*(y3 - y2)) - ((std::pow(x2,2) - std::pow(x1,2))*(y3 - y2)) + ((std::pow(x3,2) - std::pow(x2,2))*(y2 - y1));
        double x_ = xnum/(-4*A);
        double y_ =  (-1*(x2 - x1)/(y2 - y1))*(x_-0.5*(x1 + x2)) + 0.5*(y1 + y2);
        
        cc_ = {x_, y_};

        return true; 



    }
    
};
#endif
