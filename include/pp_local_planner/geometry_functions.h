#ifndef GEOMETRY_FUNCTIONS
#define GEOMETRY_FUNCTIONS

#include <iostream>
#include <cfloat>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace geometry_functions{

    /*std::tuple<double, double, double> crossProduct(const std::tuple<double, double, double> &va_ , const std::tuple<double, double, double> &vb_){
        
        std::tuple<double, double,double> cp_;
        double a_ = std::get<1>(va_) * std::get<2>(vb_) -std::get<2>(va_) * std::get<1>(vb_);
        
        double b_ = std::get<2>(va_) * std::get<0>(vb_) - std::get<0>(va_) * std::get<2>(vb_);

        double c_ = std::get<0>(va_) * std::get<1>(vb_)  - std::get<1>(va_) * std::get<0>(vb_);

        cp_ = {a_, b_, c_};

        return cp_;
        
    }*/

    double get_euclidean_dis(const std::pair<double, double> &a_, const std::pair<double, double> &b_)
    {

        double dis = sqrt(pow(a_.first - b_.first, 2) + pow(a_.second - b_.second, 2));
    }

    double get_slope_angle_from_two_points(const std::pair<double, double> &pt_a, const std::pair<double, double> &pt_b){

        double x1_ = pt_a.first, x2_ = pt_b.first; 
        double y1_ = pt_a.second, y2_ = pt_b.second;

        if(y1_ == y2_ ) {return acos(0);}

        else if(x1_ == x2_) {return 0 ;}

        else return {atan2(y2_ - y1_, x2_ - x1_)};

    }
    
    bool get_cr_(const std::pair<double, double> &a_, const std::pair<double, double> &b_, const std::pair<double, double> &c_, double &r_){

        double x1 = a_.first, x2 = b_.first, x3 = c_.first; 
        double y1 = a_.second, y2 = b_.second, y3 = c_.second;

        double den_  = 2*((x2-x1)*(y3-y2)-(y2-y1)*(x3-x2));

        double num_ =  ((std::pow((x2-x1),2)) + (std::pow((y2-y1),2))) * ((std::pow((x3-x2),2))+(std::pow((y3-y2),2))) * ((std::pow((x1-x3),2))+ (std::pow((y1-y3),2)));

        num_ = std::pow(num_, 0.5);
    
        if(den_ == 0) {

            //ROS_WARN("Failed:: Points are either not collinear or not distinct.");
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
