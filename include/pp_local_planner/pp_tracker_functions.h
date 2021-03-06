#ifndef TRACKER_FUNCTIONS
#define TRACKER_FUNCTIONS

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <pp_local_planner/helper_functions.h>
#include <pp_local_planner/geometry_functions.h>
#include <pp_local_planner/pp_ds.h>

namespace pp_tracker_functions {


    void print_pp_limits(const pp_ds::Limits &pp_limits_){

        ROS_WARN("v_mn_:%f\n", pp_limits_.v_mn_);
        ROS_WARN("v_mx_:%f\n", pp_limits_.v_mx_);
        ROS_WARN("r_mx_:%f\n", pp_limits_.r_mx_);
        ROS_WARN("r_thresh_:%f\n", pp_limits_.r_thresh_);
        ROS_WARN("la_dis_:%f\n", pp_limits_.la_dis_);
                
    }      

    void initialize_pp_limits(pp_ds::Limits &pp_limits_){
        
        //Update print_pp_limits
        pp_limits_.v_mn_ = 0.1; 
        pp_limits_.v_mx_ = 1.0;
        pp_limits_.r_mx_ = 100;
        pp_limits_.r_thresh_ = 20;
        pp_limits_.la_dis_ = 1.5;

    }

    bool get_path_rad_curvature_at_index(const int &idx, double &r_, const pp_ds::Plan_ &global_plan_)
    {

        int len_ = (int)global_plan_.size();

        if (idx == 0)
        {
            return false;
        }

        if (idx == (int)global_plan_.size() - 1)
        {
            return false;
        }

        geometry_msgs::PoseStamped pose_a_, pose_b_, pose_c_;

        pose_a_ = global_plan_.at(idx - 1);
        pose_b_ = global_plan_.at(idx);
        pose_c_ = global_plan_.at(idx + 1);

        std::pair<double, double> a_, b_, c_;

        helper_functions::convert_pose_stamped_to_pair_double(pose_a_, a_);
        helper_functions::convert_pose_stamped_to_pair_double(pose_b_, b_);
        helper_functions::convert_pose_stamped_to_pair_double(pose_c_, c_);

        bool flag_ = geometry_functions::get_cr_(a_, b_, c_, r_);

        return flag_;
    }

    bool get_lookahead_pt_idx_in_global_plan_(const pp_ds::Limits &pp_limits_, const int &closest_pt_idx_, int &la_pt_idx, const pp_ds::Plan_ &global_plan_)
    {

        bool flag_ = false;

        double dis_;

        geometry_msgs::PoseStamped closest_stamped_pose_ = global_plan_.at(closest_pt_idx_);
        std::pair<double, double> closest_pose_;
        helper_functions::convert_pose_stamped_to_pair_double(closest_stamped_pose_, closest_pose_);

        for (int i = closest_pt_idx_; i < (int)global_plan_.size(); i++)
        {

            geometry_msgs::PoseStamped stamped_pose_ = global_plan_.at(i);

            std::pair<double, double> pose_;

            helper_functions::convert_pose_stamped_to_pair_double(stamped_pose_, pose_);

            dis_ = geometry_functions::get_euclidean_dis(closest_pose_, pose_);

            if (dis_ >= pp_limits_.la_dis_)
            {

                la_pt_idx = i;
                return true;
            }
        }

        return false;
    }

    
    /**
     * @brief Sets the values of vx_ and r_ for the global path points. 
     * @param global_plan_ 
     * @param path_points_ 
     * @param pp_limit_ 
     */
    void process_global_path_points(const pp_ds::Plan_ &global_plan_, std::vector<pp_ds::PathPoint> &path_points_, const pp_ds::Limits &pp_limits_){

        
        for(int i= 0; i < (int)global_plan_.size(); i++) {
            
            geometry_msgs::PoseStamped stamped_pose_ = global_plan_.at(i);
            std::pair<double,double> pose_; 
            helper_functions::convert_pose_stamped_to_pair_double(stamped_pose_, pose_);

            pp_ds::PathPoint path_point_ = {stamped_pose_, pose_, -1, pp_limits_.v_mx_};

            //Setting r=0 for start and end points
            if( i == 0 || i == (int)global_plan_.size() - 1) {
                
                path_point_.r_ = -1;
                path_point_.vx_ = pp_limits_.v_mn_;    
                continue;
                
            }

            double r_;

            bool flag_ = get_path_rad_curvature_at_index(i, r_, global_plan_);

            if(flag_ && r_ <= pp_limits_.r_thresh_) {

                path_point_.r_ = r_;
                path_point_.vx_ = std::max(pp_limits_.v_mn_, (r_/pp_limits_.r_mx_) * pp_limits_.v_mx_);
            
            }

            else {

                path_point_.vx_ = pp_limits_.v_mx_;

            }

            path_points_.push_back(path_point_);

        }

    }


    bool get_ct_error_signum(const geometry_msgs::PoseStamped &global_pose_, const int &la_pt_idx, const pp_ds::Plan_ &global_plan_){


        std::tuple<double, double> rv_, sv_, lav_; 
        
        rv_ = {global_pose_.pose.position.x, global_pose_.pose.position.y};
        lav_ = {global_plan_[la_pt_idx].pose.position.x, global_plan_[la_pt_idx].pose.position.y};

        double theta_ = tf::getYaw(global_pose_.pose.orientation);

        double dx_ = cos(theta_), dy_ = sin(theta_);

        sv_ = {std::get<0>(rv_) + dx_ , std::get<1>(rv_) + dy_};

        double cp_ =  (std::get<1>(sv_)  - std::get<1>(rv_)) * (std::get<0>(lav_) - std::get<0>(rv_)) -  (std::get<0>(sv_) - std::get<0>(rv_)) * (std::get<1>(lav_) - std::get<1>(rv_));

        return (cp_ > 0);
    }


    bool get_closest_pt_idx_in_global_plan_(int &mn_index, const geometry_msgs::PoseStamped &global_pose_stamped_, const pp_ds::Plan_ &global_plan_)
    {

        double mn_dis_ = 1000;
        mn_index = -1;

        std::pair<double, double> global_pose_;
        helper_functions::convert_pose_stamped_to_pair_double(global_pose_stamped_, global_pose_);

        for (int i = 0; i < (int)global_plan_.size(); i++)
        {

            geometry_msgs::PoseStamped stamped_pose_ = global_plan_.at(i);

            std::pair<double, double> pose_;
            helper_functions::convert_pose_stamped_to_pair_double(stamped_pose_, pose_);

            double dis_ = geometry_functions::get_euclidean_dis(global_pose_, pose_);

            if (dis_ < mn_dis_)
            {

                mn_dis_ = dis_;
                mn_index = i;
            }
        }

        if (mn_index == -1)
        {
            return false;
        }

        return true;
    }

    void get_cross_track_error_(const int &closest_pt_idx, const int &la_pt_idx, const pp_ds::Plan_ &global_plan_, double &e_, double &alpha_){

        geometry_msgs::PoseStamped closest_pose_stamped_, la_pose_stamped_;
        std::pair<double, double> closest_pose_, la_pose_; 

        closest_pose_stamped_ = global_plan_.at(closest_pt_idx); 
        la_pose_stamped_ = global_plan_.at(la_pt_idx);

        helper_functions::convert_pose_stamped_to_pair_double(closest_pose_stamped_, closest_pose_);
        helper_functions::convert_pose_stamped_to_pair_double(la_pose_stamped_, la_pose_);

        double heading_, la_theta_;
        double dx_, dy_; 
        double la_dis_;

        la_dis_ = geometry_functions::get_euclidean_dis(closest_pose_, la_pose_);

        heading_ = tf::getYaw(closest_pose_stamped_.pose.orientation);

        dx_ = (la_pose_.first  - closest_pose_.first);
        dy_ = (la_pose_.second - closest_pose_.second);

        if(dx_ == 0){la_theta_ == 1.57; }

        else { la_theta_ = atan2(dy_, dx_) ; }


        ROS_WARN("la_theta_: %f\n", la_theta_);

        alpha_ = la_theta_ - heading_;

        e_ = la_dis_ * sin(alpha_);       

       // e_ = abs(e_);  

        //return e_;

    }

    



};

#endif