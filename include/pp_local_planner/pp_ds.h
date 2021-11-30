#ifndef PP_DS
#define PP_DS


#include <geometry_msgs/PoseStamped.h>

namespace pp_ds{

    typedef std::vector<geometry_msgs::PoseStamped> Plan_;


    struct PathPoint {

                geometry_msgs::PoseStamped stamped_pose_; 
                std::pair<double,double> pose_; 
                double r_;
                double vx_;               

    };

    struct Limits{


        //Update pp_tracker_functions::initialize_limits() and print_limits()
        double v_mn_; 
        double v_mx_;
        double r_mx_;
        double r_thresh_;
        double la_dis_;



    };

};

#endif