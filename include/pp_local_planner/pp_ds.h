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

        double v_mn_; 
        double v_mx_;
        double r_mx_;

    };

};