#ifndef MOTION_PLANNER_DATA
#define MOTION_PLANNER_DATA

//STL includes
#include<vector>

//ros includes
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/Twist.h>

namespace mpd
{
    struct ObstacleInfo
    {
        //geometry_msgs::PoseStamped obstacle_pose;
        double cost;
    };
    
    //container for holding critical obstacle information in the global path.
    struct MotionPose
    {
        geometry_msgs::PoseStamped pose;
        ObstacleInfo obstacle_info;
        geometry_msgs::Twist twist_ref;
        double acc_lin;
        double curvature;
        double arc_length; //from global robot position.
        bool in_place;
    };
   
    typedef std::vector<geometry_msgs::PoseStamped> Plan;
    typedef std::vector<MotionPose> MotionPlan;
    typedef std::vector<geometry_msgs::Twist> VelocityPlan;
    typedef std::tuple<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> MengerPoints;
    typedef std::pair<geometry_msgs::PoseStamped, double> CrossTrackInfo;

    inline double euclidean(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2)
    {
        return (sqrt(pow(pose1.pose.position.x - pose2.pose.position.x, 2) + pow(pose1.pose.position.y -
        pose2.pose.position.y, 2)));
    }
};

#endif
