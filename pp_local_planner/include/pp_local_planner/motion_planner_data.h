/*********************************************************************
 *
 * Software License Agreement 
 *
 *  Copyright (c) 2020, Botsync.
 *  All rights reserved.
 *
 * Author: Gopan B.Chandran 
 *
 *********************************************************************/

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
        public:
            double cost;
            double dynam_obst_stop_dis = 3.0;
            double static_ob_stop_dis = 1.0;
            /*std::string obstacle_type;
            double getSafeStopDistance(std::string ob_type)
            {
                return (ob_type == "dynamic") ? dynam_obst_stop_dis : static_obst_stop_dis;
            }*/
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
        int visited_count;
        bool obstacle;
        bool in_place;
        bool error;
    };

    typedef std::vector<geometry_msgs::PoseStamped> Plan;
    typedef std::vector<MotionPose> MotionPlan;
    typedef std::vector<geometry_msgs::Twist> VelocityPlan;
    typedef std::tuple<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> MengerPoints;
    typedef std::pair<geometry_msgs::PoseStamped, double> CrossTrackInfo;
    typedef std::pair<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> PosePair;


    template<typename T>
        int sign(T sign_value)
        {
            return (sign_value > T(0)) - (sign_value < T(0)); 
        }

    inline double euclidean(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2)
    {
        return (sqrt(pow(pose1.pose.position.x - pose2.pose.position.x, 2) + pow(pose1.pose.position.y -
                        pose2.pose.position.y, 2)));
    }
    /*bool operator == (const geometry_msgs::PoseStamped& pose_a, const geometry_msgs::PoseStamped& pose_b)
      {
      return (pose_a.pose.position.x == pose_b.pose.position.x && pose_a.pose.position.y == pose_b.pose.position.y &&
      pose_a.pose.position.z == pose_b.pose.position.z && pose_a.pose.orientation.x == pose_b.pose.orientation.y ==
      pose_b.pose.orientation.y && pose_a.pose.orientation.w == pose_b.pose.orientation.w && pose_a.pose.orientation.z
      == pose_b.pose.orientation.z);
      }*/
    struct PoseCompare
    { 
        geometry_msgs::PoseStamped search_pose_;
        PoseCompare(geometry_msgs::PoseStamped search_pose): search_pose_{search_pose}{
            std::cout << "searching"<< std::endl;
            }
        bool operator ()(const geometry_msgs::PoseStamped& pose) const
        {  
            std::cout << "search pose inside functor X :  Y :" <<  search_pose_.pose.position.x << " " <<
            search_pose_.pose.position.y <<" " << search_pose_.pose.position.z << " " << search_pose_.pose.orientation.x
            <<" "<<search_pose_.pose.orientation.y <<" " << search_pose_.pose.orientation.z <<" "<< search_pose_.pose.orientation.w << std::endl;
            std::cout << "pose inside functor X :  Y :" <<  pose.pose.position.x <<" "<< pose.pose.position.y <<" "<<
            pose.pose.position.z <<" "<< pose.pose.orientation.x <<" "<< pose.pose.orientation.y <<" "<<
            pose.pose.orientation.z <<" "<< pose.pose.orientation.w <<" "<< std::endl;
            return (pose.pose.position.x - search_pose_.pose.position.x < 0.0001 && pose.pose.position.y -
            search_pose_.pose.position.y < 0.0001 &&
            pose.pose.position.z - search_pose_.pose.position.z < 0.0001 && pose.pose.orientation.x -
            search_pose_.pose.orientation.x < 0.0001 &&
            pose.pose.orientation.y - search_pose_.pose.orientation.y < 0.0001 && pose.pose.orientation.w - 
            search_pose_.pose.orientation.w < 0.0001 && pose.pose.orientation.z - search_pose_.pose.orientation.z < 0.0001);
        }
    };
};

#endif
