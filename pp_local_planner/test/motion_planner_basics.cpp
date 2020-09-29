#include "motion_planner.h"
#include <gtest/gtest.h>

TEST(MotionPlannerMath, minPoseIt)
{
    motion_planner::MotionPlanner mplnr;
    mpd::MotionPlan mpl;
    mpd::MotionPose mp;
    for(int i = 0; i < 10; i++)
    {
        mp.pose.position.x = i;
        mp.pose.position.y = i;
        mp.arc_length = i;
        mpl.push_back(mp);
    }

    for(int i = 0; i < 5; i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = i;
        pose.pose.position.y = i;
        mpd::MotionPlan::const_iterator it;
        mplnr.getMinDistanceIt(mpl, pose, it);
        EXPECT_EQ(i, it->pose.position.x); 
    }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::initi(argc, argv);
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
    

