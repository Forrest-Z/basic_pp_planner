#include "pp_local_planner/motion_planner.h"

namespace motion_planner
{
    MotionPlanner::MotionPlanner(base_local_planner::LocalPlannerUtil* planner_util, double safe_factor): planner_util_{planner_util}
    {
        safe_factor_ = safe_factor;

        //current costmap information 
        //need to check whether to delete this pointer or not.
        costmap = planner_util_->getCostmap();

        //world map that we are using. Configured with costmap.
        base_local_planner::WorldModel* world_model = new base_local_planner::CostmapModel(*costmap);
    }
    MotionPlanner::MotionPlanner(){}
    MotionPlanner::~MotionPlanner()
    {
        //should properly delete world model
        /*if(world_model != NULL)
          { 
          delete world_model;
          }
          delete costmap;*/
    }

    bool MotionPlanner::constructMotionPlan(mpd::Plan& plan, const
            geometry_msgs::PoseStamped& global_pose, const geometry_msgs::Twist& robot_vel, std::vector<geometry_msgs::Point>
            footprint_spec, mpd::MotionPlan& motion_plan)
    {
        //no motion plan if robot is in obstacle.
        mpd::ObstacleInfo robot_in_obstacle;
        if(updateObstacleInfo(global_pose, footprint_spec, robot_in_obstacle))
        {
            //ROS_WARN("robot hits obstacle");
            return false;
        }
        updatePlan(plan, global_pose);
        mpd::MotionPose mp;
        //find a safe window to look forward on the path based on robot planner_parameters
        base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
        double vmax = limits.max_vel_x;
        double max_acc = limits.acc_lim_x; 
        double lateral_acc = 1.5; //need to be parameterised.
        //min distance required for the robot to stop based on the robot base parameters.
        double min_stop_dist = pow(robot_vel.linear.x,2) / (2 * max_acc);
        //increasing the window size with an extra safety distance.
        double safe_distance = min_stop_dist + safe_factor_;
        //ROS_WARN("safe_distance : %f", safe_distance);
        double arc_length = 0.0;

        geometry_msgs::PoseStamped pose_ = plan.at(0);
        //container holding safe motion plan, this plan will be up to safe factor distance away
        //obstacle in the presence of an obstacle.
        mpd::MotionPlan safe_motion_plan;

        //intial motion pose will be of zero values with position will be at initial plan pose
        mpd::MotionPose initial_safe_motion_pose;
        //initial_safe_motion_pose.pose = pose_;
        //safe_motion_plan.push_back(initial_safe_motion_pose);
        std::vector<geometry_msgs::PoseStamped>::const_iterator plan_it;

        //iterating over the global plan with in the safe window size to update velocity references.
        for(plan_it = plan.begin(); plan_it != plan.end(); plan_it++)
        {
            //estimating the arc length. 
            arc_length += mpd::euclidean(*plan_it, pose_);
            pose_ = *plan_it;

            //generating motion plan up to safe distance.
            if(arc_length > safe_distance)
            {
                //ROS_WARN("arc leght exceeds the safe distance");
                break;
            }
            mpd::ObstacleInfo obstacle_info;
            if(updateObstacleInfo((*plan_it), footprint_spec, obstacle_info))
            {
                //if obstacle detected trim out the motion plan to stop the robot at a safe distance from the robot.
                double safe_length = arc_length - safe_factor_; 
                if(arc_length > safe_length)
                {
                    //ROS_WARN("stopping motion planner, obstacle in global path.");
                    trimMotionPlan(motion_plan, safe_length);
                }
                else
                {
                    //ROS_WARN("no room for a safe motion plan");
                    motion_plan.clear(); // clear plan to put no motion plan since there is no enough space to move.
                }
                //update the obstacle informations.
                mp.pose = *plan_it;
                mp.obstacle_info = obstacle_info;
                mp.curvature = INFINITY; //at obstacle position curvature is not calculated.
                mp.arc_length = arc_length;
                mp.in_place = false;
                mp.twist_ref.linear.x = 0.0;
                mp.twist_ref.angular.z = 0.0;
                motion_plan.push_back(mp);
                break; 
            }
            else
            {
                mpd::MengerPoints path_points; 
                double path_curvature;
                if (plan_it == plan.begin() || plan_it == plan.end())
                {
                    //ROS_WARN("fixed curvature");
                    path_curvature = 0.01; //not calculating curvature for initial and final poses in the plan.
                }
                else
                {
                    //ROS_WARN("calculating menger curvature");
                    //curvature is calculated at a point using menger curvature.
                    path_points = std::make_tuple(*(std::prev(plan_it, 1)), *plan_it, *(std::next(plan_it, 1)));
                    path_curvature = pathCurvature(path_points);
                }
                double angular_vel;
                mp.pose = *plan_it;
                mp.obstacle_info = obstacle_info; // cost > 1
                mp.arc_length = arc_length;
                double yaw_dif;
                /*if(inPlace(global_pose, *plan_it, limits.xy_goal_tolerance, limits.yaw_goal_tolerance, yaw_dif))
                {
                    ROS_WARN("in-place required");
                    mp.in_place = true;
                    mp.twist_ref.linear.x = 0.0;
                    mp.twist_ref.angular.z = yaw_dif;
                    motion_plan.push_back(mp);
                    break;
                }
                else
                {*/
                    if(isGoal(global_pose, plan.back(), limits.xy_goal_tolerance, limits.yaw_goal_tolerance))
                    {
                        mp.twist_ref.linear.x = 0.0;
                        mp.twist_ref.angular.z = 0.0; 
                    }
                    else
                    {
                        //ROS_WARN("calculating forward velocity based path curvature");
                        mp.twist_ref.linear.x = std::min(vmax, sqrt(lateral_acc/path_curvature));
                        ROS_WARN("curvature : %f", path_curvature);
                        ROS_WARN("curvature velocity : %f", sqrt(lateral_acc/path_curvature));
                        ROS_WARN("max velocity : %f", vmax);
                        mp.twist_ref.angular.z = 0.0; // this is updated by purepuresuit now.
                    }
                    motion_plan.push_back(mp);
                //}
            }
        }
        return true;

    }

    bool MotionPlanner::getInstantaneousCommand(mpd::MotionPlan& mp, const geometry_msgs::PoseStamped&
            global_pose, const geometry_msgs::Twist& robot_vel, geometry_msgs::Twist& cmd_vel)
    {
        base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
        mpd::MotionPlan ramp_plan;
        rampMotionPlan(mp, global_pose, robot_vel, ramp_plan);
        //ROS_WARN("ramp plan length : %f", ramp_plan.back().arc_length);
        std::vector<mpd::MotionPlan>::const_iterator mp_it;
        if(ramp_plan.size() == 0)
        {
            ROS_WARN("No valid motion plan returning");
            return false;
        }
        else if(ramp_plan.size() == 1)
        {
            double angular_vel = 0.0;
            if(ramp_plan.at(0).in_place)
            {
                double angular_gain = 0.8;
                angular_vel = std::min(std::max(fabs(ramp_plan.at(0).twist_ref.angular.z * angular_gain), limits.min_vel_theta), limits.max_vel_theta);
            }
            ROS_WARN("plan is for inplace or obstacle instantaneous value");
            cmd_vel = ramp_plan.at(0).twist_ref;
            cmd_vel.angular.z = angular_vel;
            return true;
        }
        else
        {
            mpd::MotionPlan::const_iterator closest_pose_it;
            double short_dis_to_line, a, b, c;
            getMinDistancePoseIt(ramp_plan, global_pose, closest_pose_it);
            ROS_WARN("closest pose X : %f Y : %f", closest_pose_it->pose.pose.position.x,
            closest_pose_it->pose.pose.position.y);
            ROS_WARN("robot pose X : %f Y : %f", global_pose.pose.position.x,
            global_pose.pose.position.y);
            //ROS_WARN("min pose arc length : %f", closest_pose_it->arc_length);
            //ROS_WARN("next to min pose arc length : %f", std::next(closest_pose_it)->arc_length);
            if(getLinearEquation(closest_pose_it->pose, (std::next(closest_pose_it)->pose), a, b, c))
            {
                short_dis_to_line = getDisFromPointToLine(global_pose, a, b, c);
            }
            //distance from closest point to next reference point.
            double si = sqrt(pow(mpd::euclidean(closest_pose_it->pose, global_pose), 2) - pow(short_dis_to_line, 2));
            //ROS_WARN("karnam : %f", mpd::euclidean(closest_pose_it->pose, global_pose));
            //ROS_WARN("lambam, : %f", short_dis_to_line);
            //ROS_WARN("si : %f", si);
            double segment_length = mpd::euclidean(closest_pose_it->pose, std::next(closest_pose_it, 1)->pose); 
            double acc = (pow(std::next(closest_pose_it)->twist_ref.linear.x, 2) - pow(closest_pose_it->twist_ref.linear.x, 2))
            / (2 * segment_length);
            //ROS_WARN("acc : %f", acc);
            //ROS_WARN("vo : %f", closest_pose_it->twist_ref.linear.x);
            cmd_vel.linear.x = sqrt(2 * acc * si + pow(closest_pose_it->twist_ref.linear.x, 2));
            cmd_vel.angular.z = 0.0;
            return true;
        }
    }
    bool MotionPlanner::getLinearEquation(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& end,
            double& a, double& b, double& c)
    {
        if(mpd::euclidean(start, end) < 0.01)
        {
            return false;
        }
        //linear equation ax + by + c = 0
        // given two points p1(x1,y1), p2(x2,y2): a = y2 - y1, b = -1 * (x2 - x1), c = -1 * (y2 -y1)*x1 + (x2 - x1)*y1
        //ROS_WARN("Linear Equation Calculation");
        a = end.pose.position.y - start.pose.position.y;
        b = (-1) * (end.pose.position.x - start.pose.position.x);
        c = (-1) * (end.pose.position.y - start.pose.position.y) * start.pose.position.x + (end.pose.position.x -
        start.pose.position.x) * start.pose.position.y;
        return true;
    }


    void MotionPlanner::getMinDistancePoseIt(const mpd::MotionPlan& search_plan, const geometry_msgs::PoseStamped& origin,
            mpd::MotionPlan::const_iterator& it)
    {
        double search_radius = 4.0;
        double min_distance = 1000;
        for(auto it_ = search_plan.begin(); it_ != search_plan.end(); it_++)
        {
            if(it_->arc_length > search_radius)
            {
                break;
            }
            double origin_to_pose_dis = getPlaneDistance(origin, it_->pose);
            if(origin_to_pose_dis < min_distance)
            {
                it = it_;
                //ROS_WARN("min dis pose X : %f Y : %f", it->pose.pose.position.x, it->pose.pose.position.y);
                min_distance = origin_to_pose_dis;
            }
        }
    }
    
    double MotionPlanner::getDisFromPointToLine(const geometry_msgs::PoseStamped& pose, double a, double b, double c)
    {
        double distance = fabs(a * pose.pose.position.x + b * pose.pose.position.y + c) / sqrt(pow(a, 2) + pow(b, 2));
        return distance;
    }

    void MotionPlanner::getMinForwardVelPose(mpd::MotionPlan& mpl, const geometry_msgs::PoseStamped& global_pose,
    mpd::MotionPlan::iterator it)
    {
        //for
    }

    void MotionPlanner::rampMotionPlan(mpd::MotionPlan& mpl, const geometry_msgs::PoseStamped& global_pose, const geometry_msgs::Twist& robot_vel,
            mpd::MotionPlan& ramp_plan)
    {
        if(mpl.size() == 1)
        {
            ROS_WARN("motion plan size one no ramp plan required");
            ramp_plan.push_back(mpl.at(0));
            return;
        }
        else
        {
            //ROS_WARN("ramp plan is calculating");
            //search for min vel point in the plan.
            auto min_vel_mp = std::min_element(mpl.begin(), mpl.end(), [](const mpd::MotionPose& mp1, const
                        mpd::MotionPose& mp2) {return mp1.twist_ref.linear.x <= mp2.twist_ref.linear.x;});
            trimMotionPlan(mpl, min_vel_mp->arc_length);
            //ramp velocity plan.
            base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
            double min_vel_x = limits.min_vel_x;
            double v0 = std::max(min_vel_x, robot_vel.linear.x);
            double lin_vel_final = min_vel_mp->twist_ref.linear.x;

            //to do bounding in both acc and dec
            //double acci = std::min(fabs(pow(lin_vel_final, 2) - pow(robot_vel.linear.x, 2) / (2 *
                        //min_vel_mp->arc_length)), limits.acc_lim_x);
            double acci = (pow(lin_vel_final, 2) - pow(v0, 2)) / (2 * min_vel_mp->arc_length);

            geometry_msgs::Twist inst_vel;
            //generating velocity plan with ramp profile.
            for(auto it = mpl.begin(); it != mpl.end(); it++)
            {
                // Vi = sqrt(2 * a * Si + Vo^2)
                double si = it->arc_length;
                it->acc_lin = acci;
                it->twist_ref.linear.x = sqrt((2 * acci * si) + pow(v0, 2));
                ramp_plan.push_back(*it);
            }

        }

    }

    /*double disFromPointToLine(const geometry_msgs::PoseStamped& pose, double a, double b, double c)
    {
    }*/

    bool MotionPlanner::updateObstacleInfo(const geometry_msgs::PoseStamped& plan_pose, std::vector<geometry_msgs::Point>
            footprint_spec, mpd::ObstacleInfo& obstacle_info)
    {

        double x = plan_pose.pose.position.x;
        double y = plan_pose.pose.position.y;
        double theta = tf2::getYaw(plan_pose.pose.orientation);
        double footprint_cost = 0.0;
        //double footprint_cost = world_model->footprintCost(x, y, theta, footprint_spec);  	
        //obstacle_info.obstacle_pose = plan_pose;
        obstacle_info.cost = footprint_cost;
        if(footprint_cost < 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    mpd::CrossTrackInfo MotionPlanner::crossTrackError(const mpd::Plan& plan, const geometry_msgs::PoseStamped& global_pose)
    {
        //to be updated to point line base distance.
        double min_dis = pow(10, 5);
        mpd::CrossTrackInfo cross_track_info = std::make_pair(plan.at(0), min_dis); //setting up the pos at start of global 
        //plan dist as 10000.
        for(auto plan_it = plan.begin(); plan_it != plan.end(); plan_it++)
        {
            if(mpd::euclidean(global_pose, *plan_it) > 3.0) //limiting search 3m from robot in the global plan.
            {
                break;
            }
            double robot_shift = getPlaneDistance(global_pose, *plan_it); 
            cross_track_info = (robot_shift < std::get<1>(cross_track_info)) ? std::make_pair(*plan_it, robot_shift) :
                cross_track_info;
        }
        return cross_track_info; 
    }

    bool MotionPlanner::linInterpolatedPose(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped&
            end, const geometry_msgs::PoseStamped& global_pose, const double scale, geometry_msgs::PoseStamped& interpolated_pose)
    {
        tf2::Vector3 unit_vector; 
        getUnitVector(start, end, unit_vector);
        interpolated_pose.pose.position.x = start.pose.position.x + unit_vector.getX() * scale;
        interpolated_pose.pose.position.y = start.pose.position.y + unit_vector.getY() * scale;

        //ROS_WARN("TEST start x %f y %f", interpolated_pose.pose.position.x, interpolated_pose.pose.position.y);
        return true;

    }


    double MotionPlanner::getPlaneDistance(const geometry_msgs::PoseStamped& pose_a, const geometry_msgs::PoseStamped& pose_b)
    {
        //ROS_WARN("point a : X : %f Y : %f ", pose_a.pose.position.x, pose_a.pose.position.y);
        //ROS_WARN("point b : X : %f Y : %f ", pose_b.pose.position.b, pose_b.pose.position.y);
        tf2::Vector3 vector_a(pose_a.pose.position.x, pose_a.pose.position.y, pose_a.pose.position.z);
        vector_a.setZ(0);
        tf2::Vector3 vector_b(pose_b.pose.position.x, pose_b.pose.position.y, pose_b.pose.position.z);
        vector_a.setZ(0);
        double distance = tf2::tf2Distance(vector_b, vector_a); 
        return distance;
    }


    bool MotionPlanner::getUnitVector(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& end, tf2::Vector3& unit_vector)
    {
        tf2::Vector3 vector(end.pose.position.x - start.pose.position.x, end.pose.position.y - start.pose.position.y, 0.0);
        unit_vector = vector.normalize();
    }

    double MotionPlanner::pathCurvature(const mpd::MengerPoints& path_points)
    {
        //ROS_WARN("check");
        geometry_msgs::PoseStamped pose_a = std::get<0>(path_points); 
        geometry_msgs::PoseStamped pose_b = std::get<1>(path_points); 
        geometry_msgs::PoseStamped pose_c = std::get<2>(path_points);

        //menger curvature = 1/R = 4A/(|X-Y||Y-Z||Z-X|, X,Y,Z are given points.
        double A = fabs(pose_a.pose.position.x * (pose_b.pose.position.y - pose_c.pose.position.y) + pose_b.pose.position.x *
                (pose_c.pose.position.y - pose_a.pose.position.y) + pose_c.pose.position.x * (pose_a.pose.position.y - pose_b.pose.position.y))/2;

        double menger_curvature = (4 * A) / (mpd::euclidean(pose_a, pose_b) * mpd::euclidean(pose_b, pose_c) * 
                mpd::euclidean(pose_c, pose_a));
        return menger_curvature * 10; //scaling the curvature value for velocity calculation.
        //ROS_WARN("menger curvature : %f", menger_curvature);
    }

    bool MotionPlanner::inPlace(const geometry_msgs::PoseStamped& initial_pose, const geometry_msgs::PoseStamped&
            final_pose, const double xy_goal_tolerance, const double yaw_goal_tolerance, double& yaw_dif)
    {
        if(!isGoal(final_pose, initial_pose, xy_goal_tolerance,  yaw_goal_tolerance))
        {
            return false;
        }
        double initial_yaw = tf2::getYaw(initial_pose.pose.orientation);
        double final_yaw = tf2::getYaw(final_pose.pose.orientation);
        yaw_dif = angles::shortest_angular_distance(initial_yaw, final_yaw);
        //ROS_WARN("yaw_dif : %f", yaw_dif);
        bool in_place = (fabs(yaw_dif) > yaw_goal_tolerance) ? true : false;
        return in_place;
    }


    bool MotionPlanner::isGoal(const geometry_msgs::PoseStamped& goal_pose, const geometry_msgs::PoseStamped&
            global_pose, const double xy_goal_tolerance, const double yaw_goal_tolerance)
    {
        if(mpd::euclidean(goal_pose, global_pose) < xy_goal_tolerance)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void MotionPlanner::trimMotionPlan(mpd::MotionPlan& motion_plan, double safe_arc_length)
    {
        std::vector<mpd::MotionPose>::const_iterator plan_it;
        for(plan_it = motion_plan.begin(); plan_it != motion_plan.end(); plan_it++)
        {
            if((*plan_it).arc_length > safe_arc_length)
            {
                motion_plan.erase(std::next(plan_it, 1), motion_plan.end());
                break;
            }
        }
    }

    void MotionPlanner::updatePlan(mpd::Plan& global_plan, const geometry_msgs::PoseStamped& global_pose)
    {
        ROS_WARN("updating global plan");
        double search_radius = 6.0;
        double min_distance = 1000;
        double arc_length = 0.0;
        geometry_msgs::PoseStamped pose_;
        mpd::Plan::iterator it;
        for(auto it_ = global_plan.begin(); it_ != global_plan.end(); it_++)
        {
            arc_length += mpd::euclidean(pose_, *it_);
            pose_ = *it_;
            if(arc_length > search_radius)
            {
                break;
            }
            ROS_WARN("check it ");
            double origin_to_pose_dis = getPlaneDistance(global_pose, *it_);
            if(origin_to_pose_dis < min_distance)
            {
                it = it_;
                ROS_WARN("update plan min dis pose X : %f Y : %f", it_->pose.position.x, it_->pose.position.y);
                min_distance = origin_to_pose_dis;
            }
        }
        global_plan.erase(global_plan.begin(), it);
        ROS_WARN("updated global plan");
    }

};

