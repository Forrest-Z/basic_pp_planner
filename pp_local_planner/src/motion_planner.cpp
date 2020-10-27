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

#include "pp_local_planner/motion_planner.h"

namespace motion_planner
{
    MotionPlanner::MotionPlanner(tf2_ros::Buffer* tf, base_local_planner::LocalPlannerUtil* planner_util, double safe_factor, std::string
            motion_frame):tf_{tf}, planner_util_{planner_util}, motion_frame_{motion_frame}
    {
        safe_factor_ = safe_factor;

        //current costmap information 
        //need to check whether to delete this pointer or not.
        costmap = planner_util_->getCostmap();
        world_model = new base_local_planner::CostmapModel(*costmap);
    }
    MotionPlanner::MotionPlanner(){}

    MotionPlanner::~MotionPlanner(){}

    void MotionPlanner::clearVisitedPlan(const mpd::Plan::const_iterator upto_it)
    {
        global_plan_.erase(global_plan_.begin(), upto_it);
    }

    void MotionPlanner::clearVisitedPlan(int size)
    {
        if(!size == 0)
        {
            //ROS_INFO("clearing...");
            global_plan_.erase(global_plan_.begin(), global_plan_.begin() + size);
        }
    }

    bool MotionPlanner::constructMotionPlan(mpd::Plan& plan, const
            geometry_msgs::PoseStamped& global_pose, const geometry_msgs::Twist& robot_vel, std::vector<geometry_msgs::Point>
            footprint_spec, mpd::MotionPlan& motion_plan)
    {
        //no motion plan if robot is in obstacle.
        mpd::ObstacleInfo robot_in_obstacle;
        if(updateObstacleInfo(global_pose, footprint_spec, robot_in_obstacle))
        {
            ROS_WARN("robot hits obstacle");
            return false;
        }

        base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
        mpd::MotionPose mp;
        //getting the global plan.
        //plan.clear();
        getLocalPlan(global_pose, plan);

        //find a safe window to look forward on the path based on robot planner_parameters
        double vmax = limits.max_vel_x;
        double vmin = limits.min_vel_x;
        double max_acc = limits.acc_lim_x; 
        double lateral_acc = 3.5; //need to be parameterised.
        //min distance required for the robot to stop based on the robot base parameters.
        //double min_stop_dist = pow(robot_vel.linear.x,2) / (2 * max_acc);
        double min_stop_dist = pow(vmax, 2) / (2 * max_acc);
        //increasing the window size with an extra safety distance.
        double safe_distance = min_stop_dist + safe_factor_;
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
        for(plan_it = plan.begin(); plan_it != plan.end() - 1; plan_it++)
        {
            //ROS_WARN("constructing motion plan");
            mpd::CrossTrackInfo ct = crossTrackError(plan, global_pose);
            //ROS_INFO("cross track error : %f", std::get<1>(ct));
            //estimating the arc length. 
            arc_length += mpd::euclidean(*plan_it, pose_);
            pose_ = *plan_it;

            //generating motion plan up to safe distance.
            if(arc_length > safe_distance)
            {
                break;
            }
            mpd::ObstacleInfo obstacle_info;
            if(updateObstacleInfo((*plan_it), footprint_spec, obstacle_info))
            {
                mp.obstacle_info = obstacle_info;
                mp.curvature = INFINITY; //at obstacle position curvature is not calculated.
                mp.obstacle = true;
                mp.in_place = false;
                //if obstacle detected, trim out the motion plan to stop the robot at a safe distance from the robot.
                //double safe_length = arc_length - safe_factor_; 
                double obst_stop_dist = 1.5;
                if(arc_length < obst_stop_dist) 
                {
                    ROS_WARN("no room for a safe motion plan");
                    motion_plan.clear(); // clear plan to put no motion plan since there is no enough space to move.
                    mp.pose = *plan_it;
                    mp.arc_length = 0.0;
                    mp.twist_ref.linear.x = 0.0;
                    mp.twist_ref.angular.z = 0.0;
                    motion_plan.push_back(mp);
                }
                else 
                {
                    ROS_INFO("extra plan and setting zero vel at end");
                    trimMotionPlan(motion_plan, min_stop_dist);
                    motion_plan.back().twist_ref.linear.x = 0.0;
                    motion_plan.back().twist_ref.linear.y = 0.0;
                }
                break; 
            }
            else
            {
                mpd::MengerPoints path_points; 
                double path_curvature;
                double no_of_points = 10;
                if ((plan_it <= plan.begin() + no_of_points) || (plan_it >= plan.end() - no_of_points))
                {
                    path_curvature = 0.01; //not calculating curvature for initial and final poses in the plan.
                }
                else
                {
                    //curvature is calculated at a point using menger curvature.
                    path_points = std::make_tuple(*(std::prev(plan_it, no_of_points)), *plan_it, *(std::next(plan_it,
                                    no_of_points)));
                    path_curvature = pathCurvature(path_points);
                }
                double angular_vel;
                mp.pose = *plan_it;
                mp.obstacle_info = obstacle_info; // cost > 1
                mp.arc_length = arc_length;
                double yaw_dif;
                if(mpd::euclidean(*plan_it, *std::next(plan_it, 1)) < 0.001)
                {
                    int pose_count;
                    mp.pose = getInplacePose(plan, plan_it, pose_count);
                    if(inPlace(global_pose, mp.pose, limits.xy_goal_tolerance, limits.yaw_goal_tolerance,
                                yaw_dif))
                    {
                        mp.visited_count = pose_count;
                        mp.obstacle = false;
                        mp.in_place = true;
                        mp.twist_ref.linear.x = 0.0;
                        mp.twist_ref.angular.z = yaw_dif;
                        motion_plan.push_back(mp);
                        break;
                    }
                }
                else
                {
                    ROS_WARN("linear plan generating");
                    mp.obstacle = false;
                    mp.in_place = false;
                    auto goal_point = plan.at(plan.size() - 1);
                    if(isGoal(*plan_it, goal_point, limits.xy_goal_tolerance))
                    {
                        ROS_INFO("goal point X %f, Y %f robot point X %f Y %f", plan_it->pose.position.x,
                        plan_it->pose.position.y, global_pose.pose.position.x, global_pose.pose.position.y);
                        mp.twist_ref.linear.x = 0.0;
                        mp.twist_ref.angular.z = 0.0; 
                    }
                    else
                    {
                        path_curvature = (path_curvature <= 0.01) ? 1 : path_curvature;
                        //fix for zero velocity profile, if goal is at end of a cuve.
                        //use max(vmin, curv_vel) for reference velocity.
                        //this will keep the reference velocity minimum at min vel.
                        mp.twist_ref.linear.x = std::min(vmax, std::max(vmin, sqrt(lateral_acc/path_curvature)));
                        //ROS_INFO("curvature : %f", path_curvature);
                        //ROS_INFO("curvature velocity : %f", sqrt(lateral_acc/path_curvature));
                        //ROS_INFO("point velocity : %f", mp.twist_ref.linear.x);
                        mp.twist_ref.angular.z = 0.0; // this is updated by purepuresuit now.
                    }
                    motion_plan.push_back(mp);
                }
            }
        }
        return true;

    }

    geometry_msgs::PoseStamped MotionPlanner::getInplacePose(const mpd::Plan& plan, mpd::Plan::const_iterator it, int&
            pose_count)
    {
        geometry_msgs::PoseStamped in_place_pose;
        for(auto it_ = it; it_ != plan.end() - 1; it_++)
        {
            if(mpd::euclidean(*it_, *std::next(it_, 1)) < 0.001)
            {
                in_place_pose = *std::next(it_, 1);
            }
            else
            {
                break;
            }
            pose_count = it_ - it;
            //ROS_INFO("pose count %d", pose_count);
        }
        return in_place_pose;
    }

    bool MotionPlanner::getInstantaneousCommand(mpd::MotionPlan& mp, const geometry_msgs::PoseStamped&
            global_pose, const geometry_msgs::Twist& robot_vel, geometry_msgs::Twist& cmd_vel)
    {
        base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
        mpd::MotionPlan ramp_plan;
        rampMotionPlan(mp, global_pose, robot_vel, ramp_plan);
        std::vector<mpd::MotionPlan>::const_iterator mp_it;
        //ROS_INFO("ramp plan size %ld", ramp_plan.size());
        if(ramp_plan.size() == 0)
        {
            ROS_WARN("No valid motion plan returning");
            return false;
        }

        if(ramp_plan.at(0).obstacle)
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
            return true;
        }

        if(ramp_plan.at(0).in_place)
        {
            double angular_gain = 0.95;
            double angular_vel = ramp_plan.at(0).twist_ref.angular.z * angular_gain;
            int sign = mpd::sign(angular_vel);
            angular_vel = std::min(fabs(ramp_plan.at(0).twist_ref.angular.z * angular_gain),
                    limits.max_vel_theta);
            angular_vel *= sign;
            //int visited = closest_pose_it - ramp_plan.begin();
            if(ramp_plan.at(0).twist_ref.angular.z <= limits.yaw_goal_tolerance)
            {
                geometry_msgs::PoseStamped search_pose;
                //transformPose(global_plan_.back().header.frame_id, ramp_plan.at(0).pose, search_pose);
                //auto it = getPlanPoseIt(global_plan_, search_pose);
                //ROS_INFO("removing size after inplace %ld", it - global_plan_.begin());
                clearVisitedPlan(ramp_plan.at(0).visited_count); 
            }
            //ROS_WARN("plan is for inplace or obstacle, angular vel : %f", angular_vel);
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = angular_vel;
            return true;
        }
        else
        {
            mpd::MotionPlan::const_iterator closest_pose_it;
            double short_dis_to_line, a, b, c;
            getMinDistancePoseIt(ramp_plan, global_pose, closest_pose_it);
            if(getLinearEquation(closest_pose_it->pose, (std::next(closest_pose_it)->pose), a, b, c))
            {
                short_dis_to_line = getDisFromPointToLine(global_pose, a, b, c);
            }
            //distance from closest point to next reference point.
            /*double si = sqrt(pow(mpd::euclidean(closest_pose_it->pose, global_pose), 2) - pow(short_dis_to_line, 2));
            double segment_length = mpd::euclidean(closest_pose_it->pose, std::next(closest_pose_it, 1)->pose); 
            //to be tested //double segment_length = mpd::euclidean(closest_pose_it->pose, ramp_plan.back().pose); 
            double acc = (pow(std::next(closest_pose_it, 1)->twist_ref.linear.x, 2) - pow(closest_pose_it->twist_ref.linear.x, 2))
                / (2 * segment_length);
            //to be tested double acc = (pow(ramp_plan.back().twist_ref.linear.x, 2) - pow(closest_pose_it->twist_ref.linear.x, 2))
            ////    / (2 * segment_length);
            cmd_vel.linear.x = sqrt(2 * acc * si + pow(closest_pose_it->twist_ref.linear.x, 2));*/

            //robot dependent vel calculation.
            double ramp_segment = mpd::euclidean(ramp_plan.front().pose, ramp_plan.back().pose);
            double r_g_segment = mpd::euclidean(global_pose, ramp_plan.back().pose);
            double si = fabs(ramp_segment - r_g_segment);
            double acc = (pow(ramp_plan.back().twist_ref.linear.x, 2) - pow(robot_vel.linear.x, 2)) / (2 * r_g_segment);
            cmd_vel.linear.x = sqrt(2 * acc * si + pow(closest_pose_it->twist_ref.linear.x, 2));
            
            if(std::isnan(cmd_vel.linear.x))
            {
                ROS_WARN("linear velocity is giving nan");
                cmd_vel.linear.x = 0.0;
            }
            //intial vel to start.
            if((mpd::euclidean(global_pose, ramp_plan.front().pose) < 0.05) && cmd_vel.linear.x < 0.05)
            {
                cmd_vel.linear.x = 0.05;
            }
                
            ROS_INFO("closest point vel : %f", closest_pose_it->twist_ref.linear.x);
            ROS_INFO("closest point X : %f Y : %f", closest_pose_it->pose.pose.position.x,
            closest_pose_it->pose.pose.position.y);
            ROS_INFO("robot pose X : %f Y : %f", global_pose.pose.position.x, global_pose.pose.position.y);
            ROS_INFO("ramp end pose X : %f Y : %f", ramp_plan.back().pose.pose.position.x, ramp_plan.back().pose.pose.position.y);
            ROS_INFO("inst linear velocity : %f", cmd_vel.linear.x);
            cmd_vel.angular.z = 0.0;
            geometry_msgs::PoseStamped search_pose;
            //ROS_INFO("size %ld", ramp_plan.size());
            //ROS_INFO("clearing plan size %ld", closest_pose_it - ramp_plan.begin());
            clearVisitedPlan(closest_pose_it - ramp_plan.begin()); 
            //ROS_INFO("clearing visited plan");
            ROS_WARN("found valid vel");
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
                min_distance = origin_to_pose_dis;
            }
        }
    }

    double MotionPlanner::getDisFromPointToLine(const geometry_msgs::PoseStamped& pose, double a, double b, double c)
    {
        double distance = fabs(a * pose.pose.position.x + b * pose.pose.position.y + c) / sqrt(pow(a, 2) + pow(b, 2));
        return distance;
    }

    void MotionPlanner::getGlobalPlan(mpd::Plan& plan)
    {
        plan = global_plan_;
    }

    bool MotionPlanner::getLocalPlan(const geometry_msgs::PoseStamped& global_pose, mpd::Plan& local_plan)
    {
        mpd::Plan global_plan;
        getGlobalPlan(global_plan);
        return planner_util_->getLocalPlan(global_pose, global_plan_, local_plan);
    }

    mpd::PosePair MotionPlanner::getPlanExtendPosePair(const mpd::Plan& plan)
    {
        mpd::PosePair extension_pair;
        for(int i = plan.size() - 1; i >= 1; i--)
        {
            geometry_msgs::PoseStamped end_pose = plan.at(plan.size() - 1);
            geometry_msgs::PoseStamped start_pose = plan.at(i - 1); 
            if(mpd::euclidean(end_pose, start_pose) > 0.01)
            {
                extension_pair =  std::make_pair(start_pose, end_pose);
            }
        }
        return extension_pair;
    }

    mpd::Plan::const_iterator MotionPlanner::getPlanPoseIt(const mpd::Plan& plan, const geometry_msgs::PoseStamped&
            search_pose)
    {
        //ROS_INFO("search pose X %f Y %f", search_pose.pose.position.x, search_pose.pose.position.y);
        mpd::Plan::const_iterator it;
        //it = std::find_if(plan.begin(), plan.end(), [search_pose](const geometry_msgs::PoseStamped& pose){return pose ==
        //search_pose;});
        it = std::find_if(plan.begin(), plan.end(), mpd::PoseCompare(search_pose)); 
        //ROS_INFO("found pose X %f Y %f", it->pose.position.x, it->pose.position.y);
        return it;
    }

    void MotionPlanner::rampMotionPlan(mpd::MotionPlan& mpl, const geometry_msgs::PoseStamped& global_pose, const geometry_msgs::Twist& robot_vel,
            mpd::MotionPlan& ramp_plan)
    {
        if(mpl.size() == 1)
        {
            ramp_plan.push_back(mpl.at(0));
            return;
        }
        else
        {
            //search for min vel point in the plan.
            auto min_vel_mp = std::min_element(mpl.begin(), mpl.end(), [](const mpd::MotionPose& mp1, const
                        mpd::MotionPose& mp2) {return mp1.twist_ref.linear.x <= mp2.twist_ref.linear.x;});
            trimMotionPlan(mpl, min_vel_mp->arc_length);
            //ramp velocity plan.
            base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
            double min_vel_x = limits.min_vel_x;
            double v0 = std::max(min_vel_x, robot_vel.linear.x);
            //double v0 = robot_vel.linear.x;
            //ROS_INFO("v0 %f", v0);
            double lin_vel_final = min_vel_mp->twist_ref.linear.x;
            ROS_INFO("min vel %f", lin_vel_final);

            //to do bounding in both acc and dec
            //double acci = std::min(fabs(pow(lin_vel_final, 2) - pow(robot_vel.linear.x, 2) / (2 *
            //min_vel_mp->arc_length)), limits.acc_lim_x);
            double acci = (pow(lin_vel_final, 2) - pow(v0, 2)) / (2 * (min_vel_mp->arc_length - mpl.front().arc_length));

            geometry_msgs::Twist inst_vel;
            //generating velocity plan with ramp profile.
            for(auto it = mpl.begin(); it != mpl.end(); it++)
            {
                double si = it->arc_length;
                it->acc_lin = acci;
                it->twist_ref.linear.x = sqrt((2 * acci * si) + pow(v0, 2));
                //ramp reference velocity generating nan values.
                //making nan values to zero.
                //need to be verified this is a good fix.
                it->twist_ref.linear.x = (std::isnan(it->twist_ref.linear.x)) ? 0.0 : it->twist_ref.linear.x; 
                ramp_plan.push_back(*it);
            }
            for(auto it = ramp_plan.begin(); it != ramp_plan.end(); it++)
            {
                ROS_INFO("si : %f", it->arc_length);
                ROS_INFO("ac : %f", it->acc_lin);
                ROS_INFO("vl : %f", it->twist_ref.linear.x);
                ROS_INFO("X : %f Y : %f", it->pose.pose.position.x, it->pose.pose.position.y);
            }

        }

    }

    bool MotionPlanner::updateObstacleInfo(const geometry_msgs::PoseStamped& plan_pose, std::vector<geometry_msgs::Point>
            footprint_spec, mpd::ObstacleInfo& obstacle_info)
    {
        double x = plan_pose.pose.position.x;
        double y = plan_pose.pose.position.y;
        double theta = tf2::getYaw(plan_pose.pose.orientation);
        double footprint_cost = world_model->footprintCost(x, y, theta, footprint_spec);  	
        obstacle_info.cost = footprint_cost;
        if(footprint_cost < 0)
        {
            //ROS_INFO("obstacle");
            return true;
        }
        else
        {
            //ROS_INFO("no obstacle");
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
        interpolated_pose.pose.position.x = end.pose.position.x + unit_vector.getX() * scale;
        interpolated_pose.pose.position.y = end.pose.position.y + unit_vector.getY() * scale;
        return true;

    }


    double MotionPlanner::getPlaneDistance(const geometry_msgs::PoseStamped& pose_a, const geometry_msgs::PoseStamped& pose_b)
    {
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
        geometry_msgs::PoseStamped pose_a = std::get<0>(path_points); 
        geometry_msgs::PoseStamped pose_b = std::get<1>(path_points); 
        geometry_msgs::PoseStamped pose_c = std::get<2>(path_points);

        //menger curvature = 1/R = 4A/(|X-Y||Y-Z||Z-X|, X,Y,Z are given points.
        double A = fabs(pose_a.pose.position.x * (pose_b.pose.position.y - pose_c.pose.position.y) + pose_b.pose.position.x *
                (pose_c.pose.position.y - pose_a.pose.position.y) + pose_c.pose.position.x * (pose_a.pose.position.y - pose_b.pose.position.y))/2;

        double menger_curvature = (4 * A) / (mpd::euclidean(pose_a, pose_b) * mpd::euclidean(pose_b, pose_c) * 
                mpd::euclidean(pose_c, pose_a));
        return (A < 0.001) ? 1 : menger_curvature * 10; //scaling the curvature value for velocity calculation.
        //return menger_curvature * 10; //scaling the curvature value for velocity calculation.
    }

    bool MotionPlanner::inPlace(const geometry_msgs::PoseStamped& pose_a, const geometry_msgs::PoseStamped& pose_b, const double xy_goal_tolerance, const double yaw_goal_tolerance, double& yaw_dif)
    {
        /*if(!isGoal(pose, end_pose_, xy_goal_tolerance) && !isStart(pose, start_pose_, xy_goal_tolerance))
          {
          return false;
          }*/
        double path_a_yaw = tf2::getYaw(pose_a.pose.orientation);
        double path_b_yaw = tf2::getYaw(pose_b.pose.orientation);
        yaw_dif = angles::shortest_angular_distance(path_a_yaw, path_b_yaw);
        bool in_place = ((fabs(yaw_dif) > yaw_goal_tolerance)) ? true : false;
        return in_place;
    }


    bool MotionPlanner::isGoal(const geometry_msgs::PoseStamped& check_pose, const geometry_msgs::PoseStamped&
            end_pose, const double xy_goal_tolerance)
    {
        return (mpd::euclidean(check_pose, end_pose) < xy_goal_tolerance) ? true : false;
    }

    bool MotionPlanner::isGoalReached(const geometry_msgs::PoseStamped& robot_pose)
    {
        base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
        return (mpd::euclidean(robot_pose, end_pose_) < limits.xy_goal_tolerance) ? true : false;
    }

    bool MotionPlanner::isStart(const geometry_msgs::PoseStamped& check_pose, const geometry_msgs::PoseStamped&
            start_pose, const double xy_goal_tolerance)
    {
        return (mpd::euclidean(check_pose, start_pose) < xy_goal_tolerance) ? true : false;
    }

    bool MotionPlanner::setGlobalPlan(const mpd::Plan& orig_global_plan)
    {
        global_plan_ = orig_global_plan;
        //ROS_INFO("SBPL PLAN SIZE : %ld", orig_global_plan.size());
        if(!transformPose(motion_frame_, orig_global_plan.front(), start_pose_) && !transformPose(motion_frame_,
                    orig_global_plan.back(), end_pose_))
        {
            return false;
        }
        return true;
    }

    void MotionPlanner::trimMotionPlan(mpd::MotionPlan& motion_plan, double safe_arc_length)
    {
        std::vector<mpd::MotionPose>::const_iterator plan_it;
        for(plan_it = motion_plan.begin(); plan_it != motion_plan.end() - 1; plan_it++)
        {
            if((*plan_it).arc_length > safe_arc_length)
            {
                motion_plan.erase(std::next(plan_it, 1), motion_plan.end());
                break;
            }
        }
        //ROS_INFO("plan size after trim %ld", motion_plan.size()); 
    }

    bool MotionPlanner::transformPose(const std::string& global_frame, const
            geometry_msgs::PoseStamped& pose, geometry_msgs::PoseStamped& transformed_pose)
    {
        try{
            geometry_msgs::TransformStamped transform = tf_->lookupTransform(global_frame, ros::Time(),
                    pose.header.frame_id, pose.header.stamp, pose.header.frame_id, ros::Duration(0.5));

            tf2::doTransform(pose, transformed_pose, transform);
        }
        catch(tf2::LookupException& ex) {
            ROS_ERROR("No Transform available Error: %s\n", ex.what());
            return false;
        }
        catch(tf2::ConnectivityException& ex) {
            ROS_ERROR("Connectivity Error: %s\n", ex.what());
            return false;
        }
        catch(tf2::ExtrapolationException& ex) {
            ROS_ERROR("Extrapolation Error: %s\n", ex.what());
            return false;
        }
        return true;
    }

};

