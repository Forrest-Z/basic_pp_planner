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
    MotionPlanner::MotionPlanner(tf::TransformListener* tf, base_local_planner::LocalPlannerUtil* planner_util, double safe_factor, std::string
            motion_frame):tf_{tf}, planner_util_{planner_util}, motion_frame_{motion_frame}
    {
	initializeConfig();
        safe_factor_ = safe_factor;

        //current costmap information 
        //need to check whether to delete this pointer or not.
        costmap = planner_util_->getCostmap();
        world_model = new base_local_planner::CostmapModel(*costmap);
        debug = true;
        warning_field_status = false;
        pause_motion = false;
        ros::NodeHandle nh;
        ref_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("ref_pose", 1);
        closest_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("closest_pose", 1);
        obstacle_info_pub = nh.advertise<std_msgs::Bool>("obstacle_info", 1);
        warning_field_server = nh.advertiseService("warning_field_status", &MotionPlanner::warningFieldCb, this);
        nav_pause_server = nh.advertiseService("nav_pause", &MotionPlanner::navPauseCb, this);
    }
    MotionPlanner::MotionPlanner(){}

    MotionPlanner::~MotionPlanner(){}

    void MotionPlanner::boundControlInput(double &v, double &w)
    {
        //bounding the w values based on the configuration only wmax is considered.
        //considering wmin can create issue in tracking.
        //w = (fabs(w) < config.wmin) ? 0.0 : mpd::sign(w) * std::min(config.wmax, fabs(w));
        w = mpd::sign(w) * std::min(config.wmax, fabs(w));
        double linear_acc = (v - last_control_v) / fabs(v - last_control_v) * config.acc_x;
        double angular_acc = (w - last_control_w) / fabs(w - last_control_w) * config.acc_w;
        double delta_v = linear_acc * 0.2; //assuming running at 5hz
        double delta_w = angular_acc * 0.2;
        //bounding and profiling the v based on configuration.
        v = (linear_acc > 0.0) ? std::min(config.vmax, std::min(v, last_control_v + delta_v)) : std::max(0.0, std::max(v, last_control_v + delta_v));
        int sign = mpd::sign(last_control_w + delta_w);
        //profiling w based on the configuration.
        w = (angular_acc > 0.0) ?  std::min(w, last_control_w + delta_w) : std::max(w, last_control_w + delta_w);
        last_control_v = v;
        last_control_w = w;

    }

    void MotionPlanner::clearVisitedPlan(const mpd::Plan::const_iterator upto_it)
    {
        global_plan_.erase(global_plan_.begin(), upto_it);
    }

    void MotionPlanner::clearVisitedPlan(int size)
    {
        if(!size == 0 && !(size > global_plan_.size()))
        {
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
            ROS_ERROR("ROBOT HITS OBSTACLE");
            return false;
        }

        ROS_WARN("LOADED %d", loaded);
        base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
        mpd::MotionPose mp;
        //getting the global plan.
        getLocalPlan(global_pose, plan);
        //find a safe window to look forward on the path based on robot planner_parameters
        double vmax = config.vmax;
        double vmin = config.vmin;
        double max_acc = config.acc_x; 
        double lateral_acc = config.lat_acc; //need to be parameterised.
        xy_goal_tolerance_ = config.xy_goal_tolerance;
        //min distance required for the robot to stop based on the robot base parameters.
        //double min_stop_dist = pow(robot_vel.linear.x,2) / (2 * max_acc);
        double min_stop_dist = pow(vmax, 2) / (2 * max_acc);
        //increasing the window size with an extra safety distance.
        //double safe_distance = min_stop_dist  + safe_factor_; //safe factor should be less than half of local costmap.
        double safe_distance = min_stop_dist  + config.obst_stop_dist; //safe factor should be less than half of local costmap.
        //ROS_WARN("SAFE_DISTANCE %f", safe_distance);
        double arc_length = 0.0;

        geometry_msgs::PoseStamped pose_ = plan.at(0);
        //container holding safe motion plan, this plan will be up to safe factor distance away
        //obstacle in the presence of an obstacle.
        mpd::MotionPlan safe_motion_plan;

        //intial motion pose will be of zero values with position will be at initial plan pose
        mpd::MotionPose initial_safe_motion_pose;
        std::vector<geometry_msgs::PoseStamped>::const_iterator plan_it;
        mpd::CrossTrackInfo ct = crossTrackError(plan, global_pose);
        double cross_track_warn = config.cross_track_warn;
        double cross_track_stop = config.cross_track_error;
        critical_error = false;

        //vmax = (std::get<1>(ct) >= cross_track_control) ? limits.min_vel_x : limits.max_vel_x; //not working this mehtod.
        
        if(warning_field_status)
        {
            vmax = config.vmin;
        }

        if(std::get<1>(ct) >= cross_track_stop)
        {
            ROS_WARN("CROSS TRACK ERROR CRITICAL STOPPING");
            critical_error = true; //using this variable stop producing velocity for this plan.
            mp.error = true;
            motion_plan.push_back(mp);
            return true;
        }

        else if(std::get<1>(ct) >= cross_track_warn)
        {
            vmax = config.vmin;
            ROS_WARN("CROSS TRACK ERROR WARN REDUCING SPEED");
        }

        //position goal tolerance increasing on cross_track_warn and also robot reaches the
        //goal position with the configured tolerance.
        if(std::get<1>(ct) >= cross_track_warn || getGoalDistance(global_pose) < xy_goal_tolerance_)
        {
            xy_goal_tolerance_ = max_xy_tolerance;
        }

        //iterating over the global plan with in the safe window size to update velocity references.
        //changed iterating upto last point insted of second last point, to avoid referece point not
        //reaching near goal and overshooting (on some mprim files which generate plan with more dista
        //nce between two poses.
        //here use of std::next(plan_it, 1) might bring unpredicted behaviour, in the inplace checking
        //case.
        for(plan_it = plan.begin(); plan_it != plan.end(); plan_it++)
        {
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
                mp.error = false;
                //if obstacle detected, trim out the motion plan to stop the robot at a safe distance from the robot.
                double obst_stop_dist = config.obst_stop_dist;
                if(arc_length >= obst_stop_dist) 
                {
                    trimMotionPlan(motion_plan, min_stop_dist);
                    motion_plan.back().twist_ref.linear.x = 0.0;
                    motion_plan.back().twist_ref.linear.y = 0.0;
                }
                else 
                {
                    ROS_WARN("CLOSE TO OBSTACLE, APPLY SAFE STOP");
                    motion_plan.clear(); // clear plan to put no motion plan since there is no enough space to move.
                    mp.pose = *plan_it;
                    mp.arc_length = 0.0;
                    mp.twist_ref.linear.x = 0.0;
                    mp.twist_ref.angular.z = 0.0;
                    motion_plan.push_back(mp);
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
                mp.error = false;
                double yaw_dif;

                //fix for sudden acceleration after inplace and also unnecessary inplace.
                int pose_count = 0;
                geometry_msgs::PoseStamped temp_pose = getInplacePose(plan, plan_it, pose_count);
                //ref_pose_pub.publish(temp_pose);
                //should improve logic to avoid std::next(plan_it, 1) < 0.001 check
                //since we are iterating over entire plan. At plan end std::next(plan, 1) may create
                //issues.
                if(inPlace(global_pose, temp_pose, xy_goal_tolerance_, config.yaw_goal_tolerance, yaw_dif) && (mpd::euclidean(*plan_it, *std::next(plan_it, 1)) < 0.001))
                {
                    mp.pose = temp_pose;
                    mp.visited_count = pose_count;
                    mp.obstacle = false;
                    mp.in_place = true;
                    mp.twist_ref.linear.x = 0.0;
                    mp.twist_ref.angular.z = yaw_dif;
                    motion_plan.push_back(mp);
                    break;
                }
                else
                {
                    mp.obstacle = false;
                    mp.in_place = false;
                    auto goal_point = plan.at(plan.size() - 1);
                    if(isGoal(*plan_it, goal_point, xy_goal_tolerance_))
                    {
                        mp.twist_ref.linear.x = 0.0;
                        mp.twist_ref.angular.z = 0.0;
                        motion_plan.push_back(mp);
                        //break;
                    }
                    else
                    {
                        path_curvature = (path_curvature <= 0.01) ? 1 : path_curvature;
                        //fix for zero velocity profile, if goal is at end of a cuve.
                        //use max(vmin, curv_vel) for reference velocity.
                        //this will keep the reference velocity minimum at min vel.
                        mp.twist_ref.linear.x = std::min(vmax, std::max(vmin, sqrt(lateral_acc/path_curvature)));
                        mp.twist_ref.angular.z = 0.0; // this is updated by purepuresuit now.
                        motion_plan.push_back(mp);
                    }
                }
                    trimMotionPlan(motion_plan, min_stop_dist);
            }
        }
        //publishing obstacle information
        std_msgs::Bool is_obstacle;
        is_obstacle.data = motion_plan.back().obstacle;
        obstacle_info_pub.publish(is_obstacle);

        //pause the robot
        if(pause_motion)
        {
           motion_plan.clear(); // clear plan to put no motion plan since motion needs to be paused.
           mpd::MotionPose mp;
           mp.pause = true;
           motion_plan.push_back(mp);
        }
        
        //fix for proper lookahead on track before an inplace.
        //local plan limited upto obtacle/inplace/goal/safe_distance.
        //trimming part beyond the above scenarios of the local transformed plan.
        plan.erase(plan_it, plan.end());
        return true;
    }

    geometry_msgs::PoseStamped MotionPlanner::getInplacePose(const mpd::Plan& plan, mpd::Plan::const_iterator it, int& pose_count)
    {
        /*
         * TODO
         * avoid return uninitialized pose.
         * restrict search upto safe distance than entire local plan, since both 
         * are different.
         * improve logic for this method to find a possible inplace turn ahead.
         */

        geometry_msgs::PoseStamped in_place_pose;
        for(auto it_ = it; it_ != plan.end() - 1; it_++)
        {
            if(mpd::euclidean(*it_, *std::next(it_, 1)) < 0.001)
            {
                in_place_pose = *std::next(it_, 1);
            }
            else
            {
                return in_place_pose; //change 1.
            }
            pose_count = it_ - it;
        }
    }

    bool MotionPlanner::getInstantaneousCommand(mpd::MotionPlan& ramp_plan, const geometry_msgs::PoseStamped&
            global_pose, const geometry_msgs::Twist& robot_vel, const std::vector<geometry_msgs::Point>& footprint_spec, geometry_msgs::Twist& cmd_vel)
    {
        base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
        double min_vel = config.vmin;
        std::vector<mpd::MotionPlan>::const_iterator mp_it;
        if(ramp_plan.size() == 0)
        {
            ROS_WARN("NO VALID MOTION PLAN");
            return false;
        }

        if(ramp_plan.at(0).error)
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
            return true;
        }


        if(ramp_plan.at(0).obstacle)
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
            return true;
        }

        geometry_msgs::PoseStamped goal_pose;
        tf::Stamped<tf::Pose> tf_goal_pose;
        planner_util_->getGoal(tf_goal_pose);
        poseStampedTFToMsg(tf_goal_pose, goal_pose);
        double position = mpd::euclidean(global_pose, goal_pose);

        mpd::Plan::const_iterator closest_global_pose_it;
        mpd::MotionPlan::const_iterator closest_pose_it;
        double short_dis_to_line, a, b, c;
        getMinDistancePoseIt(ramp_plan, global_pose, closest_pose_it);
        if(getLinearEquation(closest_pose_it->pose, (std::next(closest_pose_it)->pose), a, b, c))
        {
            short_dis_to_line = getDisFromPointToLine(global_pose, a, b, c);
        }
        double min_stop_dist = pow(config.vmax, 2) / (2 * config.acc_x) ;//+ mpd::euclidean(ramp_plan.begin(), closest_pose_it->pose)

        /*std::vector<mpd::MotionPose>::iterator vel_point_it;
        for(vel_point_it = ramp_plan.begin() + (closest_pose_it - ramp_plan.begin()); vel_point_it != ramp_plan.end(); vel_point_it++)
        {
            if(mpd::euclidean(global_pose, vel_point_it->pose) >= min_stop_dist)
            {
                break;
            }
        }*/

        auto min_vel_mp = std::min_element(ramp_plan.begin(), ramp_plan.end(), [](const mpd::MotionPose& mp1, const mpd::MotionPose& mp2) {return mp1.twist_ref.linear.x <= mp2.twist_ref.linear.x;});
        

        double euclid_to_minpose = mpd::euclidean(global_pose, min_vel_mp->pose);
        ref_pose_ = min_vel_mp->pose;
        //ref_pose_pub.publish(ref_pose_);

        mpd::MotionPose inplace_mp = ramp_plan.at(0);

        if(min_vel_mp->in_place && euclid_to_minpose < xy_goal_tolerance_)
        {
            inplace_mp = *min_vel_mp;
        }

        if(inplace_mp.in_place || position <= xy_goal_tolerance_)
        {
            double angular_gain = config.acc_w;
            double goal_yaw = tf2::getYaw(goal_pose.pose.orientation);
            double robot_yaw = tf2::getYaw(global_pose.pose.orientation);
            double yaw = angles::shortest_angular_distance(robot_yaw, goal_yaw);
            //temp fix for in_place false at goal position
            //added an extra orientation accuracy check
            //generate angular velocity.
            double angular_diff = (position <= xy_goal_tolerance_) ? yaw : inplace_mp.twist_ref.angular.z;
            int sign = mpd::sign(angular_diff);
            double angular_vel = (std::min(std::max(config.wmin, fabs(angular_diff)), config.wmax)) * sign * angular_gain;
            angular_vel = (fabs(angular_diff) <= config.yaw_goal_tolerance) ? 0.0 : angular_vel;
            if(inplace_mp.twist_ref.angular.z <= config.yaw_goal_tolerance && inplace_mp.in_place)
            {
                geometry_msgs::PoseStamped search_pose;
                //clearVisitedPlan(ramp_plan.at(0).visited_count); 
                clearVisitedPlan(inplace_mp.visited_count); 
            }
            else
            {
                //setting inplace true for final goal orientation correction
                ramp_plan.at(0).in_place = true;
            }
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = angular_vel;
            return true;
        }
        else
        {

            trimMotionPlan(ramp_plan, min_vel_mp->arc_length );

            double final_vel_x = config.vmin;
            if((position <= xy_goal_tolerance_ || euclid_to_minpose <= 0.1) && !plan_executed)
            {
                final_vel_x = 0.0;
                plan_executed = true; //once reached goal then not issue any velocity until new plan.
            }
            cmd_vel.linear.x = std::min(config.vmax, std::max(final_vel_x, min_vel_mp->twist_ref.linear.x));

            if(std::isnan(cmd_vel.linear.x))
            {
                ROS_ERROR("CRITICAL ERROR STOPPING");
                cmd_vel.linear.x = 0.0;
            }
            cmd_vel.angular.z = 0.0;
            //ROS_INFO("REF VEL : %f", min_vel_mp->twist_ref.linear.x);
            clearVisitedPlan(closest_pose_it - ramp_plan.begin()); 
            //closest_pose_pub.publish(closest_pose_it->pose);
            //ref_pose_pub.publish(min_vel_mp->pose);
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
        //change 2 added plan size check.
        if(search_plan.size() == 0)
        {
            ROS_ERROR("SEARCH PLAN SIZE 0");
            return;
        }
        double search_radius = 10.0;
        double min_distance = 1000;
        double arc_length = 0.0;
        geometry_msgs::PoseStamped pose = search_plan.front().pose;
        for(auto it_ = search_plan.begin(); it_ != search_plan.end(); it_++)
        {
            arc_length += mpd::euclidean(pose, it_->pose);
            if(arc_length > search_radius)
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
            double point_dist = mpd::euclidean(end_pose, start_pose);
            if(point_dist > 0.0 && point_dist <= 1.0)
            {
                extension_pair =  std::make_pair(start_pose, end_pose);
                return extension_pair;
            }
        }
        ROS_ERROR("FAILED TO INITIALIZE PATH EXTENSION");
        return extension_pair;
    }

    /*mpd::Plan::const_iterator MotionPlanner::getPlanPoseIt(const mpd::Plan& plan, const geometry_msgs::PoseStamped&
      search_pose)
      {
      mpd::Plan::const_iterator it;
      it = std::find_if(plan.begin(), plan.end(), mpd::PoseCompare(search_pose)); 
      return it;
      }*/

    void MotionPlanner::getReferencePose(geometry_msgs::PoseStamped& ref_pose)
    {
        ref_pose = ref_pose_;
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
        interpolated_pose.pose.position.x = end.pose.position.x + unit_vector.getX() * scale;
        interpolated_pose.pose.position.y = end.pose.position.y + unit_vector.getY() * scale;
        interpolated_pose.pose.orientation = end.pose.orientation;
        return true;

    }

    double MotionPlanner::getGoalDistance(const geometry_msgs::PoseStamped& robot_pose)
    {
        geometry_msgs::PoseStamped goal_pose;
        tf::Stamped<tf::Pose> tf_goal_pose;
        planner_util_->getGoal(tf_goal_pose);
        poseStampedTFToMsg(tf_goal_pose, goal_pose);
        return mpd::euclidean(goal_pose, robot_pose); 
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

    void MotionPlanner::initializeConfig()
    {
	    config.vmax = 1.0;
	    config.vmin = 0.05;
	    config.acc_x = 0.1;
	    config.wmax = 0.5;
	    config.wmin = 0.1;
	    config.acc_w = 0.3;
	    config.lat_acc = 1.5;
	    config.obst_stop_dist = 2.0;
	    config.cross_track_warn = 0.15;
	    config.cross_track_error = 0.5;
        config.xy_goal_tolerance = 0.05;
        config.yaw_goal_tolerance = 0.05;
        max_xy_tolerance = 0.2;
        max_yaw_goal_tolerance = 0.2;
        last_control_v = 0.0;
        last_control_w = 0.0;
    }

    bool MotionPlanner::inPlace(const geometry_msgs::PoseStamped& pose_a, const geometry_msgs::PoseStamped& pose_b, const double xy_goal_tolerance, const double yaw_goal_tolerance, double& yaw_dif)
    {
        double path_a_yaw = tf2::getYaw(pose_a.pose.orientation);
        double path_b_yaw = tf2::getYaw(pose_b.pose.orientation);
        yaw_dif = angles::shortest_angular_distance(path_a_yaw, path_b_yaw);
        //bool in_place = ((fabs(yaw_dif) > yaw_goal_tolerance)) ? true : false;
        
        //to avoid oscillations when switching in_place points when yaw_goal tolerance is less.
        //inplace checking tolerance kept a higher tolerance. 
        //This will cause initial robot orientation is higher at tracking start.
        bool in_place = ((fabs(yaw_dif) > max_yaw_goal_tolerance)) ? true : false;
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
        geometry_msgs::PoseStamped goal_pose;
        tf::Stamped<tf::Pose> tf_goal_pose;
        planner_util_->getGoal(tf_goal_pose);
        poseStampedTFToMsg(tf_goal_pose, goal_pose);
        double position = mpd::euclidean(robot_pose, goal_pose);
        double goal_yaw = tf2::getYaw(goal_pose.pose.orientation);
        double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
        double angle = angles::shortest_angular_distance(robot_yaw, goal_yaw);
        if((position <= xy_goal_tolerance_) && (fabs(angle) <= config.yaw_goal_tolerance))
        {
            ROS_INFO("GOAL");
            return true;
        }
        //ROS_INFO("NOT GOAL");
        return false;

    }

    bool MotionPlanner::isStart(const geometry_msgs::PoseStamped& check_pose, const geometry_msgs::PoseStamped&
            start_pose, const double xy_goal_tolerance)
    {
        return (mpd::euclidean(check_pose, start_pose) < xy_goal_tolerance) ? true : false;
    }

    void MotionPlanner::profileVelocity(const double& ref_vel, double& profiled_vel)
    {
    }

    bool MotionPlanner::setGlobalPlan(const mpd::Plan& orig_global_plan)
    {
        global_plan_ = orig_global_plan;
        if(!transformPose(motion_frame_, orig_global_plan.front(), start_pose_) && !transformPose(motion_frame_,
                    orig_global_plan.back(), end_pose_))
        {
            return false;
        }
        plan_executed = false;
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
    }

    bool MotionPlanner::trajectoryCollision(double linear_vel, double angular_vel, double frequency,
            const std::vector<geometry_msgs::Point>& footprint_spec, const geometry_msgs::PoseStamped& robot_pose)
    {
        double x,y,theta,size;
        x = robot_pose.pose.position.x;
        y = robot_pose.pose.position.y;
        theta = tf2::getYaw(robot_pose.pose.orientation);
        size = 5;
        geometry_msgs::PoseStamped temp_pose;
        
        //for checking more trajectory collision points in inplace rotation.
        if(linear_vel < 0.01)         
        {
            size = 15;
        }
        for(int i = 0; i < size; i++)
        {
            x += linear_vel * cos(theta) * (1 / frequency);
            y += linear_vel * sin(theta) * ( 1 / frequency);
            theta += angular_vel * (1 / frequency);
            temp_pose.header.frame_id = robot_pose.header.frame_id;
            temp_pose.pose.position.x = x;
            temp_pose.pose.position.y = y;
            temp_pose.pose.position.z = 1.0;
            tf2::Quaternion quat;
            quat.setRPY( 0.0, 0.0, theta);
            //temp_pose.pose.orientation.z = sin(theta/2.0);
            temp_pose.pose.orientation.z = quat[2];
            //temp_pose.pose.orientation.w = cos(theta/2.0);
            temp_pose.pose.orientation.w = quat[3];
            //ref_pose_pub.publish(temp_pose);
            double footprint_cost = world_model->footprintCost(x, y, theta, footprint_spec);  	
            if(footprint_cost < 0)
            {
                return true;
            }
        }
        return false;


    }

    bool MotionPlanner::transformPose(const std::string& global_frame, const
            geometry_msgs::PoseStamped& pose, geometry_msgs::PoseStamped& transformed_pose)
    {
        try{
            tf::StampedTransform transform;
            tf_->waitForTransform(global_frame, ros::Time::now(), pose.header.frame_id, pose.header.stamp,
                    pose.header.frame_id, ros::Duration(0.5));

            tf_->lookupTransform(global_frame, ros::Time(), pose.header.frame_id, pose.header.stamp,
                    pose.header.frame_id, transform);
            tf::Stamped<tf::Pose> tf_pose;
            poseStampedMsgToTF(pose, tf_pose);
            tf_pose.setData(transform * tf_pose);
            tf_pose.stamp_ = transform.stamp_;
            tf_pose.frame_id_ = global_frame;
            poseStampedTFToMsg(tf_pose, transformed_pose);
        }
        catch(tf::LookupException& ex) {
            ROS_ERROR("No Transform available Error: %s\n", ex.what());
            return false;
        }
        catch(tf::ConnectivityException& ex) {
            ROS_ERROR("Connectivity Error: %s\n", ex.what());
            return false;
        }
        catch(tf::ExtrapolationException& ex) {
            ROS_ERROR("Extrapolation Error: %s\n", ex.what());
            return false;
        }
        return true;
    }
    void MotionPlanner::updateConfig(struct MotionPlannerConfig& latest_config)
    {
        boost::mutex::scoped_lock lock(config_mutex);
	    auto limits = planner_util_->getCurrentLimits();
        config.vmax = limits.max_vel_x;
	    config.vmin = limits.min_vel_x;
	    config.acc_x = limits.acc_lim_x;
	    config.wmax = limits.max_rot_vel;
	    config.wmin = limits.min_rot_vel;
	    config.acc_w = limits.acc_lim_theta;
	    config.lat_acc = latest_config.lat_acc;
	    config.obst_stop_dist = latest_config.obst_stop_dist;
	    config.cross_track_warn = latest_config.cross_track_warn;
	    config.cross_track_error = latest_config.cross_track_error;
        config.xy_goal_tolerance = latest_config.xy_goal_tolerance;
        config.yaw_goal_tolerance = latest_config.yaw_goal_tolerance;
    }

    bool MotionPlanner::warningFieldCb(std_srvs::SetBoolRequest& field_status, std_srvs::SetBoolResponse& response)
    {
        boost::recursive_mutex::scoped_lock pause_lock(warning_field_mutex);
        warning_field_status = field_status.data;
        return true; 
    }
    
    bool MotionPlanner::navPauseCb(std_srvs::SetBoolRequest& pause, std_srvs::SetBoolResponse& response)
    {
        boost::mutex::scoped_lock nav_pause_lock(nav_pause_mutex);
        pause_motion = pause.data;
        return true;
    }

    void setLoadedState(bool isloaded)
    {
        boost::mutex::scoped_lock state_set_lock(loaded_state_mutex);
        loaded = isloaded;
    }

};

