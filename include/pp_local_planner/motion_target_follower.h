#ifndef MOTION_TARGET_FOLLOWER_H
#define MOTION_TARGET_FOLLOWER_H

#include "data_struct.h"

#include <math.h>
#include <angles/angles.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace motion_target_follower
{
class MotionTargetFollower
{
	public:
		/**
		 * @brief constructor for MotionTargetFollower class
		 */
		MotionTargetFollower(double k1_ = 0.0, double k2_ = 0.0, double beta_ = 0.0, double lambda_ = 0.0);
		/**
		 * @brief destructor for MotionTargetFollower class
		 */
		~MotionTargetFollower();
		/**
		 * @brief this method find real time linear angular velocity required for the smooth navigation of the robot base through the given motion targets.
		 */
		void getControlCommands(const geometry_msgs::PoseStamped& target, double& linear_vel, double& angular_vel);
        
        void updateControlLimits(double vmax_, double vmin_, double wmax_, double wmin_);
		
	private:
		double k1, k2, beta, lambda, vmax, vmin, control_rate, delta, theta, r, curvature, wmax, wmin;
		double roll, pitch, yaw, sample_pose_yaw, odom_vel;
		motion_target_struct::MotionTarget sample;
		motion_target_struct::PoseEuler robot_pose;
		motion_target_struct::ControlInput control_input;
		geometry_msgs::PoseStamped sample_pose_world;
		geometry_msgs::PoseStamped sample_pose_base;
		geometry_msgs::Twist control_velocity;
		geometry_msgs::Quaternion quat_msg;
		tf2::Quaternion quat_;
		tf::TransformListener odom_base_transform;
		
		/*
		 * @brief method for updating goal orientation
		 */
	        void updateTheta();
		
		/*
		 * @brief method for update current delta parameter of the controller.
		 */
		void updateDelta();

		/*
		 * @brief method for updating the distance to goal parameter of controller.
		 */
		void updateR();
		/*
		 * @brief method for updating the curvature from the current position to the motion target.
		 * @param1 length of the line connecting the robot current point to motion target point.
		 * @param2 angle made by the line passing through the motion target point from current position of robot with goal orientation vector.
		 * @param3 orientation of the goal.
		 * @param4 estimated curvature from robot position to motion target.
		 */
		void updateCurvature(const double, const double, const double, double&);

		/*
		 * @brief method for estimate and update linear velocity that robot should achieve.
		 * @param1 curvature that robot is following.
		 * @param2 estimated linear velocity.
		 */
		void updateLinearVelocity(const double, double&);
		
		/*
		 * @brief method estimate angular velocity with the robot should follow based on the curvature and linear velocity.
		 * @param1 curvature that robot should follow.
		 * @param2 linear velocity that robot should achieve.
		 * @param3 estimated angular velocity.
		 */
		void updateAngularVelocity(const double, const double, double&);

		/*
		 * @brief method will find shortest angular difference between two given angles.
		 * @param1 angle between current point and the motion target.
		 * @param2 motion target orientation.
		 * @param3 shortest angular difference between the given angles.
		 */
		void findAngularDifference(const double, const double, double&);
	        
		/* 
		 * @brief method for publishing datas for debugging.
		 */	
		void debug();

        /*
         * @brief callback for getting dock pose from perception.
         * @param dock pose information.
         */
		void updateTargetPose(const geometry_msgs::PoseStamped& target); 	
};

}
#endif
