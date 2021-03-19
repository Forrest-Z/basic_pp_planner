#include "pp_local_planner/motion_target_follower.h"

namespace motion_target_follower
{

	MotionTargetFollower::MotionTargetFollower(double k1_, double k2_, double beta_, double lambda_) : 
    k1(k1_), k2(k2_), beta(beta_), lambda(lambda_)  
	{    
	}

	MotionTargetFollower::~MotionTargetFollower()
	{}

    void MotionTargetFollower::updateTargetPose(const geometry_msgs::PoseStamped& target)
    {
        sample_pose_base = target;
        sample_pose_yaw = tf::getYaw(target.pose.orientation);
    }

	void MotionTargetFollower::updateTheta()
	{
		double goal_theta = sample_pose_yaw;
		double theta_points = atan2(sample_pose_base.pose.position.y, sample_pose_base.pose.position.x);
		double theta_;
		findAngularDifference(theta_points, goal_theta, theta_);
		theta = theta_;
		//this is also correct.
		//double norm_theta = angles::normalize_angle(sample_pose_yaw + delta);
		//ROS_DEBUG("theta_from_angular diff %f", norm_theta);
		//theta = norm_theta;
	}

	void MotionTargetFollower::updateDelta()
	{
		
		delta = atan2(-sample_pose_base.pose.position.y, sample_pose_base.pose.position.x);		
	}

	void MotionTargetFollower::findAngularDifference(double theta1, double theta2, double &theta_)
	{
		{
			double dif_angle = theta2 - theta1;
			double normalize_angle_positive = fmod((fmod(dif_angle, 2.0 * 3.1416)) + (2 * 3.1416), (2.0 * 3.1416));
			if (normalize_angle_positive > 3.1416)
			{
				normalize_angle_positive -= 2.0 * 3.1416;
			}
			theta_ = normalize_angle_positive;
		}
	}

	void MotionTargetFollower::updateR()
	{
		r = sqrt(pow(sample_pose_base.pose.position.x, 2) + pow(sample_pose_base.pose.position.y, 2));
	}
	
	void MotionTargetFollower::updateCurvature(const double r_, const double delta_, double const theta_, double& curvature_)
	{
		curvature_ = -(1/r_) * ((k2 * (delta_ - atan(-k1 * theta_))) + ((1 + (k1/(1 + pow((k1 * theta_), 2)))) * sin(delta_)));
        //limit curvature value to less than 0.4, for r less than 30 cm.
        int sign = fabs(curvature_) / curvature;
        curvature_ = (r_ > 0.3) ? curvature_ : (sign * std::min(fabs(curvature_), 0.4)); 
	}

	void MotionTargetFollower::updateLinearVelocity(const double curvature_, double& linear_velocity)
	{
        //ROS_INFO("LV B : %f", linear_velocity);
        //ROS_INFO("VEL RED FACT %f", (beta * pow(fabs(curvature_), lambda)));
	    linear_velocity = linear_velocity / (1 + (beta * pow(abs(curvature_), lambda)));
        linear_velocity = (linear_velocity < vmin) ? vmin : linear_velocity;
        linear_velocity = (linear_velocity > vmax) ? vmax : linear_velocity;
        //ROS_INFO("LV A : %f", linear_velocity);
	}

	void MotionTargetFollower::updateAngularVelocity(const double curvature_, const double linear_velocity, double&
    angular_velocity)
	{
		angular_velocity = linear_velocity * curvature_;
        //ROS_INFO("AV : %f", angular_velocity);
        //angular_velocity = (angular_velocity < wmin) ? wmin : angular_velocity;
        //angular_velocity = (angular_velocity > wmax) ? wmax : angular_velocity;
	}

    void MotionTargetFollower::updateControlLimits(double vmax_, double vmin_, double wmax_, double wmin_)
    {
        vmax = vmax_;
        vmin = vmin_;
        wmax = wmax_;
        wmin = wmin_;
    }

	void MotionTargetFollower::getControlCommands(const geometry_msgs::PoseStamped& target, double& linear_vel, double&
    angular_vel)
	{
            updateTargetPose(target);
			updateTheta();
			updateDelta();
			updateR();
			updateCurvature(r, delta, theta, curvature);
			updateLinearVelocity(curvature, linear_vel);
			updateAngularVelocity(curvature, linear_vel, angular_vel);
            angular_vel = ( std::isnan(angular_vel)) ? 0.0 : angular_vel;
	}

	void debug()
	{
	}

}


