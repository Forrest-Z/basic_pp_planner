namespace motion_target_struct
{
	struct LinearVelocity
	{
		double x;
		double y;
		double z;
	};
	
	struct AngularVelocity
	{
		double x;
		double y;
		double z;
	};
	
	struct ControlInput
	{
		LinearVelocity linear;
		AngularVelocity angular;
	};
	
	struct Position
	{
		double x;
		double y;
		double z;
	};
	
	struct Euler
	{
		double roll;
		double pitch;
		double yaw;
	};
    
        //Quaternion contatiner
	struct Quaternion
	{
		double x;
		double y;
		double z;
		double w;
	};

	struct PoseEuler
	{
		Position position;
		Euler orientation;
	};

        //container for holding pose informations
	struct PoseQuaternion
	{
		Position position;
		Quaternion orientation;
	};
	
	struct MotionTarget
	{
		PoseEuler pose;
		LinearVelocity target_vel;
	};
}
