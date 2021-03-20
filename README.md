# Local Planner Plugin For Move Base
* This package can be used as a local planner plugin for move_base.
* This package generate motion plan to track a given global plan and stop for obstacles.
* Currently obstacle avoidance feature is not available.
* This package contains two controller implemantations, purepursuit and a nonlinear controller.
* Default controller is the nonlinear controller which can be used as a path tracker or a point stabilizing controller.
* Motion planner class is handling all motion plans(linear, inplace turns).
* For linear motion, motion planner generate a suitable linear velocity and controller generates angular velocity to track the path.
* Features TODO
    * Replanning and obstacle avoidance.
    * State information for debugging and diagnostics.
    * controller should be loaded from the motion planner class, currently motion planner class is initialised from purepursuit class.
    * proper state implemantation in motion planning (planning, error, obstacle ...)

## Node
* Description 
    * The node is implemented as base_local_planner plugin.
    * from the plugin class purepursuit class is initialised and which contains the motion planner class
    * move base will load configured localplanner plugin and pass the global plan for to reach a goal.

* Usage 
    * Run as a move_base plugin.

* Publishers
    * `obstacle_info` ([std_msgs::Bool](https://docs.ros.org/en/api/std_msgs/html/msg/Bool.html))
        * robot is stopped for an obstacle or not.
* Services
    * `warning_field_status` ([std_srvs::SetBool](https://docs.ros.org/en/api/std_srvs/html/srv/SetBool.html))
        * reduce speed if the warning field is active, safety node call this service to update the info.
    * `nav_pause` ([std_srvs::SetBool](https://docs.ros.org/en/api/std_srvs/html/srv/SetBool.html)).
        * pause the navigation from state machine.
