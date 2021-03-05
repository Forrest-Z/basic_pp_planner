#ifndef PLANNER_CONFIG
#define PLANNER_CONFIG

struct MotionPlannerConfig
{
    double vmax;
    double load_vmax;
    double noload_vmax;
    double vmin;
    double acc_x;
    double load_acc_x;
    double noload_acc_x;
    double wmax;
    double load_wmax;
    double noload_wmax;
    double wmin;
    double acc_w;
    double load_acc_w;
    double noload_acc_w;
    double min_lookahead;
    double max_lookahead;
    double kla;
    double lat_acc;
    double obst_stop_dist;
    double cross_track_warn;
    double cross_track_error;
    double xy_goal_tolerance;
    double yaw_goal_tolerance;
    bool update_config;
};
#endif
