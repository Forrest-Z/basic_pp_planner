# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "base_local_planner;dynamic_reconfigure;nav_msgs;pluginlib;sensor_msgs;roscpp;tf2;tf2_ros".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lpp_local_planner".split(';') if "-lpp_local_planner" != "" else []
PROJECT_NAME = "pp_local_planner"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.1.0"
