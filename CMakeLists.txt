cmake_minimum_required(VERSION 2.8.3)
project(pp_local_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
        COMPONENTS
            angles
            base_local_planner
            cmake_modules
            costmap_2d
            dynamic_reconfigure
            nav_core
            nav_msgs
            pluginlib
            sensor_msgs
            roscpp
            tf2
            tf2_geometry_msgs
            tf2_ros
        )

find_package(Eigen3 REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    )
add_definitions(${EIGEN3_DEFINITIONS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/PPLocalPlanner.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES pp_local_planner
    CATKIN_DEPENDS
        base_local_planner
        dynamic_reconfigure
        nav_msgs
        pluginlib
        sensor_msgs
        roscpp
        tf2
        tf2_ros
)

add_library(pp_local_planner 
    src/pp_local_planner.cpp 
    src/pp_local_planner_ros.cpp 
    src/motion_planner.cpp
    src/motion_target_follower.cpp)

add_dependencies(pp_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pp_local_planner ${catkin_LIBRARIES})

install(TARGETS pp_local_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(FILES pp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

#if(CATKIN_ENABLE_TESTING)
#    find_package(rostest REQUIRED)
#    add_rostest_gtest(motion_plannner_test test/motion_planner_basics.test test/test_mynode.cpp)
#    target_link_libraries(motion_planner_test ${catkin_LIBRARIES} pp_local_planner)
#endif()

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_motion_planner_basics test/motion_planner_basics.cpp)
    if(TARGET test_motion_planner_basics)
        target_link_libraries(test_motion_planner_basics pp_local_planner)
    endif()
endif()
