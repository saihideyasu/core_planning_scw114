cmake_minimum_required(VERSION 2.8.3)
project(waypoint_follower)


find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)

find_package(autoware_config_msgs REQUIRED)

find_package(tablet_socket_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        tf
        geometry_msgs
        pcl_ros
        pcl_conversions
        sensor_msgs
        autoware_msgs
        autoware_config_msgs
        tablet_socket_msgs
        autoware_health_checker
        rostest
        rosunit
        )

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES libwaypoint_follower
        CATKIN_DEPENDS roscpp
        std_msgs
        tf
        geometry_msgs
        autoware_msgs
        autoware_config_msgs
        pcl_ros
        pcl_conversions
        sensor_msgs
        tablet_socket_msgs
        autoware_health_checker
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        include
        include/twist_gate
        ${autoware_config_msgs_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
        ${tablet_socket_msgs_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

add_library(libwaypoint_follower lib/libwaypoint_follower.cpp)
add_dependencies(libwaypoint_follower
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(libwaypoint_follower ${catkin_LIBRARIES})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
