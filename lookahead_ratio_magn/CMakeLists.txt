cmake_minimum_required(VERSION 2.8.3)
project(lookahead_ratio_magn)


find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)

find_package(autoware_config_msgs REQUIRED)

find_package(tablet_socket_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        tf
        geometry_msgs
        autoware_msgs
        autoware_config_msgs
        )

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS
        CATKIN_DEPENDS roscpp
        std_msgs
        tf
        geometry_msgs
        autoware_msgs
        autoware_config_msgs
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        ${autoware_config_msgs_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

add_executable(lookahead_ratio_magn src/lookahead_ratio_magn.cpp)
target_link_libraries(lookahead_ratio_magn ${catkin_LIBRARIES})
add_dependencies(lookahead_ratio_magn
        ${catkin_EXPORTED_TARGETS})

## Install executables and/or libraries
install(TARGETS lookahead_ratio_magn
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
