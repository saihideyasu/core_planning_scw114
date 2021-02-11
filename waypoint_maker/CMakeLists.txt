cmake_minimum_required(VERSION 2.8.3)
project(waypoint_maker)

find_package(autoware_build_flags REQUIRED)

find_package(
  catkin REQUIRED COMPONENTS
    amathutils_lib
    autoware_msgs
    autoware_config_msgs
    geometry_msgs
    gnss
    lane_planner
    libwaypoint_follower
    nav_msgs
    roscpp
    std_msgs
    tablet_socket_msgs
    tf
    vector_map
)

include_directories("/usr/include/eigen3")

find_package(Boost REQUIRED)

catkin_package()

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall -fpermissive ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  waypoint_loader
  nodes/waypoint_loader/waypoint_loader_core.cpp
  nodes/waypoint_loader/waypoint_loader_node.cpp
)
target_link_libraries(waypoint_loader ${catkin_LIBRARIES})
add_dependencies(waypoint_loader ${catkin_EXPORTED_TARGETS})

add_executable(
  waypoint_loader_show_id
  nodes/waypoint_loader/waypoint_loader_core_show_id.cpp
  nodes/waypoint_loader/waypoint_loader_node.cpp
)
target_link_libraries(waypoint_loader_show_id ${catkin_LIBRARIES})
add_dependencies(waypoint_loader_show_id ${catkin_EXPORTED_TARGETS})

add_executable(
  waypoint_replanner
  nodes/waypoint_replanner/waypoint_replanner.cpp
  nodes/waypoint_replanner/waypoint_replanner_node.cpp
)

target_link_libraries(waypoint_replanner ${catkin_LIBRARIES})
add_dependencies(waypoint_replanner ${catkin_EXPORTED_TARGETS})

add_executable(waypoint_saver nodes/waypoint_saver/waypoint_saver.cpp)
target_link_libraries(waypoint_saver ${catkin_LIBRARIES})
add_dependencies(waypoint_saver ${catkin_EXPORTED_TARGETS})

add_executable(waypoint_saver_scw nodes/waypoint_saver/waypoint_saver_scw.cpp)
target_link_libraries(waypoint_saver_scw ${catkin_LIBRARIES})
add_dependencies(waypoint_saver_scw ${catkin_EXPORTED_TARGETS})

add_executable(waypoint_extractor nodes/waypoint_extractor/waypoint_extractor.cpp)
target_link_libraries(waypoint_extractor ${catkin_LIBRARIES})
add_dependencies(waypoint_extractor ${catkin_EXPORTED_TARGETS})

add_executable(waypoint_clicker nodes/waypoint_clicker/waypoint_clicker.cpp)
target_link_libraries(waypoint_clicker ${catkin_LIBRARIES})

add_executable(
  waypoint_marker_publisher
  nodes/waypoint_marker_publisher/waypoint_marker_publisher.cpp
)

target_link_libraries(waypoint_marker_publisher ${catkin_LIBRARIES})
add_dependencies(waypoint_marker_publisher ${catkin_EXPORTED_TARGETS})

add_executable(
  waypoint_marker_publisher_show_id
  nodes/waypoint_marker_publisher/waypoint_marker_publisher_show_id.cpp
)
target_link_libraries(waypoint_marker_publisher_show_id ${catkin_LIBRARIES})
add_dependencies(waypoint_marker_publisher_show_id ${catkin_EXPORTED_TARGETS})

add_executable(
  waypoint_velocity_visualizer
  nodes/waypoint_velocity_visualizer/waypoint_velocity_visualizer.cpp
)

target_link_libraries(waypoint_velocity_visualizer ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(waypoint_velocity_visualizer ${catkin_EXPORTED_TARGETS})

add_executable(
  waypoint_creator
  nodes/waypoint_creator/waypoint_creator_node.cpp
  nodes/waypoint_creator/waypoint_creator_core.cpp
  nodes/waypoint_creator/interpolate.cpp
)

target_link_libraries(waypoint_creator ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(waypoint_creator ${catkin_EXPORTED_TARGETS})

add_executable(
  waypoint_param_sender
  nodes/waypoint_param_sender/waypoint_param_sender.cpp
)
target_link_libraries(waypoint_param_sender ${catkin_LIBRARIES})
add_dependencies(waypoint_param_sender ${catkin_EXPORTED_TARGETS})

add_executable(
  waypoint_file_move
  nodes/waypoint_file_move/waypoint_file_move.cpp
)
target_link_libraries(waypoint_file_move ${catkin_LIBRARIES})
add_dependencies(waypoint_file_move ${catkin_EXPORTED_TARGETS})

add_executable(
  straight_line_fix
  nodes/straight_line_fix/straight_line_fix.cpp
)
target_link_libraries(straight_line_fix ${catkin_LIBRARIES})
add_dependencies(straight_line_fix ${catkin_EXPORTED_TARGETS})

add_executable(
  waypoint_adjustment
  nodes/waypoint_adjustment/waypoint_adjustment.cpp
)
target_link_libraries(waypoint_adjustment ${catkin_LIBRARIES})
add_dependencies(waypoint_adjustment ${catkin_EXPORTED_TARGETS})

add_executable(
  travel_route_registration
  nodes/travel_route_registration/travel_route_registration.cpp
)
target_link_libraries(travel_route_registration ${catkin_LIBRARIES})
add_dependencies(travel_route_registration ${catkin_EXPORTED_TARGETS})

install(
  TARGETS
    waypoint_loader
    waypoint_loader_show_id
    waypoint_replanner
    waypoint_saver
    waypoint_saver_scw
    waypoint_clicker
    waypoint_marker_publisher
    waypoint_marker_publisher_show_id
    waypoint_velocity_visualizer
    waypoint_extractor
    waypoint_creator
    waypoint_param_sender
    waypoint_file_move
    straight_line_fix
    waypoint_adjustment
    travel_route_registration
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
