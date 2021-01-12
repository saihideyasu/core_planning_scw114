cmake_minimum_required(VERSION 2.8.3)
project(cmd_selector)

find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  autoware_msgs
  autoware_config_msgs
  geometry_msgs
)

catkin_package()

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(cmd_selector
  src/cmd_selector.cpp
)
target_link_libraries(cmd_selector ${catkin_LIBRARIES})
add_dependencies(cmd_selector ${catkin_EXPORTED_TARGETS})

install(TARGETS cmd_selector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
