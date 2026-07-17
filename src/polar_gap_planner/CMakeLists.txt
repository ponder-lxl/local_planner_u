cmake_minimum_required(VERSION 2.8.3)
project(polar_gap_planner)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf
  pcl_ros
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf
  pcl_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

add_library(polar_analyzer src/polar_analyzer.cpp)
target_link_libraries(polar_analyzer ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(polar_gap_planner_node src/polar_gap_planner_node.cpp)
target_link_libraries(polar_gap_planner_node polar_analyzer ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS polar_analyzer polar_gap_planner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/publish_final_goal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
