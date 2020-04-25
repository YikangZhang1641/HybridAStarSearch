cmake_minimum_required(VERSION 2.8.3)
project(planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES planner
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_BINARY_DIR}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_BINARY_DIR}/../
  ${PROJECT_SOURCE_DIR}/../
)

add_executable(HybridAStarSearchTest src/HybridAStarSearchTest.cc src/HybridAStarSearchMap.cc src/GridMap.cc src/ReedsSheppPath.cc)
add_executable(RSPathTest src/ReedsSheppPathTest.cc src/ReedsSheppPath.cc)

target_link_libraries(HybridAStarSearchTest
  ${catkin_LIBRARIES}
)
target_link_libraries(RSPathTest
  ${catkin_LIBRARIES}
)

include_directories(${GTEST_INCLUDE_DIR})

