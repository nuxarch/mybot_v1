cmake_minimum_required(VERSION 2.8.3)
project(ev_r1)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
)
catkin_package(
)

include_directories( ${catkin_INCLUDE_DIRS})
include_directories(include/ev_r1 ${catkin_INCLUDE_DIRS})

add_executable(ev_r1 src/ev_r1.cpp)
target_link_libraries(ev_r1 ${catkin_LIBRARIES})
