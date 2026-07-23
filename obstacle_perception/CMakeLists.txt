cmake_minimum_required(VERSION 3.0.2)
project(obstacle_perception)

## Optimise: this node's whole point is low-latency clustering.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-O3 -std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(obstacle_circles_node src/obstacle_circles_node.cpp)
target_link_libraries(obstacle_circles_node ${catkin_LIBRARIES})

install(TARGETS obstacle_circles_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
