cmake_minimum_required(VERSION 2.8.3)
project(architect_driver)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  tf
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(architect_driver_node
  src/architect_driver_node.cpp 
  src/architect_driver_node_wrapper.cpp
)

add_dependencies(architect_driver_node architect_driver_generate_messages_cpp)

target_link_libraries(architect_driver_node
  ${catkin_LIBRARIES}
)
