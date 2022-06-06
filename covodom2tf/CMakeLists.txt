cmake_minimum_required(VERSION 3.0.2)
project(covodom2tf)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  geometry_msgs
)

catkin_package(
CATKIN_DEPENDS message_runtime
)

include_directories(
${CMAKE_CURRENT_LIST_DIR}/include
${catkin_INCLUDE_DIRS}
)

add_executable(covodom2tf_node ${CMAKE_CURRENT_LIST_DIR}/src/covodom2tf_node.cc)
target_link_libraries(covodom2tf_node ${catkin_LIBRARIES})
