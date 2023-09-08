cmake_minimum_required(VERSION 3.0.2)
project(feeding_task)

set(CMAKE_CXX_STANDARD 14)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  geometry_msgs
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  geometric_shapes
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  control_msgs
  anygrasp_generation
)

catkin_package(
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(reach_food_item
	src/nodes/food_item_fetch.cpp
	src/functions/manipulation_class.cpp
  src/functions/feeding_task.cpp
)
target_link_libraries(reach_food_item ${catkin_LIBRARIES})
add_dependencies(reach_food_item ${catkin_EXPORTED_TARGETS})

add_executable(bowl_handle_grasp
	src/nodes/door_handle_grasp.cpp
	src/functions/manipulation_class.cpp
  src/functions/feeding_task.cpp
)
target_link_libraries(bowl_handle_grasp ${catkin_LIBRARIES})
add_dependencies(bowl_handle_grasp ${catkin_EXPORTED_TARGETS})

add_executable(get_utensil
	src/nodes/get_utensil.cpp
	src/functions/manipulation_class.cpp
  src/functions/feeding_task.cpp
)
target_link_libraries(get_utensil ${catkin_LIBRARIES})
add_dependencies(get_utensil ${catkin_EXPORTED_TARGETS})