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
  # pcl_conversions
  # pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  # gpd_ros
  control_msgs
  anygrasp_generation
)

catkin_package(
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(motion_planning_test
	src/nodes/motion_planning_test.cpp
	src/functions/manipulation_class.cpp
  src/functions/gpd.cpp
	# src/functions/perception_class.cpp
  # src/functions/gpd.cpp
)
target_link_libraries(motion_planning_test ${catkin_LIBRARIES})
add_dependencies(motion_planning_test ${catkin_EXPORTED_TARGETS})

add_executable(reach_food_item
	src/nodes/food_item_fetch.cpp
	src/functions/manipulation_class.cpp
  src/functions/gpd.cpp
	# src/functions/perception_class.cpp
  # src/functions/gpd.cpp
)
target_link_libraries(reach_food_item ${catkin_LIBRARIES})
add_dependencies(reach_food_item ${catkin_EXPORTED_TARGETS})

add_executable(door_handle_grasp
	src/nodes/door_handle_grasp.cpp
	src/functions/manipulation_class.cpp
  src/functions/gpd.cpp
	# src/functions/perception_class.cpp
  # src/functions/gpd.cpp
)
target_link_libraries(door_handle_grasp ${catkin_LIBRARIES})
add_dependencies(door_handle_grasp ${catkin_EXPORTED_TARGETS})

add_executable(ompl_test
	src/nodes/ompl_test.cpp
	src/functions/manipulation_class.cpp
  src/functions/gpd.cpp
	# src/functions/perception_class.cpp
  # src/functions/gpd.cpp
)
target_link_libraries(ompl_test ${catkin_LIBRARIES})
add_dependencies(ompl_test ${catkin_EXPORTED_TARGETS})

add_executable(initial
	src/nodes/initial.cpp
	src/functions/manipulation_class.cpp
  src/functions/gpd.cpp
	# src/functions/perception_class.cpp
  # src/functions/gpd.cpp
)
target_link_libraries(initial ${catkin_LIBRARIES})
add_dependencies(initial ${catkin_EXPORTED_TARGETS})

add_executable(get_utensil
	src/nodes/get_utensil.cpp
	src/functions/manipulation_class.cpp
  src/functions/gpd.cpp
	# src/functions/perception_class.cpp
  # src/functions/gpd.cpp
)
target_link_libraries(get_utensil ${catkin_LIBRARIES})
add_dependencies(get_utensil ${catkin_EXPORTED_TARGETS})