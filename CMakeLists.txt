cmake_minimum_required(VERSION 3.0.2)
project(feeding_task)

set(CMAKE_CXX_STANDARD 14)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

if(NOT KORTEX_MOTION_PLANNING_LIB_DIRS)
  set(KORTEX_MOTION_PLANNING_LIB_DIRS "${PROJECT_SOURCE_DIR}/../../devel/lib")
endif()

SET(KORTEX_CONFIG_DIR 
  "${PROJECT_SOURCE_DIR}/../kortex_motion_planning/config"
)
add_definitions(-DKORTEX_CONFIG_DIR="${KORTEX_CONFIG_DIR}")

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
  kortex_motion_planning
  tesseract_monitoring
  tesseract_rosutils
)

find_package(tesseract_command_language REQUIRED)
find_package(tesseract_task_composer REQUIRED)
find_package(tesseract_kinematics REQUIRED)

# catkin_package(
# )

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_service_files(
  FILES
  SkewerStatusCheck.srv
  WaitForStart.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime
)

# Single food acquisition test
add_executable(reach_food_item
	src/nodes/food_item_fetch.cpp
	src/functions/manipulation_class.cpp
  src/functions/feeding_task.cpp
)
target_link_libraries(reach_food_item 
  ${catkin_LIBRARIES}
)
add_dependencies(reach_food_item 
  ${catkin_EXPORTED_TARGETS}
)

# Feeding loop for both food acquisition and transfer
link_directories(${KORTEX_MOTION_PLANNING_LIB_DIRS})
add_executable(reach_food_item_loop
	src/nodes/food_item_fetch_loop.cpp
	src/functions/manipulation_class.cpp
  src/functions/feeding_task.cpp
  src/functions/feeding_utilities.cpp
  src/functions/anygrasp_publisher.cpp

)
target_link_libraries(reach_food_item_loop 
  ${catkin_LIBRARIES}
  kortex_motion_planning
)
target_link_libraries(reach_food_item_loop
  tesseract::tesseract_environment
  tesseract::tesseract_command_language
  tesseract::tesseract_task_composer
  tesseract::tesseract_motion_planners_core
  tesseract::tesseract_motion_planners_simple
  tesseract::tesseract_motion_planners_ompl
  tesseract::tesseract_motion_planners_trajopt
  tesseract::tesseract_motion_planners_descartes
  tesseract::tesseract_time_parameterization_isp
  )
add_dependencies(reach_food_item_loop 
  ${catkin_EXPORTED_TARGETS}
)

# Grasp the bowl handle
add_executable(bowl_handle_grasp
	src/nodes/door_handle_grasp.cpp
	src/functions/manipulation_class.cpp
  src/functions/feeding_task.cpp
)
target_link_libraries(bowl_handle_grasp
  ${catkin_LIBRARIES}
)
add_dependencies(bowl_handle_grasp 
  ${catkin_EXPORTED_TARGETS}
)

# Get the utensil
add_executable(get_utensil
	src/nodes/get_utensil.cpp
	src/functions/manipulation_class.cpp
  src/functions/feeding_task.cpp
)
target_link_libraries(get_utensil 
  ${catkin_LIBRARIES}
)
add_dependencies(get_utensil 
  ${catkin_EXPORTED_TARGETS}
)

# Food transfer test, deprecated
add_executable(bite_transfer
	src/nodes/bite_transfer.cpp
	src/functions/manipulation_class.cpp
  src/functions/feeding_task.cpp
)
target_link_libraries(bite_transfer 
  ${catkin_LIBRARIES}
)
add_dependencies(bite_transfer 
  ${catkin_EXPORTED_TARGETS}
)


# voice recognization
add_executable(iat_publish
  src/ui/iat_publish.cpp
  src/ui/linuxrec.c
  src/ui/speech_recognizer.c
)
target_link_libraries(
  iat_publish
  ${catkin_LIBRARIES}
  libmsc.so -ldl -lpthread -lm -lrt -lasound
)
add_dependencies(iat_publish 
  ${catkin_EXPORTED_TARGETS}
)

# # Anygrasp publisher
# add_executable(anygrasp_publisher
# 	src/functions/anygrasp_publisher.cpp
# )
# target_link_libraries(anygrasp_publisher 
#   ${catkin_LIBRARIES}
# )
# add_dependencies(anygrasp_publisher 
#   ${catkin_EXPORTED_TARGETS}
# )

########## FOR TEST ##########
# Move to initial position
add_executable(initial
	src/nodes/initial.cpp
	src/functions/manipulation_class.cpp
  src/functions/feeding_task.cpp
)
target_link_libraries(initial 
  ${catkin_LIBRARIES}
)
add_dependencies(initial 
  ${catkin_EXPORTED_TARGETS}
)
