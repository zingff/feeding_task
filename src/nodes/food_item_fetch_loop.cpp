#include "manipulation_class.hpp"
#include <anygrasp_generation/AnyGraspGeneration.h>
#include <tf2_ros/transform_broadcaster.h>
#include <kortex_motion_planning/gen3_motion_planner.h>
#include <kortex_motion_planning/gen3_motion_executor.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "feeding_loop");

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::Duration(1.0).sleep();

  // std::string planning_group = "arm"; // Gen3 planning group
  // Manipulation manipulation(nh, planning_group);
  

  // // Manipulation variables/setup
  // manipulation.planning_scene_ptr = PlanningScenePtr(
  //     new moveit::planning_interface::PlanningSceneInterface());
  // manipulation.move_group_ptr = MoveGroupPtr(
  //     new moveit::planning_interface::MoveGroupInterface(manipulation.PLANNING_GROUP));

  // manipulation.move_group_ptr->setPlanningTime(5.0);
  // manipulation.move_group_ptr->setMaxVelocityScalingFactor(0.8); // speed scaling
  // manipulation.move_group_ptr->setPlannerId("RRT");

  std::string urdf_xml_string, srdf_xml_string;
  Eigen::VectorXd current_joint_position(7);
  Eigen::Translation3d target_translation;
  Eigen::Quaterniond target_quaternion;
  for (size_t i = 0; i < 7; i++)
  {
    current_joint_position(i) = 0;
  }
  target_translation.x() = -0.15135;
  target_translation.y() = 0.235484;
  target_translation.z() = 0.557796;

  target_quaternion.x() = 0.3872724;
  target_quaternion.y() = -0.4914169;
  target_quaternion.z() = -0.604657;
  target_quaternion.w() = 0.4928685;
  
  nh.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  Gen3MotionPlanner gen3_motion_planner;
  Gen3MotionExecutor gen3_motion_executor(nh);


  // manipulation.goTop();
  // manipulation.reach_food_item();
  // ros::Duration(0.5).sleep();

  trajectory_msgs::JointTrajectoryPoint joint_state = gen3_motion_executor.getJointState();
  for (size_t i = 0; i < 7; i++)
  {
    current_joint_position(i) = joint_state.positions[i];
  }
  trajectory_msgs::JointTrajectory motion_plan;
  motion_plan = gen3_motion_planner.createGen3MotionPlan(
    urdf_xml_string,
    srdf_xml_string,
    current_joint_position,
    target_translation,
    target_quaternion
  );

  ROS_INFO("Generated trajectory for Kinova Gen3!");

  gen3_motion_executor.executionMotionPlan(motion_plan);

  ros::spin();
  ros::shutdown();

  return 0;
}