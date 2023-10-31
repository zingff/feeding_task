#include "manipulation_class.hpp"

#include <iostream>
#include <fstream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;

Manipulation::Manipulation(ros::NodeHandle nodeHandle, std::string planning_group)
{
    PLANNING_GROUP = planning_group;
    this->gripper_command = nodeHandle.advertise<control_msgs::GripperCommandActionGoal>("robotiq_2f_85_gripper_controller/gripper_cmd/goal", 10);
    grasps_visualization_pub_ = nodeHandle.advertise<visualization_msgs::Marker>("grasps_visualization", 10);

}

void Manipulation::getCurrentState()
{
    const robot_state::JointModelGroup *joint_model_group =
        move_group_ptr->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    current_state = move_group_ptr->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
}


void Manipulation::move(std::vector<double>)
{
    move_group_ptr->setJointValueTarget(joint_group_positions);
    move_group_ptr->move();
}

void Manipulation::setJointGroup(double j0, double j1, double j2, double j3, double j4, double j5, double j6)
{
    joint_group_positions[0] = j0; // base
    joint_group_positions[1] = j1;
    joint_group_positions[2] = j2;
    joint_group_positions[3] = j3;
    joint_group_positions[4] = j4;
    joint_group_positions[5] = j5;
    joint_group_positions[6] = j6; // gripper
}

void Manipulation::get_utensil()
{
    open_gripper();
    ros::Duration(0.5).sleep();
    getCurrentState();
    ROS_INFO("Moving to utensil position");
    setJointGroup(0.302989, 5.96553-6.28, 3.09643, 3.87292-6.28, 6.26457, 5.21691-6.28, 1.81463);
    move(joint_group_positions);
    ros::Duration(0.5).sleep();
    close_gripper();
    ros::Duration(0.5).sleep();

    setJointGroup(0.206082, 5.97423-6.28, 2.64442, 3.91491-6.28, 6.09785, 5.21866-6.28, 1.36814);
    move(joint_group_positions);

    // auto current_pose = this->move_group_ptr->getCurrentPose();
    // this->target_pose.orientation.x = current_pose.pose.orientation.x;
    // this->target_pose.orientation.y = current_pose.pose.orientation.y;
    // this->target_pose.orientation.z = current_pose.pose.orientation.z;
    // this->target_pose.orientation.w = current_pose.pose.orientation.w;
    // this->target_pose.position.x = current_pose.pose.position.x;
    // this->target_pose.position.y = current_pose.pose.position.y + 0.03;
    // this->target_pose.position.z = current_pose.pose.position.z;
    // set_target_pose();
    // plan_pose_goal();
    // this->move_group_ptr->move();

    ros::Duration(0.5).sleep();
    goTop();
}

// // Not used, to detele
// void Manipulation::trajectory_replay(std::string waypoint_path)
// {
//     std::ifstream infile(waypoint_path);
//     std::string line;
//     std::vector<trajectory_msgs::JointTrajectoryPoint> waypoints;
//     while (std::getline(infile, line))
//     {
//         std::istringstream iss(line);
//         trajectory_msgs::JointTrajectoryPoint replay_point;
//         double value;
//         while (iss >> value)
//         {
//             replay_point.positions.push_back(value);
//         }
//         waypoints.push_back(replay_point);
//     }
//     infile.close();

//     robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
//     const robot_model::RobotModelPtr &robot_model = robot_model_loader.getModel();

//     for (auto &waypoint : waypoints)
//     {
//         for (size_t j = 0; j < waypoint.positions.size(); ++j)
//         {
//             const std::string &joint_name = move_group_ptr->getJointNames()[j];
//             const robot_model::JointModel *joint_model = robot_model->getJointModel(joint_name);
//             double joint_min = joint_model->getVariableBounds(joint_name).min_position_;
//             double joint_max = joint_model->getVariableBounds(joint_name).max_position_;
//             waypoint.positions[j] = std::max(joint_min, std::min(joint_max, waypoint.positions[j]));
//         }
//     }

//     trajectory_msgs::JointTrajectory trajectory;
//     trajectory.joint_names = move_group_ptr->getJointNames();

//     double max_speed = 0.8;
//     trajectory_msgs::JointTrajectoryPoint modified_first_waypoint = waypoints.front();
//     trajectory_msgs::JointTrajectoryPoint modified_last_waypoint = waypoints.back();

//     modified_first_waypoint.time_from_start = ros::Duration(1.0); 
//     modified_last_waypoint.time_from_start = ros::Duration(1.0);

//     trajectory.points.push_back(modified_first_waypoint);

//     for (const auto &waypoint : waypoints)
//     {
//         trajectory_msgs::JointTrajectoryPoint traj_point;
//         traj_point.positions = waypoint.positions;
//         traj_point.time_from_start = ros::Duration(1.0);
//         trajectory.points.push_back(traj_point);
//     }

//     trajectory.points.push_back(modified_last_waypoint);

//     TrajectoryClient client("/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory", true);
//     client.waitForServer();
//     control_msgs::FollowJointTrajectoryGoal goal;
//     goal.trajectory = trajectory;
//     client.sendGoal(goal);
//     client.waitForResult();
// }

void Manipulation::goTop()
{
    getCurrentState();
    ROS_INFO("Moving to Top Position");
    // setJointGroup(0.043, -0.1824, -0.0133, 2.208, -0.0188, 0.7828, -1.524);  // original
    // this config is closer to Home
    // setJointGroup(0, 6.12427-6.28, 3.15113, 3.9903-6.28, 0.0248641, 5.68259-6.28, 1.58098);  // normal, for pickup
    // setJointGroup(6.26513, 0.179158, 3.20562, 4.66534-6.28, 0.036471, 5.01553-6.28, 1.57978);  // vertical, for pickup
    // setJointGroup(0.319602, 6.05876-6.28, 3.1419, 4.05812-6.28, 0.0315863, 5.53112-6.28, 1.56028);  // a little tilt, for feeding
    setJointGroup(0.0212474, 6.01921-6.28, 3.15111, 4.13476-6.28, 0.0608379, 5.37321-6.28, 1.58046);  // a little tilt, for feeding new 20231019
    
    move(joint_group_positions);
}

// Not used, to detele
void Manipulation::goDown()
{
  getCurrentState();
  ROS_INFO("Moving to Down Position");
  // setJointGroup(1.15159, 0.618337, 0.593246, 2.20513, 5.16716, 0.744263, 0.126573);
  setJointGroup(1.05848, 0.377619, 3.17121, 3.86836-6.28, 0.0377704, 0.0603388, 1.58758);
  move(joint_group_positions);
}

// Not used, to detele
void Manipulation::lastJointRotation(double rotate_angle)
{
  getCurrentState();
  ROS_INFO("Rotate the last joint");
  setJointGroup(joint_group_positions[0] + rotate_angle, 
                joint_group_positions[1], 
                joint_group_positions[2],
                joint_group_positions[3],
                joint_group_positions[4],
                joint_group_positions[5],
                joint_group_positions[6]);
  move(joint_group_positions);
}

// Not used, to detele
void Manipulation::goWait()
{
    getCurrentState();
    ROS_INFO("Moving to Waiting Position");
    setJointGroup(0, pi / 5, 0, pi / 2, 0, -pi / 5, -pi / 2);
    move(joint_group_positions);
}

// Not used, to detele
void Manipulation::goTemp()
{
    getCurrentState();
    ROS_INFO("Moving to Waiting Position");
    setJointGroup(1.38351, 0.484841, 3.13812, 4.04086-6.28, 5.92295, 1.08355, 0.26627);
    move(joint_group_positions);
}

// Not used, to detele
void Manipulation::goSnapshotPostion()
{
    getCurrentState();
    ROS_INFO("Moving to snapshot Position");
    setJointGroup(0, 0, pi, -1.9*pi, 0, -0.7*pi, pi/2);
    move(joint_group_positions);
}

tf::StampedTransform Manipulation::getTransform(
                          tf::TransformListener & listener,std::string target_frame,
                          std::string source_frame)
{
  tf::StampedTransform T_target_source;
  // ros::Time now = ros::Time::now();
  ros::Time now = ros::Time(0);
  listener.waitForTransform(target_frame, source_frame, now, ros::Duration(18.0));
  listener.lookupTransform(target_frame, source_frame, now, T_target_source);
  // std::cout << "Transform from '" << source_frame << "' to '" << target_frame << "':" << std::endl;
  // std::cout << "Translation: (x=" << T_target_source.getOrigin().x()
  //           << ", y=" << T_target_source.getOrigin().y()
  //           << ", z=" << T_target_source.getOrigin().z() << ")" << std::endl;
  // std::cout << "Rotation: (x=" << T_target_source.getRotation().x()
  //           << ", y=" << T_target_source.getRotation().y()
  //           << ", z=" << T_target_source.getRotation().z()
  //           << ", w=" << T_target_source.getRotation().w() << ")" << std::endl;
  return T_target_source;
}

void Manipulation::reach_anygrasp()
{
  tf::StampedTransform T_base_grasp;
  T_base_grasp = getTransform(grasp_listener, BASE_LINK_NAME, GRASP_LINK_NAME);
  geometry_msgs::Vector3 grasp_pose;
  tf::Matrix3x3 rotation_y;
  rotation_y.setRPY(0, M_PI / 2.0 - 0.02, 0); 
  tf::Matrix3x3 rotation_z;
  rotation_z.setRPY(0, 0,  - M_PI / 2.0 + 0.02);
  tf::Matrix3x3 grasp_orientation = T_base_grasp.getBasis();
  grasp_orientation = grasp_orientation * rotation_y * rotation_z;
  grasp_orientation.getRPY(grasp_pose.x, grasp_pose.y, grasp_pose.z);

  tf::Vector3 translation_offset(0, 0, 0.01);
  tf::Vector3 grasp_translation(T_base_grasp.getOrigin().x(),
                                 T_base_grasp.getOrigin().y(),
                                 T_base_grasp.getOrigin().z());
  grasp_translation = grasp_orientation * grasp_translation + translation_offset;

  this->orientation = grasp_pose;
  this->position.x = T_base_grasp.getOrigin().x();
  this->position.y = T_base_grasp.getOrigin().y();  
  this->position.z = T_base_grasp.getOrigin().z() + 0.02; 
  // this->position.x = grasp_translation.x();
  // this->position.y = grasp_translation.y();  
  // this->position.z = grasp_translation.z();

  std::cout << grasp_translation.x() << std::endl;
  std::cout << grasp_translation.y() << std::endl;
  std::cout << grasp_translation.z() << std::endl;
  
  std::cout << grasp_pose.x*57.3 << std::endl;
  std::cout << grasp_pose.y*57.3 << std::endl;
  std::cout << grasp_pose.z*57.3 << std::endl;

  set_target_pose();
  plan_pose_goal();
  this->move_group_ptr->move();
  ros::Duration(2.0).sleep();

  close_gripper();
}

void Manipulation::reach_handle()
{
  // initial position
  getCurrentState();
  ROS_INFO("Moving to bowl grasp initial position");
  setJointGroup(1.38348, 0.467969, 3.13848, 4.03808 - 6.28, 5.9657, 1.08215, 1.69044);
  // setJointGroup(0.043, -0.1824, -0.0133, 2.208, -0.0188, 0.7828, -1.524);
  move(joint_group_positions);
  ros::Duration(2.0).sleep();

  tf::StampedTransform T_base_grasp;
  T_base_grasp = getTransform(grasp_listener, BASE_LINK_NAME, "anygrasp/grasp_0");
  geometry_msgs::Vector3 grasp_pose;
  grasp_pose.x = 0/57.3;
  grasp_pose.y = -91.1241/57.3; 
  grasp_pose.z = 89.274/57.3;

  tf::Matrix3x3 rotation_y;
  rotation_y.setRPY(0, M_PI / 2.0, 0);
  std::cout << "Rotation Matrix: 90 deg about y" << std::endl;
  for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
          std::cout << rotation_y[i][j] << " ";
      }
      std::cout << std::endl;
  }

  tf::Matrix3x3 rotation_z;
  rotation_z.setRPY(0, 0, - M_PI / 2.0);
  std::cout << "Rotation Matrix: - 90 deg about z" << std::endl;
  for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
          std::cout << rotation_z[i][j] << " ";
      }
      std::cout << std::endl;
  }

  tf::Matrix3x3 grasp_orientation = T_base_grasp.getBasis();
  grasp_orientation = grasp_orientation * rotation_y * rotation_z;
  // grasp_orientation.getRPY(grasp_pose.x, grasp_pose.y, grasp_pose.z);
  tf::Vector3 translation_offset(0, 0, 0.02);  // offset about z-axis
  tf::Vector3 grasp_translation(T_base_grasp.getOrigin().x(),
                                 T_base_grasp.getOrigin().y(),
                                 T_base_grasp.getOrigin().z());
  grasp_translation = grasp_orientation * grasp_translation + translation_offset;

  tf::Quaternion grasp_quaternion;
  grasp_orientation.getRotation(grasp_quaternion);
  std::cout << "Grasp quaternion:" << std::endl;
  std::cout << "x: " << grasp_quaternion.getX() << std::endl;
  std::cout << "y: " << grasp_quaternion.getY() << std::endl;
  std::cout << "z: " << grasp_quaternion.getZ() << std::endl;
  std::cout << "w: " << grasp_quaternion.getW() << std::endl;

  this->orientation = grasp_pose;
  this->position.x = T_base_grasp.getOrigin().x();
  this->position.y = T_base_grasp.getOrigin().y() + 0.02;  
  this->position.z = T_base_grasp.getOrigin().z(); 

  std::cout << "Translation (xyz):" << std::endl;
  std::cout << "x: " << grasp_translation.x() << std::endl;
  std::cout << "y: " << grasp_translation.y() << std::endl;
  std::cout << "z: " << grasp_translation.z() << std::endl;

  std::cout << "Pose (rpy in degrees):" << std::endl;
  std::cout << "x (roll): " << grasp_pose.x * 57.3 << std::endl;
  std::cout << "y (pitch): " << grasp_pose.y * 57.3 << std::endl;
  std::cout << "z (yaw): " << grasp_pose.z * 57.3 << std::endl;

  set_target_pose();
  plan_pose_goal();
  this->move_group_ptr->move();
  ros::Duration(2.0).sleep();

  close_gripper();

  this->position.y += 0.20;  
  set_target_pose();
  plan_pose_goal();
  this->move_group_ptr->move();
  ros::Duration(2.0).sleep();
}

void Manipulation::reach_food_item()
{  
  move(joint_group_positions);
  ros::Duration(2.0).sleep();

  // grasp pose
  tf::StampedTransform t_base_grasp;
  t_base_grasp = getTransform(grasp_listener, BASE_LINK_NAME, GRASP_LINK_NAME);
  tf::Transform T_base_grasp;
  // tf::Vector3 p_base_grasp(t_base_grasp.getOrigin());
  tf::Vector3 p_base_grasp;
  double translation_x, translation_y, translation_z;
  std::string grasp_namespace = "/reach_food_item_loop/anygrasp";
  // /reach_food_item_loop/anygrasp/grasp_8/translation/z

  std::string param_name = grasp_namespace + "/grasp_0";
  ros::param::get(param_name + "/translation/x", translation_x);
  ros::param::get(param_name + "/translation/y", translation_y);
  ros::param::get(param_name + "/translation/z", translation_z);

  p_base_grasp.setX(translation_x);
  p_base_grasp.setY(translation_y);
  p_base_grasp.setZ(translation_z);
    
      
  // TODO: remove the following tf line
  tf::Quaternion q_base_grasp(0.7071068, 0.7071068, 0, 0);  // addtional tf from anygrasp to kinova tool frame
  T_base_grasp.setRotation(q_base_grasp);
  T_base_grasp.setOrigin(p_base_grasp);

  tf::Matrix3x3 q_base_tool(t_base_grasp.getBasis());
  q_base_tool = q_base_tool; 

  // grasp orientation, vertical
  geometry_msgs::Vector3 grasp_pose;
  grasp_pose.x = 180/57.3;
  grasp_pose.y = 0/57.3; 
  grasp_pose.z = 90/57.3;

  // T_tool_grasp
  tf::Transform T_tool_grasp;
  T_tool_grasp.setIdentity();
  tf::Vector3 p_tool_grasp(0.0, -0.005, 0.175);
  tf::Quaternion q_tool_grasp(0.00, 0.00, 0.00, 1.00);  // fixed
  T_tool_grasp.setOrigin(p_tool_grasp);
  T_tool_grasp.setRotation(q_tool_grasp);

  // T_grasp_tool
  tf::Transform T_grasp_tool;
  T_grasp_tool = T_tool_grasp.inverse();
  std::cout << "T_grasp_tool: " << std::endl;
  std::cout << "Position: " << T_grasp_tool.getOrigin().getX() << ", " 
                            << T_grasp_tool.getOrigin().getY() << ", "
                            << T_grasp_tool.getOrigin().getZ() << std::endl;
  std::cout << "Orientation: " << T_grasp_tool.getRotation() << std::endl;

  // Determine EE (tool_frame) pose
  tf::Transform T_base_tool;
  T_base_tool = T_base_grasp * T_grasp_tool;
  tf::Vector3 p_base_tool; //(0.0, 0.0, 0.175);
  p_base_tool = T_base_grasp.getBasis() * p_tool_grasp + T_base_grasp.getOrigin();
  std::cout << "p_base_tool: " << std::endl;
  std::cout << "Position: " << p_base_tool.getX() << ", " 
                            << p_base_tool.getY() << ", "
                            << p_base_tool.getZ() << std::endl;

  // pre-grasp
  this->orientation = grasp_pose;
  this->position.x = T_base_tool.getOrigin().x();
  this->position.y = T_base_tool.getOrigin().y();
  this->position.z = T_base_tool.getOrigin().z() + 0.08;
  set_target_pose();
  plan_pose_goal();
  this->move_group_ptr->move();

  std::cout << "Grasp pose: " << std::endl;
  std::cout << "Position: " << T_base_tool.getOrigin().getX() << ", " 
                            << T_base_tool.getOrigin().getY() << ", "
                            << T_base_tool.getOrigin().getZ() << std::endl;
  std::cout << "Orientation: " << grasp_pose << std::endl;
  ros::Duration(1.0).sleep();  

  // pre-grasp
  this->orientation = grasp_pose;
  this->position.x = T_base_tool.getOrigin().x();
  this->position.y = T_base_tool.getOrigin().y();
  this->position.z = T_base_tool.getOrigin().z();
  set_target_pose();
  plan_pose_goal();
  this->move_group_ptr->move();
  ros::Duration(1).sleep();

  goTop();

}

// Set objects for collision detection: 
void Manipulation::set_objects()
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

    collision_objects[0].id = "object";
    collision_objects[0].header.frame_id = "base_link";

    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].CYLINDER;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.01;
    collision_objects[0].primitives[0].dimensions[1] = 0.01;

    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = this->pose.x;
    collision_objects[0].primitive_poses[0].position.y = this->pose.y;
    collision_objects[0].primitive_poses[0].position.z = this->pose.z;

    collision_objects[0].id = "table";
    collision_objects[0].header.frame_id = "base_link";
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 1;
    collision_objects[0].primitives[0].dimensions[1] = 1;
    collision_objects[0].primitives[0].dimensions[2] = 0;

    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.4;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = +0.05;

    collision_objects[0].operation = collision_objects[0].ADD;

    this->planning_scene_ptr->applyCollisionObjects(collision_objects);
}

void Manipulation::remove_objects()
{
    std::vector<std::string> object_ids;
    object_ids.push_back("object");
    object_ids.push_back("table");
    this->planning_scene_ptr->removeCollisionObjects(object_ids);
}


// Gripper Action Commands, TODO: replace with gripper planning group
void Manipulation::close_gripper()
{
    this->gripper_cmd.goal.command.position = 0.8;
    gripper_command.publish(gripper_cmd);
    ROS_WARN("Closing gripper...");
}

void Manipulation::open_gripper()
{
    this->gripper_cmd.goal.command.position = 0;
    gripper_command.publish(gripper_cmd);
    ROS_WARN("Opening gripper...");
}


//not currently used
/*
void Manipulation::closedGripper(trajectory_msgs::JointTrajectory &posture)
{
    //Add finger joints
    posture.joint_names.resize(6);
    posture.joint_names[0] = "left_inner_finger_joint";
    posture.joint_names[1] = "right_inner_knuckle_joint";
    posture.joint_names[2] = "finger_joint";
    posture.joint_names[3] = "right_inner_finger_joint";
    posture.joint_names[4] = "left_inner_knuckle_joint";
    posture.joint_names[5] = "right_outer_knuckle_joint";

    // Set them as closed.
    posture.points.resize(1);
    posture.points[0].positions.resize(6);
    posture.points[0].positions[0] = 0.8;
    posture.points[0].positions[1] = 0.8;
    posture.points[0].positions[2] = 0.8;
    posture.points[0].positions[3] = -0.8;
    posture.points[0].positions[4] = 0.8;
    posture.points[0].positions[5] = 0.8;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void Manipulation::openGripper(trajectory_msgs::JointTrajectory &posture)
{
    posture.joint_names.resize(6);
    posture.joint_names[0] = "left_inner_finger_joint";
    posture.joint_names[1] = "right_inner_knuckle_joint";
    posture.joint_names[2] = "finger_joint";
    posture.joint_names[3] = "right_inner_finger_joint";
    posture.joint_names[4] = "left_inner_knuckle_joint";
    posture.joint_names[5] = "right_outer_knuckle_joint";

    posture.points.resize(1);
    posture.points[0].positions.resize(6);
    posture.points[0].positions[0] = 0;
    posture.points[0].positions[1] = 0;
    posture.points[0].positions[2] = 0;
    posture.points[0].positions[3] = 0;
    posture.points[0].positions[4] = 0;
    posture.points[0].positions[5] = 0;

    posture.points[0].time_from_start = ros::Duration(0.5);
}
*/