#include "manipulation_class.hpp"

#include <iostream>
#include <fstream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;

// Manipulation constructor, init planning_group 
Manipulation::Manipulation(ros::NodeHandle nodeHandle, std::string planning_group)
{
    PLANNING_GROUP = planning_group;
    // grasp_config = nodeHandle.subscribe("/detect_grasps/clustered_grasps", 1, &Manipulation::callback, this); // Subscribe to GPD output
    this->gripper_command = nodeHandle.advertise<control_msgs::GripperCommandActionGoal>("robotiq_2f_85_gripper_controller/gripper_cmd/goal", 10);
    // grasps_visualization_pub_ = nodeHandle.advertise<geometry_msgs::PoseArray>("grasps_visualization", 10);
    grasps_visualization_pub_ = nodeHandle.advertise<visualization_msgs::Marker>("grasps_visualization", 10);

}


// Get robot state and move to specified joint_group_position: 
void Manipulation::getCurrentState()
{

    const robot_state::JointModelGroup *joint_model_group =
        move_group_ptr->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    current_state = move_group_ptr->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
}


// Move to specified joint values: 
void Manipulation::move(std::vector<double>)
{
    move_group_ptr->setJointValueTarget(joint_group_positions);
    move_group_ptr->move();
}

// Set joint values: 
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


// Preset arm positions: 
void Manipulation::goRight()
{
    getCurrentState();
    ROS_INFO("Moving to Right Position");
    setJointGroup(pi / 1.8, pi / 3, -pi / 3, pi / 2, 0, pi / 3, pi / 7);
    move(joint_group_positions);
}

void Manipulation::goLeft()
{
    getCurrentState();
    ROS_INFO("Moving to Left Position");
    setJointGroup(-pi / 1.8, pi / 3, pi / 3, pi / 2, 0, pi / 3, 2.8);
    move(joint_group_positions);
}

void Manipulation::goVertical()
{
    getCurrentState();
    ROS_INFO("Moving to Vertical Position");
    setJointGroup(0, 0, 0, 0, 0, 0, 0);
    move(joint_group_positions);
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

void Manipulation::trajectory_saving()
{
      // Extract waypoints
    std::vector<trajectory_msgs::JointTrajectoryPoint> waypoints = this->my_plan.trajectory_.joint_trajectory.points;

    // Save waypoints to a file
    std::ofstream outfile("/home/zing/mealAssistiveRobot/sla_ws/src/object_grasping/data/trajectory_waypoints.txt");
    for (const auto &point : waypoints)
    {
        for (const auto &position : point.positions)
        {
            outfile << position << " ";
        }
        outfile << "\n";
    }
    outfile.close();


    // // Read waypoints from the file
    // std::ifstream infile("trajectory_waypoints.txt");
    // std::string line;
    // while (std::getline(infile, line))
    // {
    //     std::istringstream iss(line);
    //     trajectory_msgs::JointTrajectoryPoint replay_point;
    //     double value;
    //     while (iss >> value)
    //     {
    //         replay_point.positions.push_back(value);
    //     }

    //     // Send the replay_point to your robot's controller for execution
    //     // You would need to implement the publishing to the robot's controller here
    // }
    // infile.close();
}

void Manipulation::trajectory_replay(std::string waypoint_path)
{
    // Load waypoints from the file
    std::ifstream infile(waypoint_path);
    std::string line;
    std::vector<trajectory_msgs::JointTrajectoryPoint> waypoints;
    while (std::getline(infile, line))
    {
        std::istringstream iss(line);
        trajectory_msgs::JointTrajectoryPoint replay_point;
        double value;
        while (iss >> value)
        {
            replay_point.positions.push_back(value);
        }
        waypoints.push_back(replay_point);
    }
    infile.close();

    // Load robot model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const robot_model::RobotModelPtr &robot_model = robot_model_loader.getModel();

    // Apply joint limits clipping to the waypoint positions
    for (auto &waypoint : waypoints)
    {
        for (size_t j = 0; j < waypoint.positions.size(); ++j)
        {
            const std::string &joint_name = move_group_ptr->getJointNames()[j];
            const robot_model::JointModel *joint_model = robot_model->getJointModel(joint_name);
            double joint_min = joint_model->getVariableBounds(joint_name).min_position_;
            double joint_max = joint_model->getVariableBounds(joint_name).max_position_;
            waypoint.positions[j] = std::max(joint_min, std::min(joint_max, waypoint.positions[j]));
        }
    }

    // Create a trajectory
    trajectory_msgs::JointTrajectory trajectory;
    trajectory.joint_names = move_group_ptr->getJointNames();

    // Modify the first and last waypoints for gradual acceleration and deceleration
    double max_speed = 0.8; // Modify this value as needed
    trajectory_msgs::JointTrajectoryPoint modified_first_waypoint = waypoints.front();
    trajectory_msgs::JointTrajectoryPoint modified_last_waypoint = waypoints.back();

    // Adjust positions of modified_first_waypoint and modified_last_waypoint if needed

    // Add gradual acceleration and deceleration
    modified_first_waypoint.time_from_start = ros::Duration(1.0); // Adjust as needed
    modified_last_waypoint.time_from_start = ros::Duration(1.0); // Adjust as needed

    // Insert modified_first_waypoint at the beginning of the trajectory
    trajectory.points.push_back(modified_first_waypoint);

    // Populate the trajectory with waypoints
    for (const auto &waypoint : waypoints)
    {
        trajectory_msgs::JointTrajectoryPoint traj_point;
        traj_point.positions = waypoint.positions;
        traj_point.time_from_start = ros::Duration(1.0); // No delay between waypoints
        trajectory.points.push_back(traj_point);
    }

    // Insert modified_last_waypoint at the end of the trajectory
    trajectory.points.push_back(modified_last_waypoint);

    // Execute the trajectory using the action interface
    TrajectoryClient client("/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory", true);
    client.waitForServer();
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = trajectory;
    client.sendGoal(goal);
    client.waitForResult();
}


void Manipulation::goTop()
{
    getCurrentState();
    ROS_INFO("Moving to Top Position");
    // setJointGroup(0.043, -0.1824, -0.0133, 2.208, -0.0188, 0.7828, -1.524);  // original
    // this config is closer to Home
    // setJointGroup(0, 6.12427-6.28, 3.15113, 3.9903-6.28, 0.0248641, 5.68259-6.28, 1.58098);  // normal, for pickup
    setJointGroup(6.26513, 0.179158, 3.20562, 4.66534-6.28, 0.036471, 5.01553-6.28, 1.57978);  // vertical, for pickup
    // setJointGroup(0.319602, 6.05876-6.28, 3.1419, 4.05812-6.28, 0.0315863, 5.53112-6.28, 1.56028);  // a little tilt, for feeding
    
    move(joint_group_positions);
}


void Manipulation::goDown()
{
  getCurrentState();
  ROS_INFO("Moving to Down Position");
  // setJointGroup(1.15159, 0.618337, 0.593246, 2.20513, 5.16716, 0.744263, 0.126573);
  setJointGroup(1.05848, 0.377619, 3.17121, 3.86836-6.28, 0.0377704, 0.0603388, 1.58758);
  move(joint_group_positions);
}


// void Manipulation::goTop()
// {
//     getCurrentState();
//     ROS_INFO("Moving to Top Position");
//     setJointGroup(0.0430224, 5.93703 + 6.28, 6.26932 - 6.28, 2.19543, 6.25847 - 6.28, 0.916653, 4.7587 - 6.28);
//     move(joint_group_positions);
// }

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

void Manipulation::goPostGrasp()
{
    getCurrentState();
    ROS_INFO("Moving to Post Grasp Position");
    setJointGroup(-0.06, 0.4309, 0.0458, 2.1915, -0.0325, -1.043, -1.567);
    move(joint_group_positions);
    // open_gripper();
}

void Manipulation::goPlace()
{
    getCurrentState();
    ROS_INFO("Moving to Place Position");
    setJointGroup(pi, pi / 4, 0, pi / 4, 0, pi / 2, -pi / 2);
    move(joint_group_positions);
}

void Manipulation::goWait()
{
    getCurrentState();
    ROS_INFO("Moving to Waiting Position");
    setJointGroup(0, pi / 5, 0, pi / 2, 0, -pi / 5, -pi / 2);
    move(joint_group_positions);
}


void Manipulation::goTemp()
{
    getCurrentState();
    ROS_INFO("Moving to Waiting Position");
    setJointGroup(1.38351, 0.484841, 3.13812, 4.04086-6.28, 5.92295, 1.08355, 0.26627);
    move(joint_group_positions);
}

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
  std::cout << "Transform from '" << source_frame << "' to '" << target_frame << "':" << std::endl;
  std::cout << "Translation: (x=" << T_target_source.getOrigin().x()
            << ", y=" << T_target_source.getOrigin().y()
            << ", z=" << T_target_source.getOrigin().z() << ")" << std::endl;
  std::cout << "Rotation: (x=" << T_target_source.getRotation().x()
            << ", y=" << T_target_source.getRotation().y()
            << ", z=" << T_target_source.getRotation().z()
            << ", w=" << T_target_source.getRotation().w() << ")" << std::endl;
  return T_target_source;
}


// Get the best prediction from anygrasp
void Manipulation::reach_anygrasp()
{

  tf::StampedTransform T_base_grasp;
  T_base_grasp = getTransform(grasp_listener, BASE_LINK_NAME, GRASP_LINK_NAME);
  geometry_msgs::Vector3 grasp_pose;
  tf::Matrix3x3 rotation_y;
  rotation_y.setRPY(0, M_PI / 2.0 - 0.02, 0); // Rotate 90 degrees about the y-axis
  tf::Matrix3x3 rotation_z;
  rotation_z.setRPY(0, 0,  - M_PI / 2.0 + 0.02); // Rotate -90 degrees about the z-axis
  tf::Matrix3x3 grasp_orientation = T_base_grasp.getBasis();
  grasp_orientation = grasp_orientation * rotation_y * rotation_z;
  grasp_orientation.getRPY(grasp_pose.x, grasp_pose.y, grasp_pose.z);



  tf::Vector3 translation_offset(0, 0, 0.01);  // offset about z-axis
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
  rotation_y.setRPY(0, M_PI / 2.0, 0); // Rotate 90 degrees about the y-axis
  std::cout << "Rotation Matrix: 90 deg about y" << std::endl;
  for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
          std::cout << rotation_y[i][j] << " ";
      }
      std::cout << std::endl;
  }

  tf::Matrix3x3 rotation_z;
  rotation_z.setRPY(0, 0, - M_PI / 2.0); // Rotate -90 degrees about the z-axis
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

  // setJointGroup(0.043, -0.1824, -0.0133, 2.208, -0.0188, 0.7828, -1.524);
  move(joint_group_positions);
  ros::Duration(2.0).sleep();

  tf::StampedTransform T_base_grasp;
  T_base_grasp = getTransform(grasp_listener, BASE_LINK_NAME, GRASP_LINK_NAME);
  
  // set grasp orientation
  geometry_msgs::Vector3 grasp_pose;
  grasp_pose.x = 176.594/57.3;
  grasp_pose.y = -0.278729/57.3; 
  grasp_pose.z = 90.8153/57.3;
  std::cout << "x: " << grasp_pose.x << std::endl;

  tf::Vector3 grasp_translation(T_base_grasp.getOrigin().x(),
                                 T_base_grasp.getOrigin().y(),
                                 T_base_grasp.getOrigin().z());

  // reach pre-grasp pose
  this->orientation = grasp_pose;
  this->position.x = T_base_grasp.getOrigin().x() - 0.027;
  this->position.y = T_base_grasp.getOrigin().y();  
  this->position.z = T_base_grasp.getOrigin().z() + 0.08 + 0.17; 

  std::cout << grasp_translation.x() << std::endl;
  std::cout << grasp_translation.y() << std::endl;
  std::cout << grasp_translation.z() << std::endl;
  
  std::cout << grasp_pose.x*57.3 << std::endl;
  std::cout << grasp_pose.y*57.3 << std::endl;
  std::cout << grasp_pose.z*57.3 << std::endl;

  set_target_pose();
  plan_pose_goal();
  this->move_group_ptr->move();
  // ros::Duration(1.0).sleep();

  // reach grasp pose
  this->position.z = T_base_grasp.getOrigin().z() + 0.17; 
  set_target_pose();
  plan_pose_goal();
  this->move_group_ptr->move();
  // ros::Duration(1.0).sleep();

  // reach pre-grasp pose
  this->position.z = T_base_grasp.getOrigin().z() + 0.08 + 0.17; 
  set_target_pose();
  plan_pose_goal();
  this->move_group_ptr->move();
  ros::Duration(1.0).sleep();  

  // // feeding
  // ROS_WARN("Feeding!");
  // // grasp_pose.x = -73.6589/57.3;
  // // grasp_pose.y = -179.238/57.3; 
  // // grasp_pose.z = 85.8695/57.3;

  // grasp_pose.x = -104.242/57.3;
  // grasp_pose.y = -175.607/57.3; 
  // grasp_pose.z = 85.9316/57.3;

  // this->orientation = grasp_pose;
  // // this->position.x = -0.0714748;
  // // this->position.y = 0.276489;  
  // // this->position.z = 0.648815;

  // this->position.x = -0.0204952;
  // this->position.y = 0.302304;  
  // this->position.z = 0.626752;
  // set_target_pose();
  // plan_pose_goal();
  // // ompl_plan();
  // this->move_group_ptr->move();
  setJointGroup(0.318251, 6.06139-6.28, 3.13404, 4.06107-6.28, 0.0316907, 6.10643-6.28, 1.55993);
  move(joint_group_positions);
  ros::Duration(2.0);

  bite_transfer();

}

void Manipulation::bite_transfer_replay()
{
  std::string bite_transfer_waypoints = "/home/zing/mealAssistiveRobot/sla_ws/src/object_grasping/data/trajectory_waypoints.txt";
  trajectory_replay(bite_transfer_waypoints);
}

// Set objects for collision detection: 
void Manipulation::set_objects()
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

    collision_objects[0].id = "object";
    collision_objects[0].header.frame_id = "base_link"; //world for NERVE workstation

    // Define the primitive and its dimensions
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].CYLINDER;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.01; // height
    collision_objects[0].primitives[0].dimensions[1] = 0.01; // radius

    // Define the pose of the object
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = this->pose.x;
    collision_objects[0].primitive_poses[0].position.y = this->pose.y;
    collision_objects[0].primitive_poses[0].position.z = this->pose.z;
    //collision_objects[0].operation = collision_objects[0].ADD;

    // Define the primitive and its dimensions.
    collision_objects[0].id = "table";
    collision_objects[0].header.frame_id = "base_link"; // world for NERVE workstation
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 1;
    collision_objects[0].primitives[0].dimensions[1] = 1;
    collision_objects[0].primitives[0].dimensions[2] = 0;

    // Define the pose of the table.
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.4;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = +0.05;

    collision_objects[0].operation = collision_objects[0].ADD;

    this->planning_scene_ptr->applyCollisionObjects(collision_objects);
}

// Remove objects from collision detection: 
void Manipulation::remove_objects()
{
    std::vector<std::string> object_ids;
    object_ids.push_back("object");
    object_ids.push_back("table");
    this->planning_scene_ptr->removeCollisionObjects(object_ids);
}


// Gripper Action Commands, TODO: replace with gripper planning group! 

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


// Working Gripper Commands for the pick() pipeline, not currently used
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