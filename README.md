# Feeding task

This package demonstrates a feeding 

Note that all experiments are conducted on Ubuntu 20.04 LTS (Focal Fossa) with a Kinova Gen3 7DOF arm.

This project is implemented with a finite state machine(FSM) utilizing smach.

## Finite state machine for feeding task

The feeding task mainly focuses on those 5 subtasks:

- Door open
- Bowl grasping
- Utensil fetching
- Food acquisition
- Food transfer

### An alternative motion planning logic for feeding cycle
 `plan_to_pre_skewer_pose`
 `plan_to_skewer_pose`
 `plan_to_post_skewer_pose`
 `execute_to_pre_skewer_pose`
 `execute_to_skewer_pose`
 `execute_to_post_skewer_pose`

### 1. Door opening
 `move_to_initial_door_open_position`
 `open_gripper_for_door_handle_grasping`
 `move_to_door_handle_pose`
 `grasp_door_handle`
 `open_door_with_admittance_control`
 `move_to_post_door_open_position` 

### 2. Bowl grasping
 `move_to_bowl_grasping_initial_position`
 `open_gripper_for_bowl_grasping`
 `bowl_grasp_generator`
 `move_to_bowl_handle_pose`
 `grasp_bowl_handle`
 `move_to_bowl_grasping_post_position`

### 3. Bowl upright transfer
 `plan_for_bowl_upright_transfer`
 `execution_for_bowl_upright_transfer`
 `release_bowl`

### 4. Utensil fetching
 `get_utensil`

### 5. Food selection and skewer
 `move_to_feeding_start_position`
 `food_item_selector`
 `move_to_pre_skewer_pose`
 `move_to_skewer_pose`
 `move_to_post_skewer_pose`
 `move_to_feeding_initial_position`
 `skewer_status_check`

### 6. Food transfer
 `plan_to_feeding_pose`
 `execute_to_feeding_pose`



算了，有时间再写





**Note:** All following contents are of an older version (deprecated).

## Requirements

First install those packages to your workspace.

1. [ros_kortex](https://github.com/Kinovarobotics/ros_kortex): *Necessary*. Driver for robot motion control.

1. [ros_kortex_vision](https://github.com/Kinovarobotics/ros_kortex_vision): *Necessary*. ROS package to launch RGBD sensor mounted on Kinova Gen3.

3. ~~[apriltag_ros](https://github.com/AprilRobotics/apriltag_ros): *Optional*. ROS package for QR code detection and localization. In this demo, `apriltag` is used in door opening part only. Note that you may first build the source package [apriltag](https://github.com/AprilRobotics/apriltag) first and then [apriltag_ros](https://github.com/AprilRobotics/apriltag_ros). For more information, refer to official document.~~

4. [anygrasp_sdk](https://github.com/graspnet/anygrasp_sdk/tree/main): *Necesssary*. In this demo, AnyGrasp is used in handle grasp generation and food item localization. Note that you need first to apply for a license registration following the [official tutorial](https://github.com/graspnet/anygrasp_sdk/tree/main/license_registration).

## Installation

1. Clone packages ~~`face_detection` (optional, since not used yet),~~ `feeding_task`  to your workspace.
2. Copy package `grasp_generation` into directory `anygrasp_sdk`. Note that you need to modify this directory using your own license, i.e., change the `src/license` with your license and replace/place `gsnet.so` and `lib_cxx.so` with your python version. More information please refer to [AnyGrasp Detection Demo](AnyGrasp Detection Demo).
3. (Maybe necessary) Run the following command to automatically install all debian dependencies listed in each package.xml file.

```bash
rosdep install --from-paths src --ignore-src -y
```

4. Build. Use `catkin build` to build your workspace. 

   - If error occurs, you can try isolated build, for example, you may run `catkin build anygrasp_generation` before `catkin build feeding_task` since `feeding_task` relies on `anygrasp_generation`.
   
   

## Usage

1. Robot

```bash
roslaunch kinova_vision kinova_vision_rgbd.launch
roslaunch kortex_driver kortex_driver.launch
```

1. Bowl grasping

```bash
roslaunch anygrasp_generation anygrasp_generation_objects.launch
roslaunch feeding_task anygrasp_bowl_handle_grasp.launch
```

1. Utensil fetching

```
roslaunch feeding_task get_utensil
```

1. Food acquisition and transfer

```bash
roslaunch anygrasp_generation anygrasp_generation_and_publish_food.launch
roslaunch feeding_task anygrasp_food_item_fetch.launch
```

