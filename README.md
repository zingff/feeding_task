# Feeding task

Those packages demonstrate a feeding pipeline including bowl grasping, utensil fetching, food acquisition and food transfer. 

Note that all experiments are conducted on Ubuntu 20.04 LTS (Focal Fossa) with a Kinova Gen3 7DOF arm.

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

