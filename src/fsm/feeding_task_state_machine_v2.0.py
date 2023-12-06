#!/usr/bin/env python

# Note: a finite state machine for feeding task
# Note: name of the manipulator: sausage lip arm (SLA)

# Update log:
  # This version is a complete state machine for feeding pipeline
  # with real time collision detection logic,
  # functional trigger

# TODO List: 
# √ concurrent sm for collision detection (cd is not always necessary)
  # √ change to a threading logic
# √ add a joint planner in skmp 
# √ modify the skewer action in a cb in food item selector 
# √ add a loop for skewer task 
# √ add simple cpe to replace the original one 
# * modify the cd to be dynamic tuning/adaptive mode
# √ add a function to check if a skewer is successful (pc)
# √ update the collision detection logic in fsm
# √ modify skewer mpe
# √ update the collision detection logic in both fsm and cd
# √ add pre states for feeding task:
  # √ get utensil
  # √ bowl upright transfer
  # √ grasp bowl
  # √ bowl grasp generator
  # √ door open
    # √ move to door open initial position
    # √ open gripper for door handle grasping
    # . (deprecated) qr recognition, to remove, leave it bgr
    # √ move to door handle pose (reach at)
    # √ grasp door handle
    # √ open door
    # √ move to post door open position
# * add force sensing for skewer state
# √ add functional trigger for feeding task
# √ format all node/service/topic name with prefix
  # . (deprecated) ftsm or feeding_task
# * consider the outcomes of the fsm
# √ mergy ku into kmp
# √ add jlc in gcp
# √ change service name to static constant form
# * change bowl handle logic
# * design voice control interface
# * add face detection logic
# * change the info color logic by subtasks


import argparse
import anygrasp_generation
import kortex_driver
import kortex_motion_planning
from kortex_motion_planning.msg import JointPositions
import rospy
import inspect
import threading
import smach
import smach_ros
import std_srvs.srv
from tf2_msgs import msg
import anygrasp_generation.srv
import kortex_motion_planning.srv
import feeding_task.srv
import door_open_task.srv
from geometry_msgs.msg import Pose, Transform, Quaternion, Vector3
from trajectory_msgs.msg import JointTrajectory
import tf2_ros
import math
import tf.transformations
from std_msgs.msg import Bool, Empty
import threading
import kortex_driver.srv
import kortex_driver.msg
import numpy as np


collision_detected = False  # deprecated
COLLISION_DETECTION_TOPIC = "/fsm/collision_detection"
STOP_SERVICE = "/base/stop"
SIMPLE_JMPE_SERVICE = "/kortex_simple_joint_motion_service"
SIMPLE_CMPE_SERVICE = "/kortex_simple_cartesian_motion_service"
GRIPPER_COMMAND_SERVICE = "/kortex_gripper_command_service"
DOOR_OPEN_SERVICE = "/feeding_task/door_open_service"
BOWL_GRASP_GENERATOR_SERVICE = "/bowl_grasp_generator"
MOTION_EXECUTOR_SERVICE = "/motion_execution_server"
MOTION_PLANNER_SERVICE = "/motion_planning_server"
FOOD_ITEM_SELECTOR_SERVICE = "/grasp_generator"
GET_UTENSIL_SERVICE = "/kortex_get_utensil_service"
SKEWER_STATUS_CHECKER_SERVICE = "skewer_status_checker"



outcomes_sm = [
    # An alternative motion planning logic for feeding cycle
      'plan_to_pre_skewer_pose'
    , 'plan_to_skewer_pose'
    , 'plan_to_post_skewer_pose'
    , 'execute_to_pre_skewer_pose'
    , 'execute_to_skewer_pose'
    , 'execute_to_post_skewer_pose'

    # deprecated/not used yet
    , 'collision_detected'  # deprecated
    , 'normal'  # deprecated
    , 'collision_status'  # deprecated
    , 'collision_detection'  # deprecated
    , 'succeeded'
    , 'aborted'
    , 'preempted'
    
    # 1. Door opening
    , 'move_to_initial_door_open_position'
    , 'open_gripper_for_door_handle_grasping'
    , 'move_to_door_handle_pose'
    , 'grasp_door_handle'
    , 'open_door_with_admittance_control'
    , 'move_to_post_door_open_position' 
    
    # 2. Bowl grasping
    , 'move_to_bowl_grasping_initial_position'
    , 'open_gripper_for_bowl_grasping'
    , 'bowl_grasp_generator'
    , 'move_to_bowl_handle_pose'
    , 'grasp_bowl_handle'
    , 'move_to_bowl_grasping_post_position'

    # 3. Bowl upright transfer
    , 'plan_for_bowl_upright_transfer'
    , 'execution_for_bowl_upright_transfer'
    , 'release_bowl'
    
    # 4. Utensil fetching
    , 'get_utensil'
    
    # 5. Food selection and skewer
    , 'move_to_feeding_start_position'
    , 'food_item_selector'
    , 'move_to_pre_skewer_pose'
    , 'move_to_skewer_pose'
    , 'move_to_post_skewer_pose'
    , 'move_to_feeding_initial_position'
    , 'skewer_status_check'
    
    # 6. Food transfer
    , 'plan_to_feeding_pose'
    , 'execute_to_feeding_pose'
               ]


# Input and output keys for the states
input_keys_sm = [
      'motion_plan'
    , 'success'
    , 'update_anygrasp'
    , 'success'
    , 'anygrasp_transforms'
    , 'anygrasp_transform'
    , 'pre_skewer_pose'
    , 'skewer_pose'
    , 'post_skewer_pose'
    , 'feeding_pose'
    , 'collision_status'
    , 'bowl_placement_pose'
    , 'bowl_handle_pose'
                 ]


transition_sm = {
    # An alternative motion planning logic for feeding cycle
      'plan_to_pre_skewer_pose': 'plan_to_pre_skewer_pose'
    , 'plan_to_skewer_pose': 'plan_to_skewer_pose'
    , 'plan_to_post_skewer_pose': 'plan_to_post_skewer_pose'
    , 'execute_to_pre_skewer_pose': 'execute_to_pre_skewer_pose'
    , 'execute_to_skewer_pose': 'execute_to_skewer_pose'
    , 'execute_to_post_skewer_pose': 'execute_to_post_skewer_pose'    
        
    # deprecated/not used yet
    , 'aborted': 'aborted'
    , 'collision_detected': 'aborted'
    , 'collision_detection': 'collision_detection'
    
    # 1. Door opening
    , 'move_to_initial_door_open_position': 'move_to_initial_door_open_position'
    , 'open_gripper_for_door_handle_grasping': 'open_gripper_for_door_handle_grasping'
    , 'move_to_door_handle_pose': 'move_to_door_handle_pose'
    , 'grasp_door_handle': 'grasp_door_handle'    
    , 'open_door_with_admittance_control': 'open_door_with_admittance_control'
    , 'move_to_post_door_open_position': 'move_to_post_door_open_position'
    
    # 2. Bowl grasping
    , 'move_to_bowl_grasping_initial_position': 'move_to_bowl_grasping_initial_position'
    , 'open_gripper_for_bowl_grasping': 'open_gripper_for_bowl_grasping'
    , 'bowl_grasp_generator': 'bowl_grasp_generator'
    , 'move_to_bowl_handle_pose': 'move_to_bowl_handle_pose'
    , 'grasp_bowl_handle': 'grasp_bowl_handle'
    , 'move_to_bowl_grasping_post_position': 'move_to_bowl_grasping_post_position'    
    
    # 3. Bowl upright transfer
    , 'plan_for_bowl_upright_transfer': 'plan_for_bowl_upright_transfer'
    , 'execution_for_bowl_upright_transfer': 'execution_for_bowl_upright_transfer'
    , 'release_bowl': 'release_bowl'
    
    # 4. Utensil fetching
    , 'get_utensil': 'get_utensil'

    # 5. Food selection and skewer
    , 'move_to_feeding_start_position': 'move_to_feeding_start_position'
    , 'food_item_selector': 'food_item_selector'
    , 'move_to_pre_skewer_pose': 'move_to_pre_skewer_pose'
    , 'move_to_skewer_pose': 'move_to_skewer_pose'
    , 'move_to_post_skewer_pose': 'move_to_post_skewer_pose'
    , 'move_to_feeding_initial_position' : 'move_to_feeding_initial_position'
    , 'skewer_status_check': 'skewer_status_check'    

    # 6. Food transfer
    , 'plan_to_feeding_pose': 'plan_to_feeding_pose'
    , 'execute_to_feeding_pose': 'execute_to_feeding_pose'
}


# from internal state machine to global variables
# keys used within the specific state: variable in sm's userdata (i.e., sm.userdata)
# io keys: userdata.data
remapping_sm = {
      'motion_plan': 'motion_plan'
    , 'update_anygrasp': 'update_anygrasp'
    , 'anygrasp_transforms': 'anygrasp_transforms'
    , 'success': 'success'
    , 'anygrasp_transform': 'anygrasp_transform'
    , 'pre_skewer_pose': 'pre_skewer_pose'
    , 'skewer_pose': 'skewer_pose'
    , 'feeding_pose': 'feeding_pose'
    , 'bowl_placement_pose': 'bowl_placement_pose'
    , 'bowl_handle_pose': 'bowl_handle_pose'
}

def normalize_quaternion(quaternion):
    return quaternion / np.linalg.norm(quaternion)


def transform_to_matrix(transform):
    translation = [transform.translation.x, transform.translation.y, transform.translation.z]
    rotation = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
    return tf.transformations.compose_matrix(translate=translation, angles=tf.transformations.euler_from_quaternion(rotation))


def matrix_to_transform(matrix):
    transform = Transform()
    transform.translation.x, transform.translation.y, transform.translation.z = tf.transformations.translation_from_matrix(matrix)
    quat = tf.transformations.quaternion_from_matrix(matrix)
    transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w = quat
    return transform


def invert_transform(transform):
    """
    Inverts a geometry_msgs.msg.Transform object.
    
    Args:
        transform (geometry_msgs.msg.Transform): The transform to be inverted.
    
    Returns:
        geometry_msgs.msg.Transform: The inverted transform.
    """
    matrix = tf.transformations.compose_matrix(
        translate=[transform.translation.x, transform.translation.y, transform.translation.z],
        angles=tf.transformations.euler_from_quaternion([
            transform.rotation.x, 
            transform.rotation.y, 
            transform.rotation.z, 
            transform.rotation.w
        ])
    )

    inverse_matrix = tf.transformations.inverse_matrix(matrix)

    inverse_transform = Transform()
    inverse_transform.translation.x, inverse_transform.translation.y, inverse_transform.translation.z = tf.transformations.translation_from_matrix(inverse_matrix)
    quat = tf.transformations.quaternion_from_matrix(inverse_matrix)
    inverse_transform.rotation.x, inverse_transform.rotation.y, inverse_transform.rotation.z, inverse_transform.rotation.w = quat

    return inverse_transform


class StateOutputStyle:
    success = '\033[1;38;5;46m'  # green
    failure = '\033[1;31m'  # red
    default = '\033[0m'  # default
    
    grey = '\033[1;30m' 
    red = '\033[1;31m' 
    green = '\033[1;32m' 
    yellow = '\033[1;33m' 
    blue = '\033[1;34m' 
    purple = '\033[1;35m'
    sky_blue = '\033[1;36m'
    
    bg_red = '\033[1;41m'
    bg_green = '\033[1;42m'
    bg_yellow = '\033[1;43m'
    bg_blue = '\033[1;44m'
    bg_purple = '\033[1;45m'
    bg_sky_blue = '\033[1;46m'
    
    collision = '\033[1;38;5;123m'  # cyan
    stop_call = '\033[1;38;5;121m'
    striking  = '\033[1;38;5;190m'
    normal = '\033[38;5;152m'
  
  
def success_loginfo(msg):
    rospy.loginfo(StateOutputStyle.success + msg + StateOutputStyle.default)


def failure_loginfo(msg):
    rospy.loginfo(StateOutputStyle.failure + msg + StateOutputStyle.default)

    
def striking_loginfo(msg):
    rospy.loginfo(StateOutputStyle.striking + msg + StateOutputStyle.default)

    
def collision_detected_loginfo(msg):
    rospy.loginfo(StateOutputStyle.collision + "Collision detected in state: "  + msg  + ", watch out!" + StateOutputStyle.default)


def stop_loginfo():
    rospy.loginfo(StateOutputStyle.collision + "Calling kortex stop service..." + StateOutputStyle.default)


def normal_loginfo(msg):
    rospy.loginfo(StateOutputStyle.normal + msg + StateOutputStyle.default)


def degrees2Radians(degrees):
    return (math.pi / 180.0) * degrees


class CustomStateMachine(smach.StateMachine):
    def __init__(self, custom_outcomes, input_keys=[], output_keys=[]):
        super(CustomStateMachine, self).__init__(custom_outcomes, input_keys, output_keys)


def generic_state_callback(userdata, response, next_state_on_success, next_state_on_failure):
    stack = inspect.stack()
    caller_frame = stack[1]
    function_name = caller_frame.function
    state_name = function_name.replace('_callback', '')

    if response.success:
        success_loginfo(f"{state_name}: success")
        return next_state_on_success
    else:
        failure_loginfo(f"{state_name}: failed")
        return next_state_on_failure


def state_skewer_status_check_callback(userdata, response, next_state_on_success, next_state_on_failure):
    stack = inspect.stack()
    caller_frame = stack[1]
    function_name = caller_frame.function
    state_name = function_name.replace('_callback', '')

    if response.skewer_status:
        success_loginfo(f"{state_name}: success")
        normal_loginfo(f"skewer_status: {response.skewer_status} ")
        normal_loginfo(f"Confidence: {response.point_number} ")
        return next_state_on_success
    else:
        failure_loginfo("No food item detected on the fork, retrying...")
        failure_loginfo(f"{state_name}: failed")
        return next_state_on_failure


def generic_userdata_state_callback(userdata, response, next_state_on_success, next_state_on_failure, success_flag_value):
    stack = inspect.stack()
    caller_frame = stack[1]
    function_name = caller_frame.function
    state_name = function_name.replace('_callback', '')

    if response.success:
        success_loginfo(f"{state_name}: success")
        userdata.success = success_flag_value
        return next_state_on_success
    else:
        failure_loginfo(f"{state_name}: failed")
        userdata.success = not success_flag_value
        return next_state_on_failure


# Define states
def move_to_initial_door_open_position_callback(userdata, response):
    return generic_state_callback(userdata, 
                                  response, 
                                  'open_gripper_for_door_handle_grasping', 
                                  'aborted')


class move_to_initial_door_open_position(smach_ros.ServiceState):
    def __init__(self
                 , target_position
                 , input_keys_sm
                 , outcomes_sm
                 , apply_collision_detection = False):
        super(move_to_initial_door_open_position, self).__init__(
            service_name=SIMPLE_JMPE_SERVICE,
            service_spec=kortex_motion_planning.srv.KortexSimpleJmpe,
            request=kortex_motion_planning.srv.KortexSimpleJmpeRequest(
              target_position),
            response_slots=['success'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=move_to_initial_door_open_position_callback
        )
        self.collision_status = False
        self.apply_collision_detection = apply_collision_detection
        
    def execute(self, userdata):
      if self.apply_collision_detection:
        stop_client = rospy.ServiceProxy(STOP_SERVICE, kortex_driver.srv.Stop)
        collision_thread_stop = threading.Event()

        def collision_detection():
            
            def callback(msg):
                if msg.data:
                    stop_client(kortex_driver.msg.Empty())
                    stop_loginfo()
                    self.collision_status = True
                    collision_thread_stop.set()

            subscriber = rospy.Subscriber(COLLISION_DETECTION_TOPIC, Bool, callback)

            while not rospy.is_shutdown():
                if collision_thread_stop.is_set():
                  break
                rospy.sleep(0.1)
                
            subscriber.unregister()
            normal_loginfo("Collision detection thread terminated.")

        collision_thread = threading.Thread(target=collision_detection)
        collision_thread.start()

        outcome = super(move_to_initial_door_open_position, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        normal_loginfo("Collision detection thread joined successfully.")
        
        if self.collision_status:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            self.collision_status = False
            return str(self.__class__.__name__)
      else:
        outcome = super(move_to_initial_door_open_position, self).execute(userdata)
          
      return outcome


def open_gripper_for_door_handle_grasping_callback(userdata, response):
    return generic_state_callback(userdata, 
                                  response, 
                                  'move_to_door_handle_pose', 
                                  'aborted')


# . update cd
class open_gripper_for_door_handle_grasping(smach_ros.ServiceState):
    def __init__(self
                 , target_position
                 , input_keys_sm
                 , outcomes_sm
                 , apply_collision_detection = False):
        super(open_gripper_for_door_handle_grasping, self).__init__(
            service_name=GRIPPER_COMMAND_SERVICE,
            service_spec=kortex_motion_planning.srv.SendGripperCommand,
            request=kortex_motion_planning.srv.SendGripperCommandRequest(
              target_position),
            response_slots=['success'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=open_gripper_for_door_handle_grasping_callback
        )
        self.collision_status = False
        self.apply_collision_detection = apply_collision_detection
        
    def execute(self, userdata):
      if self.apply_collision_detection:
        stop_client = rospy.ServiceProxy(STOP_SERVICE, kortex_driver.srv.Stop)
        collision_thread_stop = threading.Event()

        def collision_detection():
            
            def callback(msg):
                if msg.data:
                    stop_client(kortex_driver.msg.Empty())
                    stop_loginfo()
                    self.collision_status = True
                    collision_thread_stop.set()

            subscriber = rospy.Subscriber(COLLISION_DETECTION_TOPIC, Bool, callback)

            while not rospy.is_shutdown():
                if collision_thread_stop.is_set():
                  break
                rospy.sleep(0.1)
                
            subscriber.unregister()
            normal_loginfo("Collision detection thread terminated.")

        collision_thread = threading.Thread(target=collision_detection)
        collision_thread.start()

        outcome = super(open_gripper_for_door_handle_grasping, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        normal_loginfo("Collision detection thread joined successfully.")
        
        if self.collision_status:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            self.collision_status = False
            return str(self.__class__.__name__)
      else:
        outcome = super(open_gripper_for_door_handle_grasping, self).execute(userdata)
          
      return outcome


def move_to_door_handle_pose_callback(userdata, response):
    return generic_state_callback(userdata, 
                                  response, 
                                  'grasp_door_handle', 
                                  'aborted')


# . update cd
class move_to_door_handle_pose(smach_ros.ServiceState):
    def __init__(self
                 , move_to_door_handle_flag
                 , input_keys_sm
                 , outcomes_sm
                 , apply_collision_detection = False):
        super(move_to_door_handle_pose, self).__init__(
            service_name='/feeding_task/reach_apriltag_service',
            service_spec=door_open_task.srv.ReachApriltag,
            request=door_open_task.srv.ReachApriltagRequest(
              move_to_door_handle_flag),
            response_slots=['success'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=move_to_door_handle_pose_callback
        )
        self.collision_status = False
        self.apply_collision_detection = apply_collision_detection
        
    def execute(self, userdata):
      if self.apply_collision_detection:
        stop_client = rospy.ServiceProxy(STOP_SERVICE, kortex_driver.srv.Stop)
        collision_thread_stop = threading.Event()

        def collision_detection():
            
            def callback(msg):
                if msg.data:
                    stop_client(kortex_driver.msg.Empty())
                    stop_loginfo()
                    self.collision_status = True
                    collision_thread_stop.set()

            subscriber = rospy.Subscriber(COLLISION_DETECTION_TOPIC, Bool, callback)

            while not rospy.is_shutdown():
                if collision_thread_stop.is_set():
                  break
                rospy.sleep(0.1)
                
            subscriber.unregister()
            normal_loginfo("Collision detection thread terminated.")

        collision_thread = threading.Thread(target=collision_detection)
        collision_thread.start()

        outcome = super(move_to_door_handle_pose, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        normal_loginfo("Collision detection thread joined successfully.")
        
        if self.collision_status:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            self.collision_status = False
            return str(self.__class__.__name__)
      else:
        outcome = super(move_to_door_handle_pose, self).execute(userdata)
          
      return outcome


def grasp_door_handle_callback(userdata, response):
    return generic_state_callback(userdata, 
                                  response, 
                                  'open_door_with_admittance_control', 
                                  'aborted')


# . update cd
class grasp_door_handle(smach_ros.ServiceState):
    def __init__(self
                 , target_position
                 , input_keys_sm
                 , outcomes_sm
                 , apply_collision_detection = False):
        super(grasp_door_handle, self).__init__(
            service_name=GRIPPER_COMMAND_SERVICE,
            service_spec=kortex_motion_planning.srv.SendGripperCommand,
            request=kortex_motion_planning.srv.SendGripperCommandRequest(
              target_position),
            response_slots=['success'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=grasp_door_handle_callback
        )
        self.collision_status = False
        self.apply_collision_detection = apply_collision_detection
        
    def execute(self, userdata):
      if self.apply_collision_detection:
        stop_client = rospy.ServiceProxy(STOP_SERVICE, kortex_driver.srv.Stop)
        collision_thread_stop = threading.Event()

        def collision_detection():
            
            def callback(msg):
                if msg.data:
                    stop_client(kortex_driver.msg.Empty())
                    stop_loginfo()
                    self.collision_status = True
                    collision_thread_stop.set()

            subscriber = rospy.Subscriber(COLLISION_DETECTION_TOPIC, Bool, callback)

            while not rospy.is_shutdown():
                if collision_thread_stop.is_set():
                  break
                rospy.sleep(0.1)
                
            subscriber.unregister()
            normal_loginfo("Collision detection thread terminated.")

        collision_thread = threading.Thread(target=collision_detection)
        collision_thread.start()

        outcome = super(grasp_door_handle, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        normal_loginfo("Collision detection thread joined successfully.")
        
        if self.collision_status:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            self.collision_status = False
            return str(self.__class__.__name__)
      else:
        outcome = super(grasp_door_handle, self).execute(userdata)
          
      return outcome


def open_door_with_admittance_control_callback(userdata, response):
    return generic_state_callback(userdata, 
                                  response, 
                                  'move_to_post_door_open_position', 
                                  'aborted')


# . update cd
class open_door_with_admittance_control(smach_ros.ServiceState):
    def __init__(self
                 , door_open_flag
                 , input_keys_sm
                 , outcomes_sm
                 , apply_collision_detection = False):
        super(open_door_with_admittance_control, self).__init__(
            service_name=DOOR_OPEN_SERVICE,
            service_spec=door_open_task.srv.DoorOpen,
            request=door_open_task.srv.DoorOpenRequest(
              door_open_flag),
            response_slots=['success'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=open_door_with_admittance_control_callback
        )
        self.collision_status = False
        self.apply_collision_detection = apply_collision_detection
        
    def execute(self, userdata):
      if self.apply_collision_detection:
        stop_client = rospy.ServiceProxy(STOP_SERVICE, kortex_driver.srv.Stop)
        collision_thread_stop = threading.Event()

        def collision_detection():
            
            def callback(msg):
                if msg.data:
                    stop_client(kortex_driver.msg.Empty())
                    stop_loginfo()
                    self.collision_status = True
                    collision_thread_stop.set()

            subscriber = rospy.Subscriber(COLLISION_DETECTION_TOPIC, Bool, callback)

            while not rospy.is_shutdown():
                if collision_thread_stop.is_set():
                  break
                rospy.sleep(0.1)
                
            subscriber.unregister()
            normal_loginfo("Collision detection thread terminated.")

        collision_thread = threading.Thread(target=collision_detection)
        collision_thread.start()

        outcome = super(open_door_with_admittance_control, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        normal_loginfo("Collision detection thread joined successfully.")
        
        if self.collision_status:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            self.collision_status = False
            return str(self.__class__.__name__)
      else:
        outcome = super(open_door_with_admittance_control, self).execute(userdata)
          
      return outcome


def move_to_post_door_open_position_callback(userdata, response):
    return generic_state_callback(userdata, 
                                  response, 
                                  'move_to_bowl_grasping_initial_position', 
                                  'aborted')

# . update cd
class move_to_post_door_open_position(smach_ros.ServiceState):
    def __init__(self
                 , target_position
                 , input_keys_sm
                 , outcomes_sm
                 , apply_collision_detection = False):
        super(move_to_post_door_open_position, self).__init__(
            service_name=SIMPLE_JMPE_SERVICE,
            service_spec=kortex_motion_planning.srv.KortexSimpleJmpe,
            request=kortex_motion_planning.srv.KortexSimpleJmpeRequest(
              target_position),
            response_slots=['success'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=move_to_post_door_open_position_callback
        )
        self.collision_status = False
        self.apply_collision_detection = apply_collision_detection
        
    def execute(self, userdata):
      if self.apply_collision_detection:
        stop_client = rospy.ServiceProxy(STOP_SERVICE, kortex_driver.srv.Stop)
        collision_thread_stop = threading.Event()

        def collision_detection():
            
            def callback(msg):
                if msg.data:
                    stop_client(kortex_driver.msg.Empty())
                    stop_loginfo()
                    self.collision_status = True
                    collision_thread_stop.set()

            subscriber = rospy.Subscriber(COLLISION_DETECTION_TOPIC, Bool, callback)

            while not rospy.is_shutdown():
                if collision_thread_stop.is_set():
                  break
                rospy.sleep(0.1)
                
            subscriber.unregister()
            normal_loginfo("Collision detection thread terminated.")

        collision_thread = threading.Thread(target=collision_detection)
        collision_thread.start()

        outcome = super(move_to_post_door_open_position, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        normal_loginfo("Collision detection thread joined successfully.")
        
        if self.collision_status:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            self.collision_status = False
            return str(self.__class__.__name__)
      else:
        outcome = super(move_to_post_door_open_position, self).execute(userdata)
          
      return outcome



def move_to_bowl_grasping_initial_position_callback(userdata, response):
    return generic_state_callback(userdata, 
                                  response, 
                                  'open_gripper_for_bowl_grasping', 
                                  'aborted')


class move_to_bowl_grasping_initial_position(smach_ros.ServiceState):
    def __init__(self
                 , target_position
                 , input_keys_sm
                 , outcomes_sm
                 , apply_collision_detection = False):
        super(move_to_bowl_grasping_initial_position, self).__init__(
            service_name=SIMPLE_JMPE_SERVICE,
            service_spec=kortex_motion_planning.srv.KortexSimpleJmpe,
            request=kortex_motion_planning.srv.KortexSimpleJmpeRequest(
              target_position),
            response_slots=['success'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=move_to_bowl_grasping_initial_position_callback
        )
        self.collision_status = False
        self.apply_collision_detection = apply_collision_detection
        
    def execute(self, userdata):
      if self.apply_collision_detection:
        stop_client = rospy.ServiceProxy(STOP_SERVICE, kortex_driver.srv.Stop)
        collision_thread_stop = threading.Event()

        def collision_detection():
            
            def callback(msg):
                if msg.data:
                    stop_client(kortex_driver.msg.Empty())
                    stop_loginfo()
                    self.collision_status = True
                    collision_thread_stop.set()

            subscriber = rospy.Subscriber(COLLISION_DETECTION_TOPIC, Bool, callback)

            while not rospy.is_shutdown():
                if collision_thread_stop.is_set():
                  break
                rospy.sleep(0.1)
                
            subscriber.unregister()
            normal_loginfo("Collision detection thread terminated.")

        collision_thread = threading.Thread(target=collision_detection)
        collision_thread.start()

        outcome = super(move_to_bowl_grasping_initial_position, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        normal_loginfo("Collision detection thread joined successfully.")
        
        if self.collision_status:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            self.collision_status = False
            return str(self.__class__.__name__)
      else:
        outcome = super(move_to_bowl_grasping_initial_position, self).execute(userdata)
          
      return outcome


def open_gripper_for_bowl_grasping_callback(userdata, response):
    return generic_state_callback(userdata, 
                                  response, 
                                  'bowl_grasp_generator', 
                                  'aborted')


# . update cd
class open_gripper_for_bowl_grasping(smach_ros.ServiceState):
    def __init__(self
                 , target_position
                 , input_keys_sm
                 , outcomes_sm
                 , apply_collision_detection = False):
        super(open_gripper_for_bowl_grasping, self).__init__(
            service_name=GRIPPER_COMMAND_SERVICE,
            service_spec=kortex_motion_planning.srv.SendGripperCommand,
            request=kortex_motion_planning.srv.SendGripperCommandRequest(
              target_position),
            response_slots=['success'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=open_gripper_for_bowl_grasping_callback
        )
        self.collision_status = False
        self.apply_collision_detection = apply_collision_detection
        
    def execute(self, userdata):
      if self.apply_collision_detection:
        stop_client = rospy.ServiceProxy(STOP_SERVICE, kortex_driver.srv.Stop)
        collision_thread_stop = threading.Event()

        def collision_detection():
            
            def callback(msg):
                if msg.data:
                    stop_client(kortex_driver.msg.Empty())
                    stop_loginfo()
                    self.collision_status = True
                    collision_thread_stop.set()

            subscriber = rospy.Subscriber(COLLISION_DETECTION_TOPIC, Bool, callback)

            while not rospy.is_shutdown():
                if collision_thread_stop.is_set():
                  break
                rospy.sleep(0.1)
                
            subscriber.unregister()
            normal_loginfo("Collision detection thread terminated.")

        collision_thread = threading.Thread(target=collision_detection)
        collision_thread.start()

        outcome = super(open_gripper_for_bowl_grasping, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        normal_loginfo("Collision detection thread joined successfully.")
        
        if self.collision_status:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            self.collision_status = False
            return str(self.__class__.__name__)
      else:
        outcome = super(open_gripper_for_bowl_grasping, self).execute(userdata)
          
      return outcome


def bowl_grasp_generator_callback(userdata, response):
    return generic_userdata_state_callback(userdata, 
                                           response, 
                                           'move_to_bowl_handle_pose', 
                                           'aborted', 
                                           True)
    

class bowl_grasp_generator(smach_ros.ServiceState):
    def __init__(self
                 , request_key_
                 , input_keys_sm
                 , outcomes_sm
                 , apply_collision_detection = False):
        super(bowl_grasp_generator, self).__init__(
            service_name=BOWL_GRASP_GENERATOR_SERVICE,
            service_spec=anygrasp_generation.srv.AnyGraspGeneration,
            # request=anygrasp_generation.srv.AnyGraspGenerationRequest(update_anygrasp),
            request_key=request_key_,
            response_slots=['anygrasp_transforms', 'success'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=bowl_grasp_generator_callback
        )
        self.anygrasp_tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.grasp_namespace = 'anygrasp'
        self.collision_status = False
        self.apply_collision_detection = apply_collision_detection

    def execute(self, userdata):
        outcome = super(bowl_grasp_generator, self).execute(userdata)
        # if self.apply_collision_detection:
        if userdata.success:
            # todo: add grasp selection logic
            userdata.anygrasp_transform = userdata.anygrasp_transforms[0]  # * T_base_grasp
            # print('The best grasp from AnyGrasp: ')
            # print(userdata.anygrasp_transform)
            # TODO: to delete
            q_base_grasp = Quaternion()
            q_base_grasp.x = 0.7071068
            q_base_grasp.y = 0.7071068
            q_base_grasp.z = 0 
            q_base_grasp.w = 0 
            
            # Grasp pose
            grasp_orientation = Vector3()
            grasp_orientation.x = degrees2Radians(0)
            grasp_orientation.y = degrees2Radians(-90)
            grasp_orientation.z = degrees2Radians(90)
            grasp_quaternion_tf = tf.transformations.quaternion_from_euler(
              grasp_orientation.x, 
              grasp_orientation.y, 
              grasp_orientation.z)
            q_grasp = Quaternion()
            attributes = ['x', 'y', 'z', 'w']
            for attr, value in zip(attributes, grasp_quaternion_tf):
                setattr(q_grasp, attr, value)
                
            pitch_angle = np.pi / 2
            pitch_rotation = tf.transformations.euler_matrix(0, pitch_angle, 0, 'sxyz')[:3, :3]
            yaw_angle =  - np.pi / 2
            yaw_rotation = tf.transformations.euler_matrix(0, 0, yaw_angle, 'sxyz')[:3, :3]                
            
            # T_base_grasp
            T_base_grasp = Transform()
            T_base_grasp = userdata.anygrasp_transform 
            q_base_grasp = T_base_grasp.rotation
            print('q_base_grasp', q_base_grasp)
            
            quaternion_list = [q_base_grasp.x, q_base_grasp.y, q_base_grasp.z, q_base_grasp.w]
            R_base_grasp = tf.transformations.quaternion_matrix(quaternion_list)[:3, :3]
            print('R_base_grasp', R_base_grasp)
            
            R_base_grasp = np.dot(np.dot(R_base_grasp, pitch_rotation), yaw_rotation)
            
            width_offset = np.array([0, 0, 0.02])
            p_base_grasp = Vector3()
            p_base_grasp = T_base_grasp.translation
            translation_list = [p_base_grasp.x, p_base_grasp.y, p_base_grasp.z]
            p_base_grasp = np.dot(R_base_grasp, translation_list) + width_offset                
            # print("T_base_grasp: ", T_base_grasp)    
                        
            bowl_handle_pose = Pose()
            bowl_handle_pose.orientation = q_grasp
            bowl_handle_pose.position.x = T_base_grasp.translation.x
            bowl_handle_pose.position.y = T_base_grasp.translation.y + 0.02
            bowl_handle_pose.position.z = T_base_grasp.translation.z  
            
            userdata.bowl_handle_pose = bowl_handle_pose  
            print("bowl_handle_pose")     
            print(userdata.bowl_handle_pose)     

            for i, transform in enumerate(userdata.anygrasp_transforms):
                anygrasp_tf_msg = tf2_ros.TransformStamped()
                anygrasp_tf_msg.header.stamp = rospy.Time.now()
                anygrasp_tf_msg.header.frame_id = 'base_link'
                anygrasp_tf_msg.child_frame_id = f'{self.grasp_namespace}/grasp_{i}'
                anygrasp_tf_msg.transform.translation = transform.translation
                anygrasp_tf_msg.transform.rotation = transform.rotation
                self.anygrasp_tf_broadcaster.sendTransform(anygrasp_tf_msg)
        
        return outcome



def move_to_bowl_handle_pose_callback(userdata, response):
    return generic_state_callback(userdata, 
                                  response, 
                                  'grasp_bowl_handle', 
                                  'aborted')


# . update cd
class move_to_bowl_handle_pose(smach_ros.ServiceState):
    def __init__(self
                 , service_name
                 , service_spec
                 , input_keys
                 , outcomes
                 , userdata_key
                 , apply_collision_detection = False):
        smach_ros.ServiceState.__init__(
          self, 
          service_name=service_name,
          service_spec=service_spec,
          request_cb=self.request_cb,
          request_cb_args=[userdata_key],
          response_slots=['success'],
          outcomes=outcomes,
          input_keys=input_keys+[userdata_key],
          response_cb=move_to_bowl_handle_pose_callback
        )
        self.userdata_key = userdata_key
        self.collision_status = False
        self.apply_collision_detection = apply_collision_detection
        
    def request_cb(self, userdata, *args):
        target_pose = getattr(userdata, self.userdata_key)
        return kortex_motion_planning.srv.KortexSimpleCmpeRequest(target_pose)

    def execute(self, userdata):
      if self.apply_collision_detection:
        stop_client = rospy.ServiceProxy(STOP_SERVICE, kortex_driver.srv.Stop)
        collision_thread_stop = threading.Event()

        def collision_detection():
            
            def callback(msg):
                if msg.data:
                    stop_client(kortex_driver.msg.Empty())
                    stop_loginfo()
                    self.collision_status = True
                    collision_thread_stop.set()

            subscriber = rospy.Subscriber(COLLISION_DETECTION_TOPIC, Bool, callback)

            while not rospy.is_shutdown():
                if collision_thread_stop.is_set():
                  break
                rospy.sleep(0.1)
                
            subscriber.unregister()
            normal_loginfo("Collision detection thread terminated.")

        collision_thread = threading.Thread(target=collision_detection)
        collision_thread.start()

        outcome = super(move_to_bowl_handle_pose, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        normal_loginfo("Collision detection thread joined successfully.")
        
        if self.collision_status:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            self.collision_status = False
            return str(self.__class__.__name__)
      else:
        outcome = super(move_to_bowl_handle_pose, self).execute(userdata)
          
      return outcome      


def grasp_bowl_handle_callback(userdata, response):
    return generic_state_callback(userdata, 
                                  response, 
                                  'move_to_bowl_grasping_post_position', 
                                  'aborted')


# . update cd
class grasp_bowl_handle(smach_ros.ServiceState):
    def __init__(self
                 , target_position
                 , input_keys_sm
                 , outcomes_sm
                 , apply_collision_detection = False):
        super(grasp_bowl_handle, self).__init__(
            service_name=GRIPPER_COMMAND_SERVICE,
            service_spec=kortex_motion_planning.srv.SendGripperCommand,
            request=kortex_motion_planning.srv.SendGripperCommandRequest(
              target_position),
            response_slots=['success'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=grasp_bowl_handle_callback
        )
        self.collision_status = False
        self.apply_collision_detection = apply_collision_detection
        
    def execute(self, userdata):
      if self.apply_collision_detection:
        stop_client = rospy.ServiceProxy(STOP_SERVICE, kortex_driver.srv.Stop)
        collision_thread_stop = threading.Event()

        def collision_detection():
            
            def callback(msg):
                if msg.data:
                    stop_client(kortex_driver.msg.Empty())
                    stop_loginfo()
                    self.collision_status = True
                    collision_thread_stop.set()

            subscriber = rospy.Subscriber(COLLISION_DETECTION_TOPIC, Bool, callback)

            while not rospy.is_shutdown():
                if collision_thread_stop.is_set():
                  break
                rospy.sleep(0.1)
                
            subscriber.unregister()
            normal_loginfo("Collision detection thread terminated.")

        collision_thread = threading.Thread(target=collision_detection)
        collision_thread.start()

        outcome = super(grasp_bowl_handle, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        normal_loginfo("Collision detection thread joined successfully.")
        
        if self.collision_status:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            self.collision_status = False
            return str(self.__class__.__name__)
      else:
        outcome = super(grasp_bowl_handle, self).execute(userdata)
          
      return outcome




def move_to_bowl_grasping_post_position_callback(userdata, response):
    return generic_state_callback(userdata, 
                                  response, 
                                  'plan_for_bowl_upright_transfer', 
                                  'aborted')


class move_to_bowl_grasping_post_position(smach_ros.ServiceState):
    def __init__(self
                 , target_position
                 , input_keys_sm
                 , outcomes_sm
                 , apply_collision_detection = False):
        super(move_to_bowl_grasping_post_position, self).__init__(
            service_name=SIMPLE_JMPE_SERVICE,
            service_spec=kortex_motion_planning.srv.KortexSimpleJmpe,
            request=kortex_motion_planning.srv.KortexSimpleJmpeRequest(
              target_position),
            response_slots=['success'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=move_to_bowl_grasping_post_position_callback
        )
        self.collision_status = False
        self.apply_collision_detection = apply_collision_detection
        
    def execute(self, userdata):
      if self.apply_collision_detection:
        stop_client = rospy.ServiceProxy(STOP_SERVICE, kortex_driver.srv.Stop)
        collision_thread_stop = threading.Event()

        def collision_detection():
            
            def callback(msg):
                if msg.data:
                    stop_client(kortex_driver.msg.Empty())
                    stop_loginfo()
                    self.collision_status = True
                    collision_thread_stop.set()

            subscriber = rospy.Subscriber(COLLISION_DETECTION_TOPIC, Bool, callback)

            while not rospy.is_shutdown():
                if collision_thread_stop.is_set():
                  break
                rospy.sleep(0.1)
                
            subscriber.unregister()
            normal_loginfo("Collision detection thread terminated.")

        collision_thread = threading.Thread(target=collision_detection)
        collision_thread.start()

        outcome = super(move_to_bowl_grasping_post_position, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        normal_loginfo("Collision detection thread joined successfully.")
        
        if self.collision_status:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            self.collision_status = False
            return str(self.__class__.__name__)
      else:
        outcome = super(move_to_bowl_grasping_post_position, self).execute(userdata)
          
      return outcome


def plan_for_bowl_upright_transfer_callback(userdata, response):
  return generic_state_callback(userdata, 
                                response, 
                                'execution_for_bowl_upright_transfer',
                                'plan_for_bowl_upright_transfer')


# . update cd
class plan_for_bowl_upright_transfer(smach_ros.ServiceState):
    def __init__(self
                 , service_name
                 , service_spec
                 , input_keys
                 , outcomes
                 , userdata_key
                 , apply_collision_detection = False):
        smach_ros.ServiceState.__init__(
            self,
            service_name=service_name,
            service_spec=service_spec,
            request_cb=self.request_cb,
            request_cb_args=[userdata_key],
            response_slots=['motion_plan', 'success', 'message'],
            outcomes=outcomes,
            input_keys=input_keys + [userdata_key],
            response_cb=plan_for_bowl_upright_transfer_callback
        )
        self.userdata_key = userdata_key
        self.collision_status = False
        self.apply_collision_detection = apply_collision_detection
        
    def request_cb(self, userdata, *args):
          target_pose = getattr(userdata, self.userdata_key)
          return kortex_motion_planning.srv.GenerateKortexMotionPlanRequest(target_pose) 
        
    def execute(self, userdata):
      if self.apply_collision_detection:
        stop_client = rospy.ServiceProxy(STOP_SERVICE, kortex_driver.srv.Stop)
        collision_thread_stop = threading.Event()

        def collision_detection():
            
            def callback(msg):
                if msg.data:
                    stop_client(kortex_driver.msg.Empty())
                    stop_loginfo()
                    self.collision_status = True
                    collision_thread_stop.set()

            subscriber = rospy.Subscriber(COLLISION_DETECTION_TOPIC, Bool, callback)

            while not rospy.is_shutdown():
                if collision_thread_stop.is_set():
                  break
                rospy.sleep(0.1)
                
            subscriber.unregister()
            normal_loginfo("Collision detection thread terminated.")

        collision_thread = threading.Thread(target=collision_detection)
        collision_thread.start()

        outcome = super(plan_for_bowl_upright_transfer, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        normal_loginfo("Collision detection thread joined successfully.")
        
        if self.collision_status:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            self.collision_status = False
            return 'plan_for_bowl_upright_transfer'
      else:
        outcome = super(plan_for_bowl_upright_transfer, self).execute(userdata)
          
      return outcome      


def execution_for_bowl_upright_transfer_callback(userdata, response):
  return generic_state_callback(userdata, 
                                response, 
                                'release_bowl',
                                'execution_for_bowl_upright_transfer') 


# .update cd 
class execution_for_bowl_upright_transfer(smach_ros.ServiceState):
    def __init__(self
                 , request_key_
                 , input_keys_sm
                 , outcomes_sm
                 , apply_collsion_detection = False):
        super(execution_for_bowl_upright_transfer, self).__init__(
            service_name=MOTION_EXECUTOR_SERVICE,
            service_spec=kortex_motion_planning.srv.ExecuteMotionPlan,
            request_key=request_key_,
            response_slots=['success', 'message'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=execution_for_bowl_upright_transfer_callback
        )
        self.collision_status = False
        self.apply_collision_detection = apply_collsion_detection   
        
    def execute(self, userdata):
      if self.apply_collision_detection:
        stop_client = rospy.ServiceProxy(STOP_SERVICE, kortex_driver.srv.Stop)
        collision_thread_stop = threading.Event()

        def collision_detection():
            
            def callback(msg):
                if msg.data:
                    stop_client(kortex_driver.msg.Empty())
                    stop_loginfo()
                    self.collision_status = True
                    collision_thread_stop.set()

            subscriber = rospy.Subscriber(COLLISION_DETECTION_TOPIC, Bool, callback)

            while not rospy.is_shutdown():
                if collision_thread_stop.is_set():
                  break
                rospy.sleep(0.1)
                
            subscriber.unregister()
            normal_loginfo("Collision detection thread terminated.")

        collision_thread = threading.Thread(target=collision_detection)
        collision_thread.start()

        outcome = super(execution_for_bowl_upright_transfer, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        normal_loginfo("Collision detection thread joined successfully.")
        
        if self.collision_status:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            self.collision_status = False
            return 'execution_for_bowl_upright_transfer'
      else:
        outcome = super(execution_for_bowl_upright_transfer, self).execute(userdata)
          
      return outcome


def release_bowl_callback(userdata, response):
    return generic_state_callback(userdata, 
                                  response, 
                                  'get_utensil', 
                                  'aborted')


# . update cd
class release_bowl(smach_ros.ServiceState):
    def __init__(self
                 , target_position
                 , input_keys_sm
                 , outcomes_sm
                 , apply_collision_detection = False):
        super(release_bowl, self).__init__(
            service_name=GRIPPER_COMMAND_SERVICE,
            service_spec=kortex_motion_planning.srv.SendGripperCommand,
            request=kortex_motion_planning.srv.SendGripperCommandRequest(
              target_position),
            response_slots=['success'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=release_bowl_callback
        )
        self.collision_status = False
        self.apply_collision_detection = apply_collision_detection
        
    def execute(self, userdata):
      if self.apply_collision_detection:
        stop_client = rospy.ServiceProxy(STOP_SERVICE, kortex_driver.srv.Stop)
        collision_thread_stop = threading.Event()

        def collision_detection():
            
            def callback(msg):
                if msg.data:
                    stop_client(kortex_driver.msg.Empty())
                    stop_loginfo()
                    self.collision_status = True
                    collision_thread_stop.set()

            subscriber = rospy.Subscriber(COLLISION_DETECTION_TOPIC, Bool, callback)

            while not rospy.is_shutdown():
                if collision_thread_stop.is_set():
                  break
                rospy.sleep(0.1)
                
            subscriber.unregister()
            normal_loginfo("Collision detection thread terminated.")

        collision_thread = threading.Thread(target=collision_detection)
        collision_thread.start()

        outcome = super(release_bowl, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        normal_loginfo("Collision detection thread joined successfully.")
        
        if self.collision_status:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            self.collision_status = False
            return str(self.__class__.__name__)
      else:
        outcome = super(release_bowl, self).execute(userdata)
          
      return outcome


def food_item_selector_callback(userdata, response):
    return generic_userdata_state_callback(userdata, 
                                           response, 
                                           'move_to_pre_skewer_pose', 
                                           'aborted', 
                                           True)
    

class food_item_selector(smach_ros.ServiceState):
    def __init__(self
                 , request_key_
                 , input_keys_sm
                 , outcomes_sm
                 , apply_collision_detection = False):
        super(food_item_selector, self).__init__(
            service_name=FOOD_ITEM_SELECTOR_SERVICE,
            service_spec=anygrasp_generation.srv.AnyGraspGeneration,
            # request=anygrasp_generation.srv.AnyGraspGenerationRequest(update_anygrasp),
            request_key=request_key_,
            response_slots=['anygrasp_transforms', 'success'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=food_item_selector_callback
        )
        self.anygrasp_tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.grasp_namespace = 'anygrasp'
        self.collision_status = False
        self.apply_collision_detection = apply_collision_detection

    def execute(self, userdata):
        outcome = super(food_item_selector, self).execute(userdata)
        # if self.apply_collision_detection:
        if userdata.success:
            userdata.anygrasp_transform = userdata.anygrasp_transforms[0]
            # print('The best grasp from AnyGrasp: ')
            # print(userdata.anygrasp_transform)
            q_base_grasp = Quaternion()
            q_base_grasp.x = 0.7071068
            q_base_grasp.y = 0.7071068
            q_base_grasp.z = 0 
            q_base_grasp.w = 0 
            grasp_orientation = Vector3()
            grasp_orientation.x = degrees2Radians(180)
            grasp_orientation.y = degrees2Radians(0)
            grasp_orientation.z = degrees2Radians(90)
            grasp_quaternion_tf = tf.transformations.quaternion_from_euler(grasp_orientation.x, 
                                                              grasp_orientation.y, 
                                                              grasp_orientation.z)
            q_grasp = Quaternion()
            # q_grasp.x = grasp_quaternion_tf[0]
            # q_grasp.y = grasp_quaternion_tf[1]
            # q_grasp.z = grasp_quaternion_tf[2]
            # q_grasp.w = grasp_quaternion_tf[3]
            attributes = ['x', 'y', 'z', 'w']
            for attr, value in zip(attributes, grasp_quaternion_tf):
                setattr(q_grasp, attr, value)
            
            # T_base_grasp
            T_base_grasp = Transform()
            T_base_grasp = userdata.anygrasp_transform 
            T_base_grasp.rotation = q_base_grasp
            # print("T_base_grasp: ", T_base_grasp)
            
            # T_tool_grasp
            T_tool_grasp = Transform()
            p_tool_grasp = Vector3()
            p_tool_grasp.x = 0
            p_tool_grasp.y = 0 
            p_tool_grasp.z = 0.17
            # p_tool_grasp.z = 0.180
            q_tool_grasp = Quaternion()
            q_tool_grasp.x = 0.00
            q_tool_grasp.y = 0.00
            q_tool_grasp.z = 0.00
            q_tool_grasp.w = 1.00
            T_tool_grasp.translation = p_tool_grasp
            T_tool_grasp.rotation = q_tool_grasp
            
            # T_grasp_tool
            T_grasp_tool = Transform()
            T_grasp_tool = invert_transform(T_tool_grasp)
            # T_grasp_tool = T_tool_grasp
            
            # print("T_tool_grasp: ", T_tool_grasp)
            # print("T_grasp_tool: ", T_grasp_tool)
            
            # EE (tool_frame)
            T_base_tool = Transform()
            
            # T_base_tool = T_base_grasp * T_grasp_tool
            T_base_tool = matrix_to_transform(
                tf.transformations.concatenate_matrices(
                    transform_to_matrix(T_base_grasp),
                    transform_to_matrix(T_grasp_tool)
                )
            )
            # print("p_base_tool: ", T_base_tool)            
            
            # pre-skewer
            pre_skewer_pose = Pose()
            pre_skewer_pose.position.x = T_base_tool.translation.x
            pre_skewer_pose.position.y = T_base_tool.translation.y
            pre_skewer_pose.position.z = T_base_tool.translation.z + 0.08
            pre_skewer_pose.orientation = q_grasp
            userdata.pre_skewer_pose = pre_skewer_pose
            # print("pre_skewer_pose: ")
            # print(userdata.pre_skewer_pose) 
            post_skewer_pose = Pose()
            post_skewer_pose = pre_skewer_pose
            userdata.post_skewer_pose = post_skewer_pose
            # print("post_skewer_pose: ")
            # print(userdata.post_skewer_pose) 
            
            # skewer
            skewer_pose = Pose()
            skewer_pose.position.x = T_base_tool.translation.x
            skewer_pose.position.y = T_base_tool.translation.y
            skewer_pose.position.z = T_base_tool.translation.z
            skewer_pose.orientation = q_grasp
            userdata.skewer_pose = skewer_pose
            # print("skewer_pose: ")
            # print(userdata.skewer_pose)  
            normal_loginfo("Skewer pose: ")
            normal_loginfo(str(userdata.skewer_pose))           

            
            for i, transform in enumerate(userdata.anygrasp_transforms):
                anygrasp_tf_msg = tf2_ros.TransformStamped()
                anygrasp_tf_msg.header.stamp = rospy.Time.now()
                anygrasp_tf_msg.header.frame_id = 'base_link'
                anygrasp_tf_msg.child_frame_id = f'{self.grasp_namespace}/grasp_{i}'
                anygrasp_tf_msg.transform.translation = transform.translation
                anygrasp_tf_msg.transform.rotation = transform.rotation
                self.anygrasp_tf_broadcaster.sendTransform(anygrasp_tf_msg)
                
        # global collision_detected
        
        return outcome


def get_utensil_callback(userdata, response):
    return generic_state_callback(userdata, 
                                  response, 
                                  'move_to_feeding_start_position', 
                                  'aborted')


class get_utensil(smach_ros.ServiceState):
    def __init__(self
                 , get_utensil_flag
                 , holder_position
                 , utensil_position
                 , input_keys_sm
                 , outcomes_sm
                 , apply_collision_detection = False):
        super(get_utensil, self).__init__(
            service_name=GET_UTENSIL_SERVICE,
            service_spec=kortex_motion_planning.srv.GetUtensil,
            request=kortex_motion_planning.srv.GetUtensilRequest(
              get_utensil_flag
              , holder_position
              , utensil_position),
            response_slots=['success'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=get_utensil_callback
        )
        self.collision_status = False
        self.apply_collision_detection = apply_collision_detection
        
    def execute(self, userdata):
      if self.apply_collision_detection:
        stop_client = rospy.ServiceProxy(STOP_SERVICE, kortex_driver.srv.Stop)
        collision_thread_stop = threading.Event()

        def collision_detection():
            
            def callback(msg):
                if msg.data:
                    stop_client(kortex_driver.msg.Empty())
                    stop_loginfo()
                    self.collision_status = True
                    collision_thread_stop.set()

            subscriber = rospy.Subscriber(COLLISION_DETECTION_TOPIC, Bool, callback)

            while not rospy.is_shutdown():
                if collision_thread_stop.is_set():
                  break
                rospy.sleep(0.1)
                
            subscriber.unregister()
            normal_loginfo("Collision detection thread terminated.")

        collision_thread = threading.Thread(target=collision_detection)
        collision_thread.start()

        outcome = super(get_utensil, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        normal_loginfo("Collision detection thread joined successfully.")
        
        if self.collision_status:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            self.collision_status = False
            return str(self.__class__.__name__)
      else:
        outcome = super(get_utensil, self).execute(userdata)
          
      return outcome


def move_to_feeding_start_position_callback(userdata, response):
    return generic_state_callback(userdata, 
                                  response, 
                                  'food_item_selector', 
                                  'aborted')


# . update cd
class move_to_feeding_start_position(smach_ros.ServiceState):
    def __init__(self
                 , target_positions_
                 , input_keys_sm
                 , outcomes_sm
                 , apply_collision_detection = True):
        super(move_to_feeding_start_position, self).__init__(
            service_name=SIMPLE_JMPE_SERVICE,
            service_spec=kortex_motion_planning.srv.KortexSimpleJmpe,
            request=kortex_motion_planning.srv.KortexSimpleJmpeRequest(target_positions_),
            response_slots=['success'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=move_to_feeding_start_position_callback
        )
        self.collision_status = False
        self.apply_collision_detection = apply_collision_detection
        
    def execute(self, userdata):
      if self.apply_collision_detection:
        stop_client = rospy.ServiceProxy(STOP_SERVICE, kortex_driver.srv.Stop)
        collision_thread_stop = threading.Event()

        def collision_detection():
            
            def callback(msg):
                if msg.data:
                    stop_client(kortex_driver.msg.Empty())
                    stop_loginfo()
                    self.collision_status = True
                    collision_thread_stop.set()

            subscriber = rospy.Subscriber(COLLISION_DETECTION_TOPIC, Bool, callback)

            while not rospy.is_shutdown():
                if collision_thread_stop.is_set():
                  break
                rospy.sleep(0.1)
                
            subscriber.unregister()
            normal_loginfo("Collision detection thread terminated.")

        collision_thread = threading.Thread(target=collision_detection)
        collision_thread.start()

        outcome = super(move_to_feeding_start_position, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        normal_loginfo("Collision detection thread joined successfully.")
        
        if self.collision_status:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            self.collision_status = False
            return str(self.__class__.__name__)
      else:
        outcome = super(move_to_feeding_start_position, self).execute(userdata)
          
      return outcome


def move_to_pre_skewer_pose_callback(userdata, response):
    return generic_state_callback(userdata, 
                                  response, 
                                  'move_to_skewer_pose', 
                                  'aborted')


# . update cd
class move_to_pre_skewer_pose(smach_ros.ServiceState):
    def __init__(self
                 , service_name
                 , service_spec
                 , input_keys
                 , outcomes
                 , userdata_key
                 , apply_collision_detection = False):
        smach_ros.ServiceState.__init__(
          self, 
          service_name=service_name,
          service_spec=service_spec,
          request_cb=self.request_cb,
          request_cb_args=[userdata_key],
          response_slots=['success'],
          outcomes=outcomes,
          input_keys=input_keys+[userdata_key],
          response_cb=move_to_pre_skewer_pose_callback
        )
        self.userdata_key = userdata_key
        self.collision_status = False
        self.apply_collision_detection = apply_collision_detection
        
    def request_cb(self, userdata, *args):
        target_pose = getattr(userdata, self.userdata_key)
        return kortex_motion_planning.srv.KortexSimpleCmpeRequest(target_pose)

    def execute(self, userdata):
      if self.apply_collision_detection:
        stop_client = rospy.ServiceProxy(STOP_SERVICE, kortex_driver.srv.Stop)
        collision_thread_stop = threading.Event()

        def collision_detection():
            
            def callback(msg):
                if msg.data:
                    stop_client(kortex_driver.msg.Empty())
                    stop_loginfo()
                    self.collision_status = True
                    collision_thread_stop.set()

            subscriber = rospy.Subscriber(COLLISION_DETECTION_TOPIC, Bool, callback)

            while not rospy.is_shutdown():
                if collision_thread_stop.is_set():
                  break
                rospy.sleep(0.1)
                
            subscriber.unregister()
            normal_loginfo("Collision detection thread terminated.")

        collision_thread = threading.Thread(target=collision_detection)
        collision_thread.start()

        outcome = super(move_to_pre_skewer_pose, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        normal_loginfo("Collision detection thread joined successfully.")
        
        if self.collision_status:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            self.collision_status = False
            return str(self.__class__.__name__)
      else:
        outcome = super(move_to_pre_skewer_pose, self).execute(userdata)
          
      return outcome      


def move_to_skewer_pose_callback(userdata, response):
    return generic_state_callback(userdata, 
                                  response, 
                                  'move_to_post_skewer_pose', 
                                  'aborted')


class move_to_skewer_pose(smach_ros.ServiceState):
    def __init__(self, service_name, service_spec, input_keys, outcomes, userdata_key):
        smach_ros.ServiceState.__init__(
          self, 
          service_name=service_name,
          service_spec=service_spec,
          request_cb=self.request_cb,
          request_cb_args=[userdata_key],
          response_slots=['success'],
          outcomes=outcomes,
          input_keys=input_keys+[userdata_key],
          response_cb=move_to_skewer_pose_callback
        )
        self.userdata_key = userdata_key
        
    def request_cb(self, userdata, *args):
        target_pose = getattr(userdata, self.userdata_key)
        return kortex_motion_planning.srv.KortexSimpleCmpeRequest(target_pose)


def move_to_post_skewer_pose_callback(userdata, response):
    return generic_state_callback(userdata, 
                                  response, 
                                  'move_to_feeding_initial_position', 
                                  'aborted')


# . update cd
class move_to_post_skewer_pose(smach_ros.ServiceState):
    def __init__(self
                 , service_name
                 , service_spec
                 , input_keys
                 , outcomes
                 , userdata_key
                 , apply_collision_detection = False):
        smach_ros.ServiceState.__init__(
          self, 
          service_name=service_name,
          service_spec=service_spec,
          request_cb=self.request_cb,
          request_cb_args=[userdata_key],
          response_slots=['success'],
          outcomes=outcomes,
          input_keys=input_keys+[userdata_key],
          response_cb=move_to_post_skewer_pose_callback
        )
        self.userdata_key = userdata_key
        self.collision_status = False
        self.apply_collision_detection = apply_collision_detection
        
    def request_cb(self, userdata, *args):
        target_pose = getattr(userdata, self.userdata_key)
        return kortex_motion_planning.srv.KortexSimpleCmpeRequest(target_pose)

    def execute(self, userdata):
      if self.apply_collision_detection:
        stop_client = rospy.ServiceProxy(STOP_SERVICE, kortex_driver.srv.Stop)
        collision_thread_stop = threading.Event()

        def collision_detection():
            
            def callback(msg):
                if msg.data:
                    stop_client(kortex_driver.msg.Empty())
                    stop_loginfo()
                    self.collision_status = True
                    collision_thread_stop.set()

            subscriber = rospy.Subscriber(COLLISION_DETECTION_TOPIC, Bool, callback)

            while not rospy.is_shutdown():
                if collision_thread_stop.is_set():
                  break
                rospy.sleep(0.1)
                
            subscriber.unregister()
            normal_loginfo("Collision detection thread terminated.")

        collision_thread = threading.Thread(target=collision_detection)
        collision_thread.start()

        outcome = super(move_to_post_skewer_pose, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        normal_loginfo("Collision detection thread joined successfully.")
        
        if self.collision_status:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            self.collision_status = False
            return str(self.__class__.__name__)
      else:
        outcome = super(move_to_post_skewer_pose, self).execute(userdata)
          
      return outcome  


def move_to_feeding_initial_position_callback(userdata, response):
    return generic_state_callback(userdata, 
                                  response, 
                                  'skewer_status_check', 
                                  'aborted') 


# . update cd
class move_to_feeding_initial_position(smach_ros.ServiceState):
    def __init__(self
                 , target_positions_
                 , input_keys_sm
                 , outcomes_sm
                 , apply_collision_detection = True):
        super(move_to_feeding_initial_position, self).__init__(
            service_name=SIMPLE_JMPE_SERVICE,
            service_spec=kortex_motion_planning.srv.KortexSimpleJmpe,
            request=kortex_motion_planning.srv.KortexSimpleJmpeRequest(target_positions_),
            response_slots=['success'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=move_to_feeding_initial_position_callback
        )
        self.collision_status = False
        self.apply_collision_detection = apply_collision_detection
        
    def execute(self, userdata):
      if self.apply_collision_detection:
        stop_client = rospy.ServiceProxy(STOP_SERVICE, kortex_driver.srv.Stop)
        collision_thread_stop = threading.Event()

        def collision_detection():
            
            def callback(msg):
                if msg.data:
                    stop_client(kortex_driver.msg.Empty())
                    stop_loginfo()
                    self.collision_status = True
                    collision_thread_stop.set()

            subscriber = rospy.Subscriber(COLLISION_DETECTION_TOPIC, Bool, callback)

            while not rospy.is_shutdown():
                if collision_thread_stop.is_set():
                  break
                rospy.sleep(0.1)
                
            subscriber.unregister()
            normal_loginfo("Collision detection thread terminated.")

        collision_thread = threading.Thread(target=collision_detection)
        collision_thread.start()

        outcome = super(move_to_feeding_initial_position, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        normal_loginfo("Collision detection thread joined successfully.")
        
        if self.collision_status:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            self.collision_status = False
            return str(self.__class__.__name__)
      else:
        outcome = super(move_to_feeding_initial_position, self).execute(userdata)
          
      return outcome  


def skewer_status_check_callback(userdata, response):
    return state_skewer_status_check_callback(userdata, 
                                  response, 
                                  'plan_to_feeding_pose', 
                                  'food_item_selector')


class skewer_status_check(smach_ros.ServiceState):
    def __init__(self, skewer_status_check_flag_, input_keys_sm, outcomes_sm):
        super(skewer_status_check, self).__init__(
            service_name=SKEWER_STATUS_CHECKER_SERVICE,
            service_spec=feeding_task.srv.SkewerStatusCheck,
            request=feeding_task.srv.SkewerStatusCheckRequest(skewer_status_check_flag_),
            response_slots=['skewer_status', 'point_number'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=skewer_status_check_callback
        )
    def execute(self, userdata):
        global collision_detected
        collision_detected = False
        collision_thread_stop = threading.Event()

        def collision_detection():
            global collision_detected
            
            def callback(msg):
                global collision_detected
                if msg.data:
                    collision_detected = True
                    collision_thread_stop.set()

            rospy.Subscriber(COLLISION_DETECTION_TOPIC, Bool, callback)

            while not rospy.is_shutdown() and not collision_thread_stop.is_set():
                rospy.sleep(0.1)

        collision_thread = threading.Thread(target=collision_detection)
        collision_thread.start()

        outcome = super(skewer_status_check, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        
        if collision_detected:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            return 'skewer_status_check'
        
        return outcome


def plan_to_pre_skewer_pose_callback(userdata, response):
  return generic_state_callback(userdata, 
                                response, 
                                'execute_to_pre_skewer_pose',
                                'plan_to_pre_skewer_pose')


class plan_to_pre_skewer_pose(smach_ros.ServiceState):
    def __init__(self, service_name, service_spec, input_keys, outcomes, userdata_key):
        smach_ros.ServiceState.__init__(
            self,
            service_name=service_name,
            service_spec=service_spec,
            request_cb=self.request_cb,
            request_cb_args=[userdata_key],
            response_slots=['motion_plan', 'success', 'message'],
            outcomes=outcomes,
            input_keys=input_keys + [userdata_key],
            response_cb=plan_to_pre_skewer_pose_callback
        )
        self.userdata_key = userdata_key
        
    def request_cb(self, userdata, *args):
          target_pose = getattr(userdata, self.userdata_key)
          return kortex_motion_planning.srv.GenerateKortexMotionPlanRequest(target_pose)  
    def execute(self, userdata):
        global collision_detected
        collision_detected = False
        collision_thread_stop = threading.Event()

        def collision_detection():
            global collision_detected
            
            def callback(msg):
                global collision_detected
                if msg.data:
                    collision_detected = True
                    collision_thread_stop.set()

            rospy.Subscriber(COLLISION_DETECTION_TOPIC, Bool, callback)

            while not rospy.is_shutdown() and not collision_thread_stop.is_set():
                rospy.sleep(0.1)

        collision_thread = threading.Thread(target=collision_detection)
        collision_thread.start()

        outcome = super(plan_to_pre_skewer_pose, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        
        if collision_detected:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            return 'plan_to_pre_skewer_pose'
        
        return outcome


def plan_to_skewer_pose_callback(userdata, response):
  return generic_state_callback(userdata, 
                                response, 
                                'execute_to_skewer_pose',
                                'plan_to_skewer_pose')


class plan_to_skewer_pose(smach_ros.ServiceState):
    def __init__(self, service_name, service_spec, input_keys, outcomes, userdata_key):
        smach_ros.ServiceState.__init__(
            self,
            service_name=service_name,
            service_spec=service_spec,
            request_cb=self.request_cb,
            request_cb_args=[userdata_key],
            response_slots=['motion_plan', 'success', 'message'],
            outcomes=outcomes,
            input_keys=input_keys + [userdata_key],
            response_cb=plan_to_skewer_pose_callback
        )
        self.userdata_key = userdata_key
        
    def request_cb(self, userdata, *args):
          target_pose = getattr(userdata, self.userdata_key)
          return kortex_motion_planning.srv.GenerateKortexMotionPlanRequest(target_pose)  
    def execute(self, userdata):
        global collision_detected
        collision_detected = False
        collision_thread_stop = threading.Event()

        def collision_detection():
            global collision_detected
            
            def callback(msg):
                global collision_detected
                if msg.data:
                    collision_detected = True
                    collision_thread_stop.set()

            rospy.Subscriber(COLLISION_DETECTION_TOPIC, Bool, callback)

            while not rospy.is_shutdown() and not collision_thread_stop.is_set():
                rospy.sleep(0.1)

        collision_thread = threading.Thread(target=collision_detection)
        collision_thread.start()

        outcome = super(plan_to_skewer_pose, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        
        if collision_detected:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            return 'plan_to_skewer_pose'
        
        return outcome


def plan_to_post_skewer_pose_callback(userdata, response):
  return generic_state_callback(userdata, 
                                response, 
                                'execute_to_post_skewer_pose',
                                'plan_to_post_skewer_pose')


class plan_to_post_skewer_pose(smach_ros.ServiceState):
    def __init__(self, service_name, service_spec, input_keys, outcomes, userdata_key):
        smach_ros.ServiceState.__init__(
            self,
            service_name=service_name,
            service_spec=service_spec,
            request_cb=self.request_cb,
            request_cb_args=[userdata_key],
            response_slots=['motion_plan', 'success', 'message'],
            outcomes=outcomes,
            input_keys=input_keys + [userdata_key],
            response_cb=plan_to_post_skewer_pose_callback
        )
        self.userdata_key = userdata_key
        
    def request_cb(self, userdata, *args):
          target_pose = getattr(userdata, self.userdata_key)
          return kortex_motion_planning.srv.GenerateKortexMotionPlanRequest(target_pose) 
    def execute(self, userdata):
        global collision_detected
        collision_detected = False
        collision_thread_stop = threading.Event()

        def collision_detection():
            global collision_detected
            
            def callback(msg):
                global collision_detected
                if msg.data:
                    collision_detected = True
                    collision_thread_stop.set()

            rospy.Subscriber(COLLISION_DETECTION_TOPIC, Bool, callback)

            while not rospy.is_shutdown() and not collision_thread_stop.is_set():
                rospy.sleep(0.1)

        collision_thread = threading.Thread(target=collision_detection)
        collision_thread.start()

        outcome = super(plan_to_post_skewer_pose, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        
        if collision_detected:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            return 'plan_to_post_skewer_pose'
        
        return outcome


def execute_to_pre_skewer_pose_callback(userdata, response):
  return generic_state_callback(userdata, 
                                response, 
                                'plan_to_skewer_pose',
                                'plan_to_pre_skewer_pose')


class execute_to_pre_skewer_pose(smach_ros.ServiceState):
    def __init__(self, request_key_, input_keys_sm, outcomes_sm):
        super(execute_to_pre_skewer_pose, self).__init__(
            service_name=MOTION_EXECUTOR_SERVICE,
            service_spec=kortex_motion_planning.srv.ExecuteMotionPlan,
            # request=kortex_motion_planning.srv.GenerateKortexMotionPlanRequest(target_pose_),
            request_key=request_key_,
            response_slots=['success', 'message'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=execute_to_pre_skewer_pose_callback
        )
    def execute(self, userdata):
        global collision_detected
        collision_detected = False
        collision_thread_stop = threading.Event()

        def collision_detection():
            global collision_detected
            
            def callback(msg):
                global collision_detected
                if msg.data:
                    collision_detected = True
                    collision_thread_stop.set()

            rospy.Subscriber(COLLISION_DETECTION_TOPIC, Bool, callback)

            while not rospy.is_shutdown() and not collision_thread_stop.is_set():
                rospy.sleep(0.1)

        collision_thread = threading.Thread(target=collision_detection)
        collision_thread.start()

        outcome = super(execute_to_pre_skewer_pose, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        
        if collision_detected:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            return 'plan_to_pre_skewer_pose'
        
        return outcome


def execute_to_skewer_pose_callback(userdata, response):
  return generic_state_callback(userdata, 
                                response, 
                                'move_to_feeding_initial_position',
                                'plan_to_skewer_pose')
 
      
class execute_to_skewer_pose(smach_ros.ServiceState):
    def __init__(self, request_key_, input_keys_sm, outcomes_sm):
        super(execute_to_skewer_pose, self).__init__(
            service_name=MOTION_EXECUTOR_SERVICE,
            service_spec=kortex_motion_planning.srv.ExecuteMotionPlan,
            # request=kortex_motion_planning.srv.GenerateKortexMotionPlanRequest(target_pose_),
            request_key=request_key_,
            response_slots=['success', 'message'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=execute_to_skewer_pose_callback
        ) 
      

def execute_to_post_skewer_pose_callback(userdata, response):
  return generic_state_callback(userdata, 
                                response, 
                                'move_to_feeding_initial_position',
                                'plan_to_post_skewer_pose') 

        
class execute_to_post_skewer_pose(smach_ros.ServiceState):
    def __init__(self, request_key_, input_keys_sm, outcomes_sm):
        super(execute_to_post_skewer_pose, self).__init__(
            service_name=MOTION_EXECUTOR_SERVICE,
            service_spec=kortex_motion_planning.srv.ExecuteMotionPlan,
            # request=kortex_motion_planning.srv.GenerateKortexMotionPlanRequest(target_pose_),
            request_key=request_key_,
            response_slots=['success', 'message'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=execute_to_post_skewer_pose_callback
        )
    def execute(self, userdata):
        global collision_detected
        collision_detected = False
        collision_thread_stop = threading.Event()

        def collision_detection():
            global collision_detected
            
            def callback(msg):
                global collision_detected
                if msg.data:
                    collision_detected = True
                    collision_thread_stop.set()

            rospy.Subscriber(COLLISION_DETECTION_TOPIC, Bool, callback)

            while not rospy.is_shutdown() and not collision_thread_stop.is_set():
                rospy.sleep(0.1)

        collision_thread = threading.Thread(target=collision_detection)
        collision_thread.start()

        outcome = super(execute_to_post_skewer_pose, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        
        if collision_detected:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            return 'plan_to_post_skewer_pose'
        
        return outcome    
        

def plan_to_feeding_pose_callback(userdata, response):
  return generic_state_callback(userdata, 
                                response, 
                                'execute_to_feeding_pose',
                                'plan_to_feeding_pose')


# . update cd
class plan_to_feeding_pose(smach_ros.ServiceState):
    def __init__(self
                 , service_name
                 , service_spec
                 , input_keys
                 , outcomes
                 , userdata_key
                 , apply_collision_detection = False):
        smach_ros.ServiceState.__init__(
            self,
            service_name=service_name,
            service_spec=service_spec,
            request_cb=self.request_cb,
            request_cb_args=[userdata_key],
            response_slots=['motion_plan', 'success', 'message'],
            outcomes=outcomes,
            input_keys=input_keys + [userdata_key],
            response_cb=plan_to_feeding_pose_callback
        )
        self.userdata_key = userdata_key
        self.collision_status = False
        self.apply_collision_detection = apply_collision_detection
        
    def request_cb(self, userdata, *args):
          target_pose = getattr(userdata, self.userdata_key)
          return kortex_motion_planning.srv.GenerateKortexMotionPlanRequest(target_pose) 
        
    def execute(self, userdata):
      if self.apply_collision_detection:
        stop_client = rospy.ServiceProxy(STOP_SERVICE, kortex_driver.srv.Stop)
        collision_thread_stop = threading.Event()

        def collision_detection():
            
            def callback(msg):
                if msg.data:
                    stop_client(kortex_driver.msg.Empty())
                    stop_loginfo()
                    self.collision_status = True
                    collision_thread_stop.set()

            subscriber = rospy.Subscriber(COLLISION_DETECTION_TOPIC, Bool, callback)

            while not rospy.is_shutdown():
                if collision_thread_stop.is_set():
                  break
                rospy.sleep(0.1)
                
            subscriber.unregister()
            normal_loginfo("Collision detection thread terminated.")

        collision_thread = threading.Thread(target=collision_detection)
        collision_thread.start()

        outcome = super(plan_to_feeding_pose, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        normal_loginfo("Collision detection thread joined successfully.")
        
        if self.collision_status:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            self.collision_status = False
            return 'plan_to_feeding_pose'
      else:
        outcome = super(plan_to_feeding_pose, self).execute(userdata)
          
      return outcome      


def execute_to_feeding_pose_callback(userdata, response):
  return generic_state_callback(userdata, 
                                response, 
                                'move_to_feeding_start_position',
                                'plan_to_feeding_pose') 


# .update cd 
class execute_to_feeding_pose(smach_ros.ServiceState):
    def __init__(self
                 , request_key_
                 , input_keys_sm
                 , outcomes_sm
                 , apply_collsion_detection = True):
        super(execute_to_feeding_pose, self).__init__(
            service_name=MOTION_EXECUTOR_SERVICE,
            service_spec=kortex_motion_planning.srv.ExecuteMotionPlan,
            # request=kortex_motion_planning.srv.GenerateKortexMotionPlanRequest(target_pose_),
            request_key=request_key_,
            response_slots=['success', 'message'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=execute_to_feeding_pose_callback
        )
        self.collision_status = False
        self.apply_collision_detection = apply_collsion_detection   
        
    def execute(self, userdata):
      if self.apply_collision_detection:
        stop_client = rospy.ServiceProxy(STOP_SERVICE, kortex_driver.srv.Stop)
        collision_thread_stop = threading.Event()

        def collision_detection():
            
            def callback(msg):
                if msg.data:
                    stop_client(kortex_driver.msg.Empty())
                    stop_loginfo()
                    self.collision_status = True
                    collision_thread_stop.set()

            subscriber = rospy.Subscriber(COLLISION_DETECTION_TOPIC, Bool, callback)

            while not rospy.is_shutdown():
                if collision_thread_stop.is_set():
                  break
                rospy.sleep(0.1)
                
            subscriber.unregister()
            normal_loginfo("Collision detection thread terminated.")

        collision_thread = threading.Thread(target=collision_detection)
        collision_thread.start()

        outcome = super(execute_to_feeding_pose, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        normal_loginfo("Collision detection thread joined successfully.")
        
        if self.collision_status:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            self.collision_status = False
            return 'execute_to_feeding_pose'
      else:
        outcome = super(execute_to_feeding_pose, self).execute(userdata)
          
      return outcome


def get_start_state(start):
  
    # parser logic not currently used
    parser = argparse.ArgumentParser(description='Start state for the FSM')
    parser.add_argument('--start', help='Specify the start state', default='door_open')
    args = parser.parse_args(rospy.myargv()[1:])

    start_state_map = {
        'door_open': 'move_to_initial_door_open_position',
        'bowl_grasping': 'move_to_bowl_grasping_initial_position',
        'bowl_transfer': 'move_to_bowl_grasping_post_position', 
        'utensil_fetching': 'get_utensil',
        'food_skewering': 'move_to_feeding_start_position',
        'food_transfer': 'move_to_feeding_initial_position'
    }

    return start_state_map.get(start, 'door_open')


def main():
    rospy.init_node('feeding_task_state_machine')

    sm = CustomStateMachine(outcomes_sm, input_keys=input_keys_sm, output_keys=input_keys_sm)
    
    # Functional parameters
    use_opt = False
    apply_utensil_fetching = False

    # Initialize necessary data in feeding task
    feeding_pose = Pose()
    feeding_pose.position.x = -0.15135
    feeding_pose.position.y = 0.235484
    feeding_pose.position.z = 0.557796
    feeding_pose.orientation.x = 0.3872724
    feeding_pose.orientation.y = -0.4914169
    feeding_pose.orientation.z = -0.604657
    feeding_pose.orientation.w = 0.4928685
    
    feeding_initial_position = JointPositions()
    feeding_initial_position.joint_positions = [0.021247, -0.26079, 3.15111, -2.14524, 0.060838, -0.90679, 1.58046]
    holder_position = JointPositions()
    holder_position.joint_positions = [0.302989, -0.31447, 3.09643, -2.40708, 6.26457, -1.063090, 1.81463]
    utensil_position = JointPositions()    
    utensil_position.joint_positions = [0.206082, -0.305769, 2.64442, -2.3651, 6.09785, -1.06134, 1.36814]
    bowl_transfer_initial_position = JointPositions()
    bowl_transfer_initial_position.joint_positions = [1.12982, 0.575937, 3.04003, 4.18295 - 6.28, 5.67489, 1.18636, 0.290636]
    
    bowl_transfer_post_position = JointPositions()
    bowl_transfer_post_position.joint_positions = [1.38236, 0.466606, 3.09348, 4.05094 - 6.28, 6.28229, 1.04642, 0.070713] 
    
    door_open_initial_position = JointPositions()
    door_open_initial_position.joint_positions = [1.3686876090750437, 0.28184642233856894, 3.144328097731239, 4.023280977312391 - 6.28, 6.009197207678883, 0.9603996509598605, 1.6590017452006982]

    door_open_post_position = JointPositions()
    door_open_post_position.joint_positions = [0.10398898576184974, 0.6798720661633672, 3.210061920145531, 4.894967873435816 - 6.28, 4.4163636859539315, 1.9033339091773762, 2.655396283861733]

       
    bowl_placement_pose = Pose()
    bowl_placement_pose.position.x = 0.369719
    bowl_placement_pose.position.y = -0.128634    
    bowl_placement_pose.position.z = 0.118447    
    bowl_placement_pose.orientation.x = -0.5591718754025408
    bowl_placement_pose.orientation.y = -0.5122518858574863
    bowl_placement_pose.orientation.z = -0.4673708958580917
    bowl_placement_pose.orientation.w = 0.4544108987459028
    # bowl_placement_quaternion = (bowl_placement_pose.orientation.x
    #                              , bowl_placement_pose.orientation.y
    #                              , bowl_placement_pose.orientation.z
    #                              , bowl_placement_pose.orientation.w)    
    # bowl_placement_pose.orientation = normalize_quaternion(bowl_placement_quaternion)
    
    gripper_open_position = 0
    bowl_release_position = 0.5
    bowl_handle_grasp_position = 1
    grasp_door_handle_position = 0.8
    reach_apriltag_flag = True
    door_open_flag = True
    
    
    sm.userdata.motion_plan = JointTrajectory()
    sm.userdata.update_anygrasp = True
    sm.userdata.success = False
    sm.userdata.message = ''
    sm.userdata.anygrasp_transforms = []
    sm.userdata.anygrasp_transform = Transform()
    sm.userdata.pre_skewer_pose = Pose()
    sm.userdata.skewer_pose = Pose()
    sm.userdata.post_skewer_pose = Pose()
    sm.userdata.collision_status = Bool()
    sm.userdata.collision_status.data = False
    sm.userdata.skewer_check_flag = True
    
    sm.userdata.bowl_handle_pose = Pose()
    
    # Remap variables
    sm.userdata.feeding_pose = feeding_pose
    sm.userdata.feeding_start_position = feeding_initial_position
    sm.userdata.feeding_initial_position = feeding_initial_position
    sm.userdata.bowl_placement_pose = bowl_placement_pose
    # sm.userdata.update_anygrasp = update_anygrasp
    
    start_subtask = 'utensil_fetching'
    initial_state = get_start_state(start_subtask)
    
    # Open the container
    with sm:
        # sm.set_initial_state()
        sm.set_initial_state([initial_state])
        smach.StateMachine.add('move_to_initial_door_open_position'
                               , move_to_initial_door_open_position(
                                 door_open_initial_position
                                 , input_keys_sm
                                 , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm) 
 
        smach.StateMachine.add('open_gripper_for_door_handle_grasping'
                               , open_gripper_for_door_handle_grasping(
                                 gripper_open_position
                                 , input_keys_sm
                                 , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm) 
        
        smach.StateMachine.add('move_to_door_handle_pose'
                               , move_to_door_handle_pose(
                                 reach_apriltag_flag
                                 , input_keys_sm
                                 , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm)  

        smach.StateMachine.add('grasp_door_handle'
                               , grasp_door_handle(
                                 grasp_door_handle_position
                                 , input_keys_sm
                                 , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm) 

        smach.StateMachine.add('open_door_with_admittance_control'
                               , open_door_with_admittance_control(
                                 door_open_flag
                                 , input_keys_sm
                                 , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm)  

        smach.StateMachine.add('move_to_post_door_open_position'
                               , move_to_post_door_open_position(
                                 door_open_post_position
                                 , input_keys_sm
                                 , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm) 
      
        smach.StateMachine.add('move_to_bowl_grasping_initial_position'
                               , move_to_bowl_grasping_initial_position(
                                 bowl_transfer_initial_position
                                 , input_keys_sm
                                 , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm)
        
        smach.StateMachine.add('open_gripper_for_bowl_grasping'
                               , open_gripper_for_bowl_grasping(
                                 gripper_open_position
                                 , input_keys_sm
                                 , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm)  

        smach.StateMachine.add('bowl_grasp_generator'
                               , bowl_grasp_generator(
                                 'update_anygrasp'
                                , input_keys_sm
                                , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm)

        smach.StateMachine.add('move_to_bowl_handle_pose',
                              move_to_bowl_handle_pose(
                                SIMPLE_CMPE_SERVICE,
                                kortex_motion_planning.srv.KortexSimpleCmpe,
                                input_keys_sm,
                                outcomes_sm,
                                'bowl_handle_pose'),
                              transitions=transition_sm,
                              remapping=remapping_sm)  

        smach.StateMachine.add('grasp_bowl_handle'
                               , grasp_bowl_handle(
                                 bowl_handle_grasp_position
                                 , input_keys_sm
                                 , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm)  
        
        smach.StateMachine.add('move_to_bowl_grasping_post_position'
                               , move_to_bowl_grasping_post_position(
                                 bowl_transfer_post_position
                                 , input_keys_sm
                                 , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm)

        smach.StateMachine.add('plan_for_bowl_upright_transfer',
                              plan_for_bowl_upright_transfer
                              (MOTION_PLANNER_SERVICE
                              , kortex_motion_planning.srv.GenerateKortexMotionPlan
                              , input_keys_sm
                              , outcomes_sm
                              , 'bowl_placement_pose'),
                              transitions=transition_sm,
                              remapping=remapping_sm)  

        smach.StateMachine.add('execution_for_bowl_upright_transfer'
                               , execution_for_bowl_upright_transfer
                               ('motion_plan'
                               , input_keys_sm
                               , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm)

        smach.StateMachine.add('release_bowl'
                               , release_bowl(
                                 bowl_release_position
                                 , input_keys_sm
                                 , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm)

        smach.StateMachine.add('get_utensil'
                          , get_utensil(
                            True
                            , holder_position
                            , utensil_position
                            , input_keys_sm
                            , outcomes_sm)
                          , transitions=transition_sm
                          , remapping=remapping_sm)

        smach.StateMachine.add('bowl_transfer_initial_position'
                          , move_to_bowl_grasping_initial_position(
                            bowl_transfer_initial_position
                            , input_keys_sm
                            , outcomes_sm)
                          , transitions=transition_sm
                          , remapping=remapping_sm)
        
        smach.StateMachine.add('move_to_feeding_start_position'
                               , move_to_feeding_start_position(
                                 sm.userdata.feeding_initial_position
                                 , input_keys_sm
                                 , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm)
        
        smach.StateMachine.add('food_item_selector'
                               , food_item_selector(
                                 'update_anygrasp'
                                , input_keys_sm
                                , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm)

        if use_opt:
            smach.StateMachine.add('plan_to_pre_skewer_pose',
                                  plan_to_pre_skewer_pose(
                                    MOTION_PLANNER_SERVICE,
                                    kortex_motion_planning.srv.GenerateKortexMotionPlan,
                                    input_keys_sm,
                                    outcomes_sm,
                                    'pre_skewer_pose'),
                                  transitions=transition_sm,
                                  remapping=remapping_sm) 

            smach.StateMachine.add('execute_to_pre_skewer_pose'
                                   , execute_to_pre_skewer_pose(
                                     'motion_plan'
                                    , input_keys_sm
                                    , outcomes_sm)
                                   , transitions=transition_sm
                                   , remapping=remapping_sm)
            
            smach.StateMachine.add('plan_to_skewer_pose',
                                  plan_to_skewer_pose(
                                    MOTION_PLANNER_SERVICE,
                                    kortex_motion_planning.srv.GenerateKortexMotionPlan,
                                    input_keys_sm,
                                    outcomes_sm,
                                    'skewer_pose'),
                                  transitions=transition_sm,
                                  remapping=remapping_sm) 

            smach.StateMachine.add('execute_to_skewer_pose'
                                   , execute_to_skewer_pose(
                                     'motion_plan'
                                    , input_keys_sm
                                    , outcomes_sm)
                                   , transitions=transition_sm
                                   , remapping=remapping_sm)            

            smach.StateMachine.add('plan_to_post_skewer_pose',
                                  plan_to_post_skewer_pose(
                                    MOTION_PLANNER_SERVICE,
                                    kortex_motion_planning.srv.GenerateKortexMotionPlan,
                                    input_keys_sm,
                                    outcomes_sm,
                                    'post_skewer_pose'),
                                  transitions=transition_sm,
                                  remapping=remapping_sm)     
            
            smach.StateMachine.add('execute_to_post_skewer_pose'
                                   , execute_to_post_skewer_pose(
                                     'motion_plan'
                                    , input_keys_sm
                                    , outcomes_sm)
                                   , transitions=transition_sm
                                   , remapping=remapping_sm)
            
        else:
            smach.StateMachine.add('move_to_pre_skewer_pose',
                                  move_to_pre_skewer_pose(
                                    SIMPLE_CMPE_SERVICE,
                                    kortex_motion_planning.srv.KortexSimpleCmpe,
                                    input_keys_sm,
                                    outcomes_sm,
                                    'pre_skewer_pose'),
                                  transitions=transition_sm,
                                  remapping=remapping_sm)    


            smach.StateMachine.add('move_to_skewer_pose',
                                  move_to_skewer_pose(
                                    SIMPLE_CMPE_SERVICE,
                                    kortex_motion_planning.srv.KortexSimpleCmpe,
                                    input_keys_sm,
                                    outcomes_sm,
                                    'skewer_pose'),
                                  transitions=transition_sm,
                                  remapping=remapping_sm)    
            

            smach.StateMachine.add('move_to_post_skewer_pose',
                                  move_to_post_skewer_pose(
                                    SIMPLE_CMPE_SERVICE,
                                    kortex_motion_planning.srv.KortexSimpleCmpe,
                                    input_keys_sm,
                                    outcomes_sm,
                                    'post_skewer_pose'),
                                  transitions=transition_sm,
                                  remapping=remapping_sm)  

        smach.StateMachine.add('move_to_feeding_initial_position'
                               , move_to_feeding_initial_position(
                                 sm.userdata.feeding_initial_position
                               , input_keys_sm
                               , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm)

        smach.StateMachine.add('skewer_status_check'
                               , skewer_status_check
                               (sm.userdata.skewer_check_flag
                              , input_keys_sm
                              , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm)
        
        smach.StateMachine.add('plan_to_feeding_pose',
                              plan_to_feeding_pose(
                                MOTION_PLANNER_SERVICE,
                                kortex_motion_planning.srv.GenerateKortexMotionPlan,
                                input_keys_sm,
                                outcomes_sm,
                                'feeding_pose'),
                              transitions=transition_sm,
                              remapping=remapping_sm)           

        smach.StateMachine.add('execute_to_feeding_pose'
                               , execute_to_feeding_pose(
                                 'motion_plan'
                                , input_keys_sm
                                , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm)
        
    # Execute SMACH tree
    outcome = sm.execute()

    rospy.spin()


if __name__ == '__main__':
    main()
    