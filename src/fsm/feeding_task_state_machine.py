#!/usr/bin/env python

# Note: a finite state machine for feeding task
# Note: name of the manipulator: sausage lip arm (SLA)

# TODO List:
# * concurrent sm for collision detection (cd is not always necessary)
# . add a joint planner in kmp \/
# . modify the skewer action in a cb in food item selector \/
# . add a loop for skewer task \/
 
import anygrasp_generation
import kortex_motion_planning
from kortex_motion_planning.msg import JointPositions
import rospy
import inspect
import threading
import smach
from smach import StateMachine
from smach_ros import ServiceState, SimpleActionState
import smach_ros
import std_srvs.srv
from tf2_msgs import msg
import anygrasp_generation.srv
import kortex_motion_planning.srv
from geometry_msgs.msg import Pose, Transform, Quaternion, Vector3
from trajectory_msgs.msg import JointTrajectory
import tf2_ros
import math
import tf.transformations


outcomes_sm = [
      'motion_generator'
    , 'food_item_selector'
    , 'motion_executor'  # deprecated
    , 'initial_motion_generator'  # deprecated
    , 'succeeded'
    , 'aborted'
    , 'preempted'
    , 'joint_motion_generator'  # deprecated
    
    , 'move_to_feeding_start_position'
    , 'plan_to_pre_skewer_pose'
    , 'plan_to_skewer_pose'
    , 'plan_to_post_skewer_pose'
    , 'execute_to_pre_skewer_pose'
    , 'execute_to_skewer_pose'
    , 'execute_to_post_skewer_pose'
    , 'move_to_feeding_initial_position'
    , 'plan_to_feeding_pose'
    , 'execute_to_feeding_pose'
               ]


# Input and output keys for the states
input_keys_sm = [
      'motion_plan'
    , 'success'
    , 'update_anygrasp'
    # , 'update_anygrasp'  # deprecated
    , 'success'
    , 'anygrasp_transforms'
    # , 'success'  # deprecated
    # , 'anygrasp_transforms'  # deprecated
    , 'anygrasp_transform'
    # , 'anygrasp_transform'  # deprecated
    # , 'pre_skewer_pose'  # deprecated
    , 'pre_skewer_pose'
    , 'skewer_pose'
    , 'post_skewer_pose'
    , 'feeding_pose'
                 ]


transition_sm = {
      'motion_generator': 'motion_generator'  # deprecated
    , 'food_item_selector': 'food_item_selector'
    , 'motion_executor': 'motion_executor'  # deprecated
    , 'initial_motion_generator': 'initial_motion_generator'  # deprecated
    , 'joint_motion_generator' : 'joint_motion_generator'  # deprecated
    
    , 'move_to_feeding_start_position': 'move_to_feeding_start_position'
    , 'plan_to_pre_skewer_pose': 'plan_to_pre_skewer_pose'
    , 'plan_to_skewer_pose': 'plan_to_skewer_pose'
    , 'plan_to_post_skewer_pose': 'plan_to_post_skewer_pose'
    , 'execute_to_pre_skewer_pose': 'execute_to_pre_skewer_pose'
    , 'execute_to_skewer_pose': 'execute_to_skewer_pose'
    , 'execute_to_post_skewer_pose': 'execute_to_post_skewer_pose'    
    , 'move_to_feeding_initial_position' : 'move_to_feeding_initial_position'
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
    # , 'pre_skewer_pose' : 'pre_skewer_pose' # deprecated
    , 'pre_skewer_pose': 'pre_skewer_pose'
    , 'skewer_pose': 'skewer_pose'
    , 'feeding_pose': 'feeding_pose'
}


def transform_to_matrix(transform):
    """
    Converts a Transform object to a 4x4 transformation matrix.
    """
    translation = [transform.translation.x, transform.translation.y, transform.translation.z]
    rotation = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
    return tf.transformations.compose_matrix(translate=translation, angles=tf.transformations.euler_from_quaternion(rotation))


def matrix_to_transform(matrix):
    """
    Converts a 4x4 transformation matrix to a Transform object.
    """
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
    success = '\033[1;92m'  # 35m
    failure = '\033[1;91m' 
    default = '\033[0m'  
  
  
def success_loginfo(msg):
    rospy.loginfo(StateOutputStyle.default + "Please check the outcome of current state" + StateOutputStyle.success)
    rospy.loginfo(StateOutputStyle.success + msg + StateOutputStyle.default)


def failure_loginfo(msg):
    rospy.loginfo(StateOutputStyle.failure + msg + StateOutputStyle.default)


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
        success_loginfo(f"{state_name}: failed")
        userdata.success = not success_flag_value
        return next_state_on_failure


# def motion_generator_callback(userdata, response):
#   return generic_state_callback(userdata, 
#                                 response, 
#                                 'motion_executor',
#                                 'motion_generator')


def food_item_selector_callback(userdata, response):
    return generic_userdata_state_callback(userdata, 
                                           response, 
                                           'plan_to_pre_skewer_pose', 
                                           'aborted', 
                                           True)


def move_to_feeding_start_position_callback(userdata, response):
    return generic_state_callback(userdata, 
                                  response, 
                                  'food_item_selector', 
                                  'aborted')


# def motion_executor_callback(userdata, response):  # deprecated
#   return generic_state_callback(userdata, 
#                                 response, 
#                                 'food_item_selector', 
#                                 'aborted')


def plan_to_pre_skewer_pose_callback(userdata, response):
  rospy.loginfo("Received pre_skewer_pose in state plan_to_pre_skewer_pose: %s", str(userdata.pre_skewer_pose))
  return generic_state_callback(userdata, 
                                response, 
                                'execute_to_pre_skewer_pose',
                                'plan_to_pre_skewer_pose')
  
  
def plan_to_skewer_pose_callback(userdata, response):
  rospy.loginfo("Received skewer_pose in state plan_to_skewer_pose: %s", str(userdata.pre_skewer_pose))
  return generic_state_callback(userdata, 
                                response, 
                                'execute_to_skewer_pose',
                                'plan_to_skewer_pose')
  

def plan_to_post_skewer_pose_callback(userdata, response):
  rospy.loginfo("Received post_skewer_pose in state plan_to_post_skewer_pose: %s", str(userdata.pre_skewer_pose))
  return generic_state_callback(userdata, 
                                response, 
                                'execute_to_post_skewer_pose',
                                'plan_to_post_skewer_pose')


def plan_to_feeding_pose_callback(userdata, response):
  rospy.loginfo("Received feeding_pose in state plan_to_feeding_pose: %s", str(userdata.pre_skewer_pose))
  return generic_state_callback(userdata, 
                                response, 
                                'execute_to_feeding_pose',
                                'plan_to_feeding_pose')

 
def execute_to_pre_skewer_pose_callback(userdata, response):
  return generic_state_callback(userdata, 
                                response, 
                                'plan_to_skewer_pose',
                                'plan_to_pre_skewer_pose')
  
def execute_to_skewer_pose_callback(userdata, response):
  return generic_state_callback(userdata, 
                                response, 
                                'move_to_feeding_initial_position',
                                'plan_to_skewer_pose')


def execute_to_post_skewer_pose_callback(userdata, response):
  return generic_state_callback(userdata, 
                                response, 
                                'move_to_feeding_initial_position',
                                'plan_to_post_skewer_pose') 

 
def execute_to_feeding_pose_callback(userdata, response):
  return generic_state_callback(userdata, 
                                response, 
                                'move_to_feeding_start_position',
                                'plan_to_feeding_pose') 

 
def move_to_feeding_initial_position_callback(userdata, response):
    return generic_state_callback(userdata, 
                                  response, 
                                  'plan_to_feeding_pose', 
                                  'aborted') 
  
# def motion_generator_callback(userdata, response):
#     if response.success:
#         rospy.loginfo("motion_generator: success")

#         return 'motion_executor'
#     else:
#         rospy.logwarn("motion_generator: failed")
#         return 'motion_generator'
   

# def move_to_feeding_initial_position_callback(userdata, response):
#     function_name = inspect.currentframe().f_code.co_name
#     state_name = function_name.replace('_callback', '')

#     if response.success:
#         rospy.loginfo(f"{state_name}: success")
#         return 'food_item_selector'
#     else:
#         rospy.logwarn(f"{state_name}: failed")
#         return 'aborted'


# def motion_executor_callback(userdata, response):
#     # if response.success:
#     #     return 'wait'
#     if response.success:
#         rospy.loginfo("motion_executor: success")
#         return 'grasp_generator'


# def food_item_selector_callback(userdata, response):
#     if response.success:
#         rospy.loginfo("food_item_selector: success")
#         userdata.success = True
#         return 'initial_motion_generator'
#     else:
#         rospy.logwarn("food_item_selector: failed")
#         userdata.success = False
#         return 'failed'


# class wait(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=outcomes_sm, input_keys=input_keys_sm, output_keys=input_keys_sm)

#     def execute(self, ud):
#         rospy.sleep(10)


class food_item_selector(smach_ros.ServiceState):
    def __init__(self, request_key_, input_keys_sm, outcomes_sm):
        super(food_item_selector, self).__init__(
            service_name='/grasp_generator',
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

    def execute(self, userdata):
        outcome = super(food_item_selector, self).execute(userdata)

        if userdata.success:
            userdata.anygrasp_transform = userdata.anygrasp_transforms[0]
            print('The best grasp from AnyGrasp: ')
            print(userdata.anygrasp_transform)
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
            q_grasp.x = grasp_quaternion_tf[0]
            q_grasp.y = grasp_quaternion_tf[1]
            q_grasp.z = grasp_quaternion_tf[2]
            q_grasp.w = grasp_quaternion_tf[3]
            # attributes = ['x', 'y', 'z', 'w']
            # for attr, value in zip(attributes, grasp_quaternion_tf):
            #     setattr(q_grasp, attr, value)
            
            # T_base_grasp
            T_base_grasp = Transform()
            T_base_grasp = userdata.anygrasp_transform 
            T_base_grasp.rotation = q_base_grasp
            print("T_base_grasp: ", T_base_grasp)
            
            # T_tool_grasp
            T_tool_grasp = Transform()
            p_tool_grasp = Vector3()
            p_tool_grasp.x = 0
            p_tool_grasp.y = 0 
            p_tool_grasp.z = 0.172
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
            
            print("T_tool_grasp: ", T_tool_grasp)
            print("T_grasp_tool: ", T_grasp_tool)
            
            # EE (tool_frame)
            T_base_tool = Transform()
            
            # T_base_tool = T_base_grasp * T_grasp_tool
            T_base_tool = matrix_to_transform(
                tf.transformations.concatenate_matrices(
                    transform_to_matrix(T_base_grasp),
                    transform_to_matrix(T_grasp_tool)
                )
            )
            print("p_base_tool: ", T_base_tool)            
            
            # pre-skewer
            pre_skewer_pose = Pose()
            pre_skewer_pose.position.x = T_base_tool.translation.x
            pre_skewer_pose.position.y = T_base_tool.translation.y
            pre_skewer_pose.position.z = T_base_tool.translation.z + 0.08
            pre_skewer_pose.orientation = q_grasp
            userdata.pre_skewer_pose = pre_skewer_pose
            print("pre_skewer_pose: ")
            print(userdata.pre_skewer_pose) 
            post_skewer_pose = Pose()
            pre_skewer_pose = pre_skewer_pose
            userdata.post_skewer_pose = post_skewer_pose
            print("pre_skewer_pose: ")
            print(userdata.post_skewer_pose) 
            
            # skewer
            skewer_pose = Pose()
            skewer_pose.position.x = T_base_tool.translation.x
            skewer_pose.position.y = T_base_tool.translation.y
            skewer_pose.position.z = T_base_tool.translation.z
            skewer_pose.orientation = q_grasp
            userdata.skewer_pose = skewer_pose
            print("skewer_pose: ")
            print(userdata.skewer_pose)             

            
            for i, transform in enumerate(userdata.anygrasp_transforms):
                anygrasp_tf_msg = tf2_ros.TransformStamped()
                anygrasp_tf_msg.header.stamp = rospy.Time.now()
                anygrasp_tf_msg.header.frame_id = 'base_link'
                anygrasp_tf_msg.child_frame_id = f'{self.grasp_namespace}/grasp_{i}'
                anygrasp_tf_msg.transform.translation = transform.translation
                anygrasp_tf_msg.transform.rotation = transform.rotation
                self.anygrasp_tf_broadcaster.sendTransform(anygrasp_tf_msg)

        return outcome


class move_to_feeding_start_position(smach_ros.ServiceState):
    def __init__(self, target_positions_, input_keys_sm, outcomes_sm):
        super(move_to_feeding_start_position, self).__init__(
            service_name='/kortex_simple_joint_motion_service',
            service_spec=kortex_motion_planning.srv.KortexSimpleJmpe,
            request=kortex_motion_planning.srv.KortexSimpleJmpeRequest(target_positions_),
            response_slots=['success'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=move_to_feeding_start_position_callback
        )

class move_to_feeding_initial_position(smach_ros.ServiceState):
    def __init__(self, target_positions_, input_keys_sm, outcomes_sm):
        super(move_to_feeding_initial_position, self).__init__(
            service_name='/kortex_simple_joint_motion_service',
            service_spec=kortex_motion_planning.srv.KortexSimpleJmpe,
            request=kortex_motion_planning.srv.KortexSimpleJmpeRequest(target_positions_),
            response_slots=['success'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=move_to_feeding_initial_position_callback
        )
    
        
# class plan_to_pre_skewer_pose(smach_ros.ServiceState):
#     def __init__(self, target_pose_, input_keys_sm, outcomes_sm, userdata):
#         super(plan_to_pre_skewer_pose, self).__init__(
#             service_name='/motion_planning_server',
#             service_spec=kortex_motion_planning.srv.GenerateKortexMotionPlan,
#             request=kortex_motion_planning.srv.GenerateKortexMotionPlanRequest(target_pose_),
#             response_slots=['motion_plan', 'success', 'message'],
#             outcomes=outcomes_sm,
#             input_keys=input_keys_sm,
#             output_keys=input_keys_sm,
#             response_cb=plan_to_pre_skewer_pose_callback
#         )

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
 
        
# class motion_generator(smach_ros.ServiceState):
#     def __init__(self, target_pose_, input_keys_sm, outcomes_sm):
#         super(motion_generator, self).__init__(
#             service_name='/motion_planning_server',
#             service_spec=kortex_motion_planning.srv.GenerateKortexMotionPlan,
#             request=kortex_motion_planning.srv.GenerateKortexMotionPlanRequest(target_pose_),
#             response_slots=['motion_plan', 'success', 'message'],
#             outcomes=outcomes_sm,
#             input_keys=input_keys_sm,
#             output_keys=input_keys_sm,
#             response_cb=motion_generator_callback
#         )


# class motion_executor(smach_ros.ServiceState):
#     def __init__(self, request_key_, input_keys_sm, outcomes_sm):
#         super(motion_executor, self).__init__(
#             service_name='/motion_execution_server',
#             service_spec=kortex_motion_planning.srv.ExecuteMotionPlan,
#             # request=kortex_motion_planning.srv.GenerateKortexMotionPlanRequest(target_pose_),
#             request_key=request_key_,
#             response_slots=['success', 'message'],
#             outcomes=outcomes_sm,
#             input_keys=input_keys_sm,
#             output_keys=input_keys_sm,
#             response_cb=motion_executor_callback
#         )


class execute_to_pre_skewer_pose(smach_ros.ServiceState):
    def __init__(self, request_key_, input_keys_sm, outcomes_sm):
        super(execute_to_pre_skewer_pose, self).__init__(
            service_name='/motion_execution_server',
            service_spec=kortex_motion_planning.srv.ExecuteMotionPlan,
            # request=kortex_motion_planning.srv.GenerateKortexMotionPlanRequest(target_pose_),
            request_key=request_key_,
            response_slots=['success', 'message'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=execute_to_pre_skewer_pose_callback
        )

class execute_to_skewer_pose(smach_ros.ServiceState):
    def __init__(self, request_key_, input_keys_sm, outcomes_sm):
        super(execute_to_skewer_pose, self).__init__(
            service_name='/motion_execution_server',
            service_spec=kortex_motion_planning.srv.ExecuteMotionPlan,
            # request=kortex_motion_planning.srv.GenerateKortexMotionPlanRequest(target_pose_),
            request_key=request_key_,
            response_slots=['success', 'message'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=execute_to_skewer_pose_callback
        )
        
class execute_to_post_skewer_pose(smach_ros.ServiceState):
    def __init__(self, request_key_, input_keys_sm, outcomes_sm):
        super(execute_to_post_skewer_pose, self).__init__(
            service_name='/motion_execution_server',
            service_spec=kortex_motion_planning.srv.ExecuteMotionPlan,
            # request=kortex_motion_planning.srv.GenerateKortexMotionPlanRequest(target_pose_),
            request_key=request_key_,
            response_slots=['success', 'message'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=execute_to_post_skewer_pose_callback
        )
        
class plan_to_feeding_pose(smach_ros.ServiceState):
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
            response_cb=plan_to_feeding_pose_callback
        )
        self.userdata_key = userdata_key
        
    def request_cb(self, userdata, *args):
          target_pose = getattr(userdata, self.userdata_key)
          return kortex_motion_planning.srv.GenerateKortexMotionPlanRequest(target_pose) 


# class plan_to_feeding_pose(smach_ros.ServiceState):
#     def __init__(self, request_key_, input_keys_sm, outcomes_sm):
#         super(plan_to_feeding_pose, self).__init__(
#             service_name='/motion_execution_server',
#             service_spec=kortex_motion_planning.srv.ExecuteMotionPlan,
#             # request=kortex_motion_planning.srv.GenerateKortexMotionPlanRequest(target_pose_),
#             request_key=request_key_,
#             response_slots=['success', 'message'],
#             outcomes=outcomes_sm,
#             input_keys=input_keys_sm,
#             output_keys=input_keys_sm,
#             response_cb=plan_to_feeding_pose_callback
#         )        
        
        
class execute_to_feeding_pose(smach_ros.ServiceState):
    def __init__(self, request_key_, input_keys_sm, outcomes_sm):
        super(execute_to_feeding_pose, self).__init__(
            service_name='/motion_execution_server',
            service_spec=kortex_motion_planning.srv.ExecuteMotionPlan,
            # request=kortex_motion_planning.srv.GenerateKortexMotionPlanRequest(target_pose_),
            request_key=request_key_,
            response_slots=['success', 'message'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=execute_to_feeding_pose_callback
        )   
        



def main():
    rospy.init_node('feeding_task_state_machine')

    # Create a SMACH state machine
    sm = CustomStateMachine(outcomes_sm, input_keys=input_keys_sm, output_keys=input_keys_sm)

    # Initialize necessary data in feeding task
    # food_transfer
    feeding_pose = Pose()
    feeding_pose.position.x = -0.15135
    feeding_pose.position.y = 0.235484
    feeding_pose.position.z = 0.557796
    feeding_pose.orientation.x = 0.3872724
    feeding_pose.orientation.y = -0.4914169
    feeding_pose.orientation.z = -0.604657
    feeding_pose.orientation.w = 0.4928685

    target_pose_2 = Pose()  # deprecated
    target_pose_2.position.x = -0.15135
    target_pose_2.position.y = 0.235484
    target_pose_2.position.z = 0.457796
    target_pose_2.orientation.x = 0.3872724
    target_pose_2.orientation.y = -0.4914169
    target_pose_2.orientation.z = -0.604657
    target_pose_2.orientation.w = 0.4928685
    
    feeding_initial_position = JointPositions()
    feeding_initial_position.joint_positions = [0.021247, -0.26079, 3.15111, -2.14524, 0.060838, -0.90679, 1.58046]


    # update_anygrasp = True

    sm.userdata.motion_plan = JointTrajectory()
    sm.userdata.update_anygrasp = True
    sm.userdata.success = False
    sm.userdata.message = ''
    sm.userdata.anygrasp_transforms = []
    sm.userdata.anygrasp_transform = Transform()
    sm.userdata.pre_skewer_pose = Pose()
    sm.userdata.skewer_pose = Pose()
    sm.userdata.post_skewer_pose = Pose()
    
    # Remap variables
    sm.userdata.feeding_pose = feeding_pose
    sm.userdata.feeding_start_position = feeding_initial_position
    sm.userdata.target_pose_2 = target_pose_2
    sm.userdata.feeding_initial_position = feeding_initial_position
    # sm.userdata.update_anygrasp = update_anygrasp

    # Open the container
    with sm:

    
        
        # smach.StateMachine.add('motion_generator'  # deprecated
        #                        , motion_generator(sm.userdata.feeding_pose
        #                                           , input_keys_sm
        #                                           , outcomes_sm)
        #                        , transitions=transition_sm
        #                        , remapping=remapping_sm)

        
        # smach.StateMachine.add('plan_to_pre_skewer_pose'  # deprecated
        #                        , plan_to_pre_skewer_pose(sm.userdata.pre_skewer_pose
        #                                           , input_keys_sm
        #                                           , outcomes_sm
        #                                           , sm.userdata)
        #                        , transitions=transition_sm
        #                        , remapping=remapping_sm)

        smach.StateMachine.add('move_to_feeding_start_position'
                               , move_to_feeding_start_position(sm.userdata.feeding_initial_position
                                                  , input_keys_sm
                                                  , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm)

 
        smach.StateMachine.add('plan_to_pre_skewer_pose',
                              plan_to_pre_skewer_pose('/motion_planning_server',
                                                      kortex_motion_planning.srv.GenerateKortexMotionPlan,
                                                      input_keys_sm,
                                                      outcomes_sm,
                                                      'pre_skewer_pose'),
                              transitions=transition_sm,
                              remapping=remapping_sm) 

        
        smach.StateMachine.add('plan_to_skewer_pose',
                              plan_to_skewer_pose('/motion_planning_server',
                                                      kortex_motion_planning.srv.GenerateKortexMotionPlan,
                                                      input_keys_sm,
                                                      outcomes_sm,
                                                      'skewer_pose'),
                              transitions=transition_sm,
                              remapping=remapping_sm) 
        

        smach.StateMachine.add('plan_to_post_skewer_pose',
                              plan_to_post_skewer_pose('/motion_planning_server',
                                                      kortex_motion_planning.srv.GenerateKortexMotionPlan,
                                                      input_keys_sm,
                                                      outcomes_sm,
                                                      'post_skewer_pose'),
                              transitions=transition_sm,
                              remapping=remapping_sm)         


        smach.StateMachine.add('plan_to_feeding_pose',
                              plan_to_feeding_pose('/motion_planning_server',
                                                      kortex_motion_planning.srv.GenerateKortexMotionPlan,
                                                      input_keys_sm,
                                                      outcomes_sm,
                                                      'feeding_pose'),
                              transitions=transition_sm,
                              remapping=remapping_sm)           

        smach.StateMachine.add('move_to_feeding_initial_position'
                               , move_to_feeding_initial_position(sm.userdata.feeding_initial_position
                                                  , input_keys_sm
                                                  , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm)

        smach.StateMachine.add('execute_to_pre_skewer_pose'
                               , execute_to_pre_skewer_pose('motion_plan'
                                                 , input_keys_sm
                                                 , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm)
        
        smach.StateMachine.add('execute_to_skewer_pose'
                               , execute_to_skewer_pose('motion_plan'
                                                 , input_keys_sm
                                                 , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm)
        

        smach.StateMachine.add('execute_to_post_skewer_pose'
                               , execute_to_post_skewer_pose('motion_plan'
                                                 , input_keys_sm
                                                 , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm)
        
        
        smach.StateMachine.add('execute_to_feeding_pose'
                               , execute_to_feeding_pose('motion_plan'
                                                 , input_keys_sm
                                                 , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm)

        
        smach.StateMachine.add('food_item_selector'
                               , food_item_selector('update_anygrasp'
                                                 , input_keys_sm
                                                 , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm)


        # smach.StateMachine.add('motion_executor'  # deprecated
        #                        , motion_executor('motion_plan'
        #                                          , input_keys_sm
        #                                          , outcomes_sm)
        #                        , transitions=transition_sm
        #                        , remapping=remapping_sm)
        
              
        # smach.StateMachine.add('motion_executor'  # deprecated
        #                        , motion_executor('motion_plan'
        #                                          , input_keys_sm
        #                                          , outcomes_sm)
        #                        , transitions=transition_sm
        #                        , remapping=remapping_sm)

               
        # smach.StateMachine.add('initial_motion_generator'  # deprecated
        #                        , motion_generator(sm.userdata.target_pose_2
        #                                           , input_keys_sm
        #                                           , outcomes_sm)
        #                        , transitions=transition_sm
        #                        , remapping=remapping_sm)

    # sis = smach_ros.IntrospectionServer('feeding_smach_introspection_server', sm, '/Start')
    # sis.start()

    # Execute SMACH tree
    outcome = sm.execute()

    # Signal ROS shutdown (kill threads in background)
    # rospy.signal_shutdown('All done.')
    # Wait for ctrl-C to stop the application
    rospy.spin()


if __name__ == '__main__':
    main()
