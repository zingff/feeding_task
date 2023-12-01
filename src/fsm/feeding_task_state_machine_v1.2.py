#!/usr/bin/env python

# Note: a finite state machine for feeding task
# Note: name of the manipulator: sausage lip arm (SLA)

# TODO List: 
# √ concurrent sm for collision detection (cd is not always necessary)
# √ add a joint planner in kmp 
# √ modify the skewer action in a cb in food item selector 
# √ add a loop for skewer task 
# √ add simple cpe to replace the original one 
# * modify the cd to be dynamic tuning mode
# * add a function to check if a skewer is successful (pc)

# version note: just the basic state machine for feeding cycle

from email import message
from re import I
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
from std_msgs.msg import Bool
import threading

collision_detected = False
COLLISION_DETECTION_TOPIC = "/kortex_motion_planning/collision_detection"

outcomes_sm = [
      'motion_generator'
    , 'food_item_selector'
    , 'succeeded'
    , 'aborted'
    , 'preempted'
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
    , 'collision_detected'
    , 'normal'
    , 'collision_status'
    , 'collision_detection'
    , 'move_to_pre_skewer_pose'
    , 'move_to_skewer_pose'
    , 'move_to_post_skewer_pose'
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

                 ]


transition_sm = {
    'food_item_selector': 'food_item_selector'
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
    , 'aborted': 'aborted'
    , 'collision_detected': 'aborted'
    , 'collision_detection': 'collision_detection'
    , 'move_to_pre_skewer_pose': 'move_to_pre_skewer_pose'
    , 'move_to_skewer_pose': 'move_to_skewer_pose'
    , 'move_to_post_skewer_pose': 'move_to_post_skewer_pose'
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
    collision2 = '\033[1;38;5;195m' # near white
    striking  = '\033[1;38;5;190m'
    normal = '\033[38;5;152m'
  
  
def success_loginfo(msg):
    rospy.loginfo(StateOutputStyle.success + msg + StateOutputStyle.default)


def failure_loginfo(msg):
    rospy.loginfo(StateOutputStyle.failure + msg + StateOutputStyle.default)

    
def striking_loginfo(msg):
    rospy.loginfo(StateOutputStyle.striking + msg + StateOutputStyle.default)

    
def collision_detected_loginfo(msg):
    rospy.loginfo(StateOutputStyle.collision + "Collision detected in state: "  + msg  + ", aborting..." + StateOutputStyle.default)


def normal_loginfo(msg):
    rospy.loginfo(StateOutputStyle.normal + msg + StateOutputStyle.default)


def degrees2Radians(degrees):
    return (math.pi / 180.0) * degrees


class CustomStateMachine(smach.StateMachine):
    def __init__(self, custom_outcomes, input_keys=[], output_keys=[]):
        super(CustomStateMachine, self).__init__(custom_outcomes, input_keys, output_keys)


# TODO: override the execute function for state with collision detection requirement
class ServiceStateWithCollisionDetection(ServiceState):
    def __init__(self, service_name, service_spec, request_key=None, response_slots=None, outcomes=None, input_keys=None, output_keys=None, response_cb=None, request_cb=None):
        super(ServiceStateWithCollisionDetection, self).__init__(
            service_name=service_name,
            service_spec=service_spec,
            request_key=request_key,
            response_slots=response_slots,
            outcomes=outcomes,
            input_keys=input_keys,
            output_keys=output_keys,
            response_cb=response_cb,
            request_cb=request_cb
        )
        self._proxy = rospy.ServiceProxy(service_name, service_spec)
        rospy.wait_for_service(service_name, timeout=5.0)  # Wait for the service to become available

    def execute(self, userdata):
        global collision_detected
        collision_detected = False

        # Function to run service call in a separate thread
        def service_call():
            global collision_detected
            try:
                self._response = self._proxy(self._request)
            except rospy.ServiceException as ex:
                rospy.logerr("Exception when calling service '%s': %s" % (self._service_name, str(ex)))
                self._response = None

        # Start the service call in a separate thread
        service_thread = threading.Thread(target=service_call)
        service_thread.start()

        # Main loop for collision detection
        while service_thread.is_alive():
            if collision_detected:
                rospy.loginfo("Collision detected during service execution")
                return 'aborted'
            rospy.sleep(0.1)

        # Check if service call was successful
        if self._response is None:
            return 'aborted'

        # Process the response here
        # ...

        return 'succeeded'


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
        failure_loginfo(f"{state_name}: failed")
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
                                           'move_to_pre_skewer_pose', 
                                           'aborted', 
                                           True)


def move_to_feeding_start_position_callback(userdata, response):
    return generic_state_callback(userdata, 
                                  response, 
                                  'food_item_selector', 
                                  'aborted')





def plan_to_pre_skewer_pose_callback(userdata, response):
  # rospy.loginfo("Received pre_skewer_pose in state plan_to_pre_skewer_pose: %s", str(userdata.pre_skewer_pose))
  return generic_state_callback(userdata, 
                                response, 
                                'execute_to_pre_skewer_pose',
                                'plan_to_pre_skewer_pose')
  
  
def plan_to_skewer_pose_callback(userdata, response):
  # rospy.loginfo("Received skewer_pose in state plan_to_skewer_pose: %s", str(userdata.skewer_pose))
  return generic_state_callback(userdata, 
                                response, 
                                'execute_to_skewer_pose',
                                'plan_to_skewer_pose')
  

def plan_to_post_skewer_pose_callback(userdata, response):
  # rospy.loginfo("Received post_skewer_pose in state plan_to_post_skewer_pose: %s", str(userdata.post_skewer_pose))
  return generic_state_callback(userdata, 
                                response, 
                                'execute_to_post_skewer_pose',
                                'plan_to_post_skewer_pose')


def plan_to_feeding_pose_callback(userdata, response):
  # rospy.loginfo("Received feeding_pose in state plan_to_feeding_pose: %s", str(userdata.pre_skewer_pose))
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
            # print("T_base_grasp: ", T_base_grasp)
            
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

        outcome = super(food_item_selector, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        
        if collision_detected:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            return 'aborted'
        
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

        outcome = super(move_to_feeding_start_position, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        
        if collision_detected:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            return 'move_to_feeding_start_position'
        
        return outcome


def move_to_pre_skewer_pose_callback(userdata, response):
    return generic_state_callback(userdata, 
                                  response, 
                                  'move_to_skewer_pose', 
                                  'aborted')


class move_to_pre_skewer_pose(smach_ros.ServiceState):
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
          response_cb=move_to_pre_skewer_pose_callback
        )
        self.userdata_key = userdata_key
        
    def request_cb(self, userdata, *args):
        target_pose = getattr(userdata, self.userdata_key)
        return kortex_motion_planning.srv.KortexSimpleCmpeRequest(target_pose)
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

        outcome = super(move_to_pre_skewer_pose, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        
        if collision_detected:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            return 'move_to_pre_skewer_pose'
        
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

        outcome = super(move_to_skewer_pose, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        
        if collision_detected:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            return 'move_to_skewer_pose'
        
        return outcome


def move_to_post_skewer_pose_callback(userdata, response):
    return generic_state_callback(userdata, 
                                  response, 
                                  'move_to_feeding_initial_position', 
                                  'aborted')


class move_to_post_skewer_pose(smach_ros.ServiceState):
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
          response_cb=move_to_post_skewer_pose_callback
        )
        self.userdata_key = userdata_key
        
    def request_cb(self, userdata, *args):
        target_pose = getattr(userdata, self.userdata_key)
        return kortex_motion_planning.srv.KortexSimpleCmpeRequest(target_pose)
      
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

        outcome = super(move_to_post_skewer_pose, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        
        if collision_detected:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            return 'move_to_post_skewer_pose'
        
        return outcome


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

        outcome = super(move_to_feeding_initial_position, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        
        if collision_detected:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            return 'move_to_feeding_initial_position'
        
        return outcome





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

        outcome = super(plan_to_feeding_pose, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        
        if collision_detected:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            return 'plan_to_feeding_pose'
        
        return outcome

     
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

        outcome = super(execute_to_feeding_pose, self).execute(userdata)

        collision_thread_stop.set()
        collision_thread.join()
        
        if collision_detected:
            collision_detected_loginfo(self.__class__.__name__)
            outcome = 'aborted'
            return 'plan_to_feeding_pose'
        
        return outcome


def main():
    rospy.init_node('feeding_task_state_machine')

    sm = CustomStateMachine(outcomes_sm, input_keys=input_keys_sm, output_keys=input_keys_sm)

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

    # update_anygrasp = True
    # rospy.Subscriber('/kortex_motion_planning/collision_detection', Bool, lambda data: collision_detection_callback(data, sm))
    # collision_subscriber = rospy.Subscriber('/kortex_motion_planning/collision_detection', Bool, collision_detection_callback, callback_args=(collision_status))
    
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
    
    # Remap variables
    sm.userdata.feeding_pose = feeding_pose
    sm.userdata.feeding_start_position = feeding_initial_position
    sm.userdata.feeding_initial_position = feeding_initial_position
    # sm.userdata.update_anygrasp = update_anygrasp
    
    # collision_detection_thread = threading.Thread(target=collision_detection_callback)
    # collision_detection_thread.start()

    # Open the container
    with sm:    
        smach.StateMachine.add('move_to_feeding_start_position'
                               , move_to_feeding_start_position(sm.userdata.feeding_initial_position
                                                  , input_keys_sm
                                                  , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm)


        smach.StateMachine.add('move_to_pre_skewer_pose',
                              move_to_pre_skewer_pose('/kortex_simple_cartesian_motion_service',
                                                      kortex_motion_planning.srv.KortexSimpleCmpe,
                                                      input_keys_sm,
                                                      outcomes_sm,
                                                      'pre_skewer_pose'),
                              transitions=transition_sm,
                              remapping=remapping_sm)    


        smach.StateMachine.add('move_to_skewer_pose',
                              move_to_skewer_pose('/kortex_simple_cartesian_motion_service',
                                                      kortex_motion_planning.srv.KortexSimpleCmpe,
                                                      input_keys_sm,
                                                      outcomes_sm,
                                                      'skewer_pose'),
                              transitions=transition_sm,
                              remapping=remapping_sm)    
        

        smach.StateMachine.add('move_to_post_skewer_pose',
                              move_to_post_skewer_pose('/kortex_simple_cartesian_motion_service',
                                                      kortex_motion_planning.srv.KortexSimpleCmpe,
                                                      input_keys_sm,
                                                      outcomes_sm,
                                                      'post_skewer_pose'),
                              transitions=transition_sm,
                              remapping=remapping_sm)   

 
        # smach.StateMachine.add('plan_to_pre_skewer_pose',
        #                       plan_to_pre_skewer_pose('/motion_planning_server',
        #                                               kortex_motion_planning.srv.GenerateKortexMotionPlan,
        #                                               input_keys_sm,
        #                                               outcomes_sm,
        #                                               'pre_skewer_pose'),
        #                       transitions=transition_sm,
        #                       remapping=remapping_sm) 

        
        # smach.StateMachine.add('plan_to_skewer_pose',
        #                       plan_to_skewer_pose('/motion_planning_server',
        #                                               kortex_motion_planning.srv.GenerateKortexMotionPlan,
        #                                               input_keys_sm,
        #                                               outcomes_sm,
        #                                               'skewer_pose'),
        #                       transitions=transition_sm,
        #                       remapping=remapping_sm) 
        

        # smach.StateMachine.add('plan_to_post_skewer_pose',
        #                       plan_to_post_skewer_pose('/motion_planning_server',
        #                                               kortex_motion_planning.srv.GenerateKortexMotionPlan,
        #                                               input_keys_sm,
        #                                               outcomes_sm,
        #                                               'post_skewer_pose'),
        #                       transitions=transition_sm,
        #                       remapping=remapping_sm)         


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

        # smach.StateMachine.add('execute_to_pre_skewer_pose'
        #                        , execute_to_pre_skewer_pose('motion_plan'
        #                                          , input_keys_sm
        #                                          , outcomes_sm)
        #                        , transitions=transition_sm
        #                        , remapping=remapping_sm)
        
        # smach.StateMachine.add('execute_to_skewer_pose'
        #                        , execute_to_skewer_pose('motion_plan'
        #                                          , input_keys_sm
        #                                          , outcomes_sm)
        #                        , transitions=transition_sm
        #                        , remapping=remapping_sm)
        

        # smach.StateMachine.add('execute_to_post_skewer_pose'
        #                        , execute_to_post_skewer_pose('motion_plan'
        #                                          , input_keys_sm
        #                                          , outcomes_sm)
        #                        , transitions=transition_sm
        #                        , remapping=remapping_sm)
        
        
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

    # Execute SMACH tree
    outcome = sm.execute()

    rospy.spin()


if __name__ == '__main__':
    main()
