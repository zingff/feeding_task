#!/usr/bin/env python

from contextlib import nullcontext
from urllib import response
import anygrasp_generation
import kortex_motion_planning
import rospy

import threading

import smach
from smach import StateMachine
from smach_ros import ServiceState, SimpleActionState
import smach_ros

import std_srvs.srv
from tf2_msgs import msg
# from tree_of_thougnt.arm import output
import turtlesim.srv
import anygrasp_generation.srv
import kortex_motion_planning.srv
from geometry_msgs.msg import Pose, Transform
from trajectory_msgs.msg import JointTrajectory

outcomes_sm = [  'motion_generator'
               , 'grasp_generator'
               , 'motion_executor'
               , 'initial_motion_generator'
               , 'succeeded','aborted','preempted'
               ]

input_keys_sm = [  'motion_plan'
                 , 'success']

transition_sm = {
    'motion_generator': 'motion_generator'
  , 'grasp_generator': 'grasp_generator'
  , 'motion_executor': 'motion_executor'
  , 'initial_motion_generator': 'initial_motion_generator'
}

remapping_sm = {
    'motion_plan': 'motion_plan'
  , 'success': 'success'
}

class CustomStateMachine(smach.StateMachine):
    def __init__(self, custom_outcomes, input_keys=[], output_keys=[]):
        super(CustomStateMachine, self).__init__(custom_outcomes, input_keys, output_keys)

# Define states here
def motion_generator_callback(userdata, response):
    if response.success:
        return 'motion_executor'
    else:
        return 'motion_generator'
      
def motion_executor_callback(userdata, response):
    # if response.success:
    #     return 'wait'
    if response.success:
        return 'grasp_generator'
      
def grasp_generator_callback(userdata, response):
    # if response.success:
    #     return 'wait'
    if response.success:
        return 'initial_motion_generator'

class wait(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=outcomes_sm, input_keys=input_keys_sm, output_keys=input_keys_sm)
  
  def execute(self, ud):
    rospy.sleep(10)


class grasp_generator(smach_ros.ServiceState):
    def __init__(self, update_anygrasp, input_keys_sm, outcomes_sm):
        super(grasp_generator, self).__init__(
            service_name='/grasp_generator',
            service_spec=anygrasp_generation.srv.AnyGraspGeneration,
            request=anygrasp_generation.srv.AnyGraspGenerationRequest(update_anygrasp),
            response_slots=['anygrasp_transforms', 'success'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=grasp_generator_callback
            # response_cb=motion_generator_callback
        )

class motion_generator(smach_ros.ServiceState):
    def __init__(self, target_pose_, input_keys_sm, outcomes_sm):
        super(motion_generator, self).__init__(
            service_name='/motion_planning_server',
            service_spec=kortex_motion_planning.srv.GenerateKortexMotionPlan,
            request=kortex_motion_planning.srv.GenerateKortexMotionPlanRequest(target_pose_),
            response_slots=['motion_plan', 'success', 'message'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=motion_generator_callback
        )
        
        

class motion_executor(smach_ros.ServiceState):
    def __init__(self, request_key_, input_keys_sm, outcomes_sm):
        super(motion_executor, self).__init__(
            service_name='/motion_execution_server',
            service_spec=kortex_motion_planning.srv.ExecuteMotionPlan,
            # request=kortex_motion_planning.srv.GenerateKortexMotionPlanRequest(target_pose_),
            request_key=request_key_,
            response_slots=['success', 'message'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=motion_executor_callback
        )
        
  


def main():
    rospy.init_node('feeding_task_state_machine')

    # Create a SMACH state machine
    # sm = StateMachine(outcomes=['succeeded','aborted','preempted'])
    # sm = smach.StateMachine(outcomes=[])
    sm = CustomStateMachine(outcomes_sm, input_keys_sm, input_keys_sm)
    
    target_pose_ = Pose()
    target_pose_.position.x = -0.15135
    target_pose_.position.y = 0.235484
    target_pose_.position.z = 0.557796
    target_pose_.orientation.x = 0.3872724
    target_pose_.orientation.y = -0.4914169
    target_pose_.orientation.z = -0.604657
    target_pose_.orientation.w = 0.4928685
    
    target_pose_2 = Pose()
    target_pose_2.position.x = -0.15135
    target_pose_2.position.y = 0.235484
    target_pose_2.position.z = 0.457796
    target_pose_2.orientation.x = 0.3872724
    target_pose_2.orientation.y = -0.4914169
    target_pose_2.orientation.z = -0.604657
    target_pose_2.orientation.w = 0.4928685
    
    update_anygrasp = True

    sm.userdata.ud_motion_plan = JointTrajectory()
    # sm0.userdata.ud_motion_plan = JointTrajectory()
    sm.userdata.ud_success = False
    sm.userdata.ud_message = ''
    sm.userdata.ud_anygrasp_transforms = []
    
    sm.userdata.success = False
    sm.userdata.motion_plan = JointTrajectory()
    success = False
    motion_plan = JointTrajectory()

     

    # Open the container
    with sm:
      smach.StateMachine.add('motion_generator', motion_generator(target_pose_, input_keys_sm, outcomes_sm),
                             transitions=transition_sm,
                             remapping=remapping_sm)
      smach.StateMachine.add('motion_executor', motion_executor('motion_plan', input_keys_sm, outcomes_sm),
                        transitions=transition_sm,
                        remapping=remapping_sm)
      smach.StateMachine.add('grasp_generator', grasp_generator(update_anygrasp, input_keys_sm, outcomes_sm),
                        transitions=transition_sm,
                        remapping=remapping_sm)
      smach.StateMachine.add('initial_motion_generator', motion_generator(target_pose_2, input_keys_sm, outcomes_sm),
                             transitions=transition_sm,
                             remapping=remapping_sm)

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
