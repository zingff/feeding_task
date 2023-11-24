#!/usr/bin/env python

# This is a state machine demo for test
# Deprecated: 20231122

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
import anygrasp_generation.srv
import kortex_motion_planning.srv
from geometry_msgs.msg import Pose, Transform, Quaternion
from trajectory_msgs.msg import JointTrajectory
import tf2_ros

outcomes_sm = ['motion_generator'
    , 'grasp_generator'
    , 'motion_executor'
    , 'initial_motion_generator'
    , 'succeeded'
    , 'aborted'
    , 'preempted'
               ]

# Input and output keys for the states
input_keys_sm = ['motion_plan'
    # , 'ud_success'
    # , 'ud_update_anygrasp'
    , 'update_anygrasp'
    , 'success'
    , 'anygrasp_transforms'
    # , 'ud_anygrasp_transforms'
    , 'ud_success'
    , 'ud_anygrasp_transforms'
    , 'ud_anygrasp_transform'
    , 'anygrasp_transform'
                 ]

# output_keys=['ud_success', 'ud_anygrasp_transforms']

transition_sm = {
      'motion_generator': 'motion_generator'
    , 'grasp_generator': 'grasp_generator'
    , 'motion_executor': 'motion_executor'
    , 'initial_motion_generator': 'initial_motion_generator'
}

# from internal state machine to global variables
# keys used within the specific state: variable in sm's userdata (i.e., sm.userdata)
# io keys: userdata.data
remapping_sm = {
      'motion_plan': 'ud_motion_plan'
    , 'update_anygrasp': 'ud_update_anygrasp'
    , 'anygrasp_transforms': 'ud_anygrasp_transforms'
    , 'success': 'ud_success'
    , 'anygrasp_transform': 'ud_anygrasp_transform'
}


class CustomStateMachine(smach.StateMachine):
    def __init__(self, custom_outcomes, input_keys=[], output_keys=[]):
        super(CustomStateMachine, self).__init__(custom_outcomes, input_keys, output_keys)


# Define states here
def motion_generator_callback(userdata, response):
    if response.success:
        rospy.loginfo("motion_generator: success")

        return 'motion_executor'
    else:
        rospy.logwarn("motion_generator: failed")
        return 'motion_generator'


def motion_executor_callback(userdata, response):
    # if response.success:
    #     return 'wait'
    if response.success:
        rospy.loginfo("motion_executor: success")
        return 'grasp_generator'


def grasp_generator_callback(userdata, response):
    if response.success:
        rospy.loginfo("grasp_generator: success")
        userdata.ud_success = True
        return 'initial_motion_generator'
    else:
        rospy.logwarn("grasp_generator: failed")
        userdata.ud_success = False
        return 'failed'


class wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=outcomes_sm, input_keys=input_keys_sm, output_keys=input_keys_sm)

    def execute(self, ud):
        rospy.sleep(10)


class grasp_generator(smach_ros.ServiceState):
    def __init__(self, request_key_, input_keys_sm, outcomes_sm):
        super(grasp_generator, self).__init__(
            service_name='/grasp_generator',
            service_spec=anygrasp_generation.srv.AnyGraspGeneration,
            # request=anygrasp_generation.srv.AnyGraspGenerationRequest(update_anygrasp),
            request_key=request_key_,
            response_slots=['anygrasp_transforms', 'success'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=grasp_generator_callback
            # response_cb=motion_generator_callback
        )
        self.anygrasp_tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.grasp_namespace = 'anygrasp'

    def execute(self, userdata):
        # Call parent execute (makes the service call)
        outcome = super(grasp_generator, self).execute(userdata)

        # Publish TF transforms if the service call was successful
        if userdata.success:
            userdata.ud_anygrasp_transform = userdata.anygrasp_transforms[0]
            print('The best grasp from AnyGrasp: ')
            print(userdata.anygrasp_transform)
            # Publish each transform once
            for i, transform in enumerate(userdata.anygrasp_transforms):
                anygrasp_tf_msg = tf2_ros.TransformStamped()
                anygrasp_tf_msg.header.stamp = rospy.Time.now()
                anygrasp_tf_msg.header.frame_id = 'base_link'
                anygrasp_tf_msg.child_frame_id = f'{self.grasp_namespace}/grasp_{i}'
                anygrasp_tf_msg.transform.translation = transform.translation
                anygrasp_tf_msg.transform.rotation = transform.rotation
                self.anygrasp_tf_broadcaster.sendTransform(anygrasp_tf_msg)

        return outcome


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

    target_pose_2 = Pose()
    target_pose_2.position.x = -0.15135
    target_pose_2.position.y = 0.235484
    target_pose_2.position.z = 0.457796
    target_pose_2.orientation.x = 0.3872724
    target_pose_2.orientation.y = -0.4914169
    target_pose_2.orientation.z = -0.604657
    target_pose_2.orientation.w = 0.4928685

    # update_anygrasp = True

    sm.userdata.ud_motion_plan = JointTrajectory()
    sm.userdata.ud_update_anygrasp = True
    sm.userdata.ud_success = False
    sm.userdata.ud_message = ''
    sm.userdata.ud_anygrasp_transforms = []
    sm.userdata.ud_anygrasp_transform = Transform()
    # Skewer action
    sm.userdata.ud_pre_skewer_pose = Pose()
    sm.userdata.ud_skewer_pose = Pose()

    # Remap variables
    sm.userdata.ud_feeding_pose = feeding_pose
    sm.userdata.ud_target_pose_2 = target_pose_2
    # sm.userdata.ud_update_anygrasp = update_anygrasp

    # Open the container
    with sm:
        smach.StateMachine.add('motion_generator'
                               , motion_generator(sm.userdata.ud_feeding_pose
                                                  , input_keys_sm
                                                  , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm)
        smach.StateMachine.add('motion_executor'
                               , motion_executor('ud_motion_plan'
                                                 , input_keys_sm
                                                 , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm)
        smach.StateMachine.add('grasp_generator'
                               , grasp_generator('update_anygrasp'
                                                 , input_keys_sm
                                                 , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm)
        smach.StateMachine.add('initial_motion_generator'
                               , motion_generator(sm.userdata.ud_target_pose_2
                                                  , input_keys_sm
                                                  , outcomes_sm)
                               , transitions=transition_sm
                               , remapping=remapping_sm)

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
