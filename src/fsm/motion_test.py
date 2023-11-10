#!/usr/bin/env python
"""
Description:

Usage:
    $> roslaunch turtle_nodes.launch
    $> ./executive_step_02.py

Output:
    [INFO] : State machine starting in initial state 'RESET' with userdata: 
                []
    [INFO] : State machine transitioning 'RESET':'succeeded'-->'SPAWN'
    [INFO] : State machine terminating 'SPAWN':'succeeded':'succeeded'

"""

from contextlib import nullcontext
from urllib import response
import kortex_motion_planning
import rospy

import threading

import smach
from smach import StateMachine
from smach_ros import ServiceState, SimpleActionState

import std_srvs.srv
from tf2_msgs import msg
import turtlesim.srv
from kortex_motion_planning.srv import ExecuteMotionPlan, ExecuteMotionPlanRequest, ExecuteMotionPlanResponse
from kortex_motion_planning.srv import GenerateKortexMotionPlan, GenerateKortexMotionPlanRequest, GenerateKortexMotionPlanResponse
import kortex_motion_planning.srv
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String

def main():
    rospy.init_node('feeding_task')

    # Create a SMACH state machine
    sm0 = StateMachine(outcomes=['succeeded','aborted','preempted'])
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

    
    sm0.userdata.ud_motion_plan = JointTrajectory()
    # sm0.userdata.ud_motion_plan = JointTrajectory()
    sm0.userdata.ud_success = False
    sm0.userdata.ud_message = ''
     

    # Open the container
    with sm0:
      StateMachine.add('MOTION_PLAN_GENERATION',
                      ServiceState('/motion_planning_server',
                                    kortex_motion_planning.srv.GenerateKortexMotionPlan
                                    , request=kortex_motion_planning.srv.GenerateKortexMotionPlanRequest(target_pose_)
                                    , response_slots=['motion_plan', 'success', 'message']
                      )
                      , remapping={'motion_plan': 'ud_motion_plan'
                                  , 'success': 'ud_success'
                                  , 'message': 'ud_message'}
                      , transitions={'succeeded': 'MOTION_PLAN_EXECUTION'}
                      )
      
      StateMachine.add('MOTION_PLAN_GENERATION_2',
                      ServiceState('/motion_planning_server'
                                    , kortex_motion_planning.srv.GenerateKortexMotionPlan
                                    , request=kortex_motion_planning.srv.GenerateKortexMotionPlanRequest(target_pose_2)
                                    , response_slots=['motion_plan', 'success', 'message']
                      )
                      , remapping={'motion_plan': 'ud_motion_plan'
                                  , 'success': 'ud_success'
                                  , 'message': 'ud_message'}
                      , transitions={'succeeded': 'MOTION_PLAN_EXECUTION'}
                      )


      StateMachine.add('MOTION_PLAN_EXECUTION',
                      ServiceState('/motion_execution_server',
                                    kortex_motion_planning.srv.ExecuteMotionPlan
                                    , request_key='ud_execute_motion_plan_request'
                                    , response_slots=['success', 'message']
                      )
                      , remapping={'ud_execute_motion_plan_request': 'ud_motion_plan'}
                      , transitions={'succeeded': 'MOTION_PLAN_GENERATION_2'}
                      )



    # Execute SMACH tree
    outcome = sm0.execute()

    # Signal ROS shutdown (kill threads in background)
    rospy.signal_shutdown('All done.')

if __name__ == '__main__':
    main()
