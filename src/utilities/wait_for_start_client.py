#!/usr/bin/env python
import rospy
from feeding_task.srv import WaitForStart, WaitForStartRequest


def call_skewer_status_check(skewer_checker_flag):
    rospy.wait_for_service('/fsm/wait_for_start')
    try:
        wait_for_start_ = rospy.ServiceProxy('/fsm/wait_for_start', WaitForStart)
        resp = wait_for_start_(skewer_checker_flag)
        return resp.start_success, resp.start_command
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('wait_for_start_client')
    # Example call to service
    skewer_checker_flag = True
    skewer_status, point_number = call_skewer_status_check(skewer_checker_flag)
    print(f"start success: {skewer_status}, start command: {point_number}")
